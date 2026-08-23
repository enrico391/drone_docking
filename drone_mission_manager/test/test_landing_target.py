"""Tests for native ArduPilot precision-landing message generation."""

import math
import struct

import pytest

from drone_mission_manager.landing_target import (
    camera_dock_orientation_to_body_yaw_error,
    camera_target_to_body_frd,
    decode_command_acks,
    encode_aux_function,
    encode_condition_yaw,
    encode_condition_yaw_absolute,
    encode_landing_target,
    measurement_time_usec,
    ros_enu_quaternion_to_mavlink_heading_deg,
)


def test_camera_target_converts_to_vehicle_body_frd():
    """Optical axes and the downward camera offset map into Body-FRD."""
    forward, right, down = camera_target_to_body_frd(
        camera_x_m=0.2,
        camera_y_m=-0.3,
        camera_z_m=2.0,
        camera_down_m=0.14,
    )

    assert forward == pytest.approx(0.3)
    assert right == pytest.approx(0.2)
    assert down == pytest.approx(2.14)


def test_landing_target_matches_known_mavlink2_frame():
    """Encoding matches a frame independently decoded by pymavlink."""
    packet = encode_landing_target(
        sequence=7,
        source_system=254,
        source_component=191,
        time_usec=123456,
        forward_m=1.25,
        right_m=-0.5,
        down_m=3.0,
    )

    assert packet.hex() == (
        "fd3c000007febf95000040e2010000000000000000000000000078725240"
        "0000000000000000000c0000a03f000000bf000040400000803f00000000"
        "0000000000000000020166fc"
    )


def test_measurement_timestamp_prefers_ros_acquisition_time():
    timestamp = measurement_time_usec(
        stamp_sec=12,
        stamp_nanosec=345_678_999,
        fallback_usec=99,
    )

    assert timestamp == 12_345_678


def test_measurement_timestamp_uses_fallback_for_empty_stamp():
    timestamp = measurement_time_usec(
        stamp_sec=0,
        stamp_nanosec=0,
        fallback_usec=99,
    )

    assert timestamp == 99


@pytest.mark.parametrize(
    ("quaternion", "expected_degrees"),
    [
        ((0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)), 0.0),
        ((0.0, 0.0, 0.0, 1.0), 90.0),
        ((0.0, 0.0, 1.0, 0.0), -90.0),
    ],
)
def test_camera_dock_orientation_converts_to_body_yaw_error(
    quaternion, expected_degrees
):
    yaw_error = camera_dock_orientation_to_body_yaw_error(*quaternion)

    assert math.degrees(yaw_error) == pytest.approx(expected_degrees)


def test_condition_yaw_encodes_relative_right_turn():
    packet = encode_condition_yaw(
        sequence=8,
        source_system=254,
        source_component=191,
        yaw_error_rad=math.radians(30.0),
    )

    assert packet.hex() == (
        "fd20000008febf4c00000000f041000000000000803f0000803f00000000"
        "000000000000000073000101b464"
    )


def test_absolute_condition_yaw_encodes_heading_and_rate():
    """An absolute command can be retried without accumulating rotation."""
    packet = encode_condition_yaw_absolute(
        sequence=9,
        source_system=254,
        source_component=191,
        heading_deg=120.0,
        yaw_rate_deg_s=5.0,
    )

    params = struct.unpack_from("<7fHBB", packet, 10)
    assert params[:4] == pytest.approx((120.0, 5.0, 0.0, 0.0))
    assert params[7:] == (115, 1, 1)


def test_aux_function_encodes_precision_loiter_high():
    """Auxiliary function 39 HIGH enables ArduPilot Precision Loiter."""
    packet = encode_aux_function(
        sequence=10,
        source_system=254,
        source_component=191,
        function=39,
        switch_position=2,
    )

    params = struct.unpack_from("<7fHBB", packet, 10)
    assert params[:2] == pytest.approx((39.0, 2.0))
    assert params[7:] == (218, 1, 1)


def test_command_ack_decoder_finds_mavlink2_ack():
    """The sender can confirm autonomous yaw and auxiliary commands."""
    payload = struct.pack("<HBBiBB", 218, 0, 100, 0, 254, 191)
    header = bytes((len(payload), 0, 0, 7, 1, 1, 77, 0, 0))
    datagram = bytes((0xFD,)) + header + payload + b"\x00\x00"

    assert decode_command_acks(datagram) == [(218, 0)]


@pytest.mark.parametrize(
    ("quaternion", "expected_heading_deg"),
    [
        ((0.0, 0.0, 0.0, 1.0), 90.0),
        ((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), 0.0),
        ((0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)), 180.0),
    ],
)
def test_ros_enu_yaw_converts_to_mavlink_heading(
    quaternion, expected_heading_deg
):
    heading = ros_enu_quaternion_to_mavlink_heading_deg(*quaternion)

    assert heading == pytest.approx(expected_heading_deg)

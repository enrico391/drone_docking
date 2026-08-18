"""Tests for native ArduPilot precision-landing message generation."""

import math

import pytest

from drone_mission_manager.landing_target import (
    camera_dock_orientation_to_body_yaw_error,
    camera_target_to_body_frd,
    encode_condition_yaw,
    encode_landing_target,
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

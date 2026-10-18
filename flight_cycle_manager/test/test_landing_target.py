"""Tests for transport-independent camera/body frame and timestamp helpers."""

import math

import pytest

from flight_cycle_manager.landing_target import (
    camera_dock_orientation_to_body_yaw_error,
    camera_target_to_body_frd,
    measurement_time_usec,
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

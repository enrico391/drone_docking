"""Tests for precision-landing frame conversion."""

import pytest

from flight_cycle_manager.control import camera_offset_to_body_velocity


def test_camera_right_commands_body_right():
    """Optical +X is body right, which is negative Y in FLU."""
    velocity_x, velocity_y = camera_offset_to_body_velocity(
        camera_x_m=0.2,
        camera_y_m=0.0,
        gain=1.0,
        max_speed_m_s=1.0,
    )

    assert velocity_x == pytest.approx(0.0)
    assert velocity_y == pytest.approx(-0.2)


def test_camera_bottom_commands_body_backward():
    """Optical +Y is body rear, which is negative X in FLU."""
    velocity_x, velocity_y = camera_offset_to_body_velocity(
        camera_x_m=0.0,
        camera_y_m=0.3,
        gain=1.0,
        max_speed_m_s=1.0,
    )

    assert velocity_x == pytest.approx(-0.3)
    assert velocity_y == pytest.approx(0.0)


def test_camera_correction_is_speed_limited():
    """Large visual errors cannot exceed the precision-flight speed limit."""
    velocity_x, velocity_y = camera_offset_to_body_velocity(
        camera_x_m=3.0,
        camera_y_m=4.0,
        gain=1.0,
        max_speed_m_s=0.5,
    )

    assert (velocity_x**2 + velocity_y**2) ** 0.5 == pytest.approx(0.5)

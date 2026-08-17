"""Tests for standalone dock-pose MAVLink message generation."""

import pytest

from dock_apriltag_localizer.dock_pose_mavlink import (
    camera_target_to_body_frd,
    encode_landing_target,
)


def test_camera_target_converts_to_vehicle_body_frd():
    """Optical axes and the downward camera offset map into Body-FRD."""
    forward, right, down = camera_target_to_body_frd(
        camera_x_m=0.2,
        camera_y_m=-0.3,
        camera_z_m=2.0,
        camera_down_m=0.206,
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

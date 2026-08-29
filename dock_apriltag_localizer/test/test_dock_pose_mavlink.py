"""Tests for the standalone dock-pose pymavlink transport."""

import pytest

from dock_apriltag_localizer.dock_pose_mavlink import (
    MavlinkLandingTargetSender,
    camera_target_to_body_frd,
    measurement_time_usec,
)


class FakeMav:
    def __init__(self):
        self.landing_targets = []

    def landing_target_send(self, *args):
        self.landing_targets.append(args)


class FakeConnection:
    def __init__(self):
        self.mav = FakeMav()
        self.closed = False

    def close(self):
        self.closed = True


class FakeMavlink:
    MAV_FRAME_BODY_FRD = 12


class FakeMavutil:
    mavlink = FakeMavlink()

    def __init__(self):
        self.connection = FakeConnection()
        self.connection_calls = []

    def mavlink_connection(self, endpoint, **kwargs):
        self.connection_calls.append((endpoint, kwargs))
        return self.connection


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
    assert down == pytest.approx(2.206)


def test_sender_opens_configured_direct_pymavlink_endpoint():
    """Break caught: the standalone bridge still depends on MAVProxy UDP."""
    mavutil = FakeMavutil()

    sender = MavlinkLandingTargetSender(
        connection="/dev/ttyAMA0",
        baud=921600,
        source_system=254,
        source_component=191,
        mavutil_module=mavutil,
    )

    assert mavutil.connection_calls == [
        (
            "/dev/ttyAMA0",
            {
                "baud": 921600,
                "source_system": 254,
                "source_component": 191,
                "dialect": "ardupilotmega",
                "autoreconnect": True,
                "robust_parsing": True,
            },
        )
    ]

    sender.close()
    assert mavutil.connection.closed is True


def test_sender_uses_generated_pymavlink_landing_target_message():
    """Break caught: target fields bypass or misuse pymavlink generation."""
    mavutil = FakeMavutil()
    sender = MavlinkLandingTargetSender(
        connection="tcp:127.0.0.1:5760",
        baud=921600,
        source_system=254,
        source_component=191,
        mavutil_module=mavutil,
    )

    sent = sender.send(
        time_usec=123456,
        forward_m=1.25,
        right_m=-0.5,
        down_m=3.0,
    )

    assert sent is True
    call = mavutil.connection.mav.landing_targets[-1]
    assert call[0] == 123456
    assert call[1] == 0
    assert call[2] == 12
    assert call[3:5] == (0.0, 0.0)
    assert call[5] == pytest.approx(3.288236609)
    assert call[6:8] == (0.0, 0.0)
    assert call[8:11] == pytest.approx((1.25, -0.5, 3.0))
    assert call[11] == (1.0, 0.0, 0.0, 0.0)
    assert call[12] == 2
    assert call[13] == 1


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

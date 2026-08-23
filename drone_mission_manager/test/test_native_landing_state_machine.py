"""Test native landing state ownership and recovery."""

import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from ardupilot_msgs.msg import Status

from drone_mission_manager.mission_manager import MissionManager, MissionState


MISSION_FILE = (
    Path(__file__).parents[1]
    / "config"
    / "missions"
    / "simple_circle.yaml"
)


class DoneFuture:
    """Minimal completed service future used by state transition tests."""

    def __init__(self, result) -> None:
        self._result = result

    def done(self) -> bool:
        """Report immediate completion."""
        return True

    def result(self):
        """Return the configured service response."""
        return self._result


class NoAckSender:
    """MAVLink sender stub that never receives a command acknowledgement."""

    def take_command_ack(self, command):
        """Model a lost COMMAND_ACK."""
        return None


@pytest.fixture(scope="module")
def manager():
    """Create one non-spinning mission manager for transition tests."""
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"mission_file:={MISSION_FILE}",
            ]
        )
    node = MissionManager()
    yield node
    node.destroy_node()
    if initialized_here and rclpy.ok():
        rclpy.shutdown()


def test_guided_hold_does_not_override_an_active_condition_yaw(
    manager, monkeypatch
):
    """No Twist yaw-rate command is published after autonomous yaw starts."""
    calls = []
    manager._pending = None
    manager._state = MissionState.ALIGN_DOCK_YAW
    manager._status = Status(armed=True, mode=4, flying=True)
    manager._has_seen_armed = True
    monkeypatch.setattr(
        manager, "_stream_native_landing_target", lambda now: None
    )
    monkeypatch.setattr(manager, "_pose_is_fresh", lambda now: True)
    monkeypatch.setattr(
        manager, "_publish_guided_hold", lambda: calls.append("hold")
    )
    monkeypatch.setattr(
        manager,
        "_update_native_yaw_alignment",
        lambda now: calls.append("yaw"),
    )

    manager._alignment_yaw_command_sent = True
    manager._update()

    assert calls == ["yaw"]


def test_land_mode_rejection_returns_to_precision_loiter(
    manager,
):
    """A rejected LOITER-to-LAND switch revalidates instead of descending."""
    now = time.monotonic()
    manager._state = MissionState.SET_LAND
    manager._pending_name = "land_mode"
    manager._pending = DoneFuture(
        SimpleNamespace(curr_mode=5, status=False)
    )
    manager._pending_deadline = now + 1.0

    manager._handle_pending_call(now)

    assert manager._state == MissionState.PRECISION_LOITER_ALIGN


def test_checkpoint_ack_loss_is_nonfatal(manager, monkeypatch):
    """Lost checkpoint ACK cannot stop the active native LAND operation."""
    now = time.monotonic()
    manager._state = MissionState.WAIT_LANDED
    monkeypatch.setattr(manager, "_landing_target_sender", NoAckSender())
    manager._begin_mavlink_command(
        command_id=115,
        label="test checkpoint",
        send=lambda: True,
        now=now,
        fatal=False,
    )

    assert manager._update_mavlink_command(now) == "pending"
    assert manager._update_mavlink_command(now + 4.0) == "failed"
    assert manager._state == MissionState.WAIT_LANDED

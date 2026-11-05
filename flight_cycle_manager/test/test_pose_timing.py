"""Measure the age of the exact pose passed to the real landing backend."""

import json

from geometry_msgs.msg import PoseStamped
import pytest
from std_msgs.msg import String

from flight_cycle_manager.pid_landing_backend import PidLandingBackend
from test_px4_manager import manager, telemetry


def prepare_landing(node, monkeypatch):
    telemetry(node, armed=True, landed=False, offboard=True, z=-3)
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.z = 3.0
    pose.pose.orientation.w = 1.0
    node._on_dock_pose(pose)
    node._on_dock_mode(String(data="OUTER"))
    node._landing = PidLandingBackend(node._pid_config)
    node._landing.start(now=node._now(), dock=node._dock(node._now()), vehicle=node._vehicle())
    node._state = "LANDING_BACKEND"
    captured = []
    monkeypatch.setattr(node._debug_pub, "publish", captured.append)
    stamp = pose.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec, captured


def test_publish_to_pid_includes_transport_and_timer_wait(manager, monkeypatch):
    stamp_ns, captured = prepare_landing(manager, monkeypatch)
    manager._on_dock_timing(String(data=json.dumps({
        "source_stamp_ns": stamp_ns, "pose_publish_monotonic_s": 99.99,
    })))
    manager.test_clock[0] = 100.025
    manager._advance_docking(manager._now())
    timing = json.loads(captured[-1].data)["pose_timing"]
    assert timing["source_stamp_ns"] == stamp_ns
    assert timing["publish_to_receive_s"] == pytest.approx(0.01)
    assert timing["receive_to_pid_s"] == pytest.approx(0.025)
    assert timing["publish_to_pid_s"] == pytest.approx(0.035)
    assert timing["consumed_monotonic_s"] == 100.025
    assert timing["source_age_at_pid_s"] >= 0
    # A second control update using the same frame must report its increased age.
    manager.test_clock[0] = 100.075
    manager._advance_docking(manager._now())
    timing = json.loads(captured[-1].data)["pose_timing"]
    assert timing["publish_to_pid_s"] == pytest.approx(0.085)
    assert timing["receive_to_pid_s"] == pytest.approx(0.075)


def test_control_entry_is_measured_after_observation_conversion(manager, monkeypatch):
    _, captured = prepare_landing(manager, monkeypatch)
    original_dock = manager._dock

    def delayed_dock(now):
        dock = original_dock(now)
        manager.test_clock[0] += 0.02
        return dock

    monkeypatch.setattr(manager, "_dock", delayed_dock)
    manager._advance_docking(100.0)
    timing = json.loads(captured[-1].data)["pose_timing"]
    assert timing["receive_to_pid_s"] == pytest.approx(0.02)


def test_timing_matches_exact_source_stamp_even_when_messages_arrive_out_of_order(
        manager, monkeypatch):
    stamp_ns, captured = prepare_landing(manager, monkeypatch)
    for source_ns, published in ((stamp_ns, 99.99), (stamp_ns + 1, 99.999)):
        manager._on_dock_timing(String(data=json.dumps({
            "source_stamp_ns": source_ns, "pose_publish_monotonic_s": published,
        })))
    manager._advance_docking(manager._now())
    assert json.loads(captured[-1].data)["pose_timing"]["publish_to_pid_s"] == pytest.approx(.01)


@pytest.mark.parametrize("payload", [
    "{}", "not json", "[]", "null",
    '{"source_stamp_ns":1,"pose_publish_monotonic_s":NaN}',
])
def test_missing_or_invalid_publication_timing_does_not_block_control(
        manager, monkeypatch, payload):
    _, captured = prepare_landing(manager, monkeypatch)
    manager._on_dock_timing(String(data=payload))
    manager.test_clock[0] += .025
    manager._advance_docking(manager._now())
    timing = json.loads(captured[-1].data)["pose_timing"]
    assert timing["publish_to_pid_s"] is None
    assert timing["publish_to_receive_s"] is None
    assert timing["receive_to_pid_s"] == pytest.approx(.025)
    assert manager._velocity is not None


def test_late_timing_is_used_on_next_update_and_stale_pose_clears_timing(manager, monkeypatch):
    stamp_ns, captured = prepare_landing(manager, monkeypatch)
    manager._advance_docking(manager._now())
    assert json.loads(captured[-1].data)["pose_timing"]["publish_to_pid_s"] is None
    manager._on_dock_timing(String(data=json.dumps({
        "source_stamp_ns": stamp_ns, "pose_publish_monotonic_s": 99.99,
    })))
    manager.test_clock[0] += .025
    manager._advance_docking(manager._now())
    assert json.loads(captured[-1].data)["pose_timing"]["publish_to_pid_s"] == pytest.approx(.035)
    manager.test_clock[0] += 2
    manager._advance_docking(manager._now())
    assert json.loads(captured[-1].data)["pose_timing"] is None


def test_future_publication_timestamp_is_unavailable(manager, monkeypatch):
    stamp_ns, captured = prepare_landing(manager, monkeypatch)
    manager._on_dock_timing(String(data=json.dumps({
        "source_stamp_ns": stamp_ns, "pose_publish_monotonic_s": 100.01,
    })))
    manager.test_clock[0] += .025
    manager._advance_docking(manager._now())
    timing = json.loads(captured[-1].data)["pose_timing"]
    assert timing["publish_to_pid_s"] is None
    assert timing["publish_to_receive_s"] is None

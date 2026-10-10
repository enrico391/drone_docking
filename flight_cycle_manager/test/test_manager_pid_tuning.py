"""Live tuning must affect the active backend, not only the parameter store."""

import json
from dataclasses import replace

import pytest

from vehicle_fakes import install_ros_fakes

install_ros_fakes()

import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from test_manager_pid_recovery import manager_at_landing_boundary  # noqa: E402
from test_manager_start import airborne_state  # noqa: E402
from test_pid_landing_backend import dock_observation  # noqa: E402
from flight_cycle_manager.pid_landing import PidLandingState  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_live_gains_change_active_output_and_reject_invalid_batch(monkeypatch):
    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock_observation(now))
        vehicle.state = airborne_state(1.1)
        manager.tick(1.1)
        assert vehicle.velocity_calls[-1][1] == pytest.approx(0.1)
        backend = manager.backend
        result = manager.set_parameters_atomically([
            Parameter("pid_x_kp", value=2.0), Parameter("pid_y_kp", value=2.0),
        ])
        assert result.successful
        for step in range(1, 12):
            now = 1.1 + step * 0.05
            vehicle.state = airborne_state(now)
            manager.tick(now)
        assert manager.backend is backend
        assert vehicle.velocity_calls[-1][1:3] == pytest.approx((0.2, -0.2))
        bad = manager.set_parameters_atomically([
            Parameter("pid_x_kp", value=3.0), Parameter("pid_y_ki", value=-0.1),
        ])
        assert not bad.successful
        assert manager.get_parameter("pid_x_kp").value == 2.0
        vehicle.state = airborne_state(1.7)
        manager.tick(1.7)
        assert vehicle.velocity_calls[-1][1] == pytest.approx(0.2)
    finally:
        manager.close()


def test_live_velocity_feedback_changes_active_backend_and_rejects_invalid_batch(monkeypatch):
    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock_observation(now))
        vehicle.state = airborne_state(1.1)
        manager.tick(1.1)
        backend = manager.backend
        result = manager.set_parameters_atomically([
            Parameter("pid_x_k_v", value=0.4),
            Parameter("pid_y_k_v", value=0.2),
            Parameter("pid_z_k_v", value=0.3),
        ])
        assert result.successful
        assert manager.backend is backend
        assert backend.diagnostics()["velocity_feedback"] == {
            "x": 0.4, "y": 0.2, "z": 0.3,
        }
        vehicle.state = replace(airborne_state(1.2), heading_rad=0.0,
                                velocity_enu_m_s=(-0.5, 0.5, 0.05))
        manager.tick(1.2)
        assert vehicle.velocity_calls[-1][1:3] == pytest.approx((-0.1, -0.2))
        result = manager.set_parameters_atomically([
            Parameter("pid_x_k_v", value=0.8),
            Parameter("pid_y_k_v", value=-0.1),
        ])
        assert not result.successful
        assert manager.get_parameter("pid_x_k_v").value == 0.4
        assert backend.diagnostics()["velocity_feedback"]["x"] == 0.4
    finally:
        manager.close()


def test_live_hold_changes_active_backend_and_rejects_passed_stage(monkeypatch):
    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        backend = manager.backend
        assert manager.set_parameters_atomically([
            Parameter("pid_hold_at_state", value="PID_ALIGN"),
        ]).successful
        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock_observation(now))
        for now in (1.1, 2.2):
            vehicle.state = airborne_state(now)
            manager.tick(now)
        assert manager.backend is backend
        assert backend._controller._state is PidLandingState.PID_ALIGN

        assert manager.set_parameters_atomically([
            Parameter("pid_hold_at_state", value="HOLD_CENTRAL"),
        ]).successful
        for now in (2.3, 3.4):
            vehicle.state = airborne_state(now)
            manager.tick(now)
        assert backend._controller._state is PidLandingState.DESCEND_OUTER
        assert backend._controller._hold_at_state is PidLandingState.HOLD_CENTRAL

        rejected = manager.set_parameters_atomically([
            Parameter("pid_hold_at_state", value="PID_ALIGN"),
            Parameter("pid_x_kp", value=2.0),
        ])
        assert not rejected.successful
        assert manager.get_parameter("pid_hold_at_state").value == "HOLD_CENTRAL"
        assert manager.get_parameter("pid_x_kp").value == 1.0
        assert backend._controller._hold_at_state is PidLandingState.HOLD_CENTRAL
        assert manager.set_parameters_atomically([
            Parameter("pid_hold_at_state", value="none"),
        ]).successful
        assert backend._controller._hold_at_state is None
        assert manager._pid_hold_at_state is None
    finally:
        manager.close()


def test_live_hold_rejects_updates_after_touchdown_commitment(monkeypatch):
    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        controller = manager.backend._controller
        controller._state = PidLandingState.TOUCHDOWN_APPROACH
        rejected = manager.set_parameters_atomically([
            Parameter("pid_hold_at_state", value="HOLD_CENTRAL"),
        ])
        assert not rejected.successful
        assert "commitment" in rejected.reason
        assert manager.get_parameter("pid_hold_at_state").value == "none"
        assert controller._hold_at_state is None
    finally:
        manager.close()


def test_live_fixed_reference_changes_active_backend_and_future_attempts(monkeypatch):
    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        backend = manager.backend
        assert manager.set_parameters_atomically([
            Parameter("pid_fixed_central_reference_mode", value="descend_central"),
        ]).successful
        assert backend.diagnostics()["fixed_central_reference_mode"] == "descend_central"
        assert manager._build_pid_landing_config().fixed_central_reference_mode == "descend_central"
        rejected = manager.set_parameters_atomically([
            Parameter("pid_fixed_central_reference_mode", value="invalid"),
            Parameter("pid_x_kp", value=3.0),
        ])
        assert not rejected.successful
        assert manager.get_parameter("pid_fixed_central_reference_mode").value == "descend_central"
        assert backend.diagnostics()["fixed_central_reference_mode"] == "descend_central"
        backend._controller._state = PidLandingState.TOUCHDOWN_APPROACH
        rejected = manager.set_parameters_atomically([
            Parameter("pid_fixed_central_reference_mode", value="off"),
        ])
        assert not rejected.successful
        assert "touchdown" in rejected.reason
        assert backend.diagnostics()["fixed_central_reference_mode"] == "descend_central"
    finally:
        manager.close()


def test_pid_diagnostics_match_command_and_body_velocity(monkeypatch):
    from dataclasses import replace
    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        published = []
        monkeypatch.setattr(
            manager._pid_debug_publisher,
            "publish",
            lambda msg: published.append(msg.data),
        )
        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock_observation(now))
        vehicle.state = replace(airborne_state(1.1), heading_rad=0.0)
        manager.tick(1.1)
        data = json.loads(published[-1])
        assert data["state"] == "PID_ALIGN"
        assert data["error"]["x"] == pytest.approx(0.1)
        assert data["command_body_flu"][0] == pytest.approx(0.1)
        assert data["measured_velocity_body_flu"] == [0.0, 0.0, 0.05]
        assert data["measured_dock_position"] == {
            "forward_m": 0.1, "left_m": -0.1, "distance_m": 2.0,
        }
        assert data["axes"]["x"]["p"] == pytest.approx(0.1)
        assert data["axes"]["x"]["kp"] == 1.0
        assert data["dock_age_s"] == 0.0
        monkeypatch.setattr(manager, "_dock_observation", lambda now: None)
        vehicle.state = airborne_state(1.2)
        manager.tick(1.2)
        missing = json.loads(published[-1])
        assert missing["measured_dock_position"] is None
        assert missing["measured_velocity_body_flu"] is None
        assert missing["measured_velocity_enu"] == [0.0, 0.0, 0.05]
    finally:
        manager.close()


def test_frame_age_uses_ros_clock_and_tracks_last_pose_during_loss(monkeypatch):
    from types import SimpleNamespace

    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        published = []
        clock = SimpleNamespace(nanoseconds=100_200_000_000)
        monkeypatch.setattr(manager, "get_clock", lambda: SimpleNamespace(now=lambda: clock),
                            raising=False)
        monkeypatch.setattr(manager._pid_debug_publisher, "publish",
                            lambda msg: published.append(json.loads(msg.data)))
        dock = replace(dock_observation(1.1), source_stamp_s=100.0)
        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock)
        vehicle.state = airborne_state(1.1)
        manager.tick(1.1)
        assert published[-1]["frame_age_s"] == pytest.approx(0.2)
        assert published[-1]["dock_age_s"] == 0.0
        assert published[-1]["frame_age_status"] == "ok"

        manager._dock_pose = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)))
        monkeypatch.setattr(manager, "_dock_observation", lambda now: None)
        clock.nanoseconds = 100_500_000_000
        vehicle.state = airborne_state(1.2)
        manager.tick(1.2)
        assert published[-1]["frame_age_s"] == pytest.approx(0.5)
        assert published[-1]["dock_usable"] is False

        clock.nanoseconds = 99_000_000_000
        vehicle.state = airborne_state(1.3)
        manager.tick(1.3)
        assert published[-1]["frame_age_s"] == -1.0
        assert published[-1]["frame_age_status"] == "future_stamp"

        manager._dock_pose.header.stamp.sec = 0
        vehicle.state = airborne_state(1.4)
        manager.tick(1.4)
        assert published[-1]["frame_age_s"] is None
        assert published[-1]["frame_age_status"] == "missing_stamp"
    finally:
        manager.close()


def test_live_distance_schedule_updates_and_invalid_batches(monkeypatch):
    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        monkeypatch.setattr(
            manager, "_dock_observation",
            lambda now: dock_observation(now, distance_m=2.0),
        )
        vehicle.state = airborne_state(1.1)
        manager.tick(1.1)
        backend = manager.backend
        result = manager.set_parameters_atomically([
            Parameter("pid_gain_near_x_kp", value=0.4),
            Parameter("pid_gain_far_x_kp", value=1.6),
            Parameter("pid_gain_far_distance_m", value=2.0),
        ])
        assert result.successful
        for step in range(1, 12):
            now = 1.1 + step * 0.05
            vehicle.state = airborne_state(now)
            manager.tick(now)
        assert manager.backend is backend
        assert backend.diagnostics()["axes"]["x"]["kp"] == pytest.approx(1.6)
        bad = manager.set_parameters_atomically([
            Parameter("pid_gain_near_x_kp", value=0.8),
            Parameter("pid_gain_far_distance_m", value=0.2),
        ])
        assert not bad.successful
        assert manager.get_parameter("pid_gain_near_x_kp").value == 0.4
        assert manager.get_parameter("pid_gain_far_distance_m").value == 2.0
        assert not manager.set_parameters_atomically([
            Parameter("pid_gain_far_x_ki", value=-0.5),
        ]).successful
    finally:
        manager.close()

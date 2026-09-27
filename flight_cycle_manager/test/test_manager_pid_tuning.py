"""Live tuning must affect the active backend, not only the parameter store."""

import json

import pytest

from vehicle_fakes import install_ros_fakes

install_ros_fakes()

import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from test_manager_pid_recovery import manager_at_landing_boundary  # noqa: E402
from test_manager_start import airborne_state  # noqa: E402
from test_pid_landing_backend import dock_observation  # noqa: E402


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
        assert data["axes"]["x"]["p"] == pytest.approx(0.1)
        assert data["axes"]["x"]["kp"] == 1.0
        assert data["dock_age_s"] == 0.0
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

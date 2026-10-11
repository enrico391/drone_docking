"""Approach-height settings must reach the controller and reject unsafe batches."""

import json
from types import SimpleNamespace

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from flight_cycle_manager.manager import CycleState, FlightCycleManager, GUIDED_MODE  # noqa: E402
from test_manager_start import airborne_state  # noqa: E402
from test_pid_landing_backend import dock_observation  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_startup_braking_settings_reach_backend_and_trace(tmp_path, monkeypatch):
    vehicle = FakeVehicle(airborne_state(2.0, mode=5))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_align_debug", "pid_align_auto_start": True,
        "pid_touchdown_commit_height_m": 0.35,
        "pid_approach_vertical_response_time_s": 0.7,
        "pid_approach_braking_accel_m_s2": 0.4,
        "pid_approach_height_tolerance_m": 0.015,
        "touchdown_trace_enabled": True, "landing_attempt_log_directory": str(tmp_path),
    })
    monkeypatch.setattr(manager, "get_clock", lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)), raising=False)
    try:
        manager.tick(2.0)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(2.1)
        manager.tick(2.1)
        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock_observation(now))
        vehicle.state = airborne_state(2.2)
        manager.tick(2.2)
        assert manager.state is CycleState.LANDING_BACKEND
        row = json.loads(next(tmp_path.glob("touchdown_*.jsonl")).read_text().splitlines()[-1])
        guard = row["diagnostics"]["approach_height_guard"]
        assert guard["height_m"] == 0.35
        assert guard["response_time_s"] == 0.7
        assert guard["braking_accel_m_s2"] == 0.4
        assert guard["arrival_tolerance_m"] == 0.015
    finally:
        manager.close()


@pytest.mark.parametrize("field", ["pid_approach_vertical_response_time_s",
                                  "pid_approach_braking_accel_m_s2",
                                  "pid_approach_height_tolerance_m"])
def test_braking_settings_are_startup_only_and_reject_atomic_tuning_batch(field):
    manager = FlightCycleManager(vehicle=FakeVehicle(), parameter_values={
        "operation_mode": "pid_align_debug",
    })
    try:
        previous = manager.get_parameter(field).value
        result = manager.set_parameters_atomically([
            Parameter(field, value=0.1), Parameter("pid_x_k_v", value=0.8),
        ])
        assert not result.successful
        assert "startup" in result.reason
        assert manager.get_parameter(field).value == previous
        assert manager.get_parameter("pid_x_k_v").value == 0.0
    finally:
        manager.close()


def test_below_floor_setpoint_rejected_before_mutating_active_feedback(monkeypatch):
    vehicle = FakeVehicle(airborne_state(2.0, mode=5))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_align_debug", "pid_align_auto_start": True,
        "pid_touchdown_commit_height_m": 0.35,
    })
    monkeypatch.setattr(manager, "get_clock", lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)), raising=False)
    try:
        manager.tick(2.0)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(2.1)
        manager.tick(2.1)
        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock_observation(now))
        vehicle.state = airborne_state(2.2)
        manager.tick(2.2)
        assert manager.state is CycleState.LANDING_BACKEND
        backend = manager.backend
        result = manager.set_parameters_atomically([
            Parameter("pid_x_k_v", value=0.8),
            Parameter("pid_align_distance_setpoint_m", value=0.3),
        ])
        assert not result.successful
        assert backend.diagnostics()["velocity_feedback"]["x"] == 0.0
        assert backend.diagnostics()["distance_setpoint_m"] == 2.0
        assert manager.get_parameter("pid_x_k_v").value == 0.0
    finally:
        manager.close()

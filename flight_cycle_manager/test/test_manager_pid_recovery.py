"""Real PID backend integration at the manager's shared-search boundary."""

from dataclasses import replace

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy

from flight_cycle_manager.manager import CycleState
from test_manager_start import (
    airborne_state,
    finish_startup_validation,
    make_manager,
)
from test_pid_landing_backend import central_observation, dock_observation


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def manager_at_landing_boundary(monkeypatch, **parameters):
    vehicle = FakeVehicle()
    manager = make_manager(vehicle, landing_backend="pid", **parameters)
    finish_startup_validation(manager, vehicle)
    # Start at shared-search completion, independently of the takeoff altitude
    # workaround. Keep the real manager and real PID backend for all decisions.
    manager._state = CycleState.SEARCH_DOCK
    vehicle.state = airborne_state(1.0)
    monkeypatch.setattr(manager, "_dock_observation", lambda now: None)
    manager._start_selected_backend(1.0)
    assert manager.state is CycleState.LANDING_BACKEND
    return manager, vehicle


def test_manager_hands_back_only_after_pid_recovery_is_exhausted(monkeypatch):
    manager, vehicle = manager_at_landing_boundary(
        monkeypatch, pid_max_recovery_attempts=1,
        pid_target_loss_grace_s=0.7,
    )
    try:
        for now, altitude in [(1.1, 4.0), (1.81, 4.0), (3.3, 4.3)]:
            vehicle.state = airborne_state(now, z=altitude)
            manager.tick(now)
            assert manager.state is CycleState.LANDING_BACKEND
            assert vehicle.set_mission_current_calls == []
            assert vehicle.land_calls == 0

        vehicle.state = airborne_state(6.31, z=4.3)
        manager.tick(6.31)
        assert manager.state is CycleState.RETURN_TO_FC
        assert vehicle.set_mission_current_calls == [(2, True)]
        assert vehicle.land_calls == 0
        assert "recovery attempts exhausted" in manager.reason
    finally:
        manager.close()


def test_real_pid_completion_reaches_docked_and_requires_explicit_restart(monkeypatch):
    manager, vehicle = manager_at_landing_boundary(monkeypatch)
    try:
        for now, dock in [
            (1.1, dock_observation(1.1)),
            (2.11, dock_observation(2.11)),
            (2.2, central_observation(2.2)),
            (2.3, central_observation(2.3)),
            (2.81, central_observation(2.81)),
            (2.9, central_observation(2.9, distance_m=0.2)),
            (3.91, central_observation(3.91, distance_m=0.2)),
        ]:
            monkeypatch.setattr(manager, "_dock_observation", lambda now, dock=dock: dock)
            vehicle.state = airborne_state(now)
            manager.tick(now)
        monkeypatch.setattr(manager, "_dock_observation", lambda now: None)
        for now in [4.0, 4.51]:
            vehicle.state = replace(airborne_state(now), flying=False)
            manager.tick(now)
        assert vehicle.arm_calls == [False]
        vehicle.finish_command()
        vehicle.state = replace(airborne_state(4.6), flying=False, armed=False)
        manager.tick(4.6)
        assert manager.state is CycleState.DOCKED
        assert manager.backend is None
        command_count = len(vehicle.velocity_calls)
        manager.tick(4.7)
        assert manager.state is CycleState.DOCKED
        assert len(vehicle.velocity_calls) == command_count
        assert vehicle.arm_calls == [False]
    finally:
        manager.close()

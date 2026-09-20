"""Repeatable LAND-mode trigger tests for mission-free PID docking debug."""

from __future__ import annotations

from dataclasses import replace

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy
from std_srvs.srv import Trigger

from flight_cycle_manager.manager import (
    GUIDED_MODE,
    LAND_MODE,
    CycleState,
    FlightCycleManager,
)
from test_manager_start import airborne_state, ground_state


LOITER_MODE = 5


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_debug_manager(vehicle: FakeVehicle) -> FlightCycleManager:
    return FlightCycleManager(
        vehicle=vehicle,
        parameter_values={"operation_mode": "pid_landing_debug"},
    )


def test_debug_mode_starts_passive_without_downloading_a_mission():
    """Break caught: debug startup regains the production mission dependency."""
    vehicle = FakeVehicle(ground_state(1.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        assert manager.state is CycleState.WAITING_LAND_TRIGGER
        assert vehicle.download_calls == 0
    finally:
        manager.close()


def test_start_cycle_service_cannot_enable_mission_flow_in_debug_mode():
    """Break caught: an inherited service call silently leaves landing-only mode."""
    vehicle = FakeVehicle(ground_state(1.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager._state = CycleState.DOCKED
        response = manager._start_cycle_callback(
            Trigger.Request(), Trigger.Response()
        )
        assert response.success is False
        assert manager.state is CycleState.DOCKED
        assert vehicle.download_calls == 0
    finally:
        manager.close()


def test_only_a_fresh_airborne_land_edge_requests_guided():
    """Break caught: cached LAND or a grounded LAND report starts PID control."""
    vehicle = FakeVehicle(airborne_state(1.0, mode=LAND_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(1.0)
        assert manager.state is CycleState.WAITING_LAND_TRIGGER
        assert vehicle.set_mode_calls == []

        vehicle.state = replace(
            airborne_state(1.1, mode=LOITER_MODE),
            armed=False,
            flying=False,
        )
        manager.tick(1.1)
        vehicle.state = replace(
            airborne_state(1.2, mode=LAND_MODE),
            armed=False,
            flying=False,
        )
        manager.tick(1.2)
        vehicle.state = airborne_state(1.25, mode=LAND_MODE)
        manager.tick(1.25)
        assert vehicle.set_mode_calls == []

        vehicle.state = airborne_state(1.3, mode=LOITER_MODE)
        manager.tick(1.3)
        vehicle.state = airborne_state(1.4, mode=LAND_MODE)
        manager.tick(1.4)

        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert vehicle.set_mode_calls == [GUIDED_MODE]
    finally:
        manager.close()


def test_mode_change_cancels_active_attempt_and_later_land_edge_restarts():
    """Break caught: cancelling once permanently latches debug docking passive."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(2.0)
        vehicle.state = airborne_state(2.1, mode=LAND_MODE)
        manager.tick(2.1)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(2.2, mode=GUIDED_MODE)
        manager.tick(2.2)
        assert manager.state is CycleState.SEARCH_DOCK

        vehicle.state = airborne_state(2.3, mode=LOITER_MODE)
        manager.tick(2.3)
        assert manager.state is CycleState.WAITING_LAND_TRIGGER
        assert manager.search_controller is None
        assert vehicle.land_calls == 0

        vehicle.state = airborne_state(2.4, mode=LAND_MODE)
        manager.tick(2.4)
        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert vehicle.set_mode_calls == [GUIDED_MODE, GUIDED_MODE]
    finally:
        manager.close()


def test_guided_handoff_waits_for_fresh_streams_before_starting_search():
    """Break caught: a stream-phase gap after GUIDED causes immediate LAND."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(2.0)
        vehicle.state = airborne_state(2.1, mode=LAND_MODE)
        manager.tick(2.1)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = replace(
            airborne_state(2.2, mode=GUIDED_MODE),
            position_at=1.0,
            landed_at=1.0,
        )

        manager.tick(2.2)

        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert vehicle.land_calls == 0
        assert vehicle.set_mode_calls == [GUIDED_MODE]

        vehicle.state = airborne_state(2.25, mode=GUIDED_MODE)
        manager.tick(2.25)

        assert manager.state is CycleState.SEARCH_DOCK
        assert vehicle.land_calls == 0
    finally:
        manager.close()


def test_delayed_land_heartbeat_during_guided_telemetry_wait_does_not_cancel():
    """Break caught: the trigger LAND sample is mistaken for pilot override."""
    vehicle = FakeVehicle(airborne_state(3.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(3.0)
        vehicle.state = airborne_state(3.1, mode=LAND_MODE)
        manager.tick(3.1)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = replace(
            airborne_state(3.2, mode=LAND_MODE),
            position_at=2.0,
            landed_at=2.0,
        )

        manager.tick(3.2)

        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert vehicle.land_calls == 0

        vehicle.state = replace(
            airborne_state(3.25, mode=LAND_MODE),
            position_at=2.0,
            landed_at=2.0,
        )
        manager.tick(3.25)

        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert vehicle.land_calls == 0

        vehicle.state = airborne_state(3.3, mode=GUIDED_MODE)
        manager.tick(3.3)

        assert manager.state is CycleState.SEARCH_DOCK
    finally:
        manager.close()


def test_link_recovery_in_land_requires_non_land_rearm_before_restart():
    """Break caught: LAND selected during link loss restarts on first heartbeat."""
    vehicle = FakeVehicle(airborne_state(3.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(3.0)
        vehicle.state = airborne_state(3.1, mode=LAND_MODE)
        manager.tick(3.1)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(3.2, mode=GUIDED_MODE)
        manager.tick(3.2)
        assert manager.state is CycleState.SEARCH_DOCK

        vehicle.state = replace(
            airborne_state(3.3, mode=GUIDED_MODE), heartbeat_at=0.0
        )
        manager.tick(6.0)
        assert manager.state is CycleState.WAITING_LAND_TRIGGER

        vehicle.state = airborne_state(6.1, mode=LAND_MODE)
        manager.tick(6.1)
        assert vehicle.set_mode_calls == [GUIDED_MODE]

        vehicle.state = airborne_state(6.2, mode=LOITER_MODE)
        manager.tick(6.2)
        vehicle.state = airborne_state(6.3, mode=LAND_MODE)
        manager.tick(6.3)
        assert vehicle.set_mode_calls == [GUIDED_MODE, GUIDED_MODE]
    finally:
        manager.close()


@pytest.mark.parametrize("cached_flying", [False, None])
def test_stale_link_cancels_even_when_cached_flying_is_not_true(cached_flying):
    """Break caught: ambiguous flight status routes link loss to direct LAND."""
    vehicle = FakeVehicle(airborne_state(4.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(4.0)
        vehicle.state = airborne_state(4.1, mode=LAND_MODE)
        manager.tick(4.1)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(4.2, mode=GUIDED_MODE)
        manager.tick(4.2)
        assert manager.state is CycleState.SEARCH_DOCK

        vehicle.state = replace(
            airborne_state(4.3, mode=GUIDED_MODE),
            heartbeat_at=0.0,
            flying=cached_flying,
        )
        manager.tick(7.0)

        assert manager.state is CycleState.WAITING_LAND_TRIGGER
        assert manager.search_controller is None
        assert vehicle.land_calls == 0
    finally:
        manager.close()


def test_land_edge_with_stale_landed_state_does_not_start_docking():
    """Break caught: cached flying=True is trusted after its telemetry expires."""
    vehicle = FakeVehicle(airborne_state(7.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(7.0)
        vehicle.state = replace(
            airborne_state(10.0, mode=LAND_MODE), landed_at=7.0
        )
        manager.tick(10.0)
        assert manager.state is CycleState.WAITING_LAND_TRIGGER
        assert vehicle.set_mode_calls == []
    finally:
        manager.close()

"""Real transport authority regressions using the MAVLink fake connection."""

from __future__ import annotations

import math

import pytest

from vehicle_fakes import install_ros_fakes

install_ros_fakes()

import rclpy

import flight_cycle_manager.manager as manager_module
from flight_cycle_manager.landing_backend import LandingBackendDecision, LandingBackendOutcome
from flight_cycle_manager.manager import (
    AUTO_RTL_MODE,
    GUIDED_MODE,
    LAND_MODE,
    MISSION_STATE_ACTIVE,
    CycleState,
    FlightCycleManager,
    _PendingOperation,
)
from flight_cycle_manager.mission_contract import MissionValidationConfig, validate_docking_mission
from flight_cycle_manager.pymavlink_vehicle import MavlinkVehicle, VelocityFrame
from test_manager_docking import StubPidLandingBackend, StubPreciseLandingBackend, StubSearchController, publish_dock_target
from test_manager_start import valid_contract_items
from test_pymavlink_vehicle import FakeConnection, FakeMessage, MAVLINK, push_ack, push_heartbeat


LOITER_MODE = 5
MISSION_TOTAL = len(valid_contract_items())


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def stub_non_native_dependencies(monkeypatch):
    monkeypatch.setattr(manager_module, "DockSearchController", StubSearchController)
    monkeypatch.setattr(manager_module, "PidLandingBackend", StubPidLandingBackend)
    monkeypatch.setattr(manager_module, "PreciseLandingBackend", StubPreciseLandingBackend)


def validated_contract():
    return validate_docking_mission(
        valid_contract_items(),
        MissionValidationConfig(
            min_handoff_loiter_s=5.0,
            landing_location_tolerance_m=2.0,
        ),
    )


def make_real_manager(connection: FakeConnection, **parameters: object):
    vehicle = MavlinkVehicle(
        connection="tcp:127.0.0.1:5760",
        target_system=1,
        target_component=1,
        source_system=254,
        source_component=191,
        connection_factory=lambda *args, **kwargs: connection,
    )
    manager = FlightCycleManager(vehicle=vehicle, parameter_values=parameters)
    manager.vehicle.cancel_pending_operation()
    manager._pending = None
    manager._state = CycleState.READY
    manager._reason = "ready for wire authority tests"
    manager._contract = validated_contract()
    manager._validated_mission_total = MISSION_TOTAL
    manager._validated_mission_id = None
    return manager, vehicle


def push_airborne_snapshot(
    connection: FakeConnection,
    *,
    mode: int,
    now: float,
    seq: int,
    total: int = MISSION_TOTAL,
    mission_id: int | None = None,
) -> None:
    push_heartbeat(connection, mode=mode, armed=True)
    connection.incoming.extend(
        [
            FakeMessage(
                "LOCAL_POSITION_NED",
                x=0.0,
                y=0.0,
                z=-4.0,
                vx=0.0,
                vy=0.0,
                vz=0.0,
            ),
            FakeMessage(
                "EXTENDED_SYS_STATE",
                landed_state=MAVLINK.MAV_LANDED_STATE_IN_AIR,
            ),
            FakeMessage(
                "MISSION_CURRENT",
                seq=seq,
                total=total,
                mission_state=MISSION_STATE_ACTIVE,
                mission_mode=0,
                mission_id=mission_id,
            ),
        ]
    )


def prime_airborne(vehicle: MavlinkVehicle, connection: FakeConnection, *, seq: int) -> None:
    push_airborne_snapshot(connection, mode=GUIDED_MODE, now=1.0, seq=seq)
    vehicle.receive_telemetry(now=1.0)
    vehicle.service_pending(now=1.0)
    connection.mav.calls.clear()


def flight_wire_calls(connection: FakeConnection):
    return [
        (name, args)
        for name, args in connection.mav.calls
        if name
        in {
            "command_long_send",
            "set_position_target_local_ned_send",
            "landing_target_send",
        }
    ]


def command_ids(connection: FakeConnection):
    return [args[2] for args in connection.mav.calls_for("command_long_send")]


def test_takeoff_wire_altitude_adds_requested_climb_to_height_above_home():
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection, takeoff_altitude_m=4.0)
    try:
        manager._operation_succeeded("arm", None, now=1.0)
        push_heartbeat(connection, mode=GUIDED_MODE, armed=True)
        connection.incoming.extend([
            FakeMessage(
                "LOCAL_POSITION_NED", x=0.0, y=0.0, z=-1.0,
                vx=0.0, vy=0.0, vz=0.0,
            ),
            FakeMessage("GLOBAL_POSITION_INT", relative_alt=1250, alt=512345),
            FakeMessage(
                "EXTENDED_SYS_STATE", landed_state=MAVLINK.MAV_LANDED_STATE_ON_GROUND,
            ),
        ])
        manager.tick(1.1)
        takeoffs = [
            args for args in connection.mav.calls_for("command_long_send")
            if args[2] == MAVLINK.MAV_CMD_NAV_TAKEOFF
        ]
        assert len(takeoffs) == 1
        assert takeoffs[0][10] == 5.25
    finally:
        manager.close()


def test_pilot_override_detection_tick_and_later_passive_ticks_do_not_retry_handoff():
    """Break caught: receive+retry happens before the passive override latch."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        prime_airborne(vehicle, connection, seq=manager.contract.handoff_seq)
        now = 20.0
        future = vehicle.set_mode_async(GUIDED_MODE, now=now)
        manager._state = CycleState.GUIDED_TAKEOVER
        manager._pending = _PendingOperation("handoff_guided", future, now + 5.0)
        before = list(flight_wire_calls(connection))

        push_airborne_snapshot(
            connection,
            mode=LOITER_MODE,
            now=20.6,
            seq=manager.contract.handoff_seq,
        )
        manager.tick(now=20.6)

        assert manager.state is CycleState.PILOT_OVERRIDE
        assert flight_wire_calls(connection) == before

        manager.tick(now=21.2)

        assert flight_wire_calls(connection) == before
    finally:
        manager.close()


def test_simultaneous_mode_override_beats_mission_integrity_rtl():
    """Break caught: same-tick pilot takeover still sends RTL before becoming passive."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        prime_airborne(vehicle, connection, seq=manager.contract.handoff_seq)
        manager._state = CycleState.SEARCH_DOCK
        manager._search_controller = StubSearchController(manager._build_search_config())

        push_airborne_snapshot(
            connection,
            mode=LOITER_MODE,
            now=20.0,
            seq=manager.contract.handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.0)

        assert manager.state is CycleState.PILOT_OVERRIDE
        assert flight_wire_calls(connection) == []
    finally:
        manager.close()


def test_precise_warmup_land_mode_is_passive_override_until_native_land_request(
    monkeypatch,
):
    """Break caught: native backend warmup accepts LAND before the manager requested it."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(
        connection,
        landing_backend="precise_landing",
    )
    try:
        handoff_seq = manager.contract.handoff_seq
        prime_airborne(vehicle, connection, seq=handoff_seq)
        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.0,
            seq=handoff_seq,
        )
        vehicle.receive_telemetry(now=20.0)
        manager._state = CycleState.SEARCH_DOCK
        publish_dock_target(manager, monkeypatch, received_at=20.0)
        manager._start_selected_backend(now=20.0)
        connection.mav.calls.clear()

        push_airborne_snapshot(
            connection,
            mode=LAND_MODE,
            now=20.1,
            seq=handoff_seq,
        )
        manager.tick(now=20.1)

        assert manager.state is CycleState.PILOT_OVERRIDE
        assert flight_wire_calls(connection) == []
    finally:
        manager.close()


def test_mission_integrity_breach_cancels_pending_handoff_guided_before_success():
    """Break caught: a stale GUIDED success callback can resume docking after authority was revoked."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        prime_airborne(vehicle, connection, seq=manager.contract.handoff_seq)
        now = 20.0
        future = vehicle.set_mode_async(GUIDED_MODE, now=now)
        manager._state = CycleState.GUIDED_TAKEOVER
        manager._pending = _PendingOperation("handoff_guided", future, now + 5.0)

        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.1,
            seq=manager.contract.handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.1)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert manager.contract is None
        assert command_ids(connection) == [
            MAVLINK.MAV_CMD_DO_SET_MODE,
            MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        ]

        push_ack(connection, MAVLINK.MAV_CMD_DO_SET_MODE)
        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.2,
            seq=manager.contract.handoff_seq if manager.contract is not None else 1,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.2)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert manager.search_controller is None
        assert command_ids(connection) == [
            MAVLINK.MAV_CMD_DO_SET_MODE,
            MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        ]
    finally:
        manager.close()


def test_mission_integrity_breach_cancels_pending_handoff_guided_before_rejection():
    """Break caught: a stale GUIDED rejection callback can override mission-integrity RTL handoff."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        prime_airborne(vehicle, connection, seq=manager.contract.handoff_seq)
        now = 20.0
        future = vehicle.set_mode_async(GUIDED_MODE, now=now)
        manager._state = CycleState.GUIDED_TAKEOVER
        manager._pending = _PendingOperation("handoff_guided", future, now + 5.0)

        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.1,
            seq=manager.contract.handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.1)

        push_ack(
            connection,
            MAVLINK.MAV_CMD_DO_SET_MODE,
            result=MAVLINK.MAV_RESULT_DENIED,
        )
        manager.tick(now=20.2)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert command_ids(connection) == [
            MAVLINK.MAV_CMD_DO_SET_MODE,
            MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        ]
        assert MAVLINK.MAV_CMD_NAV_LAND not in command_ids(connection)
    finally:
        manager.close()


def test_mission_integrity_breach_cancels_pending_fallback_select_before_success():
    """Break caught: a stale fallback select ACK can restart the old validated mission after integrity loss."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        prime_airborne(vehicle, connection, seq=manager.contract.handoff_seq)
        now = 20.0
        future = vehicle.set_mission_current_async(
            manager.contract.fallback_land_seq,
            reset=True,
            now=now,
        )
        manager._state = CycleState.RETURN_TO_FC
        manager._pending = _PendingOperation("fallback_select", future, now + 5.0)

        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.1,
            seq=manager.contract.handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.1)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert manager.contract is None
        assert command_ids(connection) == [
            MAVLINK.MAV_CMD_DO_SET_MISSION_CURRENT,
            MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        ]

        push_ack(connection, MAVLINK.MAV_CMD_DO_SET_MISSION_CURRENT)
        manager.tick(now=20.2)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert MAVLINK.MAV_CMD_MISSION_START not in command_ids(connection)
    finally:
        manager.close()


def test_direct_land_ack_waits_for_land_heartbeat_before_override_window_closes():
    """Break caught: direct LAND ACK narrows expected modes before LAND is observed."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        handoff_seq = manager.contract.handoff_seq
        prime_airborne(vehicle, connection, seq=handoff_seq)
        manager._state = CycleState.SEARCH_DOCK
        manager._search_controller = StubSearchController(manager._build_search_config())
        manager._command_direct_land("forced direct LAND for wire test")

        push_ack(connection, MAVLINK.MAV_CMD_NAV_LAND)
        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.1,
            seq=handoff_seq,
        )
        manager.tick(now=20.1)
        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.2,
            seq=handoff_seq,
        )
        manager.tick(now=20.2)

        assert manager.state is CycleState.FC_FALLBACK_LAND

        push_airborne_snapshot(
            connection,
            mode=LAND_MODE,
            now=20.3,
            seq=handoff_seq,
        )
        manager.tick(now=20.3)

        assert manager.state is not CycleState.PILOT_OVERRIDE
    finally:
        manager.close()


def test_mission_integrity_rtl_ack_waits_for_rtl_heartbeat_before_override_window_closes():
    """Break caught: RTL ACK narrows expected modes before AUTO_RTL is observed."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        handoff_seq = manager.contract.handoff_seq
        prime_airborne(vehicle, connection, seq=handoff_seq)
        manager._state = CycleState.SEARCH_DOCK
        manager._search_controller = StubSearchController(manager._build_search_config())

        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.0,
            seq=handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.0)

        push_ack(connection, MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.1,
            seq=handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.1)
        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.2,
            seq=handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.2)

        assert manager.state is CycleState.FC_FALLBACK_LAND

        push_airborne_snapshot(
            connection,
            mode=AUTO_RTL_MODE,
            now=20.3,
            seq=handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        manager.tick(now=20.3)

        assert manager.state is not CycleState.PILOT_OVERRIDE
    finally:
        manager.close()


def test_native_land_ack_waits_for_land_heartbeat_before_override_window_closes():
    """Break caught: native LAND ACK narrows expected modes before LAND is observed."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection, landing_backend="precise_landing")
    try:
        handoff_seq = manager.contract.handoff_seq
        prime_airborne(vehicle, connection, seq=handoff_seq)
        manager._state = CycleState.LANDING_BACKEND
        manager._landing_controller = StubPreciseLandingBackend(
            manager._build_precise_landing_config()
        )
        manager.backend.queue_decision(
            LandingBackendDecision(
                LandingBackendOutcome.RUNNING,
                "REQUEST_LAND",
                request_land=True,
            )
        )

        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.0,
            seq=handoff_seq,
        )
        manager.tick(now=20.0)

        push_ack(connection, MAVLINK.MAV_CMD_NAV_LAND)
        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.1,
            seq=handoff_seq,
        )
        manager.tick(now=20.1)
        push_airborne_snapshot(
            connection,
            mode=GUIDED_MODE,
            now=20.2,
            seq=handoff_seq,
        )
        manager.tick(now=20.2)

        assert manager.state is CycleState.LANDING_BACKEND

        push_airborne_snapshot(
            connection,
            mode=LAND_MODE,
            now=20.3,
            seq=handoff_seq,
        )
        manager.tick(now=20.3)

        assert manager.state is not CycleState.PILOT_OVERRIDE
    finally:
        manager.close()


@pytest.mark.parametrize("state", [CycleState.PILOT_OVERRIDE, CycleState.LINK_LOSS])
def test_manager_close_skips_final_zero_velocity_in_passive_states(state: CycleState):
    """Break caught: passive shutdown sends a final velocity command after authority is lost."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        prime_airborne(vehicle, connection, seq=manager.contract.handoff_seq)
        vehicle.send_velocity(VelocityFrame.MAP_ENU, 1.0, 2.0, 3.0, now=30.0)
        manager._state = state

        manager.close()

        calls = connection.mav.calls_for("set_position_target_local_ned_send")
        assert len(calls) == 1
        assert calls[0][8:11] == (2.0, 1.0, -3.0)
    finally:
        if not connection.closed:
            manager.close()


def test_manager_close_still_sends_final_zero_velocity_while_manager_owns_authority():
    """Break caught: normal authoritative shutdown stops suppressing the safety zero."""
    connection = FakeConnection()
    manager, vehicle = make_real_manager(connection)
    try:
        prime_airborne(vehicle, connection, seq=manager.contract.handoff_seq)
        vehicle.send_velocity(VelocityFrame.MAP_ENU, 1.0, 2.0, 3.0, now=30.0)
        manager._state = CycleState.SEARCH_DOCK

        manager.close()

        calls = connection.mav.calls_for("set_position_target_local_ned_send")
        assert len(calls) == 2
        assert calls[-1][8:11] == (0.0, 0.0, 0.0)
    finally:
        if not connection.closed:
            manager.close()

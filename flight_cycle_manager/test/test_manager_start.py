"""Lifecycle tests for validation, vertical takeoff, and AUTO handoff."""

from __future__ import annotations

from dataclasses import replace

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy
from std_srvs.srv import Trigger

import flight_cycle_manager.manager as manager_module
from flight_cycle_manager.manager import (
    AUTO_MODE,
    GUIDED_MODE,
    MISSION_STATE_ACTIVE,
    CycleState,
    FlightCycleManager,
)
from flight_cycle_manager.mission_contract import (
    MAV_CMD_DO_LAND_START,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_LOITER_TIME,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    MissionItem,
)
from flight_cycle_manager.pymavlink_vehicle import VehicleState

def valid_contract_items() -> tuple[MissionItem, ...]:
    """A hand-checked minimal legal mission with the required landing tail."""
    return (
        MissionItem(
            0,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_CMD_DO_LAND_START,
            0.0,
            0.0,
            0.0,
            0.0,
            450000000,
            90000000,
            4.0,
        ),
        MissionItem(
            1,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_CMD_NAV_LOITER_TIME,
            5.0,
            0.0,
            0.0,
            0.0,
            450000000,
            90000000,
            4.0,
        ),
        MissionItem(
            2,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_CMD_NAV_LAND,
            0.0,
            0.0,
            0.0,
            0.0,
            450000000,
            90000000,
            0.0,
        ),
    )


def items_without_land_contract() -> tuple[MissionItem, ...]:
    """Break caught: validation accidentally accepts a mission without its marker."""
    items = valid_contract_items()
    return (replace(items[0], command=MAV_CMD_NAV_LAND), *items[1:])


def ground_state(
    now: float,
    *,
    mode: int | None = None,
    armed: bool = False,
    prearm_ok: bool | None = True,
) -> VehicleState:
    return VehicleState(
        connected=True,
        armed=armed,
        flying=False,
        mode=mode,
        system_status=4,
        position_enu_m=(0.0, 0.0, 0.0),
        velocity_enu_m_s=(0.0, 0.0, 0.0),
        prearm_ok=prearm_ok,
        heartbeat_at=now,
        position_at=now,
        landed_at=now,
    )


def airborne_state(
    now: float,
    *,
    z: float = 4.0,
    vz: float = 0.05,
    mode: int = GUIDED_MODE,
) -> VehicleState:
    return VehicleState(
        connected=True,
        armed=True,
        flying=True,
        mode=mode,
        system_status=4,
        position_enu_m=(0.0, 0.0, z),
        velocity_enu_m_s=(0.0, 0.0, vz),
        prearm_ok=True,
        heartbeat_at=now,
        position_at=now,
        landed_at=now,
    )


def auto_active_state(now: float, *, seq: int = 0) -> VehicleState:
    return replace(
        airborne_state(now, mode=AUTO_MODE),
        mission_seq=seq,
        mission_total=3,
        mission_state=MISSION_STATE_ACTIVE,
        mission_mode=0,
        mission_current_at=now,
    )


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_manager(
    vehicle: FakeVehicle, **parameters: object
) -> FlightCycleManager:
    return FlightCycleManager(vehicle=vehicle, parameter_values=parameters)


def finish_startup_validation(
    manager: FlightCycleManager, vehicle: FakeVehicle, now: float = 0.0
) -> None:
    vehicle.finish_download(valid_contract_items())
    manager.tick(now=now)
    if manager.landing_backend == "precise_landing":
        vehicle.finish_parameters({"PLND_ENABLED": 1.0, "PLND_TYPE": 1.0})
        manager.tick(now=now)
    assert manager.state is CycleState.READY


def request_valid_start(
    manager: FlightCycleManager, vehicle: FakeVehicle, now: float = 1.0
) -> None:
    response = manager._start_cycle_callback(Trigger.Request(), Trigger.Response())
    assert response.success
    vehicle.finish_download(valid_contract_items())
    manager.tick(now=now)
    if manager.landing_backend == "precise_landing":
        vehicle.finish_parameters({"PLND_ENABLED": 1.0, "PLND_TYPE": 1.0})
        manager.tick(now=now)
    assert manager.state is CycleState.PREARM


def drive_to_wait_takeoff(
    manager: FlightCycleManager, vehicle: FakeVehicle
) -> None:
    vehicle.state = ground_state(1.0)
    manager.tick(now=1.0)
    assert manager.state is CycleState.PREARM
    assert vehicle.prearm_calls == 1
    vehicle.finish_command()
    manager.tick(now=1.1)
    assert manager.state is CycleState.SET_GUIDED

    manager.tick(now=1.2)
    assert vehicle.set_mode_calls == [GUIDED_MODE]
    vehicle.state = ground_state(1.2, mode=GUIDED_MODE)
    vehicle.finish_command()
    manager.tick(now=1.3)
    assert manager.state is CycleState.ARM

    manager.tick(now=1.4)
    assert vehicle.arm_calls == [True]
    vehicle.state = ground_state(1.4, mode=GUIDED_MODE, armed=True)
    vehicle.finish_command()
    manager.tick(now=1.5)
    assert manager.state is CycleState.TAKEOFF

    manager.tick(now=1.6)
    assert vehicle.takeoff_calls == [4.0]
    vehicle.finish_command()
    vehicle.state = airborne_state(1.7)
    manager.tick(now=1.7)
    assert manager.state is CycleState.WAIT_TAKEOFF_STABLE


def drive_to_takeoff_pending(
    manager: FlightCycleManager, vehicle: FakeVehicle
) -> None:
    """Advance fresh pre-arm, GUIDED, and arm transactions to one takeoff ACK."""
    vehicle.state = ground_state(1.0)
    manager.tick(now=1.0)
    vehicle.finish_command()
    manager.tick(now=1.1)
    manager.tick(now=1.2)
    vehicle.state = ground_state(1.2, mode=GUIDED_MODE)
    vehicle.finish_command()
    manager.tick(now=1.3)
    manager.tick(now=1.4)
    vehicle.state = ground_state(1.4, mode=GUIDED_MODE, armed=True)
    vehicle.finish_command()
    manager.tick(now=1.5)
    manager.tick(now=1.6)
    assert manager.state is CycleState.TAKEOFF
    assert vehicle.takeoff_calls == [4.0]


def drive_to_reset_mission(
    manager: FlightCycleManager, vehicle: FakeVehicle
) -> None:
    drive_to_wait_takeoff(manager, vehicle)
    vehicle.state = airborne_state(5.0, z=4.0, vz=0.05)
    manager.tick(now=5.0)
    vehicle.state = airborne_state(6.1, z=4.0, vz=0.05)
    manager.tick(now=6.1)
    assert manager.state is CycleState.RESET_ONBOARD_MISSION
    assert vehicle.set_mission_current_calls == [(0, True)]


def drive_to_start_mission_pending(
    manager: FlightCycleManager, vehicle: FakeVehicle
) -> None:
    """Advance reset acknowledgement to one pending MISSION_START command."""
    drive_to_reset_mission(manager, vehicle)
    vehicle.finish_command()
    manager.tick(now=6.2)
    assert manager.state is CycleState.START_AUTO
    manager.tick(now=6.3)
    assert vehicle.start_mission_calls == 1


def test_startup_downloads_and_validates_before_becoming_ready():
    """Break caught: startup can mark a mission ready without a fresh download."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        assert manager.state is CycleState.VALIDATING_MISSION
        assert vehicle.download_calls == 1
        assert vehicle.arm_calls == []
        finish_startup_validation(manager, vehicle)
    finally:
        manager.close()


def test_start_service_queues_fresh_validation_without_blocking():
    """Break caught: the synchronous service arms or trusts startup validation."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        response = manager._start_cycle_callback(Trigger.Request(), Trigger.Response())

        assert response.success is True
        assert response.message == "cycle request accepted for validation"
        assert manager.state is CycleState.VALIDATING_MISSION
        assert vehicle.download_calls == 2
        assert vehicle.arm_calls == []
    finally:
        manager.close()


def test_invalid_contract_publishes_invalid_state_without_arming():
    """Break caught: invalid onboard missions reach any arm operation."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        response = manager._start_cycle_callback(Trigger.Request(), Trigger.Response())
        assert response.success
        vehicle.finish_download(items_without_land_contract())
        manager.tick(now=1.0)

        assert manager.state is CycleState.INVALID_ONBOARD_MISSION
        assert "DO_LAND_START" in manager.reason
        assert vehicle.arm_calls == []
        assert vehicle.land_calls == 0
        assert vehicle.mission_mutation_calls == []
    finally:
        manager.close()


def test_start_service_rejects_an_armed_vehicle():
    """Break caught: a second cycle request can run while the aircraft is armed."""
    vehicle = FakeVehicle(ground_state(0.0, armed=True))
    manager = make_manager(vehicle)
    try:
        response = manager._start_cycle_callback(Trigger.Request(), Trigger.Response())

        assert response.success is False
        assert "armed" in response.message
        assert vehicle.download_calls == 1
    finally:
        manager.close()


def test_auto_start_is_rejected_outside_simulation_mode():
    """Break caught: production configuration can silently begin a flight cycle."""
    vehicle = FakeVehicle(ground_state(0.0))

    with pytest.raises(ValueError, match="simulation_mode"):
        make_manager(vehicle, auto_start=True, simulation_mode=False)


def test_simulation_auto_start_queues_a_cycle_after_startup_validation():
    """Break caught: allowed simulation auto-start never advances past startup validation."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle, auto_start=True, simulation_mode=True)
    try:
        vehicle.finish_download(valid_contract_items())
        manager.tick(now=0.0)

        assert manager.state is CycleState.VALIDATING_MISSION
        assert vehicle.download_calls == 2
    finally:
        manager.close()


def test_native_precision_validation_requires_both_flight_controller_parameters():
    """Break caught: native precision landing proceeds without its FC parameter gate."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle, landing_backend="precise_landing")
    try:
        finish_startup_validation(manager, vehicle)
        request = manager._start_cycle_callback(Trigger.Request(), Trigger.Response())
        assert request.success
        vehicle.finish_download(valid_contract_items())
        manager.tick(now=1.0)
        assert vehicle.parameter_calls[-1] == ("PLND_ENABLED", "PLND_TYPE")
        vehicle.state = ground_state(1.1)
        vehicle.finish_parameters({"PLND_ENABLED": 1.0, "PLND_TYPE": 0.0})
        manager.tick(now=1.1)

        assert manager.state is CycleState.INVALID_ONBOARD_MISSION
        assert "PLND_ENABLED=1.0" in manager.reason
        assert "PLND_TYPE=0.0" in manager.reason
        assert vehicle.arm_calls == []
    finally:
        manager.close()


def test_pid_start_does_not_read_native_precision_landing_parameters():
    """Break caught: PID starts acquire native-only PLND configuration."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle, landing_backend="pid")
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)

        assert vehicle.parameter_calls == []
        assert manager.state is CycleState.PREARM
    finally:
        manager.close()


def test_stale_heartbeat_with_cached_ground_flags_commands_direct_land_once():
    """Break caught: stale disarmed flags are trusted as a current safe state."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        vehicle.state = replace(ground_state(3.0), heartbeat_at=0.0)
        manager.tick(now=3.0)
        manager.tick(now=3.1)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert "fresh heartbeat, position, and landed state" in manager.reason
        assert vehicle.prearm_calls == 0
        assert vehicle.arm_calls == []
        assert vehicle.land_calls == 1
    finally:
        manager.close()


def test_stale_landed_state_with_cached_ground_flags_commands_direct_land_once():
    """Break caught: an old ON_GROUND report is trusted after validation fails."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        vehicle.state = replace(ground_state(3.0), landed_at=0.0)
        manager.tick(now=3.0)
        manager.tick(now=3.1)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert "fresh heartbeat, position, and landed state" in manager.reason
        assert vehicle.prearm_calls == 0
        assert vehicle.arm_calls == []
        assert vehicle.land_calls == 1
    finally:
        manager.close()


def test_armed_telemetry_race_before_prearm_commands_direct_land_once():
    """Break caught: a post-acceptance arm race is mislabeled FAILED_DISARMED."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        vehicle.state = airborne_state(1.0)
        manager.tick(now=1.0)
        manager.tick(now=1.1)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 1
        assert vehicle.arm_calls == []
    finally:
        manager.close()


def test_armed_invalid_mission_race_commands_direct_land_once():
    """Break caught: failed validation ignores that the accepted cycle is airborne."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        response = manager._start_cycle_callback(Trigger.Request(), Trigger.Response())
        assert response.success
        vehicle.state = airborne_state(1.0)
        vehicle.finish_download(items_without_land_contract())
        manager.tick(now=1.0)
        manager.tick(now=1.1)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 1
    finally:
        manager.close()


def test_armed_native_parameter_validation_race_commands_direct_land_once():
    """Break caught: a bad PLND gate ignores a vehicle that armed during validation."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle, landing_backend="precise_landing")
    try:
        finish_startup_validation(manager, vehicle)
        response = manager._start_cycle_callback(Trigger.Request(), Trigger.Response())
        assert response.success
        vehicle.finish_download(valid_contract_items())
        manager.tick(now=1.0)
        vehicle.state = airborne_state(1.1)
        vehicle.finish_parameters({"PLND_ENABLED": 0.0, "PLND_TYPE": 0.0})
        manager.tick(now=1.1)
        manager.tick(now=1.2)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 1
    finally:
        manager.close()


def test_fresh_telemetry_runs_prearm_before_any_arm_command():
    """Break caught: a valid start bypasses fresh pre-arm health confirmation."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        vehicle.state = ground_state(1.0)
        manager.tick(now=1.0)

        assert manager.state is CycleState.PREARM
        assert vehicle.prearm_calls == 1
        assert vehicle.arm_calls == []
    finally:
        manager.close()


def test_takeoff_ack_rejection_while_airborne_commands_direct_land():
    """Break caught: a rejected airborne operation leaves the aircraft in GUIDED."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        drive_to_takeoff_pending(manager, vehicle)
        vehicle.state = airborne_state(2.0)
        vehicle.finish_command(success=False, message="takeoff rejected")
        manager.tick(now=2.1)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 1
        assert "takeoff rejected" in manager.reason
    finally:
        manager.close()


def test_takeoff_is_sent_once_while_its_ack_is_pending():
    """Break caught: timer ticks retransmit a second takeoff command."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        vehicle.state = ground_state(1.0)
        manager.tick(now=1.0)
        vehicle.finish_command()
        manager.tick(now=1.1)
        manager.tick(now=1.2)
        vehicle.state = ground_state(1.2, mode=GUIDED_MODE)
        vehicle.finish_command()
        manager.tick(now=1.3)
        manager.tick(now=1.4)
        vehicle.state = ground_state(1.4, mode=GUIDED_MODE, armed=True)
        vehicle.finish_command()
        manager.tick(now=1.5)
        manager.tick(now=1.6)
        manager.tick(now=1.7)

        assert manager.state is CycleState.TAKEOFF
        assert vehicle.takeoff_calls == [4.0]
    finally:
        manager.close()


def test_takeoff_stability_requires_low_vertical_speed_for_full_dwell():
    """Break caught: target altitude alone completes takeoff without settling."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        drive_to_wait_takeoff(manager, vehicle)
        vehicle.state = airborne_state(5.0, z=4.0, vz=0.5)
        manager.tick(now=5.0)
        vehicle.state = airborne_state(6.5, z=4.0, vz=0.5)
        manager.tick(now=6.5)
        assert manager.state is CycleState.WAIT_TAKEOFF_STABLE
        assert vehicle.set_mission_current_calls == []

        vehicle.state = airborne_state(7.0, z=4.0, vz=0.05)
        manager.tick(now=7.0)
        manager.tick(now=7.9)
        assert vehicle.set_mission_current_calls == []
        vehicle.state = airborne_state(8.1, z=4.0, vz=0.05)
        manager.tick(now=8.1)

        assert manager.state is CycleState.RESET_ONBOARD_MISSION
        assert vehicle.set_mission_current_calls == [(0, True)]
    finally:
        manager.close()


def test_reset_then_mission_start_wait_for_each_ack_in_order():
    """Break caught: mission start skips reset or overlaps its acknowledgement."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        drive_to_reset_mission(manager, vehicle)

        manager.tick(now=6.2)
        assert vehicle.start_mission_calls == 0
        vehicle.finish_command()
        manager.tick(now=6.3)
        assert manager.state is CycleState.START_AUTO
        assert vehicle.start_mission_calls == 0
        manager.tick(now=6.4)

        assert vehicle.start_mission_calls == 1
        vehicle.finish_command()
        manager.tick(now=6.5)
        assert manager.state is CycleState.START_AUTO
    finally:
        manager.close()


def test_auto_mission_requires_auto_heartbeat_and_active_mission_current():
    """Break caught: start acknowledgement alone marks the stored mission active."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        drive_to_reset_mission(manager, vehicle)
        vehicle.finish_command()
        manager.tick(now=6.2)
        manager.tick(now=6.3)
        vehicle.finish_command()
        manager.tick(now=6.4)
        assert manager.state is CycleState.START_AUTO

        vehicle.state = auto_active_state(6.5, seq=0)
        manager.tick(now=6.5)
        assert manager.state is CycleState.AUTO_MISSION
    finally:
        manager.close()


def test_auto_confirmation_requires_observations_newer_than_start_ack():
    """Break caught: pre-ACK AUTO telemetry can complete a later mission start."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        drive_to_start_mission_pending(manager, vehicle)
        vehicle.state = auto_active_state(6.3, seq=0)
        vehicle.finish_command()
        manager.tick(now=6.4)
        manager.tick(now=6.5)

        assert manager.state is CycleState.START_AUTO

        vehicle.state = auto_active_state(6.6, seq=0)
        manager.tick(now=6.6)
        assert manager.state is CycleState.AUTO_MISSION
    finally:
        manager.close()


def test_auto_confirmation_accepts_absent_mission_state_after_start_ack():
    """Break caught: a MAVLink v1 MISSION_CURRENT can never confirm AUTO."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        drive_to_start_mission_pending(manager, vehicle)
        vehicle.finish_command()
        manager.tick(now=6.4)
        vehicle.state = replace(auto_active_state(6.5, seq=0), mission_state=None)
        manager.tick(now=6.5)

        assert manager.state is CycleState.AUTO_MISSION
    finally:
        manager.close()


def test_mission_confirmation_timeout_commands_direct_land():
    """Break caught: AUTO confirmation can wait indefinitely after takeoff."""
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle, mission_start_confirmation_timeout_s=0.5)
    try:
        finish_startup_validation(manager, vehicle)
        request_valid_start(manager, vehicle)
        drive_to_reset_mission(manager, vehicle)
        vehicle.finish_command()
        manager.tick(now=6.2)
        manager.tick(now=6.3)
        vehicle.finish_command()
        manager.tick(now=6.4)
        vehicle.state = airborne_state(7.0, mode=GUIDED_MODE)
        manager.tick(now=7.0)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 1
        assert "mission confirmation timed out" in manager.reason
    finally:
        manager.close()


def test_main_constructs_spins_and_closes_the_node(monkeypatch):
    """Break caught: executable startup leaves the vehicle connection unclosed."""
    events: list[str] = []

    class FakeNode:
        def close(self):
            events.append("close")

    class FakeRclpy:
        @staticmethod
        def init(args=None):
            assert args is None
            events.append("init")

        @staticmethod
        def spin(node):
            assert isinstance(node, FakeNode)
            events.append("spin")

        @staticmethod
        def shutdown():
            events.append("shutdown")

    monkeypatch.setattr(manager_module, "rclpy", FakeRclpy)
    monkeypatch.setattr(manager_module, "FlightCycleManager", FakeNode)

    manager_module.main()

    assert events == ["init", "spin", "close", "shutdown"]

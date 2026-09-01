"""Authority, link-loss, and terminal completion lifecycle tests."""

from __future__ import annotations

from dataclasses import replace

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy
from std_srvs.srv import Trigger

import flight_cycle_manager.manager as manager_module
from flight_cycle_manager.landing_backend import (
    LandingBackendDecision,
    LandingBackendOutcome,
)
from flight_cycle_manager.manager import (
    AUTO_MODE,
    AUTO_RTL_MODE,
    GUIDED_MODE,
    LAND_MODE,
    MISSION_STATE_ACTIVE,
    CycleState,
    _PendingOperation,
)
from test_manager_docking import (
    StubPidLandingBackend,
    StubPreciseLandingBackend,
    StubSearchController,
    publish_dock_target,
)
from test_manager_start import (
    airborne_state,
    auto_active_state,
    finish_startup_validation,
    ground_state,
    make_manager,
    valid_contract_items,
)


LOITER_MODE = 5
MISSION_TOTAL = len(valid_contract_items())


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def stub_manager_dependencies(monkeypatch):
    StubPidLandingBackend.instances = []
    StubPreciseLandingBackend.instances = []
    monkeypatch.setattr(manager_module, "DockSearchController", StubSearchController)
    monkeypatch.setattr(manager_module, "PidLandingBackend", StubPidLandingBackend)
    monkeypatch.setattr(
        manager_module,
        "PreciseLandingBackend",
        StubPreciseLandingBackend,
    )


def mission_progress_state(
    now: float,
    *,
    mode: int,
    seq: int,
    total: int = MISSION_TOTAL,
    mission_id: int | None = None,
):
    return replace(
        airborne_state(now, mode=mode),
        mission_seq=seq,
        mission_total=total,
        mission_state=MISSION_STATE_ACTIVE,
        mission_mode=0,
        mission_id=mission_id,
        mission_current_at=now,
    )


def on_ground_mission_state(
    now: float,
    *,
    mode: int = LAND_MODE,
    armed: bool,
    total: int = MISSION_TOTAL,
    mission_id: int | None = None,
):
    return replace(
        ground_state(now, mode=mode, armed=armed),
        mission_seq=2,
        mission_total=total,
        mission_state=MISSION_STATE_ACTIVE,
        mission_mode=0,
        mission_id=mission_id,
        mission_current_at=now,
    )


def ready_manager(*, backend_name: str = "pid"):
    vehicle = FakeVehicle(ground_state(0.0))
    manager = make_manager(vehicle, landing_backend=backend_name)
    finish_startup_validation(manager, vehicle)
    setattr(manager, "_validated_mission_total", MISSION_TOTAL)
    setattr(manager, "_validated_mission_id", None)
    return manager, vehicle


def flight_output_snapshot(vehicle: FakeVehicle) -> dict[str, object]:
    return {
        "set_mode_calls": list(vehicle.set_mode_calls),
        "arm_calls": list(vehicle.arm_calls),
        "takeoff_calls": list(vehicle.takeoff_calls),
        "set_mission_current_calls": list(vehicle.set_mission_current_calls),
        "start_mission_calls": vehicle.start_mission_calls,
        "request_dock_calls": vehicle.request_dock_calls,
        "land_calls": vehicle.land_calls,
        "rtl_calls": vehicle.rtl_calls,
        "velocity_calls": list(vehicle.velocity_calls),
        "landing_target_calls": list(vehicle.landing_target_calls),
        "condition_yaw_calls": list(vehicle.condition_yaw_calls),
    }


def configure_wait_takeoff_stable(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.WAIT_TAKEOFF_STABLE
    manager._stable_takeoff_since = None
    vehicle.state = mission_progress_state(now, mode=GUIDED_MODE, seq=0)
    return manager, vehicle, now


def configure_reset_onboard_mission(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.RESET_ONBOARD_MISSION
    vehicle.state = mission_progress_state(now, mode=GUIDED_MODE, seq=0)
    return manager, vehicle, now


def configure_start_auto_pre_ack(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.START_AUTO
    manager._mission_confirmation_deadline = None
    vehicle.state = mission_progress_state(now, mode=GUIDED_MODE, seq=0)
    return manager, vehicle, now


def configure_start_auto_confirming_auto(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.START_AUTO
    manager._mission_confirmation_deadline = now + 5.0
    manager._mission_confirmation_expected_seq = 0
    manager._auto_heartbeat_baseline = now
    manager._mission_current_baseline = now
    vehicle.state = auto_active_state(now, seq=0)
    return manager, vehicle, now


def configure_auto_mission(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.AUTO_MISSION
    vehicle.state = replace(auto_active_state(now, seq=99), mission_total=MISSION_TOTAL)
    return manager, vehicle, now


def configure_wait_for_handoff_auto(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.WAIT_FOR_HANDOFF
    vehicle.state = replace(
        auto_active_state(now, seq=manager.contract.land_start_seq),
        mode=AUTO_MODE,
        mission_total=MISSION_TOTAL,
    )
    return manager, vehicle, now


def configure_wait_for_handoff_auto_rtl(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.WAIT_FOR_HANDOFF
    vehicle.state = replace(
        auto_active_state(now, seq=manager.contract.land_start_seq),
        mode=AUTO_RTL_MODE,
        mission_total=MISSION_TOTAL,
    )
    return manager, vehicle, now


def configure_search_dock(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.SEARCH_DOCK
    manager._search_controller = StubSearchController(manager._build_search_config())
    vehicle.state = mission_progress_state(
        now,
        mode=GUIDED_MODE,
        seq=manager.contract.handoff_seq,
    )
    return manager, vehicle, now


def configure_landing_backend_guided(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager(backend_name="pid")
    now = 20.0
    manager._state = CycleState.LANDING_BACKEND
    manager._landing_controller = StubPidLandingBackend(
        manager._build_pid_landing_config()
    )
    vehicle.state = mission_progress_state(
        now,
        mode=GUIDED_MODE,
        seq=manager.contract.handoff_seq,
    )
    return manager, vehicle, now


def configure_landing_backend_land(monkeypatch):
    manager, vehicle = ready_manager(backend_name="precise_landing")
    now = 20.0
    manager._state = CycleState.LANDING_BACKEND
    manager._landing_expected_modes = frozenset({LAND_MODE})
    manager._landing_controller = StubPreciseLandingBackend(
        manager._build_precise_landing_config()
    )
    manager.backend.queue_decision(
        LandingBackendDecision(
            LandingBackendOutcome.RUNNING,
            "WAIT_LANDED",
            stream_landing_target=True,
        )
    )
    publish_dock_target(manager, monkeypatch, received_at=now, mode="OUTER")
    vehicle.state = mission_progress_state(
        now,
        mode=LAND_MODE,
        seq=manager.contract.handoff_seq,
    )
    return manager, vehicle, now


def configure_return_to_fc(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.RETURN_TO_FC
    manager._mission_confirmation_deadline = now + 5.0
    manager._mission_confirmation_expected_seq = manager.contract.fallback_land_seq
    manager._auto_heartbeat_baseline = now
    manager._mission_current_baseline = now
    vehicle.state = replace(
        auto_active_state(now, seq=manager.contract.fallback_land_seq),
        mission_total=MISSION_TOTAL,
    )
    return manager, vehicle, now


def configure_fc_fallback_land(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.FC_FALLBACK_LAND
    manager._fc_fallback_expected_modes = frozenset({AUTO_RTL_MODE, LAND_MODE})
    vehicle.state = mission_progress_state(
        now,
        mode=AUTO_RTL_MODE,
        seq=manager.contract.handoff_seq,
    )
    return manager, vehicle, now


def configure_wait_landed(monkeypatch):
    del monkeypatch
    manager, vehicle = ready_manager()
    now = 20.0
    manager._state = CycleState.WAIT_LANDED
    vehicle.state = mission_progress_state(
        now,
        mode=LAND_MODE,
        seq=manager.contract.handoff_seq,
    )
    return manager, vehicle, now


ACTIVE_MODE_SCENARIOS = [
    ("wait_takeoff_stable_guided", configure_wait_takeoff_stable),
    ("reset_onboard_mission_guided", configure_reset_onboard_mission),
    ("start_auto_pre_ack_guided", configure_start_auto_pre_ack),
    ("start_auto_confirming_auto", configure_start_auto_confirming_auto),
    ("auto_mission_auto", configure_auto_mission),
    ("wait_for_handoff_auto", configure_wait_for_handoff_auto),
    ("wait_for_handoff_auto_rtl", configure_wait_for_handoff_auto_rtl),
    ("search_dock_guided", configure_search_dock),
    ("landing_backend_guided", configure_landing_backend_guided),
    ("landing_backend_land", configure_landing_backend_land),
    ("return_to_fc_auto", configure_return_to_fc),
    ("fc_fallback_land_auto_rtl", configure_fc_fallback_land),
    ("wait_landed_land", configure_wait_landed),
]


@pytest.mark.parametrize(("label", "configure"), ACTIVE_MODE_SCENARIOS)
def test_expected_active_modes_do_not_latch_override(label: str, configure, monkeypatch):
    """Break caught: expected manager-owned or FC-owned modes are rejected as overrides."""
    del label
    manager, vehicle, now = configure(monkeypatch)
    try:
        manager.tick(now=now)
        assert manager.state is not CycleState.PILOT_OVERRIDE
    finally:
        manager.close()


@pytest.mark.parametrize(("label", "configure"), ACTIVE_MODE_SCENARIOS)
def test_unexpected_active_modes_latch_passive_override_without_output(
    label: str, configure, monkeypatch
):
    """Break caught: an airborne mode departure can emit another flight command first."""
    del label
    manager, vehicle, now = configure(monkeypatch)
    try:
        vehicle.state = replace(vehicle.state, mode=LOITER_MODE)
        before = flight_output_snapshot(vehicle)

        manager.tick(now=now)

        assert manager.state is CycleState.PILOT_OVERRIDE
        assert flight_output_snapshot(vehicle) == before
    finally:
        manager.close()


def test_pending_guided_mode_match_is_not_treated_as_pilot_override():
    """Break caught: telemetry can acknowledge a requested mode before its command future."""
    manager, vehicle = ready_manager()
    try:
        now = 20.0
        manager._state = CycleState.GUIDED_TAKEOVER
        vehicle.state = mission_progress_state(
            now,
            mode=GUIDED_MODE,
            seq=manager.contract.handoff_seq,
        )
        future = vehicle.set_mode_async(GUIDED_MODE, now)
        manager._pending = _PendingOperation("handoff_guided", future, now + 5.0)

        manager.tick(now=now)

        assert manager.state is CycleState.GUIDED_TAKEOVER

        vehicle.finish_command()
        manager.tick(now=now + 0.1)

        assert manager.state is CycleState.SEARCH_DOCK
    finally:
        manager.close()


def test_pilot_override_stays_passive_until_docked_then_explicit_start_clears_latch():
    """Break caught: override can resume autonomously or block the next explicit cycle forever."""
    manager, vehicle = ready_manager()
    try:
        manager._state = CycleState.SEARCH_DOCK
        manager._search_controller = StubSearchController(manager._build_search_config())
        vehicle.state = mission_progress_state(
            20.0,
            mode=LOITER_MODE,
            seq=manager.contract.handoff_seq,
        )

        manager.tick(now=20.0)

        assert manager.state is CycleState.PILOT_OVERRIDE

        response = manager._start_cycle_callback(
            Trigger.Request(),
            Trigger.Response(),
        )
        assert response.success is False
        assert "armed" in response.message

        before = flight_output_snapshot(vehicle)
        vehicle.state = mission_progress_state(
            21.0,
            mode=GUIDED_MODE,
            seq=manager.contract.handoff_seq,
        )
        manager.tick(now=21.0)
        assert manager.state is CycleState.PILOT_OVERRIDE
        assert flight_output_snapshot(vehicle) == before

        vehicle.state = on_ground_mission_state(22.0, armed=True)
        manager.tick(now=22.0)
        assert manager.state is CycleState.WAIT_DISARM

        vehicle.state = on_ground_mission_state(22.1, armed=False)
        manager.tick(now=22.1)
        assert manager.state is CycleState.DOCKED

        restart = manager._start_cycle_callback(
            Trigger.Request(),
            Trigger.Response(),
        )
        assert restart.success is True
        vehicle.finish_download(valid_contract_items())
        manager.tick(now=22.2)

        assert manager.state is CycleState.PREARM
    finally:
        manager.close()


def test_stale_heartbeat_enters_passive_link_loss_without_recovery_commands():
    """Break caught: stale heartbeat still tries to steer or re-command the aircraft."""
    manager, vehicle = ready_manager()
    try:
        manager._state = CycleState.SEARCH_DOCK
        manager._search_controller = StubSearchController(manager._build_search_config())
        vehicle.state = replace(
            mission_progress_state(
                20.0,
                mode=GUIDED_MODE,
                seq=manager.contract.handoff_seq,
            ),
            position_at=22.9,
            landed_at=22.9,
        )
        before = flight_output_snapshot(vehicle)

        manager.tick(now=23.0)

        assert manager.state is CycleState.LINK_LOSS
        assert flight_output_snapshot(vehicle) == before

        vehicle.state = mission_progress_state(
            23.1,
            mode=GUIDED_MODE,
            seq=manager.contract.handoff_seq,
        )
        manager.tick(now=23.1)
        assert manager.state is CycleState.LINK_LOSS
        assert flight_output_snapshot(vehicle) == before
    finally:
        manager.close()


@pytest.mark.parametrize(
    ("field_name", "changed_total", "validated_id", "changed_id"),
    [
        ("mission_total", MISSION_TOTAL + 1, None, None),
        ("mission_id", MISSION_TOTAL, 17, 18),
    ],
)
def test_mission_integrity_change_requests_rtl_once_then_direct_land_on_rejection(
    field_name: str,
    changed_total: int,
    validated_id: int | None,
    changed_id: int | None,
):
    """Break caught: changed mission identity/count can keep using stale sequence authority."""
    manager, vehicle = ready_manager()
    try:
        now = 20.0
        manager._state = CycleState.SEARCH_DOCK
        manager._search_controller = StubSearchController(manager._build_search_config())
        setattr(manager, "_validated_mission_total", MISSION_TOTAL)
        setattr(manager, "_validated_mission_id", validated_id)
        vehicle.state = mission_progress_state(
            now,
            mode=GUIDED_MODE,
            seq=manager.contract.handoff_seq,
            total=changed_total,
            mission_id=changed_id if field_name == "mission_id" else validated_id,
        )

        manager.tick(now=now)
        manager.tick(now=now + 0.05)

        assert vehicle.rtl_calls == 1
        assert vehicle.land_calls == 0

        vehicle.reject_pending("RTL rejected")
        manager.tick(now=now + 0.1)

        assert vehicle.rtl_calls == 1
        assert vehicle.land_calls == 1
        assert manager.state is CycleState.FC_FALLBACK_LAND
    finally:
        manager.close()


def test_mission_integrity_rtl_timeout_defers_direct_land_until_transport_clears():
    """Break caught: timed-out RTL can drop the fallback LAND while MAVLink is still busy."""
    manager, vehicle = ready_manager()
    try:
        now = 20.0
        manager._state = CycleState.SEARCH_DOCK
        manager._search_controller = StubSearchController(manager._build_search_config())
        setattr(manager, "_validated_mission_total", MISSION_TOTAL)
        setattr(manager, "_validated_mission_id", 23)
        vehicle.state = mission_progress_state(
            now,
            mode=GUIDED_MODE,
            seq=manager.contract.handoff_seq,
            total=MISSION_TOTAL,
            mission_id=24,
        )

        manager.tick(now=now)
        manager.tick(now=now + 0.05)
        vehicle.state = mission_progress_state(
            now + 10.1,
            mode=GUIDED_MODE,
            seq=manager.contract.handoff_seq,
            total=MISSION_TOTAL,
            mission_id=24,
        )
        manager.tick(now=now + 10.1)

        assert vehicle.rtl_calls == 1
        assert vehicle.land_calls == 0
        assert manager.state is CycleState.FC_FALLBACK_LAND

        vehicle.finish_command(success=False, message="late RTL completion")
        manager.tick(now=now + 10.2)

        assert vehicle.rtl_calls == 1
        assert vehicle.land_calls == 1
        assert manager.state is CycleState.FC_FALLBACK_LAND
    finally:
        manager.close()


@pytest.mark.parametrize("fallback_mode", [AUTO_RTL_MODE, LAND_MODE])
def test_mission_integrity_fallback_accepts_rtl_and_land_modes(fallback_mode: int):
    """Break caught: fallback authority fights the FC after mission-integrity handback."""
    manager, vehicle = ready_manager()
    try:
        now = 20.0
        manager._state = CycleState.SEARCH_DOCK
        manager._search_controller = StubSearchController(manager._build_search_config())
        setattr(manager, "_validated_mission_total", MISSION_TOTAL)
        vehicle.state = mission_progress_state(
            now,
            mode=GUIDED_MODE,
            seq=manager.contract.handoff_seq,
            total=MISSION_TOTAL + 1,
        )

        manager.tick(now=now)
        vehicle.state = mission_progress_state(
            now + 0.1,
            mode=fallback_mode,
            seq=manager.contract.handoff_seq,
            total=MISSION_TOTAL + 1,
        )
        vehicle.finish_command()
        manager.tick(now=now + 0.1)

        assert manager.state is not CycleState.PILOT_OVERRIDE
        assert vehicle.land_calls == 0
    finally:
        manager.close()


def test_touchdown_and_disarm_finish_docked_without_repeat():
    """Break caught: completed landing can re-run commands instead of staying terminal."""
    manager, vehicle = ready_manager()
    try:
        manager._state = CycleState.WAIT_LANDED
        vehicle.state = on_ground_mission_state(30.0, armed=True)

        manager.tick(now=30.0)

        assert manager.state is CycleState.WAIT_DISARM

        vehicle.state = on_ground_mission_state(31.0, armed=False)
        manager.tick(now=31.0)

        assert manager.state is CycleState.DOCKED
        before = flight_output_snapshot(vehicle)

        manager.tick(now=100.0)

        assert manager.state is CycleState.DOCKED
        assert flight_output_snapshot(vehicle) == before
    finally:
        manager.close()


def test_pid_disarm_timeout_defers_direct_land_until_live_transport_clears():
    """Break caught: fallback LAND is dropped while a real lower disarm command is still pending."""
    manager, vehicle = ready_manager(backend_name="pid")
    try:
        now = 20.0
        backend = StubPidLandingBackend(manager._build_pid_landing_config())
        backend.queue_decision(
            LandingBackendDecision(
                LandingBackendOutcome.FAIL_AFTER_COMMIT,
                "FAILED",
                reason="touchdown controller failed",
            )
        )
        manager._state = CycleState.LANDING_BACKEND
        manager._landing_controller = backend
        vehicle.state = mission_progress_state(
            now,
            mode=GUIDED_MODE,
            seq=manager.contract.handoff_seq,
        )
        future = vehicle.arm_async(False, now)
        manager._pending = _PendingOperation("pid_disarm", future, now + 0.1)
        manager._pending_pid_disarm_request_id = 7

        manager.tick(now=now + 0.2)

        assert backend.disarm_results == [(7, False, now + 0.2)]
        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 0

        vehicle.finish_command(success=False, message="disarm rejected")
        manager.tick(now=now + 0.3)

        assert vehicle.land_calls == 1
    finally:
        manager.close()

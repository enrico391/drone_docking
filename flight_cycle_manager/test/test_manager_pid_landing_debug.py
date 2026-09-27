"""Repeatable LAND-mode trigger tests for mission-free PID docking debug."""

from __future__ import annotations

from dataclasses import replace
import json

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from flight_cycle_manager.manager import (
    GUIDED_MODE,
    LAND_MODE,
    CycleState,
    FlightCycleManager,
)
from flight_cycle_manager.landing_backend import (
    LandingBackendDecision,
    LandingBackendOutcome,
)
from test_manager_start import airborne_state, ground_state


LOITER_MODE = 5


def acknowledged_guided_confirmation_timeout(observed_mode: int = LAND_MODE) -> str:
    return (
        "MAVLink mode change acknowledged but timed out waiting for mode "
        f"{GUIDED_MODE}; observed mode {observed_mode}"
    )


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_debug_manager(
    vehicle: FakeVehicle, **parameter_values: object
) -> FlightCycleManager:
    return FlightCycleManager(
        vehicle=vehicle,
        parameter_values={
            "operation_mode": "pid_landing_debug",
            **parameter_values,
        },
    )


def make_align_manager(
    vehicle: FakeVehicle, *, auto_start: bool
) -> FlightCycleManager:
    return FlightCycleManager(
        vehicle=vehicle,
        parameter_values={
            "operation_mode": "pid_align_debug",
            "pid_align_auto_start": auto_start,
        },
    )


def test_align_debug_auto_start_requests_guided_without_land_trigger():
    """Break caught: auto-start alignment remains blocked on a LAND edge."""
    vehicle = FakeVehicle(airborne_state(1.0, mode=LOITER_MODE))
    manager = make_align_manager(vehicle, auto_start=True)
    try:
        manager.tick(1.0)

        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert vehicle.set_mode_calls == [GUIDED_MODE]
    finally:
        manager.close()


def test_align_debug_manual_start_still_requires_fresh_land_edge():
    """Break caught: disabling auto-start no longer keeps alignment passive."""
    vehicle = FakeVehicle(airborne_state(1.0, mode=LOITER_MODE))
    manager = make_align_manager(vehicle, auto_start=False)
    try:
        manager.tick(1.0)
        assert manager.state is CycleState.WAITING_LAND_TRIGGER
        assert vehicle.set_mode_calls == []

        vehicle.state = airborne_state(1.1, mode=LAND_MODE)
        manager.tick(1.1)
        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert vehicle.set_mode_calls == [GUIDED_MODE]
    finally:
        manager.close()


def test_align_debug_starts_backend_without_shared_search():
    """Break caught: alignment tuning runs the height-changing dock search."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=LOITER_MODE))
    manager = make_align_manager(vehicle, auto_start=True)
    try:
        manager.tick(2.0)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(2.1, mode=GUIDED_MODE)
        manager.tick(2.1)

        assert manager.state is CycleState.LANDING_BACKEND
        assert manager.search_controller is None
        assert manager.backend is not None
    finally:
        manager.close()


def test_align_debug_dashboard_setpoint_commands_z_and_reports_error(monkeypatch):
    from test_pid_landing_backend import dock_observation

    vehicle = FakeVehicle(airborne_state(2.0, mode=LOITER_MODE))
    manager = make_align_manager(vehicle, auto_start=True)
    published = []
    try:
        manager.tick(2.0)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(2.1, mode=GUIDED_MODE)
        manager.tick(2.1)
        assert not manager.set_parameters_atomically([
            Parameter("pid_align_distance_setpoint_m", value=2.5)
        ]).successful

        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock_observation(now))
        monkeypatch.setattr(manager._pid_debug_publisher, "publish",
                            lambda message: published.append(json.loads(message.data)))
        vehicle.state = airborne_state(2.2, mode=GUIDED_MODE)
        manager.tick(2.2)
        assert vehicle.velocity_calls[-1][3] == pytest.approx(0.0)
        assert published[-1]["distance_setpoint_m"] == pytest.approx(2.0)
        assert published[-1]["dock_distance_m"] == pytest.approx(2.0)

        assert manager.set_parameters_atomically([
            Parameter("pid_align_distance_setpoint_m", value=2.5)
        ]).successful
        vehicle.state = airborne_state(2.3, mode=GUIDED_MODE)
        manager.tick(2.3)
        assert vehicle.velocity_calls[-1][3] > 0.0
        assert published[-1]["error"]["z"] == pytest.approx(0.5)
        assert published[-1]["alignment_only"] is True
        for value in (float("nan"), 0.0, 25.0):
            assert not manager.set_parameters_atomically([
                Parameter("pid_align_distance_setpoint_m", value=value)
            ]).successful
        assert manager.get_parameter("pid_align_distance_setpoint_m").value == 2.5
    finally:
        manager.close()


def test_align_debug_guided_rejection_stops_without_commanding_land():
    """Break caught: failed tuning engagement unexpectedly starts a landing."""
    vehicle = FakeVehicle(airborne_state(3.0, mode=LOITER_MODE))
    manager = make_align_manager(vehicle, auto_start=True)
    try:
        manager.tick(3.0)
        vehicle.finish_command(success=False, message="GUIDED rejected")

        manager.tick(3.1)

        assert manager.state is CycleState.WAITING_LAND_TRIGGER
        assert vehicle.land_calls == 0
        assert "GUIDED rejected" in manager.reason
    finally:
        manager.close()


class _OneDecisionBackend:
    def __init__(self, decision: LandingBackendDecision) -> None:
        self._decision = decision
        self.disarm_results: list[tuple[int, bool, float]] = []

    def update(self, **kwargs) -> LandingBackendDecision:
        del kwargs
        return self._decision

    def notify_disarm_result(
        self, request_id: int, success: bool, now: float
    ) -> None:
        self.disarm_results.append((request_id, success, now))


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


def test_debug_guided_handoff_retries_acknowledged_confirmation_timeout():
    """Break caught: PID debug cancels instead of retrying an ACKed LAND handoff."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(2.0)
        vehicle.state = airborne_state(2.1, mode=LAND_MODE)
        manager.tick(2.1)
        vehicle.finish_command(
            success=False, message=acknowledged_guided_confirmation_timeout()
        )

        manager.tick(2.2)

        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert "retrying GUIDED (1/1)" in manager.reason
        assert vehicle.set_mode_calls == [GUIDED_MODE, GUIDED_MODE]

        vehicle.state = airborne_state(2.3, mode=GUIDED_MODE)
        vehicle.finish_command()
        manager.tick(2.3)

        assert manager.state is CycleState.SEARCH_DOCK
    finally:
        manager.close()


def test_debug_guided_handoff_does_not_retry_after_mode_change():
    """Break caught: a pilot mode change is retried as if the mode remained LAND."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=LOITER_MODE))
    manager = make_debug_manager(vehicle)
    try:
        manager.tick(2.0)
        vehicle.state = airborne_state(2.1, mode=LAND_MODE)
        manager.tick(2.1)
        vehicle.state = airborne_state(2.2, mode=LOITER_MODE)
        vehicle.finish_command(
            success=False,
            message=acknowledged_guided_confirmation_timeout(observed_mode=LOITER_MODE)
        )

        manager.tick(2.2)

        assert manager.state is CycleState.WAITING_LAND_TRIGGER
        assert vehicle.set_mode_calls == [GUIDED_MODE]
        assert vehicle.land_calls == 0
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


def test_debug_logging_creates_a_new_file_for_each_attempt(tmp_path):
    """Break caught: a later landing approach overwrites the earlier trace."""
    vehicle = FakeVehicle(airborne_state(1.0, mode=LOITER_MODE))
    manager = make_debug_manager(
        vehicle,
        landing_attempt_log_enabled=True,
        landing_attempt_log_directory=str(tmp_path),
    )
    try:
        for start in (1.0, 2.0):
            vehicle.state = airborne_state(start, mode=LOITER_MODE)
            manager.tick(start)
            vehicle.state = airborne_state(start + 0.1, mode=LAND_MODE)
            manager.tick(start + 0.1)
            vehicle.state = airborne_state(start + 0.2, mode=LOITER_MODE)
            manager.tick(start + 0.2)

        traces = sorted(tmp_path.glob("landing_attempt_*.txt"))
        assert len(traces) == 2
        assert traces[0].name != traces[1].name
        for trace in traces:
            text = trace.read_text(encoding="utf-8")
            assert "state=GUIDED_TAKEOVER" in text
            assert "outcome=CANCELLED" in text
            assert "flight mode changed away from GUIDED" in text
    finally:
        manager.close()


def test_debug_log_records_pid_retry_observations_and_failure(tmp_path):
    """Break caught: traces omit the PID retry count or terminal failure reason."""
    vehicle = FakeVehicle(airborne_state(3.0, mode=LOITER_MODE))
    manager = make_debug_manager(
        vehicle,
        landing_attempt_log_enabled=True,
        landing_attempt_log_directory=str(tmp_path),
        pid_max_recovery_attempts=3,
    )
    try:
        manager.tick(3.0)
        vehicle.state = airborne_state(3.1, mode=LAND_MODE)
        manager.tick(3.1)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(3.2, mode=GUIDED_MODE)
        manager.tick(3.2)

        manager._state = CycleState.LANDING_BACKEND
        manager._landing_controller = _OneDecisionBackend(
            LandingBackendDecision(
                outcome=LandingBackendOutcome.RUNNING,
                substate="RECOVERY_CLIMB",
                recovery_attempts=2,
                reason="dock target lost",
            )
        )
        manager.tick(3.3)

        manager._landing_controller = _OneDecisionBackend(
            LandingBackendDecision(
                outcome=LandingBackendOutcome.FAIL_BEFORE_COMMIT,
                substate="FAILED",
                recovery_attempts=3,
                reason="dock target recovery attempts exhausted",
            )
        )
        manager.tick(3.4)

        trace = next(tmp_path.glob("landing_attempt_*.txt"))
        text = trace.read_text(encoding="utf-8")
        assert "PID state=RECOVERY_CLIMB recovery=2/3" in text
        assert "vehicle_mode=4 armed=True flying=True" in text
        assert "dock=none" in text
        assert "PID state=FAILED recovery=3/3" in text
        assert "dock target recovery attempts exhausted" in text
        assert "outcome=FAILED" in text
    finally:
        manager.close()


def test_debug_logging_is_disabled_by_default(tmp_path):
    """Break caught: normal manager construction starts writing debug artifacts."""
    vehicle = FakeVehicle(airborne_state(4.0, mode=LOITER_MODE))
    manager = make_debug_manager(
        vehicle,
        landing_attempt_log_directory=str(tmp_path),
    )
    try:
        manager.tick(4.0)
        vehicle.state = airborne_state(4.1, mode=LAND_MODE)
        manager.tick(4.1)
        assert list(tmp_path.iterdir()) == []
    finally:
        manager.close()


def test_disabled_logging_does_not_expand_an_invalid_home_directory():
    """Break caught: a disabled optional logger prevents manager startup."""
    vehicle = FakeVehicle(airborne_state(5.0, mode=LOITER_MODE))
    manager = make_debug_manager(
        vehicle,
        landing_attempt_log_enabled=False,
        landing_attempt_log_directory="~flight-cycle-user-does-not-exist/logs",
    )
    manager.close()


def test_logger_exceptions_do_not_change_landing_control(tmp_path):
    """Break caught: trace failures trigger fallback flight behavior."""
    vehicle = FakeVehicle(airborne_state(6.0, mode=LOITER_MODE))
    manager = make_debug_manager(
        vehicle,
        landing_attempt_log_enabled=True,
        landing_attempt_log_directory=str(tmp_path),
    )
    try:
        manager.tick(6.0)
        manager._landing_attempt_log.start = lambda *args: (_ for _ in ()).throw(
            RuntimeError("trace start failed")
        )
        vehicle.state = airborne_state(6.1, mode=LAND_MODE)
        manager.tick(6.1)
        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert vehicle.set_mode_calls == [GUIDED_MODE]

        manager._landing_attempt_log.start = lambda *args: None
        manager._landing_attempt_log.write = lambda *args: (_ for _ in ()).throw(
            RuntimeError("trace write failed")
        )
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(6.2, mode=GUIDED_MODE)
        manager.tick(6.2)
        manager._state = CycleState.LANDING_BACKEND
        manager._landing_controller = _OneDecisionBackend(
            LandingBackendDecision(
                outcome=LandingBackendOutcome.RUNNING,
                substate="PID_ALIGN",
            )
        )
        manager.tick(6.3)

        assert manager.state is CycleState.LANDING_BACKEND
        assert vehicle.land_calls == 0
    finally:
        manager.close()


def test_debug_log_records_rejected_disarm_result(tmp_path):
    """Break caught: disarm retries lack the command failure that caused them."""
    vehicle = FakeVehicle(airborne_state(7.0, mode=LOITER_MODE))
    manager = make_debug_manager(
        vehicle,
        landing_attempt_log_enabled=True,
        landing_attempt_log_directory=str(tmp_path),
    )
    try:
        manager.tick(7.0)
        vehicle.state = airborne_state(7.1, mode=LAND_MODE)
        manager.tick(7.1)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(7.2, mode=GUIDED_MODE)
        manager.tick(7.2)

        backend = _OneDecisionBackend(
            LandingBackendDecision(
                outcome=LandingBackendOutcome.RUNNING,
                substate="DISARM",
                request_disarm=True,
                disarm_request_id=4,
            )
        )
        manager._state = CycleState.LANDING_BACKEND
        manager._landing_controller = backend
        manager.tick(7.3)
        vehicle.finish_command(success=False, message="command rejected")
        manager.tick(7.4)

        trace = next(tmp_path.glob("landing_attempt_*.txt"))
        text = trace.read_text(encoding="utf-8")
        assert "DISARM_RESULT request=4 success=False" in text
        assert "reason=command rejected" in text
        assert backend.disarm_results[0] == (4, False, 7.4)
    finally:
        manager.close()

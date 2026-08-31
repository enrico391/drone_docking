"""Docking-lifecycle tests for request handoff, shared search, and fallback."""

from __future__ import annotations

from dataclasses import replace

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

import flight_cycle_manager.manager as manager_module
from flight_cycle_manager.dock_search import SearchDecision, SearchOutcome
from flight_cycle_manager.landing_backend import (
    LandingBackendDecision,
    LandingBackendOutcome,
    VelocityCommand,
    VelocityFrame,
)
from flight_cycle_manager.manager import (
    AUTO_MODE,
    AUTO_RTL_MODE,
    GUIDED_MODE,
    LAND_MODE,
    CycleState,
    FlightCycleManager,
)
from test_manager_start import (
    auto_active_state,
    drive_to_start_mission_pending,
    finish_startup_validation,
    make_manager,
    request_valid_start,
)


class StubSearchController:
    def __init__(self, config):
        self.config = config
        self.starts: list[tuple[float, tuple[float, float, float]]] = []
        self.updates = []
        self.center_enu_m: tuple[float, float, float] | None = None
        self._queued: list[SearchDecision] = []

    def start(self, *, now: float, center_enu_m: tuple[float, float, float]) -> None:
        self.starts.append((now, center_enu_m))
        self.center_enu_m = center_enu_m

    def queue_decision(
        self,
        outcome: SearchOutcome,
        *,
        velocity_enu_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
        reason: str = "stub search decision",
    ) -> None:
        self._queued.append(SearchDecision(outcome, velocity_enu_m_s, reason))

    def update(self, observation):
        self.updates.append(observation)
        if self._queued:
            return self._queued.pop(0)
        return SearchDecision(
            SearchOutcome.SEARCHING,
            (0.0, 0.0, 0.0),
            "stub search holding",
        )


class StubLandingBackend:
    def __init__(self, config):
        self.config = config
        self.starts = []
        self.updates = []
        self.disarm_results = []
        self.land_results = []
        self._queued: list[LandingBackendDecision] = []

    def start(self, *, now: float, dock, vehicle) -> None:
        self.starts.append((now, dock, vehicle))

    def queue_decision(self, decision: LandingBackendDecision) -> None:
        self._queued.append(decision)

    def update(self, *, now: float, dock, vehicle):
        self.updates.append((now, dock, vehicle))
        if self._queued:
            return self._queued.pop(0)
        return LandingBackendDecision(
            LandingBackendOutcome.RUNNING,
            "RUNNING",
        )

    def notify_disarm_result(self, request_id: int, success: bool, now: float) -> None:
        self.disarm_results.append((request_id, success, now))

    def notify_land_result(self, success: bool, now: float) -> None:
        self.land_results.append((success, now))


class StubPidLandingBackend(StubLandingBackend):
    instances: list["StubPidLandingBackend"] = []

    def __init__(self, config):
        super().__init__(config)
        self.__class__.instances.append(self)


class StubPreciseLandingBackend(StubLandingBackend):
    instances: list["StubPreciseLandingBackend"] = []

    def __init__(self, config):
        super().__init__(config)
        self.__class__.instances.append(self)


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


def publish_dock_target(
    manager: FlightCycleManager,
    monkeypatch,
    *,
    received_at: float,
    mode: str = "OUTER",
    camera_x_m: float = 0.1,
    camera_y_m: float = -0.1,
    camera_z_m: float = 2.0,
) -> None:
    pose = PoseStamped()
    pose.header.stamp.sec = int(received_at)
    pose.header.stamp.nanosec = int((received_at - int(received_at)) * 1_000_000_000)
    pose.pose.position.x = camera_x_m
    pose.pose.position.y = camera_y_m
    pose.pose.position.z = camera_z_m
    pose.pose.orientation.w = 1.0

    monkeypatch.setattr(manager_module.time, "monotonic", lambda: received_at)
    manager._dock_pose_callback(pose)
    mode_message = String()
    mode_message.data = mode
    manager._dock_mode_callback(mode_message)


def drive_to_auto_mission(
    manager: FlightCycleManager,
    vehicle: FakeVehicle,
) -> None:
    finish_startup_validation(manager, vehicle)
    request_valid_start(manager, vehicle)
    drive_to_start_mission_pending(manager, vehicle)
    vehicle.finish_command()
    manager.tick(now=6.4)
    vehicle.state = auto_active_state(6.5, seq=0)
    manager.tick(now=6.5)
    assert manager.state is CycleState.AUTO_MISSION
    assert manager.contract is not None


def call_request_dock(manager: FlightCycleManager, *, now: float) -> Trigger.Response:
    response = manager._request_dock_callback(Trigger.Request(), Trigger.Response())
    manager.tick(now=now)
    return response


def guided_state(
    now: float,
    *,
    seq: int,
    mode: int = GUIDED_MODE,
    position_enu_m: tuple[float, float, float] = (1.0, 2.0, 4.5),
):
    return replace(
        auto_active_state(now, seq=seq),
        mode=mode,
        position_enu_m=position_enu_m,
        position_at=now,
        landed_at=now,
    )


def enter_search(
    manager: FlightCycleManager,
    vehicle: FakeVehicle,
    *,
    handoff_at: float = 10.0,
    guided_at: float = 10.1,
    center_enu_m: tuple[float, float, float] = (1.0, 2.0, 4.5),
) -> None:
    vehicle.state = auto_active_state(handoff_at, seq=manager.contract.handoff_seq)
    manager.tick(now=handoff_at)
    vehicle.state = guided_state(
        guided_at,
        seq=manager.contract.handoff_seq,
        position_enu_m=center_enu_m,
    )
    vehicle.finish_command()
    manager.tick(now=guided_at)
    assert manager.state is CycleState.SEARCH_DOCK


def start_backend(
    manager: FlightCycleManager,
    vehicle: FakeVehicle,
    monkeypatch,
    *,
    backend_name: str,
) -> StubLandingBackend:
    publish_dock_target(manager, monkeypatch, received_at=10.05)
    enter_search(manager, vehicle)
    manager.search_controller.queue_decision(SearchOutcome.ACQUIRED)
    vehicle.state = guided_state(10.2, seq=manager.contract.handoff_seq)
    manager.tick(now=10.2)
    assert manager.state is CycleState.LANDING_BACKEND
    assert manager.backend is not None
    assert isinstance(manager.backend, StubLandingBackend)
    return manager.backend


def fallback_from_search_timeout(
    manager: FlightCycleManager,
    vehicle: FakeVehicle,
) -> None:
    enter_search(manager, vehicle)
    manager.search_controller.queue_decision(
        SearchOutcome.TIMED_OUT,
        reason="shared search timeout",
    )
    vehicle.state = guided_state(40.0, seq=manager.contract.handoff_seq)
    manager.tick(now=40.0)
    assert manager.state is CycleState.RETURN_TO_FC


def backend_manager(*, backend_name: str = "pid", **parameters: object):
    vehicle = FakeVehicle()
    manager = make_manager(vehicle, landing_backend=backend_name, **parameters)
    drive_to_auto_mission(manager, vehicle)
    return manager, vehicle


def test_request_dock_is_rejected_outside_auto_mission():
    """Break caught: request_dock can start from any non-AUTO lifecycle state."""
    vehicle = FakeVehicle()
    manager = make_manager(vehicle)
    try:
        finish_startup_validation(manager, vehicle)

        response = manager._request_dock_callback(
            Trigger.Request(),
            Trigger.Response(),
        )

        assert response.success is False
        assert "AUTO_MISSION" in response.message
        assert vehicle.request_dock_calls == 0
    finally:
        manager.close()


def test_request_dock_is_idempotent_and_queues_do_land_start():
    """Break caught: repeated dock requests duplicate DO_LAND_START commands."""
    manager, vehicle = backend_manager()
    try:
        first = call_request_dock(manager, now=7.0)
        second = call_request_dock(manager, now=7.1)

        assert first.success is True
        assert second.success is True
        assert manager.state is CycleState.WAIT_FOR_HANDOFF
        assert vehicle.request_dock_calls == 1
    finally:
        manager.close()


def test_request_dock_ack_failure_commands_direct_land_once():
    """Break caught: a rejected DO_LAND_START leaves the aircraft in AUTO."""
    manager, vehicle = backend_manager()
    try:
        call_request_dock(manager, now=7.0)
        vehicle.reject_pending("DO_LAND_START rejected")
        manager.tick(now=7.1)
        manager.tick(now=7.2)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert "DO_LAND_START rejected" in manager.reason
        assert vehicle.land_calls == 1
    finally:
        manager.close()


def test_wait_for_handoff_accepts_auto_rtl_approach_progress():
    """Break caught: AUTO_RTL approach progress is treated as an unexpected mode."""
    manager, vehicle = backend_manager()
    try:
        vehicle.state = replace(
            auto_active_state(8.0, seq=manager.contract.land_start_seq),
            mode=AUTO_RTL_MODE,
        )
        manager.tick(now=8.0)
        manager.tick(now=8.5)

        assert manager.state is CycleState.WAIT_FOR_HANDOFF
        assert vehicle.land_calls == 0
        assert vehicle.set_mode_calls == [GUIDED_MODE]
    finally:
        manager.close()


def test_handoff_sequence_switches_to_guided_and_starts_search_from_fresh_pose():
    """Break caught: search centers on stale pre-GUIDED telemetry."""
    manager, vehicle = backend_manager()
    try:
        vehicle.state = auto_active_state(10.0, seq=manager.contract.handoff_seq)
        manager.tick(now=10.0)
        fresh_center = (3.0, -1.5, 4.2)
        vehicle.state = replace(
            auto_active_state(10.1, seq=manager.contract.handoff_seq),
            mode=GUIDED_MODE,
            position_enu_m=fresh_center,
            position_at=10.1,
        )
        vehicle.finish_command()
        manager.tick(now=10.1)

        assert manager.state is CycleState.SEARCH_DOCK
        assert manager.search_controller.center_enu_m == pytest.approx(fresh_center)
    finally:
        manager.close()


def test_shared_search_velocity_routes_through_map_enu():
    """Break caught: shared search emits a velocity in the wrong transport frame."""
    manager, vehicle = backend_manager()
    try:
        enter_search(manager, vehicle)
        manager.search_controller.queue_decision(
            SearchOutcome.SEARCHING,
            velocity_enu_m_s=(0.2, -0.1, 0.05),
        )
        vehicle.state = guided_state(10.2, seq=manager.contract.handoff_seq)

        manager.tick(now=10.2)

        assert vehicle.velocity_calls[-1] == (
            VelocityFrame.MAP_ENU,
            0.2,
            -0.1,
            0.05,
            None,
            10.2,
        )
    finally:
        manager.close()


@pytest.mark.parametrize("backend_name", ["pid", "precise_landing"])
def test_shared_search_acquisition_starts_selected_backend_once(
    backend_name: str,
    monkeypatch,
):
    """Break caught: search acquisition can restart or skip the selected backend."""
    manager, vehicle = backend_manager(backend_name=backend_name)
    try:
        backend = start_backend(
            manager,
            vehicle,
            monkeypatch,
            backend_name=backend_name,
        )
        manager.tick(now=10.3)

        assert len(manager.search_controller.starts) == 1
        assert len(backend.starts) == 1
    finally:
        manager.close()


def test_pid_backend_velocity_and_disarm_results_are_correlated(monkeypatch):
    """Break caught: PID backend outputs are not routed through one tracked transport."""
    manager, vehicle = backend_manager(backend_name="pid")
    try:
        backend = start_backend(
            manager,
            vehicle,
            monkeypatch,
            backend_name="pid",
        )
        backend.queue_decision(
            LandingBackendDecision(
                LandingBackendOutcome.RUNNING,
                "DISARM",
                velocity=VelocityCommand(
                    VelocityFrame.BODY_FLU,
                    0.3,
                    -0.2,
                    0.0,
                    yaw_rate_rad_s=0.15,
                ),
                request_disarm=True,
                disarm_request_id=7,
            )
        )

        vehicle.state = guided_state(12.0, seq=manager.contract.handoff_seq)
        manager.tick(now=12.0)
        vehicle.state = guided_state(12.1, seq=manager.contract.handoff_seq)
        vehicle.finish_command()
        manager.tick(now=12.1)

        assert vehicle.velocity_calls[-1] == (
            VelocityFrame.BODY_FLU,
            0.3,
            -0.2,
            0.0,
            0.15,
            12.0,
        )
        assert vehicle.arm_calls[-1] is False
        assert backend.disarm_results == [(7, True, 12.1)]
    finally:
        manager.close()


def test_native_backend_streams_fresh_generations_and_notifies_land_result(
    monkeypatch,
):
    """Break caught: native landing reuses stale targets or loses LAND ACK correlation."""
    manager, vehicle = backend_manager(backend_name="precise_landing")
    try:
        backend = start_backend(
            manager,
            vehicle,
            monkeypatch,
            backend_name="precise_landing",
        )
        backend.queue_decision(
            LandingBackendDecision(
                LandingBackendOutcome.RUNNING,
                "REQUEST_LAND",
                stream_landing_target=True,
                request_land=True,
            )
        )

        publish_dock_target(manager, monkeypatch, received_at=12.0, mode="OUTER")
        vehicle.state = guided_state(12.0, seq=manager.contract.handoff_seq)
        manager.tick(now=12.0)
        assert len(vehicle.landing_target_calls) == 1
        assert vehicle.land_calls == 1

        vehicle.state = guided_state(
            12.1,
            seq=manager.contract.handoff_seq,
            mode=LAND_MODE,
        )
        vehicle.finish_command()
        manager.tick(now=12.1)
        assert backend.land_results == [(True, 12.1)]

        backend.queue_decision(
            LandingBackendDecision(
                LandingBackendOutcome.RUNNING,
                "WAIT_LANDED",
                stream_landing_target=True,
            )
        )
        vehicle.state = guided_state(
            12.2,
            seq=manager.contract.handoff_seq,
            mode=LAND_MODE,
        )
        manager.tick(now=12.2)
        assert len(vehicle.landing_target_calls) == 1

        publish_dock_target(manager, monkeypatch, received_at=12.3, mode="OUTER")
        backend.queue_decision(
            LandingBackendDecision(
                LandingBackendOutcome.RUNNING,
                "WAIT_LANDED",
                stream_landing_target=True,
            )
        )
        vehicle.state = guided_state(
            12.3,
            seq=manager.contract.handoff_seq,
            mode=LAND_MODE,
        )
        manager.tick(now=12.3)

        assert len(vehicle.landing_target_calls) == 2
    finally:
        manager.close()

def test_search_timeout_restarts_validated_nav_land_and_needs_fresh_confirmation():
    """Break caught: fallback handback trusts pre-ACK AUTO progress."""
    manager, vehicle = backend_manager()
    try:
        baseline_start_calls = vehicle.start_mission_calls
        fallback_from_search_timeout(manager, vehicle)
        vehicle.finish_command()
        manager.tick(now=40.1)
        manager.tick(now=40.2)
        assert vehicle.start_mission_calls == baseline_start_calls + 1

        vehicle.state = auto_active_state(40.2, seq=manager.contract.fallback_land_seq)
        vehicle.finish_command()
        manager.tick(now=40.3)
        manager.tick(now=40.4)
        assert manager.state is CycleState.RETURN_TO_FC

        vehicle.state = auto_active_state(40.5, seq=manager.contract.fallback_land_seq)
        manager.tick(now=40.5)

        assert vehicle.set_mission_current_calls[-1] == (
            manager.contract.fallback_land_seq,
            True,
        )
        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 0
    finally:
        manager.close()


@pytest.mark.parametrize("reject_stage", ["set_mission_current", "start_mission"])
def test_failed_handback_commands_direct_land(reject_stage: str):
    """Break caught: a rejected fallback handback does not escalate to direct LAND."""
    manager, vehicle = backend_manager()
    try:
        fallback_from_search_timeout(manager, vehicle)
        if reject_stage == "set_mission_current":
            vehicle.reject_pending("mission current rejected")
            manager.tick(now=40.1)
        else:
            vehicle.finish_command()
            manager.tick(now=40.1)
            manager.tick(now=40.2)
            vehicle.reject_pending("mission start rejected")
            manager.tick(now=40.3)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 1
    finally:
        manager.close()


def test_handback_confirmation_timeout_commands_direct_land_once():
    """Break caught: fallback confirmation can wait forever after mission start ACK."""
    manager, vehicle = backend_manager(mission_start_confirmation_timeout_s=0.2)
    try:
        fallback_from_search_timeout(manager, vehicle)
        vehicle.finish_command()
        manager.tick(now=40.1)
        manager.tick(now=40.2)
        vehicle.finish_command()
        manager.tick(now=40.3)
        manager.tick(now=40.6)
        manager.tick(now=40.7)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert "timed out" in manager.reason
        assert vehicle.land_calls == 1
    finally:
        manager.close()


def test_precommit_backend_failure_returns_control_to_validated_nav_land(
    monkeypatch,
):
    """Break caught: a precommit backend failure skips the stored NAV_LAND fallback."""
    manager, vehicle = backend_manager(backend_name="pid")
    try:
        backend = start_backend(
            manager,
            vehicle,
            monkeypatch,
            backend_name="pid",
        )
        backend.queue_decision(
            LandingBackendDecision(
                LandingBackendOutcome.FAIL_BEFORE_COMMIT,
                "FAILED",
                reason="central target recovery exhausted",
            )
        )

        vehicle.state = guided_state(13.0, seq=manager.contract.handoff_seq)
        manager.tick(now=13.0)

        assert manager.state is CycleState.RETURN_TO_FC
        assert vehicle.set_mission_current_calls[-1] == (
            manager.contract.fallback_land_seq,
            True,
        )
    finally:
        manager.close()


def test_postcommit_backend_failure_commands_direct_land_once(monkeypatch):
    """Break caught: a committed backend failure tries to rejoin the approach route."""
    manager, vehicle = backend_manager(backend_name="precise_landing")
    try:
        backend = start_backend(
            manager,
            vehicle,
            monkeypatch,
            backend_name="precise_landing",
        )
        backend.queue_decision(
            LandingBackendDecision(
                LandingBackendOutcome.FAIL_AFTER_COMMIT,
                "FAILED",
                reason="vehicle left LAND",
            )
        )

        vehicle.state = guided_state(
            13.0,
            seq=manager.contract.handoff_seq,
            mode=LAND_MODE,
        )
        manager.tick(now=13.0)
        manager.tick(now=13.1)

        assert manager.state is CycleState.FC_FALLBACK_LAND
        assert vehicle.land_calls == 1
    finally:
        manager.close()

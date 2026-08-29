"""Integration boundary tests for MissionManager's direct vehicle transport."""

from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_msgs.msg import String

import drone_mission_manager.mission_manager as mission_manager_module
from drone_mission_manager.mission_manager import (
    GUIDED_MODE,
    MissionManager,
    MissionState,
    resolve_landing_mode,
    selected_update_rate_hz,
)
from drone_mission_manager.pid_landing import (
    DockTargetMode,
    PidLandingDecision,
    PidLandingOutcome,
    PidLandingState,
    VelocityCommand,
)
from drone_mission_manager.pymavlink_vehicle import (
    VehicleCommandResult,
    VehicleState,
    VelocityFrame,
)


MISSION_FILE = (
    Path(__file__).parents[1] / "config" / "missions" / "simple_circle.yaml"
)


class FakeVehicle:
    """Deterministic command-capable direct-vehicle boundary."""

    def __init__(self):
        self.state = VehicleState()
        self.closed = False
        self.poll_calls = []
        self.velocity_calls = []
        self.arm_calls = []
        self.mode_calls = []
        self.parameter_calls = []
        self.landing_target_calls = []
        self.condition_yaw_calls = []
        self.arm_futures = []

    def poll(self, now):
        self.poll_calls.append(now)

    def ready(self, now, heartbeat_timeout_s=2.5):
        return True

    def send_velocity(
        self, frame, velocity_x, velocity_y, velocity_z, *, now,
        yaw_rate_rad_s=None,
    ):
        self.velocity_calls.append(
            (frame, velocity_x, velocity_y, velocity_z, yaw_rate_rad_s, now)
        )
        return True

    def arm_async(self, arm, now):
        self.arm_calls.append((arm, now))
        if self.arm_futures:
            return self.arm_futures.pop(0)
        return FakeFuture(VehicleCommandResult(True))

    def set_mode_async(self, mode, now):
        self.mode_calls.append((mode, now))
        return FakeFuture(VehicleCommandResult(True, current_mode=mode))

    def get_parameters_async(self, names, now):
        self.parameter_calls.append((tuple(names), now))
        return FakeFuture(
            VehicleCommandResult(
                True, parameters={"PLND_ENABLED": 1.0, "PLND_TYPE": 1.0}
            )
        )

    def send_landing_target(self, **values):
        self.landing_target_calls.append(values)
        return True

    def send_condition_yaw(self, **values):
        self.condition_yaw_calls.append(values)
        return True

    def close(self):
        self.closed = True


class FakeFuture:
    def __init__(self, result=None, exception=None, done=True):
        self._result = result
        self._exception = exception
        self._done = done

    def done(self):
        return self._done

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result


class StubPidLanding:
    def __init__(self, decisions):
        self.decisions = list(decisions)
        self.starts = []
        self.updates = []
        self.disarm_results = []

    def start(self, now, dock_x_m, dock_y_m, hover_altitude_m):
        self.starts.append((now, dock_x_m, dock_y_m, hover_altitude_m))

    def update(self, now, dock, vehicle):
        self.updates.append((now, dock, vehicle))
        return self.decisions.pop(0)

    def notify_disarm_result(self, request_id, success, now):
        self.disarm_results.append((request_id, success, now))


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


@pytest.fixture
def manager_factory():
    resources = []

    def create(mode="pid", **parameter_values):
        context = Context()
        rclpy.init(context=context)
        vehicle = FakeVehicle()
        overrides = [
            Parameter("mission_file", value=str(MISSION_FILE)),
            Parameter("landing_mode", value=mode),
        ]
        overrides.extend(
            Parameter(name, value=value)
            for name, value in parameter_values.items()
        )
        node = MissionManager(
            vehicle_factory=lambda _node: vehicle,
            parameter_overrides=overrides,
            context=context,
        )
        resources.append((node, context))
        return node, vehicle

    yield create

    for node, context in reversed(resources):
        node.destroy_node()
        rclpy.shutdown(context=context)


def fresh_vehicle_state(
    now, *, position=(0.0, 0.0, 5.0), velocity=(0.0, 0.0, 0.0),
    armed=True, flying=True, mode=GUIDED_MODE, landed_at=None,
):
    return VehicleState(
        connected=True,
        armed=armed,
        flying=flying,
        mode=mode,
        system_status=4,
        position_enu_m=position,
        velocity_enu_m_s=velocity,
        last_message_at=now,
        heartbeat_at=now,
        position_at=now,
        landed_at=now if landed_at is None else landed_at,
    )


def dock_pose(now, *, x=0.2, y=-0.3, z=2.0):
    message = PoseStamped()
    message.header.stamp.sec = int(now)
    message.header.stamp.nanosec = int((now - int(now)) * 1_000_000_000)
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z
    message.pose.orientation.w = 1.0
    return message


def running_decision(
    state=PidLandingState.PID_ALIGN,
    command=VelocityCommand(
        VelocityFrame.BODY_FLU, 0.4, -0.2, -0.1, 0.3
    ),
):
    return PidLandingDecision(state, command)


def disarm_decision(request_id):
    return PidLandingDecision(
        PidLandingState.DISARM,
        VelocityCommand(VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0, 0.0),
        request_disarm=True,
        disarm_request_id=request_id,
    )


def terminal_decision(outcome, reason=""):
    state = (
        PidLandingState.COMPLETE
        if outcome is PidLandingOutcome.COMPLETE
        else PidLandingState.FAILED
    )
    return PidLandingDecision(
        state,
        VelocityCommand(VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0, 0.0),
        outcome=outcome,
        reason=reason,
    )


@pytest.mark.parametrize(
    ("canonical", "legacy", "expected", "warns"),
    [
        ("pid", "", "pid", False),
        ("precise_landing", "", "precise_landing", False),
        ("", "ros", "pid", True),
        ("", "ardupilot", "precise_landing", True),
        ("", "", "pid", False),
        ("PID", "", "pid", False),
        ("pid", "ros", "pid", True),
        ("precise_landing", "ardupilot", "precise_landing", True),
    ],
)
def test_resolve_landing_mode_accepts_canonical_and_legacy_values(
    canonical, legacy, expected, warns
):
    """Break caught: startup selects the wrong landing implementation."""
    mode, warning = resolve_landing_mode(canonical, legacy)

    assert mode == expected
    assert (warning is not None) is warns


@pytest.mark.parametrize(
    ("canonical", "legacy"),
    [
        ("unknown", ""),
        ("", "unknown"),
        ("pid", "ardupilot"),
        ("precise_landing", "ros"),
    ],
)
def test_resolve_landing_mode_rejects_invalid_or_conflicting_values(
    canonical, legacy
):
    """Break caught: invalid or conflicting configuration starts a mission."""
    with pytest.raises(ValueError):
        resolve_landing_mode(canonical, legacy)


@pytest.mark.parametrize(
    ("mode", "expected"),
    [("pid", 20.0), ("precise_landing", 10.0)],
)
def test_selected_update_rate_uses_pid_control_rate_only_for_pid(
    mode, expected
):
    """Break caught: PID control runs at the slower native update rate."""
    assert selected_update_rate_hz(mode, 20.0, 10.0) == expected


@pytest.mark.parametrize("pid_rate", [0.0, -1.0, float("nan")])
def test_selected_update_rate_rejects_invalid_selected_rate(pid_rate):
    """Break caught: an invalid selected timer rate reaches create_timer."""
    with pytest.raises(ValueError, match="greater than zero"):
        selected_update_rate_hz("pid", pid_rate, 10.0)


def test_manager_creates_no_ap_dds_vehicle_entities():
    """Break caught: an AP_DDS entity remains in the direct manager."""
    context = Context()
    rclpy.init(context=context)
    vehicle = FakeVehicle()
    node = None
    try:
        node = MissionManager(
            vehicle_factory=lambda _node: vehicle,
            parameter_overrides=[
                Parameter("mission_file", value=str(MISSION_FILE)),
                Parameter("landing_backend", value="ros"),
            ],
            context=context,
        )

        topics = {name for name, _types in node.get_topic_names_and_types()}
        services = {
            name for name, _types in node.get_service_names_and_types()
        }
        assert not {
            "/ap/v1/pose/filtered",
            "/ap/v1/status",
            "/ap/v1/cmd_vel",
        } & topics
        assert not {
            "/ap/v1/prearm_check",
            "/ap/v1/mode_switch",
            "/ap/v1/arm_motors",
            "/ap/v1/experimental/takeoff",
            "/ap/v1/get_parameters",
        } & services
        assert {"/dock/pose", "/dock/mode", "/mission_manager/state"} <= topics
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown(context=context)

    assert vehicle.closed is True


def test_pid_handoff_starts_controller_at_resolved_dock_position(
    manager_factory, monkeypatch
):
    """Break caught: return completion enters the old ROS landing states."""
    now = 100.0
    monkeypatch.setattr(mission_manager_module.time, "monotonic", lambda: now)
    node, vehicle = manager_factory("pid")
    controller = StubPidLanding([])
    node._pid_landing = controller
    node._mission_origin = (0.0, 0.0)
    node._state = MissionState.RETURN_TO_DOCK
    vehicle.state = fresh_vehicle_state(now)

    node._update()

    assert node._state is MissionState.PID_LANDING
    assert controller.starts == [(100.0, 0.0, 0.0, 5.0)]


def test_pid_update_converts_dock_observation_once_and_routes_command(
    manager_factory, monkeypatch
):
    """Break caught: PID output or camera axes bypass the MAVLink boundary."""
    now = 120.25
    monkeypatch.setattr(mission_manager_module.time, "monotonic", lambda: now)
    node, vehicle = manager_factory("pid")
    controller = StubPidLanding([running_decision()])
    node._pid_landing = controller
    node._state = MissionState.PID_LANDING
    node._state_publisher = RecordingPublisher()
    vehicle.state = fresh_vehicle_state(
        now, position=(1.0, 2.0, 3.0), velocity=(0.1, 0.2, 0.3)
    )
    node._dock_pose_callback(dock_pose(now))
    node._dock_mode_callback(String(data="OUTER"))

    node._update()

    observation = controller.updates[0][1]
    assert observation.forward_m == pytest.approx(0.3)
    assert observation.left_m == pytest.approx(-0.2)
    assert observation.distance_m == pytest.approx(2.0)
    assert observation.yaw_error_rad == pytest.approx(1.5707963267948966)
    assert observation.mode is DockTargetMode.OUTER
    assert observation.source_stamp_s == pytest.approx(120.25)
    assert observation.received_at == pytest.approx(120.25)
    assert vehicle.velocity_calls[-1][:5] == (
        VelocityFrame.BODY_FLU, 0.4, -0.2, -0.1, 0.3
    )
    assert node._state_publisher.messages == ["PID_PID_ALIGN"]


def test_stale_pose_or_mode_is_not_passed_to_pid(
    manager_factory, monkeypatch
):
    """Break caught: half-fresh dock data feeds a stale PID input."""
    now = 140.0
    monkeypatch.setattr(mission_manager_module.time, "monotonic", lambda: now)
    node, vehicle = manager_factory("pid", dock_pose_timeout_s=0.5)
    controller = StubPidLanding([running_decision(), running_decision()])
    node._pid_landing = controller
    node._state = MissionState.PID_LANDING
    vehicle.state = fresh_vehicle_state(now)

    node._dock_pose_callback(dock_pose(now))
    node._dock_mode = "OUTER"
    node._dock_mode_received_at = now - 1.0
    node._update()

    node._dock_pose_received_at = now - 1.0
    node._dock_mode_received_at = now
    node._update()

    assert [call[1] for call in controller.updates] == [None, None]


def test_precise_landing_keeps_native_acquire_and_land_handoff(
    manager_factory, monkeypatch
):
    """Break caught: canonical migration accidentally replaces native LAND."""
    now = 160.0
    monkeypatch.setattr(mission_manager_module.time, "monotonic", lambda: now)
    node, vehicle = manager_factory("precise_landing")
    node._mission_origin = (0.0, 0.0)
    node._state = MissionState.RETURN_TO_DOCK
    vehicle.state = fresh_vehicle_state(now)

    node._update()
    assert node._state is MissionState.ACQUIRE_DOCK

    node._dock_pose_callback(dock_pose(now))
    node._dock_mode_callback(String(data="CENTRAL"))
    node._landing_target_started_at = now - 2.0
    node._update()

    assert node._state is MissionState.SET_LAND
    assert vehicle.landing_target_calls


def test_pid_config_reads_recovery_tolerance_and_axis_tuning(
    manager_factory,
):
    """Break caught: a controller field is hard-coded or left undeclared."""
    node, _vehicle = manager_factory(
        "pid",
        pid_recovery_climb_m=0.4,
        pid_recovery_altitude_tolerance_m=0.06,
        pid_x_kp=1.25,
        pid_max_horizontal_speed_m_s=0.45,
    )

    config = node._build_pid_landing_config()

    assert config.recovery_climb_m == pytest.approx(0.4)
    assert config.recovery_altitude_tolerance_m == pytest.approx(0.06)
    assert config.x_pid.kp == pytest.approx(1.25)
    assert config.x_pid.output_limit == pytest.approx(0.45)


def test_pending_pid_disarm_does_not_freeze_controller_updates(
    manager_factory, monkeypatch
):
    """Break caught: a future freezes PID bounce and freshness logic."""
    now = 180.0
    monkeypatch.setattr(mission_manager_module.time, "monotonic", lambda: now)
    node, vehicle = manager_factory("pid")
    pending = FakeFuture(done=False)
    vehicle.arm_futures.append(pending)
    controller = StubPidLanding([disarm_decision(17), running_decision()])
    node._pid_landing = controller
    node._state = MissionState.PID_LANDING
    vehicle.state = fresh_vehicle_state(now, flying=False)

    node._update()
    node._update()

    assert vehicle.arm_calls == [(False, 180.0)]
    assert len(controller.updates) == 2
    assert node._pending_name == "pid_disarm"
    assert node._pending_pid_disarm_request_id == 17


@pytest.mark.parametrize(
    ("future", "expected_success"),
    [
        (FakeFuture(VehicleCommandResult(True)), True),
        (FakeFuture(VehicleCommandResult(False, "rejected")), False),
        (FakeFuture(exception=RuntimeError("link failed")), False),
    ],
)
def test_pid_disarm_result_is_correlated_without_failing_mission(
    manager_factory, monkeypatch, future, expected_success
):
    """Break caught: disarm ACK loses its ID or fails the mission."""
    now = [200.0]
    monkeypatch.setattr(
        mission_manager_module.time, "monotonic", lambda: now[0]
    )
    node, vehicle = manager_factory("pid")
    future._done = False
    vehicle.arm_futures.append(future)
    controller = StubPidLanding([disarm_decision(23), running_decision()])
    node._pid_landing = controller
    node._state = MissionState.PID_LANDING
    vehicle.state = fresh_vehicle_state(now[0], flying=False)

    node._update()
    future._done = True
    now[0] = 200.1
    vehicle.state = fresh_vehicle_state(now[0], flying=False)
    node._update()

    assert controller.disarm_results == [(23, expected_success, 200.1)]
    assert len(controller.updates) == 2
    assert node._state is MissionState.PID_LANDING


def test_pid_disarm_timeout_notifies_rejection_and_continues(
    manager_factory, monkeypatch
):
    """Break caught: a transport timeout immediately fails PID landing."""
    now = [220.0]
    monkeypatch.setattr(
        mission_manager_module.time, "monotonic", lambda: now[0]
    )
    node, vehicle = manager_factory("pid", mavlink_operation_timeout_s=0.05)
    vehicle.arm_futures.append(FakeFuture(done=False))
    controller = StubPidLanding([disarm_decision(31), running_decision()])
    node._pid_landing = controller
    node._state = MissionState.PID_LANDING
    vehicle.state = fresh_vehicle_state(now[0], flying=False)

    node._update()
    now[0] = 220.1
    vehicle.state = fresh_vehicle_state(now[0], flying=False)
    node._update()

    assert controller.disarm_results == [(31, False, 220.1)]
    assert len(controller.updates) == 2
    assert node._state is MissionState.PID_LANDING


@pytest.mark.parametrize(
    ("decision", "armed", "expected_state"),
    [
        (
            terminal_decision(PidLandingOutcome.COMPLETE),
            False,
            MissionState.COMPLETE,
        ),
        (
            terminal_decision(PidLandingOutcome.FAILED, "search timed out"),
            True,
            MissionState.FAILED,
        ),
    ],
)
def test_pid_terminal_outcome_uses_existing_mission_completion_or_failure(
    manager_factory, monkeypatch, decision, armed, expected_state
):
    """Break caught: PID terminal status does not reach mission lifecycle."""
    now = 240.0
    monkeypatch.setattr(mission_manager_module.time, "monotonic", lambda: now)
    node, vehicle = manager_factory("pid")
    controller = StubPidLanding([decision])
    node._pid_landing = controller
    node._state = MissionState.PID_LANDING
    node._has_seen_armed = True
    vehicle.state = fresh_vehicle_state(now, armed=armed, flying=False)

    node._update()

    assert node._state is expected_state
    if expected_state is MissionState.FAILED:
        assert vehicle.velocity_calls[-1][1:4] == (0.0, 0.0, 0.0)


def test_pid_landing_fails_if_vehicle_leaves_guided(
    manager_factory, monkeypatch
):
    """Break caught: PID commands continue after an unexpected mode change."""
    now = 260.0
    monkeypatch.setattr(mission_manager_module.time, "monotonic", lambda: now)
    node, vehicle = manager_factory("pid")
    controller = StubPidLanding([running_decision()])
    node._pid_landing = controller
    node._state = MissionState.PID_LANDING
    vehicle.state = fresh_vehicle_state(now, mode=9)

    node._update()

    assert node._state is MissionState.FAILED
    assert controller.updates == []
    assert vehicle.velocity_calls[-1][1:4] == (0.0, 0.0, 0.0)

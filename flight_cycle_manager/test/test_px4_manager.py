"""Exercise native PX4 messages with real Humble nodes and deterministic time."""

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import subprocess

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleCommandAck, VehicleLandDetected,
    VehicleLocalPosition, VehicleStatus,
)
from rclpy.serialization import deserialize_message, serialize_message
from std_srvs.srv import Trigger

from flight_cycle_manager.landing_backend import VelocityCommand, VelocityFrame
from flight_cycle_manager.px4_control import dds_topic, velocity_to_ned
from flight_cycle_manager.px4_manager import Px4FlightCycleManager


ROOT = Path(__file__).parents[2]
DDS_MESSAGES = (
    ("in", "offboard_control_mode", OffboardControlMode, None),
    ("in", "trajectory_setpoint", TrajectorySetpoint, 0),
    ("in", "vehicle_command", VehicleCommand, 0),
    ("out", "vehicle_command_ack", VehicleCommandAck, 1),
    ("out", "vehicle_land_detected", VehicleLandDetected, 0),
    ("out", "vehicle_local_position", VehicleLocalPosition, 1),
    ("out", "vehicle_status", VehicleStatus, 4),
)


@pytest.fixture
def manager(monkeypatch, request):
    monkeypatch.setattr('flight_cycle_manager.px4_manager.get_package_share_directory',
                        lambda _: str(ROOT / 'flight_cycle_manager'))
    context = Context()
    rclpy.init(context=context)
    clock = [100.0]
    node = Px4FlightCycleManager(
        context=context, now=lambda: clock[0],
        namespace=getattr(request, "param", ""),
        parameter_overrides=[Parameter("use_sim_time", value=False)],
    )
    node._timer.cancel()
    node.test_clock = clock
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


@pytest.fixture
def control_messages(manager, monkeypatch):
    captured = {name: [] for name in manager._dds_publishers}
    for name, publisher in manager._dds_publishers.items():
        monkeypatch.setattr(publisher, "publish", captured[name].append)
    return captured


def telemetry(node, *, armed=False, landed=True, offboard=False, z=0.0, nav_state=None):
    if nav_state is None:
        nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if offboard
                     else VehicleStatus.NAVIGATION_STATE_POSCTL)
    node._on_status(VehicleStatus(
        arming_state=(VehicleStatus.ARMING_STATE_ARMED if armed else VehicleStatus.ARMING_STATE_DISARMED),
        nav_state=nav_state,
        pre_flight_checks_pass=True, system_id=1, component_id=1,
    ))
    node._on_local(VehicleLocalPosition(
        xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
        heading_good_for_control=True, z=float(z), heading=0.0,
    ))
    node._on_land(VehicleLandDetected(landed=landed))


def ack(node, result=VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED, **overrides):
    values = dict(command=node._pending.command, result=result,
                  target_system=254, target_component=191)
    values.update(overrides)
    node._on_ack(VehicleCommandAck(**values))


def start_to_hover(node):
    telemetry(node)
    assert node._start(None, Trigger.Response()).success
    node.test_clock[0] += 1.6
    telemetry(node)
    node._tick()
    assert node._state == "SET_OFFBOARD"
    assert node._pending.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    ack(node)
    telemetry(node, offboard=True)
    node._tick()
    assert node._state == "ARM"
    ack(node)
    telemetry(node, armed=True, offboard=True)
    node._tick()
    assert node._state == "TAKEOFF"
    assert node._target[2] == -5.0
    telemetry(node, armed=True, landed=False, offboard=True, z=-5)
    node._tick()
    node.test_clock[0] += 1.1
    telemetry(node, armed=True, landed=False, offboard=True, z=-5)
    node._tick()
    assert node._state == "HOVER"


def test_cycle_uses_acks_and_status_before_climbing(manager):
    start_to_hover(manager)
    assert manager._request_dock(None, Trigger.Response()).success
    assert manager._state == "SEARCH_DOCK"
    manager._tick()
    assert all(math.isfinite(value) for value in manager._velocity)


def test_start_requires_fresh_disarmed_landed_preflight(manager):
    assert not manager._start(None, Trigger.Response()).success
    telemetry(manager, armed=True)
    assert not manager._start(None, Trigger.Response()).success
    telemetry(manager, landed=False)
    assert not manager._start(None, Trigger.Response()).success
    telemetry(manager)
    manager._status.pre_flight_checks_pass = False
    assert not manager._start(None, Trigger.Response()).success
    telemetry(manager)
    manager._local.xy_valid = False
    assert not manager._start(None, Trigger.Response()).success


def test_ground_heading_waits_for_px4_inflight_alignment(manager):
    telemetry(manager)
    manager._local.heading_good_for_control = False
    assert manager._start(None, Trigger.Response()).success


def test_takeoff_tracks_expected_px4_heading_reset(manager):
    start_to_hover(manager)
    manager._state = "TAKEOFF"
    manager._stable_since = None
    previous = manager._yaw
    manager._local.heading_reset_counter += 1
    manager._local.delta_heading = 0.5
    manager._tick()
    assert manager._state == "TAKEOFF"
    assert manager._yaw == pytest.approx(previous + 0.5)


def test_ack_is_filtered_and_does_not_replace_mode_confirmation(manager):
    telemetry(manager)
    manager._start(None, Trigger.Response())
    manager.test_clock[0] += 1.6
    telemetry(manager)
    manager._tick()
    ack(manager, target_component=99)
    assert manager._pending is not None
    ack(manager, command=400)
    assert manager._pending is not None
    ack(manager)
    manager._tick()
    assert manager._state == "SET_OFFBOARD"


def test_denied_arm_never_starts_takeoff(manager):
    telemetry(manager)
    manager._start(None, Trigger.Response())
    manager.test_clock[0] += 1.6
    telemetry(manager)
    manager._tick()
    ack(manager)
    telemetry(manager, offboard=True)
    manager._tick()
    ack(manager, result=VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED)
    assert manager._state == "FAILED"
    assert manager._target[2] == 0.0


def test_pilot_override_latches_and_stops_stream(manager):
    start_to_hover(manager)
    telemetry(manager, armed=True, landed=False, offboard=False)
    manager._tick()
    assert manager._state == "PILOT_OVERRIDE"
    telemetry(manager, armed=True, landed=False, offboard=True)
    manager._tick()
    assert manager._state == "PILOT_OVERRIDE"
    assert not manager._start(None, Trigger.Response()).success


@pytest.mark.parametrize("nav_state", [
    VehicleStatus.NAVIGATION_STATE_POSCTL,
    VehicleStatus.NAVIGATION_STATE_ALTCTL,
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
])
def test_dock_trigger_acquires_offboard_before_search(manager, control_messages, nav_state):
    telemetry(manager, armed=True, landed=False, z=-3, nav_state=nav_state)
    assert manager._request_dock(None, Trigger.Response()).success
    assert manager._state == "WARMUP"
    assert manager._target == [0.0, 0.0, -3.0]
    assert manager._velocity is None
    assert manager._search is None
    manager._tick()
    assert control_messages["offboard_control_mode"][-1].position
    assert not control_messages["vehicle_command"]

    manager.test_clock[0] += 1.6
    telemetry(manager, armed=True, landed=False, z=-3, nav_state=nav_state)
    manager._tick()
    assert manager._state == "SET_OFFBOARD"
    command, = control_messages["vehicle_command"]
    assert command.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    assert (command.param1, command.param2) == (1.0, 6.0)
    ack(manager)
    manager._tick()
    assert manager._state == "SET_OFFBOARD"
    assert manager._search is None

    telemetry(manager, armed=True, landed=False, offboard=True, z=-3)
    manager._tick()
    assert manager._state == "SEARCH_DOCK"
    assert manager._search is not None
    assert manager._velocity == (0.0, 0.0, 0.0, 0.0)
    assert len(control_messages["vehicle_command"]) == 1


def test_dock_entry_requires_ack_even_when_offboard_status_arrives_first(manager):
    telemetry(manager, armed=True, landed=False, z=-3)
    assert manager._request_dock(None, Trigger.Response()).success
    manager.test_clock[0] += 1.6
    telemetry(manager, armed=True, landed=False, z=-3)
    manager._tick()
    telemetry(manager, armed=True, landed=False, offboard=True, z=-3)
    manager._tick()
    assert manager._state == "SET_OFFBOARD"
    assert manager._search is None
    ack(manager)
    manager._tick()
    assert manager._state == "SEARCH_DOCK"


def test_dock_trigger_in_offboard_skips_mode_request(manager, control_messages):
    telemetry(manager, armed=True, landed=False, offboard=True, z=-3)
    assert manager._request_dock(None, Trigger.Response()).success
    assert manager._state == "SEARCH_DOCK"
    assert not control_messages["vehicle_command"]


@pytest.mark.parametrize("manager", ["/test_dock_entry", "/drone1/test_dock_entry"], indirect=True)
def test_real_dock_trigger_service_queues_offboard_entry(manager):
    telemetry(manager, armed=True, landed=False, z=-3)
    executor = SingleThreadedExecutor(context=manager.context)
    executor.add_node(manager)
    client = manager.create_client(Trigger, "~/request_dock")
    try:
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=3.0)
        assert future.done()
        response = future.result()
        assert response.success, response.message
        assert manager._state == "WARMUP"
    finally:
        manager.destroy_client(client)
        executor.shutdown()


def test_start_already_in_offboard_does_not_send_redundant_mode_command(manager, control_messages):
    telemetry(manager, offboard=True)
    assert manager._start(None, Trigger.Response()).success
    manager.test_clock[0] += 1.6
    telemetry(manager, offboard=True)
    manager._tick()
    assert manager._state == "ARM"
    command, = control_messages["vehicle_command"]
    assert command.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM


@pytest.mark.parametrize("invalid", ["stale", "disarmed", "landed", "heading", "failsafe", "busy"])
def test_dock_trigger_preserves_safety_gates(manager, control_messages, invalid):
    telemetry(manager, armed=True, landed=False, z=-3)
    if invalid == "stale":
        manager.test_clock[0] += 3.0
    elif invalid == "disarmed":
        manager._status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    elif invalid == "landed":
        manager._land.landed = True
    elif invalid == "heading":
        manager._local.heading_good_for_control = False
    elif invalid == "failsafe":
        manager._status.failsafe = True
    else:
        manager._state = "TAKEOFF"
    assert not manager._request_dock(None, Trigger.Response()).success
    assert manager._pending is None
    assert manager._search is None
    assert not control_messages["vehicle_command"]


@pytest.mark.parametrize("stage", ["WARMUP", "SET_OFFBOARD", "OFFBOARD_BEFORE_ACK", "ARM",
                                   "TAKEOFF", "HOVER", "SEARCH_DOCK", "LANDING_BACKEND"])
def test_operator_mode_switch_stops_all_control_and_ignores_late_ack(
        manager, control_messages, stage):
    telemetry(manager)
    assert manager._start(None, Trigger.Response()).success
    if stage != "WARMUP":
        manager.test_clock[0] += 1.6
        telemetry(manager)
        manager._tick()
    if stage == "OFFBOARD_BEFORE_ACK":
        telemetry(manager, offboard=True)
    elif stage not in {"WARMUP", "SET_OFFBOARD"}:
        ack(manager)
        telemetry(manager, offboard=True)
        manager._tick()
        if stage != "ARM":
            ack(manager)
            telemetry(manager, armed=True, offboard=True)
            manager._tick()
            if stage != "TAKEOFF":
                manager._state = stage
    pending = manager._pending
    before = {name: len(messages) for name, messages in control_messages.items()}
    manager._on_status(VehicleStatus(
        arming_state=manager._status.arming_state,
        nav_state=VehicleStatus.NAVIGATION_STATE_ALTCTL,
    ))
    assert manager._state == "PILOT_OVERRIDE"
    assert manager._pending is None
    if pending is not None:
        manager._on_ack(VehicleCommandAck(
            command=pending.command, result=VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
            target_system=254, target_component=191,
        ))
    # A quick switch back must not hide the override or resume the cycle.
    telemetry(manager, armed=True, landed=False, offboard=True, z=-3)
    manager._tick()
    assert manager._state == "PILOT_OVERRIDE"
    assert {name: len(messages) for name, messages in control_messages.items()} == before


def test_dock_requires_new_trigger_after_pilot_override(manager, control_messages):
    start_to_hover(manager)
    telemetry(manager, armed=True, landed=False, z=-5)
    manager._tick()
    assert manager._state == "PILOT_OVERRIDE"
    before = len(control_messages["vehicle_command"])
    manager._tick()
    assert len(control_messages["vehicle_command"]) == before
    assert manager._request_dock(None, Trigger.Response()).success
    assert manager._state == "WARMUP"


@pytest.mark.parametrize("result", [None, VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED])
def test_dock_offboard_entry_failure_never_starts_search(manager, control_messages, result):
    telemetry(manager, armed=True, landed=False, z=-3)
    assert manager._request_dock(None, Trigger.Response()).success
    manager.test_clock[0] += 1.6
    telemetry(manager, armed=True, landed=False, z=-3)
    manager._tick()
    if result is None:
        manager.test_clock[0] += 11.0
        telemetry(manager, armed=True, landed=False, z=-3)
        manager._tick()
    else:
        ack(manager, result=result)
    assert manager._state == "FC_FALLBACK_LAND"
    assert manager._search is None
    assert manager._pending.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
    assert [message.command for message in control_messages["vehicle_command"]] == [
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, VehicleCommand.VEHICLE_CMD_NAV_LAND,
    ]


def test_dock_mode_confirmation_timeout_does_not_retry_offboard(manager, control_messages):
    telemetry(manager, armed=True, landed=False, z=-3)
    assert manager._request_dock(None, Trigger.Response()).success
    manager.test_clock[0] += 1.6
    telemetry(manager, armed=True, landed=False, z=-3)
    manager._tick()
    ack(manager)
    manager.test_clock[0] += 11.0
    telemetry(manager, armed=True, landed=False, z=-3)
    manager._tick()
    assert manager._state == "FC_FALLBACK_LAND"
    assert manager._search is None
    assert [message.command for message in control_messages["vehicle_command"]] == [
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, VehicleCommand.VEHICLE_CMD_NAV_LAND,
    ]


@pytest.mark.parametrize("invalid", ["disarmed", "landed", "heading"])
def test_dock_entry_rechecks_airborne_prerequisites_before_search(manager, invalid):
    telemetry(manager, armed=True, landed=False, z=-3)
    assert manager._request_dock(None, Trigger.Response()).success
    manager.test_clock[0] += 1.6
    telemetry(manager, armed=True, landed=False, z=-3)
    manager._tick()
    ack(manager)
    telemetry(manager, armed=invalid != "disarmed", landed=invalid == "landed",
              offboard=True, z=-3)
    manager._local.heading_good_for_control = invalid != "heading"
    manager._tick()
    assert manager._state in {"FAILED", "FC_FALLBACK_LAND"}
    assert manager._search is None


def test_failsafe_during_entry_releases_authority(manager, control_messages):
    telemetry(manager)
    assert manager._start(None, Trigger.Response()).success
    manager._status.failsafe = True
    manager._tick()
    assert manager._state == "PILOT_OVERRIDE"
    assert all(not messages for messages in control_messages.values())


def test_operator_takeover_during_fallback_ignores_pending_land_ack(manager, control_messages):
    start_to_hover(manager)
    manager._fallback("test")
    telemetry(manager, armed=True, landed=False,
              nav_state=VehicleStatus.NAVIGATION_STATE_AUTO_LAND, z=-3)
    assert manager._state == "FC_FALLBACK_LAND"
    before = {name: len(messages) for name, messages in control_messages.items()}
    telemetry(manager, armed=True, landed=False, z=-3)
    assert manager._state == "PILOT_OVERRIDE"
    assert manager._pending is None
    manager._on_ack(VehicleCommandAck(
        command=VehicleCommand.VEHICLE_CMD_NAV_LAND,
        result=VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
        target_system=254, target_component=191,
    ))
    manager._tick()
    assert manager._state == "PILOT_OVERRIDE"
    assert {name: len(messages) for name, messages in control_messages.items()} == before


def test_telemetry_loss_stops_even_with_sim_clock_paused(manager):
    start_to_hover(manager)
    manager.test_clock[0] += 3.0
    manager._tick()
    assert manager._state == "LINK_LOSS"
    assert manager._state not in manager.STREAM_STATES


def test_estimator_reset_requests_native_land(manager):
    start_to_hover(manager)
    manager._local.z_reset_counter += 1
    manager._tick()
    assert manager._state == "FC_FALLBACK_LAND"
    assert manager._pending.command == VehicleCommand.VEHICLE_CMD_NAV_LAND


def test_search_timeout_lands_instead_of_reentering_offboard(manager):
    start_to_hover(manager)
    manager._request_dock(None, Trigger.Response())
    manager.test_clock[0] += 31.0
    telemetry(manager, armed=True, landed=False, offboard=True, z=-5)
    manager._tick()
    assert manager._state == "FC_FALLBACK_LAND"
    ack(manager)
    manager._status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
    manager._tick()
    assert manager._state == "FC_FALLBACK_LAND"


def test_landed_and_auto_disarmed_is_complete_even_if_px4_left_offboard(manager):
    start_to_hover(manager)
    telemetry(manager)
    manager._tick()
    assert manager._state == "DOCKED"


def test_dock_rejects_stale_acquisition_stamp_and_invalid_quaternion(manager):
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    pose = PoseStamped()
    pose.pose.position.z = 2.0
    pose.pose.orientation.w = 1.0
    pose.header.stamp = manager.get_clock().now().to_msg()
    manager._on_dock_pose(pose)
    manager._on_dock_mode(String(data="OUTER"))
    assert manager._dock(manager._now()) is not None
    pose.header.stamp.sec -= 5
    assert manager._dock(manager._now()) is None
    pose.header.stamp = manager.get_clock().now().to_msg()
    pose.pose.orientation.w = 0.0
    assert manager._dock(manager._now()) is None


def test_search_acquisition_runs_shared_pid_without_mavlink(manager):
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    start_to_hover(manager)
    manager._request_dock(None, Trigger.Response())
    pose = PoseStamped()
    pose.header.stamp = manager.get_clock().now().to_msg()
    pose.pose.position.z = 5.0
    # Aligned dock +X projects along camera -Y.
    pose.pose.orientation.z = -math.sqrt(0.5)
    pose.pose.orientation.w = math.sqrt(0.5)
    manager._on_dock_pose(pose)
    manager._on_dock_mode(String(data="OUTER"))
    manager._tick()
    manager.test_clock[0] += 0.6
    telemetry(manager, armed=True, landed=False, offboard=True, z=-5)
    manager._on_dock_pose(pose)
    manager._on_dock_mode(String(data="OUTER"))
    manager._tick()
    assert manager._state == "LANDING_BACKEND"
    manager._tick()
    assert all(math.isfinite(value) for value in manager._velocity)


@pytest.mark.parametrize("frame,heading,xyz,expected", [
    (VelocityFrame.MAP_ENU, 0.0, (1, 2, 3), (2, 1, -3, -0.2)),
    (VelocityFrame.BODY_FLU, 0.0, (1, 2, 3), (1, -2, -3, -0.2)),
    (VelocityFrame.BODY_FLU, math.pi / 2, (1, 2, 3), (2, 1, -3, -0.2)),
])
def test_frames_and_yaw_sign(frame, heading, xyz, expected):
    assert velocity_to_ned(VelocityCommand(frame, *xyz, 0.2), heading) == pytest.approx(expected)


@pytest.mark.parametrize("namespace", ["", "/px4_1/"])
@pytest.mark.parametrize("direction,name,message_type,version", DDS_MESSAGES)
def test_topic_versions_follow_rc1(namespace, direction, name, message_type, version):
    prefix = "/px4_1" if namespace else ""
    suffix = f"_v{version}" if version else ""
    assert dds_topic(namespace, direction, name, message_type) == (
        f"{prefix}/fmu/{direction}/{name}{suffix}")


@pytest.mark.parametrize("direction,name,message_type,version", DDS_MESSAGES)
def test_generated_rc1_interfaces_serialize(direction, name, message_type, version):
    assert getattr(message_type, "MESSAGE_VERSION", None) == version
    message = message_type(timestamp=123_456_789)
    assert deserialize_message(serialize_message(message), message_type) == message


def test_manager_uses_rc1_dds_endpoints(manager):
    assert {publisher.topic_name for publisher in manager._dds_publishers.values()} == {
        "/fmu/in/offboard_control_mode",
        "/fmu/in/trajectory_setpoint",
        "/fmu/in/vehicle_command",
    }
    assert {subscription.topic_name for subscription in manager._dds_subscriptions[:4]} == {
        "/fmu/out/vehicle_status_v4",
        "/fmu/out/vehicle_local_position_v1",
        "/fmu/out/vehicle_land_detected",
        "/fmu/out/vehicle_command_ack_v1",
    }


def test_rc1_status_layout_serializes_new_fields():
    message = VehicleStatus(
        timestamp=123_456_789,
        nav_state_display=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
        accepts_offboard_setpoints=True,
        traffic_avoidance_system_present=True,
    )
    assert "failure_detector_status" not in message.get_fields_and_field_types()
    assert deserialize_message(serialize_message(message), VehicleStatus) == message


def test_transport_timestamp_tracks_px4_dds_clock_not_wall_or_sim_clock(manager):
    telemetry(manager)
    manager._local.timestamp = 123_456_000
    manager.test_clock[0] += 0.05
    assert manager._timestamp() == pytest.approx(123_506_000, abs=1)


def test_native_manager_has_no_ardupilot_imports():
    script = ROOT / "flight_cycle_manager/flight_cycle_manager/px4_manager.py"
    text = script.read_text()
    assert "pymavlink" not in text
    assert "from .manager" not in text
    result = subprocess.run([sys.executable, "-c", """
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if 'pymavlink' in name or name == 'flight_cycle_manager.manager':
        raise AssertionError('ArduPilot import: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
import flight_cycle_manager.px4_manager
"""], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr


def test_setpoint_stream_nan_masks_and_transport_clock(manager):
    telemetry(manager)
    manager._target = [1.0, 2.0, -5.0]
    manager._yaw = 0.4
    manager._local.timestamp = 123_000_000
    captured = {}

    class Publisher:
        def __init__(self, name):
            self.name = name

        def publish(self, message):
            captured[self.name] = message

    manager._dds_publishers = {name: Publisher(name) for name in (
        "offboard_control_mode", "trajectory_setpoint")}
    manager._stream()
    mode, target = captured["offboard_control_mode"], captured["trajectory_setpoint"]
    assert mode.position and not mode.velocity
    assert list(target.position) == [1.0, 2.0, -5.0]
    assert all(math.isnan(value) for value in target.velocity)
    assert target.timestamp == mode.timestamp == 123_000_000
    manager._velocity = (0.1, 0.2, -0.3, -0.4)
    manager._stream()
    mode, target = captured["offboard_control_mode"], captured["trajectory_setpoint"]
    assert mode.velocity and not mode.position
    assert all(math.isnan(value) for value in target.position)
    assert list(target.velocity) == pytest.approx([0.1, 0.2, -0.3])
    assert math.isnan(target.yaw)
    assert target.yawspeed == pytest.approx(-0.4)


def test_command_timeout_releases_stream_and_requests_land(manager):
    start_to_hover(manager)
    manager._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1, param2=6)
    manager.test_clock[0] += 11.0
    telemetry(manager, armed=True, landed=False, offboard=True, z=-5)
    manager._tick()
    assert manager._state == "FC_FALLBACK_LAND"
    assert manager._pending.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
    assert manager._state not in manager.STREAM_STATES
    ack(manager, result=VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED)
    assert manager._state == "FAILED"


def test_pid_disarm_ack_preserves_request_correlation(manager):
    start_to_hover(manager)
    calls = []

    class Landing:
        def notify_disarm_result(self, *args):
            calls.append(args)

    manager._landing = Landing()
    manager._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                     param1=0, disarm_request_id=7)
    ack(manager)
    assert calls == [(7, True, manager._now())]


def test_message_snapshot_checksums_and_metadata_match_rc1():
    package = ROOT / "px4_msgs"
    snapshot = json.loads((package / "firmware_snapshot.json").read_text())
    assert snapshot["tag"] == "v1.18.0-rc1"
    assert snapshot["commit"] == "fca3df865af36124a28c9d607e850f111dbaaea9"
    assert set(snapshot["messages"]) == {
        f"{message_type.__name__}.msg" for _, _, message_type, _ in DDS_MESSAGES
    }
    assert {path.name for path in (package / "msg").glob("*.msg")} == set(snapshot["messages"])
    for name, origin in snapshot["messages"].items():
        assert hashlib.sha256((package / "msg" / name).read_bytes()).hexdigest() == origin["sha256"]


def test_message_snapshot_matches_pinned_firmware_commit():
    firmware = ROOT.parents[1] / ".worktrees/PX4-Autopilot"
    if not firmware.is_dir():
        pytest.skip("local PX4 checkout unavailable")
    snapshot = json.loads((ROOT / "px4_msgs/firmware_snapshot.json").read_text())
    commit = snapshot["commit"]
    available = subprocess.run(
        ["git", "-C", str(firmware), "cat-file", "-e", f"{commit}^{{commit}}"],
        capture_output=True, timeout=10,
    )
    if available.returncode:
        pytest.skip("pinned PX4 firmware commit unavailable locally")
    for name, origin in snapshot["messages"].items():
        result = subprocess.run(
            ["git", "-C", str(firmware), "show", f"{commit}:{origin['path']}"],
            capture_output=True, check=True, timeout=10,
        )
        assert (ROOT / "px4_msgs/msg" / name).read_bytes() == result.stdout


def test_px4_launch_has_only_native_manager():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    path = ROOT / "flight_cycle_manager/launch/px4_flight_cycle.launch.py"
    spec = importlib.util.spec_from_file_location("px4_cycle_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    assert [node._Node__node_executable for node in nodes] == ["px4_flight_cycle_manager", "pid_tuner"]
    assert context.launch_configurations["auto_start"] == "false"
    assert context.launch_configurations["pid_tuner"] == "true"
    assert context.launch_configurations["pid_tuner_port"] == "8765"

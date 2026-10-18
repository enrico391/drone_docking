"""Exercise native PX4 messages with real Humble nodes and deterministic time."""

import importlib.util
import math
from pathlib import Path
import sys
import subprocess

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from px4_msgs.msg import (
    VehicleCommand, VehicleCommandAck, VehicleLandDetected,
    VehicleLocalPosition, VehicleStatus,
)
from std_srvs.srv import Trigger

from flight_cycle_manager.landing_backend import VelocityCommand, VelocityFrame
from flight_cycle_manager.px4_control import dds_topic, velocity_to_ned
from flight_cycle_manager.px4_manager import Px4FlightCycleManager


ROOT = Path(__file__).parents[2]


@pytest.fixture
def manager(monkeypatch):
    monkeypatch.setattr('flight_cycle_manager.px4_manager.get_package_share_directory',
                        lambda _: str(ROOT / 'flight_cycle_manager'))
    context = Context()
    rclpy.init(context=context)
    clock = [100.0]
    node = Px4FlightCycleManager(
        context=context, now=lambda: clock[0],
        parameter_overrides=[Parameter("use_sim_time", value=False)],
    )
    node._timer.cancel()
    node.test_clock = clock
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


def telemetry(node, *, armed=False, landed=True, offboard=False, z=0.0):
    node._on_status(VehicleStatus(
        arming_state=(VehicleStatus.ARMING_STATE_ARMED if armed else VehicleStatus.ARMING_STATE_DISARMED),
        nav_state=(VehicleStatus.NAVIGATION_STATE_OFFBOARD if offboard else VehicleStatus.NAVIGATION_STATE_POSCTL),
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


def test_topic_versions_follow_firmware():
    assert dds_topic("", "out", "vehicle_status", VehicleStatus) == "/fmu/out/vehicle_status_v1"
    assert dds_topic("/px4_1/", "out", "vehicle_local_position", VehicleLocalPosition) == (
        "/px4_1/fmu/out/vehicle_local_position")


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


def test_message_snapshot_matches_workspace_firmware():
    firmware = ROOT.parents[1] / ".worktrees/PX4-Autopilot"
    if not firmware.is_dir():
        pytest.skip("local PX4 checkout unavailable")
    for path in (ROOT / "px4_msgs/msg").glob("*.msg"):
        origin = firmware / "msg" / path.name
        if not origin.is_file():
            origin = firmware / "msg/versioned" / path.name
        assert path.read_bytes() == origin.read_bytes()


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
    assert [node._Node__node_executable for node in nodes] == ["px4_flight_cycle_manager"]
    assert context.launch_configurations["auto_start"] == "false"

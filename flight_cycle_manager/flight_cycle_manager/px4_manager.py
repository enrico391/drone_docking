"""Non-blocking PX4 flight cycle using native ROS 2/uXRCE-DDS topics only."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,
    VehicleLandDetected, VehicleLocalPosition, VehicleStatus,
)
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .dock_search import DockSearchController, SearchConfig, SearchObservation, SearchOutcome
from .landing_backend import (
    DockObservation, DockTargetMode, LandingBackendOutcome, VehicleObservation,
)
from .landing_target import camera_dock_orientation_to_body_yaw_error
from .pid_landing_backend import PidLandingBackend
from .px4_control import (
    LIVE_PID_PARAMETERS, PID_AXES, dds_topic, pid_config,
    pose_tracking_telemetry, velocity_to_ned,
)


@dataclass
class PendingCommand:
    command: int
    deadline: float
    disarm_request_id: int | None = None


class Px4FlightCycleManager(Node):
    """Take off, hold, search and PID-dock without an ArduPilot adapter."""

    STREAM_STATES = frozenset({
        "WARMUP", "SET_OFFBOARD", "ARM", "TAKEOFF", "HOVER",
        "SEARCH_DOCK", "LANDING_BACKEND",
    })
    IDLE_STATES = frozenset({"READY", "DOCKED", "FAILED", "PILOT_OVERRIDE", "LINK_LOSS"})

    def __init__(self, *, now=time.monotonic, **kwargs):
        super().__init__("flight_cycle_manager", **kwargs)
        self._now = now
        # Watchdogs must keep running even when Gazebo /clock is paused.
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        share = Path(get_package_share_directory("flight_cycle_manager"))
        defaults = {}
        for filename in ("px4_flight_cycle.yaml", "pid_landing.yaml"):
            defaults.update(yaml.safe_load((share / "config" / filename).read_text())[
                "flight_cycle_manager"]["ros__parameters"])
        for name, value in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)
        self._p = {name: self.get_parameter(name).value for name in defaults}
        for name in (
            "update_rate_hz", "offboard_warmup_s", "command_timeout_s",
            "telemetry_timeout_s", "takeoff_timeout_s", "takeoff_altitude_m",
            "takeoff_altitude_tolerance_m", "takeoff_vertical_speed_tolerance_m_s",
            "takeoff_stable_time_s", "dock_pose_timeout_s",
        ):
            if not math.isfinite(self._p[name]) or self._p[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self._p["update_rate_hz"] < 10 or self._p["offboard_warmup_s"] < 1.0:
            raise ValueError("Offboard requires >=10 Hz and >=1 s warmup")
        if self._p["auto_start"] and not self._p["simulation_mode"]:
            raise ValueError("auto_start requires simulation_mode=true")
        for name in ("source_system", "source_component"):
            if not 1 <= self._p[name] <= 255:
                raise ValueError(f"{name} must be in [1, 255]")
        self._pid_config = pid_config(self._p)
        self._search_config = SearchConfig(**{
            name: self._p[f"search_{name}"] for name in SearchConfig.__dataclass_fields__
        })
        self._state = "READY"
        self._entered = now()
        self._pending = None
        self._status = self._local = self._land = None
        self._status_at = self._local_at = self._land_at = -math.inf
        self._dock_pose = None
        self._dock_mode = "NO_TARGET"
        self._dock_pose_at = self._dock_mode_at = -math.inf
        self._target = None
        self._yaw = math.nan
        self._velocity = None
        self._stable_since = None
        self._search = self._landing = None
        self._reset_counters = None
        self._entry_action = None
        self._entry_nav_state = None
        self._offboard_acquired = False
        self._auto_requested = False
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._dds_publishers = {}
        for name, message_type in (
            ("offboard_control_mode", OffboardControlMode),
            ("trajectory_setpoint", TrajectorySetpoint),
            ("vehicle_command", VehicleCommand),
        ):
            self._dds_publishers[name] = self.create_publisher(
                message_type, self._topic("in", name, message_type), 10)
        self._dds_subscriptions = []
        for name, message_type, callback in (
            ("vehicle_status", VehicleStatus, self._on_status),
            ("vehicle_local_position", VehicleLocalPosition, self._on_local),
            ("vehicle_land_detected", VehicleLandDetected, self._on_land),
            ("vehicle_command_ack", VehicleCommandAck, self._on_ack),
        ):
            self._dds_subscriptions.append(self.create_subscription(
                message_type, self._topic("out", name, message_type), callback, sensor_qos))
        self._dds_subscriptions.extend([
            self.create_subscription(PoseStamped, self._p["dock_pose_topic"],
                                     self._on_dock_pose, sensor_qos),
            self.create_subscription(String, self._p["dock_mode_topic"],
                                     self._on_dock_mode, sensor_qos),
        ])
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, "~/state", state_qos)
        self._reason_pub = self.create_publisher(String, "~/reason", state_qos)
        self._debug_pub = self.create_publisher(String, "~/pid_debug", 10)
        self._start_service = self.create_service(Trigger, "~/start_cycle", self._start)
        self._dock_service = self.create_service(Trigger, "~/request_dock", self._request_dock)
        self._timer = self.create_timer(
            1.0 / self._p["update_rate_hz"], self._tick, clock=self._watchdog_clock)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self._transition("READY", "PX4 DDS ready; waiting for fresh telemetry and start_cycle")

    def _on_parameters_changed(self, parameters):
        updates = {parameter.name: parameter.value for parameter in parameters}
        changed = {name: value for name, value in updates.items()
                   if name not in self._p or value != self._p[name]}
        unsupported = changed.keys() - LIVE_PID_PARAMETERS
        if unsupported:
            return SetParametersResult(
                successful=False, reason="restart required for: " + ", ".join(sorted(unsupported)))
        if not changed:
            return SetParametersResult(successful=True)
        try:
            for name, value in changed.items():
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"{name} must be a number")
                if not math.isfinite(value):
                    raise ValueError(f"{name} must be finite")
            candidate = dict(self._p, **changed)
            config = pid_config(candidate)
            # Validate the complete candidate before touching a running controller.
            if self._landing is not None and self._state == "LANDING_BACKEND":
                self._landing.validate_touchdown_transition(config)
                self._landing.reconfigure_pids(
                    tuple(getattr(config, f"{axis}_pid") for axis in PID_AXES),
                    config.near_pids, config.far_pids,
                    config.gain_near_distance_m, config.gain_far_distance_m,
                )
                self._landing.reconfigure_velocity_feedback(
                    x=config.x_k_v, y=config.y_k_v, z=config.z_k_v)
                self._landing.reconfigure_touchdown_transition(config)
        except (TypeError, ValueError, OverflowError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self._p, self._pid_config = candidate, config
        return SetParametersResult(successful=True)

    def _publish_pid_debug(self, now, dock=None, substate=None):
        active = self._state == "LANDING_BACKEND" and self._landing is not None
        diagnostics = self._landing.diagnostics() if active else {}
        data = pose_tracking_telemetry(
            now=now, state=substate or self._state, dock=dock,
            diagnostics=diagnostics, parameters=self._p,
        )
        data["cycle_state"] = self._state
        self._debug_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def _topic(self, direction, name, message_type):
        return dds_topic(self._p["px4_namespace"], direction, name, message_type)

    def _transition(self, state, reason):
        self._state, self._entered = state, self._now()
        self._state_pub.publish(String(data=state))
        self._reason_pub.publish(String(data=reason))
        self.get_logger().info(f"{state}: {reason}")
        if state != "LANDING_BACKEND":
            self._publish_pid_debug(self._now())

    def _on_status(self, message):
        self._status, self._status_at = message, self._now()
        # Status and land detection are separate DDS topics. On auto-disarm,
        # let the next tick evaluate their combined touchdown confirmation.
        if (self._state in {"HOVER", "SEARCH_DOCK", "LANDING_BACKEND"}
                and not self._armed() and not message.failsafe):
            return
        # Latch a takeover on receipt, even if another status restores Offboard
        # before the next timer tick or an outstanding command ACK arrives.
        self._check_authority()

    def _check_authority(self):
        if self._state not in self.STREAM_STATES | {"FC_FALLBACK_LAND"}:
            return True
        nav_state = self._status.nav_state
        offboard = nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if self._state == "FC_FALLBACK_LAND":
            lost = nav_state not in {
                VehicleStatus.NAVIGATION_STATE_AUTO_LAND, VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            }
        else:
            # Landed/disarmed completion takes precedence over the normal mode
            # change PX4 may perform at the end of a cycle.
            if (self._state in {"HOVER", "SEARCH_DOCK", "LANDING_BACKEND"}
                    and self._land is not None and self._land.landed and not self._armed()):
                return True
            entering = self._state in {"WARMUP", "SET_OFFBOARD"}
            lost = (self._status.failsafe or (not offboard and (
                not entering or self._offboard_acquired or nav_state != self._entry_nav_state)))
            if offboard:
                self._offboard_acquired = True
        if lost:
            self._pending = None
            self._transition("PILOT_OVERRIDE", "PX4 changed mode or entered failsafe; "
                             "authority released without re-entry")
        return not lost

    def _on_local(self, message):
        self._local, self._local_at = message, self._now()

    def _on_land(self, message):
        self._land, self._land_at = message, self._now()

    def _on_dock_pose(self, message):
        self._dock_pose, self._dock_pose_at = message, self._now()

    def _on_dock_mode(self, message):
        self._dock_mode, self._dock_mode_at = message.data, self._now()

    def _fresh(self):
        now, timeout = self._now(), self._p["telemetry_timeout_s"]
        if any(now - stamp > timeout for stamp in (
            self._status_at, self._local_at, self._land_at,
        )):
            return False
        local = self._local
        # PX4's heading_good_for_control stays false until magnetometer
        # alignment completes IN FLIGHT. Requiring it on the ground deadlocks
        # takeoff; position-mode Offboard is gated by PX4's own preflight checks.
        return (all((local.xy_valid, local.z_valid, local.v_xy_valid, local.v_z_valid))
                and all(math.isfinite(value) for value in (
                    local.x, local.y, local.z, local.vx, local.vy, local.vz, local.heading)))

    def _armed(self):
        return self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _start(self, request, response):
        del request
        response.success = False
        if self._state not in self.IDLE_STATES:
            response.message = "a cycle is already active"
        elif not self._fresh():
            response.message = "fresh valid PX4 status, position, heading and land telemetry required"
        elif self._armed() or not self._land.landed:
            response.message = "start requires disarmed and landed PX4"
        elif self._status.failsafe or not self._status.pre_flight_checks_pass:
            response.message = "PX4 preflight checks have not passed"
        else:
            self._prepare_entry("TAKEOFF")
            self._transition("WARMUP", "streaming position hold before Offboard request")
            response.success, response.message = True, "one PX4 cycle queued"
        return response

    def _request_dock(self, request, response):
        del request
        response.success = (self._state in self.IDLE_STATES | {"HOVER"} and self._fresh()
                            and self._armed() and not self._land.landed
                            and self._local.heading_good_for_control
                            and not self._status.failsafe)
        response.message = ("docking requires an idle manager and fresh, armed, airborne PX4 "
                            "with valid heading and no failsafe")
        if response.success:
            self._prepare_entry("DOCK")
            if self._offboard_acquired:
                self._begin_dock_search()
                response.message = "dock search started"
            else:
                self._transition("WARMUP", "docking queued; streaming position hold before Offboard request")
                response.message = "docking queued; acquiring PX4 Offboard mode"
        return response

    def _prepare_entry(self, action):
        self._pending = None
        self._landing = self._search = None
        self._target = [self._local.x, self._local.y, self._local.z]
        self._yaw = self._local.heading
        self._velocity = None
        self._stable_since = None
        self._reset_counters = self._counters()
        self._entry_action = action
        self._entry_nav_state = self._status.nav_state
        self._offboard_acquired = (
            self._entry_nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _begin_dock_search(self):
        self._search = DockSearchController(self._search_config)
        self._search.start(now=self._now(), center_enu_m=self._vehicle().position_enu_m)
        self._velocity = (0.0, 0.0, 0.0, 0.0)
        self._transition("SEARCH_DOCK", "explicit docking request accepted; Offboard confirmed")

    def _offboard_ready(self):
        if self._entry_action == "DOCK":
            if not self._armed() or self._land.landed or not self._local.heading_good_for_control:
                self._fallback("airborne docking prerequisites lost during Offboard entry")
                return
            self._begin_dock_search()
        else:
            self._transition("ARM", "Offboard confirmed; requesting arm")
            self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1)

    def _counters(self):
        return (self._local.xy_reset_counter, self._local.z_reset_counter,
                self._local.heading_reset_counter)

    def _timestamp(self):
        # The DDS client applies its current timesync offset in BOTH
        # directions. Echo its latest adjusted local-position timestamp so
        # accelerated/slow lockstep simulation cannot make wall time stale
        # before the client's timesync filter catches up. Telemetry freshness
        # is independently checked using monotonic receipt times.
        elapsed = max(0.0, self._now() - self._local_at)
        return int(self._local.timestamp + elapsed * 1_000_000)

    def _command(self, command, *, disarm_request_id=None, **parameters):
        message = VehicleCommand()
        message.timestamp = self._timestamp()
        message.command = command
        message.target_system = self._status.system_id
        message.target_component = self._status.component_id
        message.source_system = self._p["source_system"]
        message.source_component = self._p["source_component"]
        message.from_external = True
        for key, value in parameters.items():
            setattr(message, key, float(value))
        self._pending = PendingCommand(
            command, self._now() + self._p["command_timeout_s"], disarm_request_id)
        self._dds_publishers["vehicle_command"].publish(message)

    def _on_ack(self, message):
        pending = self._pending
        if (pending is None or message.command != pending.command
                or message.target_system != self._p["source_system"]
                or message.target_component != self._p["source_component"]
                or message.from_external):
            return
        if message.result == VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS:
            return
        self._pending = None
        success = message.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        if pending.disarm_request_id is not None:
            self._landing.notify_disarm_result(pending.disarm_request_id, success, self._now())
        elif not success:
            if self._state == "FC_FALLBACK_LAND":
                self._transition("FAILED", f"PX4 rejected LAND: result {message.result}; stream stopped")
            else:
                self._fallback(f"PX4 rejected command {message.command}: result {message.result}")

    def _fallback(self, reason):
        self._pending = None
        if not self._armed():
            self._transition("FAILED", reason)
            return
        self._transition("FC_FALLBACK_LAND", reason)
        self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _tick(self):
        now = self._now()
        if self._state != "LANDING_BACKEND":
            self._publish_pid_debug(now)
        if self._p["auto_start"] and not self._auto_requested and self._state == "READY":
            result = self._start(None, Trigger.Response())
            self._auto_requested = result.success
        if self._state not in self.STREAM_STATES | {"FC_FALLBACK_LAND"}:
            return
        if not self._fresh():
            self._pending = None
            self._transition("LINK_LOSS", "invalid/stale PX4 telemetry; stream stopped for PX4 failsafe")
            return
        if (self._state in {"HOVER", "SEARCH_DOCK", "LANDING_BACKEND"}
                and self._land.landed and not self._armed()):
            self._pending = None
            self._transition("DOCKED", "fresh landed and disarmed telemetry confirmed")
            return
        if not self._check_authority():
            return
        counters = self._counters()
        if (self._state == "TAKEOFF" and counters[:2] == self._reset_counters[:2]
                and counters[2] != self._reset_counters[2]):
            # PX4 intentionally performs final magnetic yaw alignment after
            # liftoff. Keep the physical yaw setpoint continuous across it.
            self._yaw = math.atan2(math.sin(self._yaw + self._local.delta_heading),
                                   math.cos(self._yaw + self._local.delta_heading))
            self._reset_counters = counters
        if self._state in self.STREAM_STATES and counters != self._reset_counters:
            self._fallback("PX4 estimator reference reset during cycle")
            return
        if self._pending and now > self._pending.deadline:
            pending, self._pending = self._pending, None
            if pending.disarm_request_id is not None:
                self._landing.notify_disarm_result(pending.disarm_request_id, False, now)
            elif self._state == "FC_FALLBACK_LAND":
                self._transition("FAILED", "LAND acknowledgement timed out; stream stopped")
                return
            else:
                self._fallback("PX4 command acknowledgement timed out")
                return
        if self._state == "FC_FALLBACK_LAND":
            if self._land.landed and not self._armed():
                self._transition("DOCKED", "PX4 landed and disarmed (fallback; dock not guaranteed)")
            elif (self._status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                  and now - self._entered > self._p["command_timeout_s"]):
                self._transition("FAILED", "PX4 did not enter LAND; Offboard stream remains stopped")
            return
        if self._state == "WARMUP" and now - self._entered >= self._p["offboard_warmup_s"]:
            if self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._offboard_ready()
            else:
                self._transition("SET_OFFBOARD", "requesting PX4 Offboard mode")
                self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1, param2=6)
        elif self._state == "SET_OFFBOARD":
            if self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self._pending is None:
                self._offboard_ready()
            elif now - self._entered > self._p["command_timeout_s"]:
                self._fallback("Offboard mode was not confirmed")
        elif self._state == "ARM":
            if self._armed() and self._pending is None:
                self._target[2] -= self._p["takeoff_altitude_m"]
                self._transition("TAKEOFF", "climbing above the cycle's resting position")
            elif now - self._entered > self._p["command_timeout_s"]:
                self._fallback("arming was not confirmed")
        elif self._state == "TAKEOFF":
            stable = (not self._land.landed and self._armed()
                      and abs(self._local.z - self._target[2]) <= self._p["takeoff_altitude_tolerance_m"]
                      and abs(self._local.vz) <= self._p["takeoff_vertical_speed_tolerance_m_s"])
            if stable and self._stable_since is None:
                self._stable_since = now
            elif not stable:
                self._stable_since = None
            if self._stable_since is not None and now - self._stable_since >= self._p["takeoff_stable_time_s"]:
                self._transition("HOVER", "takeoff stable; waiting for request_dock")
            elif not self._armed() or now - self._entered > self._p["takeoff_timeout_s"]:
                self._fallback("takeoff failed or timed out")
        elif self._state in {"HOVER", "SEARCH_DOCK", "LANDING_BACKEND"}:
            if self._land.landed and not self._armed():
                self._pending = None
                self._transition("DOCKED", "fresh landed and disarmed telemetry confirmed")
                return
            if not self._armed():
                self._transition("FAILED", "PX4 disarmed without landed confirmation")
                return
            if self._state != "HOVER" and not self._local.heading_good_for_control:
                self._fallback("PX4 heading no longer valid for body velocity control")
                return
            if self._state != "HOVER":
                self._advance_docking(now)
        if self._state in self.STREAM_STATES:
            self._stream()

    def _stream(self):
        timestamp = self._timestamp()
        mode = OffboardControlMode(timestamp=timestamp)
        mode.position = self._velocity is None
        mode.velocity = self._velocity is not None
        setpoint = TrajectorySetpoint(timestamp=timestamp)
        setpoint.position = self._target if mode.position else [math.nan] * 3
        setpoint.velocity = [math.nan] * 3 if mode.position else list(self._velocity[:3])
        setpoint.acceleration = setpoint.jerk = [math.nan] * 3
        setpoint.yaw = self._yaw if mode.position else math.nan
        setpoint.yawspeed = math.nan if mode.position else self._velocity[3]
        self._dds_publishers["offboard_control_mode"].publish(mode)
        self._dds_publishers["trajectory_setpoint"].publish(setpoint)

    def _vehicle(self):
        local = self._local
        return VehicleObservation(
            (local.y, local.x, -local.z), (local.vy, local.vx, -local.vz),
            self._armed(), not self._land.landed, self._status.nav_state,
            self._land_at, local.heading,
        )

    def _dock(self, now):
        timeout = self._p["dock_pose_timeout_s"]
        if (self._dock_pose is None or now - self._dock_pose_at > timeout
                or now - self._dock_mode_at > timeout):
            return None
        mode = DockTargetMode.__members__.get(self._dock_mode)
        pose = self._dock_pose.pose
        p, q = pose.position, pose.orientation
        if mode is None or p.z <= 0 or not all(math.isfinite(value) for value in (
            p.x, p.y, p.z, q.x, q.y, q.z, q.w,
        )):
            return None
        # Reject delayed camera frames as well as stale receipt times.
        stamp = self._dock_pose.header.stamp
        source = stamp.sec + stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - source
        if source <= 0 or age < -0.1 or age > timeout:
            return None
        try:
            yaw = -camera_dock_orientation_to_body_yaw_error(q.x, q.y, q.z, q.w)
        except ValueError:
            return None
        return DockObservation(-p.y, -p.x, p.z, yaw, mode, source, self._dock_pose_at)

    def _advance_docking(self, now):
        vehicle, dock = self._vehicle(), self._dock(now)
        if self._state == "SEARCH_DOCK":
            decision = self._search.update(SearchObservation(now, vehicle.position_enu_m, dock is not None))
            east, north, up = decision.velocity_enu_m_s
            self._velocity = (north, east, -up, 0.0)
            if decision.outcome is SearchOutcome.ACQUIRED:
                self._landing = PidLandingBackend(self._pid_config)
                self._landing.start(now=now, dock=dock, vehicle=vehicle)
                self._transition("LANDING_BACKEND", "dock acquired; native DDS velocity control")
            elif decision.outcome is not SearchOutcome.SEARCHING:
                self._fallback(decision.reason)
        else:
            decision = self._landing.update(now=now, dock=dock, vehicle=vehicle)
            if decision.velocity is not None:
                self._velocity = velocity_to_ned(decision.velocity, self._local.heading)
            self._publish_pid_debug(now, dock, decision.substate)
            if decision.request_disarm and self._pending is None:
                # Never force-disarm: PX4 must independently confirm touchdown.
                if self._land.landed:
                    self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                  param1=0, disarm_request_id=decision.disarm_request_id)
                else:
                    self._landing.notify_disarm_result(decision.disarm_request_id, False, now)
            if decision.outcome in {
                LandingBackendOutcome.FAIL_BEFORE_COMMIT, LandingBackendOutcome.FAIL_AFTER_COMMIT,
            }:
                self._fallback(decision.reason)
            elif decision.outcome is LandingBackendOutcome.COMPLETE:
                if self._land.landed and not self._armed():
                    self._transition("DOCKED", "PID docking complete; landed and disarmed")
                else:
                    self._fallback("PID completed without PX4 touchdown confirmation")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Px4FlightCycleManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

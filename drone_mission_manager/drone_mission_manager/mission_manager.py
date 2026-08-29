"""Non-blocking circle mission and camera-guided precision landing manager."""

import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .control import camera_offset_to_body_velocity
from .landing_target import (
    camera_dock_orientation_to_body_yaw_error,
    camera_target_to_body_frd,
    measurement_time_usec,
)
from .landing_wait import (
    LandingWaitAction,
    disarm_wait_action,
    touchdown_wait_action,
)
from .mission import CircleMission, load_mission
from .pymavlink_vehicle import MavlinkVehicle, VelocityFrame


GUIDED_MODE = 4
LAND_MODE = 9


@dataclass(frozen=True)
class _VehicleStatusView:
    """Status fields consumed by the unchanged landing state machine."""

    armed: bool
    flying: bool


def create_mavlink_vehicle(node: Node) -> MavlinkVehicle:
    """Construct the direct flight-controller transport from ROS parameters."""
    return MavlinkVehicle(
        connection=str(node.get_parameter("mavlink_connection").value),
        baud=int(node.get_parameter("mavlink_baud").value),
        target_system=int(node.get_parameter("mavlink_target_system").value),
        target_component=int(
            node.get_parameter("mavlink_target_component").value
        ),
        source_system=int(node.get_parameter("mavlink_source_system").value),
        source_component=int(
            node.get_parameter("mavlink_source_component").value
        ),
    )


class MissionState(Enum):
    """States used by the asynchronous mission controller."""

    IDLE = auto()
    WAITING_FOR_AP = auto()
    CHECK_LANDING_CONFIG = auto()
    PREARM = auto()
    SET_GUIDED = auto()
    ARM = auto()
    TAKEOFF = auto()
    WAIT_TAKEOFF = auto()
    GO_TO_CIRCLE = auto()
    CIRCLE = auto()
    RETURN_TO_DOCK = auto()
    HOVER_OVER_DOCK = auto()
    ACQUIRE_DOCK = auto()
    ALIGN_DOCK = auto()
    DESCEND_OUTER = auto()
    DESCEND_CENTRAL = auto()
    SET_LAND = auto()
    WAIT_LANDED = auto()
    WAIT_DISARM = auto()
    REPEAT_DELAY = auto()
    COMPLETE = auto()
    FAILED = auto()


class MissionManager(Node):
    """Execute one configured simulation mission without blocking callbacks."""

    def __init__(
        self,
        vehicle_factory: Optional[Callable[[Node], MavlinkVehicle]] = None,
        **node_kwargs,
    ) -> None:
        super().__init__("mission_manager", **node_kwargs)
        self.declare_parameter("mission_file", "")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("position_gain", 0.7)
        self.declare_parameter("max_horizontal_speed_m_s", 1.5)
        self.declare_parameter("max_vertical_speed_m_s", 0.7)
        self.declare_parameter("position_tolerance_m", 0.35)
        self.declare_parameter("altitude_tolerance_m", 0.25)
        self.declare_parameter("pose_timeout_s", 1.0)
        self.declare_parameter("mavlink_operation_timeout_s", 10.0)
        self.declare_parameter("prearm_timeout_s", 60.0)
        self.declare_parameter("prearm_retry_period_s", 1.0)
        self.declare_parameter("dock_pose_timeout_s", 0.5)
        self.declare_parameter("dock_acquire_timeout_s", 20.0)
        self.declare_parameter("dock_horizontal_gain", 0.7)
        self.declare_parameter("dock_max_horizontal_speed_m_s", 0.5)
        self.declare_parameter("dock_coarse_tolerance_m", 0.20)
        self.declare_parameter("dock_fine_tolerance_m", 0.07)
        self.declare_parameter("dock_alignment_stable_time_s", 1.0)
        self.declare_parameter("dock_central_transition_distance_m", 1.50)
        self.declare_parameter("dock_outer_descent_speed_m_s", 0.30)
        self.declare_parameter("dock_central_descent_speed_m_s", 0.12)
        self.declare_parameter("dock_land_switch_distance_m", 0.55)
        self.declare_parameter("dock_land_timeout_s", 30.0)
        self.declare_parameter("dock_disarm_timeout_s", 20.0)
        self.declare_parameter("landing_backend", "ros")
        self.declare_parameter("landing_yaw_correction_enabled", True)
        self.declare_parameter("landing_target_warmup_s", 1.0)
        self.declare_parameter("landing_target_camera_forward_m", 0.0)
        self.declare_parameter("landing_target_camera_right_m", 0.0)
        self.declare_parameter("landing_target_camera_down_m", 0.19)
        self.declare_parameter("mavlink_connection", "tcp:127.0.0.1:5760")
        self.declare_parameter("mavlink_baud", 921600)
        self.declare_parameter("mavlink_target_system", 1)
        self.declare_parameter("mavlink_target_component", 1)
        self.declare_parameter("mavlink_source_system", 254)
        self.declare_parameter("mavlink_source_component", 191)
        self.declare_parameter("mavlink_heartbeat_timeout_s", 2.5)
        self.declare_parameter("dock_pose_topic", "/dock/pose")
        self.declare_parameter("dock_mode_topic", "/dock/mode")

        mission_path = str(self.get_parameter("mission_file").value)
        self._mission: CircleMission = load_mission(mission_path)
        self._validate_landing_parameters()
        self._landing_backend = str(
            self.get_parameter("landing_backend").value
        ).lower()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._dock_pose_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("dock_pose_topic").value),
            self._dock_pose_callback,
            sensor_qos,
        )
        self._dock_mode_subscription = self.create_subscription(
            String,
            str(self.get_parameter("dock_mode_topic").value),
            self._dock_mode_callback,
            sensor_qos,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._state_publisher = self.create_publisher(
            String, "~/state", state_qos
        )

        factory = vehicle_factory or create_mavlink_vehicle
        self._vehicle = factory(self)

        self._pose: Optional[PoseStamped] = None
        self._pose_received_at = 0.0
        self._mission_origin: Optional[Tuple[float, float]] = None
        self._status: Optional[_VehicleStatusView] = None
        self._has_seen_armed = False
        self._dock_pose: Optional[PoseStamped] = None
        self._dock_pose_received_at = 0.0
        self._dock_pose_generation = 0
        self._last_streamed_dock_pose_generation = -1
        self._dock_mode = "NO_TARGET"
        self._dock_mode_received_at = 0.0
        self._pending = None
        self._pending_name = ""
        self._pending_deadline = 0.0
        self._next_prearm_at = 0.0
        self._circle_progress = 0.0
        self._iteration = 1
        self._repeat_at = 0.0
        self._alignment_started_at: Optional[float] = None
        self._landing_target_started_at: Optional[float] = None
        self._landing_yaw_command_sent = False
        self._landing_timeout_warned = False
        self._disarm_timeout_warned = False
        self._state_started_at = time.monotonic()
        self._last_update_at = time.monotonic()

        initial_state = (
            MissionState.WAITING_FOR_AP
            if bool(self.get_parameter("auto_start").value)
            else MissionState.IDLE
        )
        self._state = initial_state
        self._publish_state()

        update_rate = float(self.get_parameter("update_rate_hz").value)
        if update_rate <= 0.0:
            raise ValueError("update_rate_hz must be greater than zero")
        self._timer = self.create_timer(1.0 / update_rate, self._update)
        self.get_logger().info(
            f"Loaded {self._mission.mission_type.value} mission "
            f"'{self._mission.name}' from {mission_path}; "
            f"repeat_count={self._mission.repeat_count}; "
            f"landing_backend={self._landing_backend}; "
            f"state={self._state.name}"
        )

    def _validate_landing_parameters(self) -> None:
        positive_parameters = (
            "prearm_timeout_s",
            "prearm_retry_period_s",
            "dock_pose_timeout_s",
            "dock_acquire_timeout_s",
            "dock_horizontal_gain",
            "dock_max_horizontal_speed_m_s",
            "dock_coarse_tolerance_m",
            "dock_fine_tolerance_m",
            "dock_alignment_stable_time_s",
            "dock_central_transition_distance_m",
            "dock_outer_descent_speed_m_s",
            "dock_central_descent_speed_m_s",
            "dock_land_switch_distance_m",
            "dock_land_timeout_s",
            "dock_disarm_timeout_s",
            "landing_target_warmup_s",
        )
        for name in positive_parameters:
            if float(self.get_parameter(name).value) <= 0.0:
                raise ValueError(f"{name} must be greater than zero")

        coarse = float(self.get_parameter("dock_coarse_tolerance_m").value)
        fine = float(self.get_parameter("dock_fine_tolerance_m").value)
        if fine >= coarse:
            raise ValueError(
                "dock_fine_tolerance_m must be smaller than "
                "dock_coarse_tolerance_m"
            )
        transition = float(
            self.get_parameter("dock_central_transition_distance_m").value
        )
        land_switch = float(
            self.get_parameter("dock_land_switch_distance_m").value
        )
        if land_switch >= transition:
            raise ValueError(
                "dock_land_switch_distance_m must be smaller than "
                "dock_central_transition_distance_m"
            )

        backend = str(self.get_parameter("landing_backend").value).lower()
        if backend not in ("ros", "ardupilot"):
            raise ValueError(
                "landing_backend must be either 'ros' or 'ardupilot'"
            )
        heartbeat_timeout = float(
            self.get_parameter("mavlink_heartbeat_timeout_s").value
        )
        if heartbeat_timeout <= 0.0:
            raise ValueError("mavlink_heartbeat_timeout_s must be greater than zero")

    def _sync_vehicle_state(self) -> None:
        """Expose direct telemetry in the shapes used by existing states."""
        state = self._vehicle.state
        if (
            state.position_enu_m is not None
            and state.position_at >= self._pose_received_at
        ):
            pose = PoseStamped()
            pose.pose.position.x = state.position_enu_m[0]
            pose.pose.position.y = state.position_enu_m[1]
            pose.pose.position.z = state.position_enu_m[2]
            self._pose = pose
            self._pose_received_at = state.position_at
        if state.armed is not None and state.flying is not None:
            self._status = _VehicleStatusView(
                armed=state.armed,
                flying=state.flying,
            )
            self._has_seen_armed = self._has_seen_armed or state.armed

    def _dock_pose_callback(self, message: PoseStamped) -> None:
        self._dock_pose = message
        self._dock_pose_received_at = time.monotonic()
        self._dock_pose_generation += 1

    def _dock_mode_callback(self, message: String) -> None:
        self._dock_mode = message.data
        self._dock_mode_received_at = time.monotonic()

    def _set_state(self, state: MissionState) -> None:
        if state == self._state:
            return
        self._state = state
        self._state_started_at = time.monotonic()
        self._alignment_started_at = None
        if state == MissionState.SET_LAND:
            self._landing_yaw_command_sent = False
        if state == MissionState.WAIT_LANDED:
            self._landing_timeout_warned = False
        elif state == MissionState.WAIT_DISARM:
            self._disarm_timeout_warned = False
        self._publish_state()
        self.get_logger().info(f"Mission state: {state.name}")

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.name
        self._state_publisher.publish(message)

    def _vehicle_ready(self, now: float) -> bool:
        return self._vehicle.ready(
            now,
            heartbeat_timeout_s=float(
                self.get_parameter("mavlink_heartbeat_timeout_s").value
            ),
        )

    def _pose_is_fresh(self, now: float) -> bool:
        timeout = float(self.get_parameter("pose_timeout_s").value)
        return self._pose is not None and now - self._pose_received_at <= timeout

    def _dock_target_is_fresh(self, now: float) -> bool:
        timeout = float(self.get_parameter("dock_pose_timeout_s").value)
        return (
            self._dock_pose is not None
            and self._dock_mode != "NO_TARGET"
            and now - self._dock_pose_received_at <= timeout
            and now - self._dock_mode_received_at <= timeout
        )

    def _begin_call(self, name: str, future) -> None:
        self._pending_name = name
        self._pending = future
        timeout = float(
            self.get_parameter("mavlink_operation_timeout_s").value
        )
        self._pending_deadline = time.monotonic() + timeout

    def _fail(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._pending = None
        self._publish_velocity(0.0, 0.0, 0.0)
        self._set_state(MissionState.FAILED)

    def _handle_pending_call(self, now: float) -> bool:
        if self._pending is None:
            return False
        if now > self._pending_deadline:
            self._fail(f"{self._pending_name} MAVLink operation timed out")
            return True
        if not self._pending.done():
            return True

        try:
            result = self._pending.result()
        except Exception as exception:
            self._fail(
                f"{self._pending_name} MAVLink operation failed: {exception}"
            )
            return True

        name = self._pending_name
        self._pending = None
        if name == "prearm":
            if not result.success:
                elapsed = now - self._state_started_at
                timeout = float(self.get_parameter("prearm_timeout_s").value)
                if elapsed >= timeout:
                    self._fail(
                        f"pre-arm check did not pass within {timeout:.1f}s: "
                        f"{result.message}"
                    )
                else:
                    retry_period = float(
                        self.get_parameter("prearm_retry_period_s").value
                    )
                    self._next_prearm_at = now + retry_period
                    self.get_logger().warning(
                        f"Vehicle not armable yet ({result.message}); "
                        f"retrying pre-arm check in {retry_period:.1f}s"
                    )
            else:
                self._set_state(MissionState.SET_GUIDED)
        elif name == "mode":
            if not (
                result.success or result.current_mode == GUIDED_MODE
            ):
                self._fail("ArduPilot rejected GUIDED mode")
            else:
                self._set_state(MissionState.ARM)
        elif name == "arm":
            if not result.success:
                self._fail("ArduPilot rejected the arm request")
            else:
                self._set_state(MissionState.TAKEOFF)
        elif name == "takeoff":
            if not result.success:
                self._fail("ArduPilot rejected the takeoff request")
            else:
                self._set_state(MissionState.WAIT_TAKEOFF)
        elif name == "land_mode":
            if not (result.success or result.current_mode == LAND_MODE):
                self._fail("ArduPilot rejected LAND mode")
            else:
                self._set_state(MissionState.WAIT_LANDED)
        elif name == "landing_config":
            enabled = result.parameters.get("PLND_ENABLED")
            backend = result.parameters.get("PLND_TYPE")
            if enabled != 1.0 or backend != 1.0:
                self._fail(
                    "ArduPilot native precision landing is not configured: "
                    f"PLND_ENABLED={enabled}, PLND_TYPE={backend}; restart "
                    "the simulation with the updated X500 parameter defaults"
                )
            else:
                self._set_state(MissionState.PREARM)
        return True

    def _update(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self._last_update_at, 0.0), 0.5)
        self._last_update_at = now

        self._vehicle.poll(now)
        self._sync_vehicle_state()

        # USED TO SEND DOCK POSE TO PRECISION LANDING VIA PYMAVLINK
        self._stream_native_landing_target(now)

        if self._handle_pending_call(now):
            return
        if self._state == MissionState.IDLE:
            return
        if self._state == MissionState.FAILED:
            self._publish_velocity(0.0, 0.0, 0.0)
            return
        if self._state == MissionState.REPEAT_DELAY:
            if now >= self._repeat_at:
                self._start_next_iteration(now)
            return
        if self._state == MissionState.WAITING_FOR_AP:
            if self._vehicle_ready(now) and self._pose_is_fresh(now):
                if self._mission_origin is None:
                    position = self._pose.pose.position
                    self._mission_origin = (position.x, position.y)
                    self.get_logger().info(
                        "Captured mission initial pose at "
                        f"({position.x:.2f}, {position.y:.2f}) m; "
                        "coordinate_reference="
                        f"{self._mission.coordinate_reference.value}"
                    )
                next_state = (
                    MissionState.CHECK_LANDING_CONFIG
                    if self._landing_backend == "ardupilot"
                    else MissionState.PREARM
                )
                self._set_state(next_state)
            return
        if self._state == MissionState.CHECK_LANDING_CONFIG:
            self._begin_call(
                "landing_config",
                self._vehicle.get_parameters_async(
                    ("PLND_ENABLED", "PLND_TYPE"), now
                ),
            )
            return
        if self._state == MissionState.PREARM:
            if now >= self._next_prearm_at:
                self._begin_call(
                    "prearm", self._vehicle.prearm_async(now)
                )
            return
        if self._state == MissionState.SET_GUIDED:
            self._begin_call(
                "mode", self._vehicle.set_mode_async(GUIDED_MODE, now)
            )
            return
        if self._state == MissionState.ARM:
            self._begin_call("arm", self._vehicle.arm_async(True, now))
            return
        if self._state == MissionState.TAKEOFF:
            self._begin_call(
                "takeoff",
                self._vehicle.takeoff_async(
                    self._mission.takeoff_altitude_m, now
                ),
            )
            return
        if self._state == MissionState.SET_LAND:
            self._begin_call(
                "land_mode", self._vehicle.set_mode_async(LAND_MODE, now)
            )
            return
        if self._state == MissionState.WAIT_LANDED:
            self._send_native_landing_yaw(now)
            status = self._status
            action = touchdown_wait_action(
                armed=status.armed if status is not None else True,
                flying=status.flying if status is not None else True,
                elapsed_s=now - self._state_started_at,
                warn_after_s=float(
                    self.get_parameter("dock_land_timeout_s").value
                ),
                warning_emitted=self._landing_timeout_warned,
            )
            if action == LandingWaitAction.COMPLETE:
                self._complete_landing()
            elif action == LandingWaitAction.TOUCHDOWN:
                self._set_state(MissionState.WAIT_DISARM)
                self.get_logger().info(
                    "ArduPilot reports touchdown; waiting for automatic disarm"
                )
            elif action == LandingWaitAction.WARN:
                self._landing_timeout_warned = True
                timeout = float(
                    self.get_parameter("dock_land_timeout_s").value
                )
                self.get_logger().warning(
                    f"LAND has not reported touchdown after {timeout:.1f}s; "
                    "continuing precision-landing target stream and waiting"
                )
            return
        if self._state == MissionState.WAIT_DISARM:
            status = self._status
            action = disarm_wait_action(
                armed=status.armed if status is not None else True,
                flying=status.flying if status is not None else False,
                elapsed_s=now - self._state_started_at,
                warn_after_s=float(
                    self.get_parameter("dock_disarm_timeout_s").value
                ),
                warning_emitted=self._disarm_timeout_warned,
            )
            if action == LandingWaitAction.COMPLETE:
                self._complete_landing()
            elif action == LandingWaitAction.AIRBORNE:
                self.get_logger().warning(
                    "ArduPilot reports the vehicle flying again after "
                    "touchdown; returning to touchdown wait"
                )
                self._set_state(MissionState.WAIT_LANDED)
            elif action == LandingWaitAction.WARN:
                self._disarm_timeout_warned = True
                timeout = float(
                    self.get_parameter("dock_disarm_timeout_s").value
                )
                self.get_logger().warning(
                    f"Vehicle remains armed {timeout:.1f}s after touchdown; "
                    "continuing to wait for ArduPilot automatic disarm"
                )
            return
        if self._state == MissionState.COMPLETE:
            return

        if not self._pose_is_fresh(now):
            self._fail("local pose became stale; stopping mission commands")
            return
        if (
            self._status is not None
            and self._has_seen_armed
            and not self._status.armed
        ):
            self._fail("vehicle disarmed while mission was active")
            return

        if self._state == MissionState.WAIT_TAKEOFF:
            altitude_error = abs(
                self._pose.pose.position.z - self._mission.takeoff_altitude_m
            )
            tolerance = float(self.get_parameter("altitude_tolerance_m").value)
            if altitude_error <= tolerance:
                self._set_state(MissionState.GO_TO_CIRCLE)
            return
        if self._state == MissionState.GO_TO_CIRCLE:
            center_x, center_y = self._mission_xy(
                self._mission.circle_center_x_m,
                self._mission.circle_center_y_m,
            )
            target = (
                center_x + self._mission.circle_radius_m,
                center_y,
                self._mission.takeoff_altitude_m,
            )
            if self._fly_to(target):
                self._circle_progress = 0.0
                self._set_state(MissionState.CIRCLE)
            return
        if self._state == MissionState.CIRCLE:
            self._fly_circle(dt)
            return
        if self._state == MissionState.RETURN_TO_DOCK:
            dock_x, dock_y = self._mission_xy(
                self._mission.dock_x_m, self._mission.dock_y_m
            )
            target = (
                dock_x,
                dock_y,
                self._mission.hover_altitude_m,
            )
            if self._fly_to(target):
                if self._mission.precision_landing_enabled:
                    self._set_state(MissionState.ACQUIRE_DOCK)
                    self.get_logger().info(
                        "Coarse dock approach complete; acquiring visual target"
                    )
                else:
                    self._set_state(MissionState.HOVER_OVER_DOCK)
                    self.get_logger().info(
                        "Circle mission complete; holding over dock"
                    )
            return
        if self._state == MissionState.HOVER_OVER_DOCK:
            dock_x, dock_y = self._mission_xy(
                self._mission.dock_x_m, self._mission.dock_y_m
            )
            self._fly_to(
                (
                    dock_x,
                    dock_y,
                    self._mission.hover_altitude_m,
                )
            )
            return
        if self._state == MissionState.ACQUIRE_DOCK:
            self._publish_velocity(0.0, 0.0, 0.0, frame_id="base_link")
            if self._dock_target_is_fresh(now):
                # IF ROS BACKEND START TO ALIGN WITH CMD_VEL
                if self._landing_backend == "ros":
                    self._set_state(MissionState.ALIGN_DOCK)
                # For native precision landing, LAND consumes the dock pose.
                elif self._native_target_is_ready(now):
                    self._set_state(MissionState.SET_LAND)
                    self.get_logger().info(
                        "LANDING_TARGET stream is stable; handing descent "
                        "control to ArduPilot precision landing"
                    )
            if now - self._state_started_at > float(
                self.get_parameter("dock_acquire_timeout_s").value
            ):
                reason = "dock target was not acquired before timeout"
                if (
                    self._landing_backend == "ardupilot"
                    and self._dock_target_is_fresh(now)
                ):
                    reason = (
                        "LANDING_TARGET stream did not become ready before "
                        "timeout"
                    )
                self._fail(reason)
            return
        if self._state == MissionState.ALIGN_DOCK:
            if not self._dock_target_is_fresh(now):
                self._publish_velocity(0.0, 0.0, 0.0, frame_id="base_link")
                self._set_state(MissionState.ACQUIRE_DOCK)
            else:
                self._align_dock(now)
            return
        if self._state == MissionState.DESCEND_OUTER:
            self._descend_outer(now)
            return
        if self._state == MissionState.DESCEND_CENTRAL:
            self._descend_central(now)
            return

    def _dock_horizontal_error(self) -> float:
        position = self._dock_pose.pose.position
        return math.hypot(position.x, position.y)

    def _complete_landing(self) -> None:
        if self._mission.has_next_iteration(self._iteration):
            completed_iteration = self._iteration
            self._iteration += 1
            self._repeat_at = time.monotonic() + self._mission.repeat_delay_s
            self._set_state(MissionState.REPEAT_DELAY)
            self.get_logger().info(
                f"Completed iteration {completed_iteration}/"
                f"{self._mission.repeat_count}; vehicle is disarmed on the "
                f"dock; next iteration starts in "
                f"{self._mission.repeat_delay_s:.1f}s"
            )
            return

        self._set_state(MissionState.COMPLETE)
        self.get_logger().info(
            f"Precision landing complete after {self._iteration} iteration(s); "
            "vehicle is disarmed on the dock"
        )

    def _start_next_iteration(self, now: float) -> None:
        """Reset transient flight state and begin the next configured cycle."""
        self._pending = None
        self._pending_name = ""
        self._next_prearm_at = 0.0
        self._circle_progress = 0.0
        self._alignment_started_at = None
        self._landing_target_started_at = None
        self._landing_yaw_command_sent = False
        self._landing_timeout_warned = False
        self._disarm_timeout_warned = False
        self._has_seen_armed = False
        self._last_update_at = now
        self._set_state(MissionState.WAITING_FOR_AP)
        self.get_logger().info(
            f"Starting mission iteration {self._iteration}/"
            f"{self._mission.repeat_count}"
        )

    def _stream_native_landing_target(self, now: float) -> None:
        if self._landing_backend != "ardupilot":
            return
        active_states = (
            MissionState.ACQUIRE_DOCK,
            MissionState.SET_LAND,
            MissionState.WAIT_LANDED,
            MissionState.WAIT_DISARM,
        )
        if self._state not in active_states or not self._dock_target_is_fresh(now):
            if self._state == MissionState.ACQUIRE_DOCK:
                self._landing_target_started_at = None
            return
        if (
            self._dock_pose_generation
            == self._last_streamed_dock_pose_generation
        ):
            return

        position = self._dock_pose.pose.position
        forward_m, right_m, down_m = camera_target_to_body_frd(
            camera_x_m=position.x,
            camera_y_m=position.y,
            camera_z_m=position.z,
            camera_forward_m=float(
                self.get_parameter("landing_target_camera_forward_m").value
            ),
            camera_right_m=float(
                self.get_parameter("landing_target_camera_right_m").value
            ),
            camera_down_m=float(
                self.get_parameter("landing_target_camera_down_m").value
            ),
        )
        sent = self._vehicle.send_landing_target(
            time_usec=measurement_time_usec(
                stamp_sec=self._dock_pose.header.stamp.sec,
                stamp_nanosec=self._dock_pose.header.stamp.nanosec,
                fallback_usec=int(now * 1_000_000),
            ),
            forward_m=forward_m,
            right_m=right_m,
            down_m=down_m,
        )
        if sent:
            self._last_streamed_dock_pose_generation = (
                self._dock_pose_generation
            )
        if sent and self._landing_target_started_at is None:
            self._landing_target_started_at = now

    def _native_target_is_ready(self, now: float) -> bool:
        if self._landing_target_started_at is None:
            return False
        warmup = float(self.get_parameter("landing_target_warmup_s").value)
        return now - self._landing_target_started_at >= warmup

    def _send_native_landing_yaw(self, now: float) -> None:
        if (
            self._landing_backend != "ardupilot"
            or not bool(
                self.get_parameter("landing_yaw_correction_enabled").value
            )
            or self._landing_yaw_command_sent
            or not self._dock_target_is_fresh(now)
        ):
            return
        orientation = self._dock_pose.pose.orientation
        try:
            yaw_error = camera_dock_orientation_to_body_yaw_error(
                quaternion_x=orientation.x,
                quaternion_y=orientation.y,
                quaternion_z=orientation.z,
                quaternion_w=orientation.w,
            )
        except ValueError as exception:
            self.get_logger().warning(str(exception))
            return

        if self._vehicle.send_condition_yaw(yaw_error_rad=yaw_error):
            self._landing_yaw_command_sent = True
            self.get_logger().info(
                "Sent MAV_CMD_CONDITION_YAW during ArduPilot LAND; "
                "current drone-to-dock yaw error="
                f"{math.degrees(yaw_error):.1f} deg"
            )

    def _dock_body_velocity(self) -> Tuple[float, float]:
        position = self._dock_pose.pose.position
        return camera_offset_to_body_velocity(
            camera_x_m=position.x,
            camera_y_m=position.y,
            gain=float(self.get_parameter("dock_horizontal_gain").value),
            max_speed_m_s=float(
                self.get_parameter("dock_max_horizontal_speed_m_s").value
            ),
        )

    def _alignment_is_stable(
        self, now: float, horizontal_error: float, tolerance: float
    ) -> bool:
        if horizontal_error > tolerance:
            self._alignment_started_at = None
            return False
        if self._alignment_started_at is None:
            self._alignment_started_at = now
        stable_time = float(
            self.get_parameter("dock_alignment_stable_time_s").value
        )
        return now - self._alignment_started_at >= stable_time

    def _align_dock(self, now: float) -> None:
        velocity_x, velocity_y = self._dock_body_velocity()
        self._publish_velocity(
            velocity_x, velocity_y, 0.0, frame_id="base_link"
        )

        distance = self._dock_pose.pose.position.z
        transition_distance = float(
            self.get_parameter("dock_central_transition_distance_m").value
        )
        tolerance = float(
            self.get_parameter(
                "dock_fine_tolerance_m"
                if distance <= transition_distance
                else "dock_coarse_tolerance_m"
            ).value
        )
        if not self._alignment_is_stable(
            now, self._dock_horizontal_error(), tolerance
        ):
            return

        if distance > transition_distance:
            self._set_state(MissionState.DESCEND_OUTER)
        elif self._dock_mode == "CENTRAL":
            self._set_state(MissionState.DESCEND_CENTRAL)

    def _descend_outer(self, now: float) -> None:
        if not self._dock_target_is_fresh(now):
            self._publish_velocity(0.0, 0.0, 0.0, frame_id="base_link")
            self._set_state(MissionState.ACQUIRE_DOCK)
            return

        distance = self._dock_pose.pose.position.z
        transition_distance = float(
            self.get_parameter("dock_central_transition_distance_m").value
        )
        if distance <= transition_distance:
            self._publish_velocity(0.0, 0.0, 0.0, frame_id="base_link")
            self._set_state(MissionState.ALIGN_DOCK)
            return

        velocity_x, velocity_y = self._dock_body_velocity()
        tolerance = float(
            self.get_parameter("dock_coarse_tolerance_m").value
        )
        descent_speed = 0.0
        if self._dock_horizontal_error() <= tolerance:
            descent_speed = -float(
                self.get_parameter("dock_outer_descent_speed_m_s").value
            )
        self._publish_velocity(
            velocity_x, velocity_y, descent_speed, frame_id="base_link"
        )

    def _descend_central(self, now: float) -> None:
        if not self._dock_target_is_fresh(now) or self._dock_mode != "CENTRAL":
            self._publish_velocity(0.0, 0.0, 0.0, frame_id="base_link")
            self._set_state(MissionState.ACQUIRE_DOCK)
            return

        velocity_x, velocity_y = self._dock_body_velocity()
        horizontal_error = self._dock_horizontal_error()
        fine_tolerance = float(
            self.get_parameter("dock_fine_tolerance_m").value
        )
        aligned = self._alignment_is_stable(
            now, horizontal_error, fine_tolerance
        )
        distance = self._dock_pose.pose.position.z
        land_distance = float(
            self.get_parameter("dock_land_switch_distance_m").value
        )
        if aligned and distance <= land_distance:
            self._publish_velocity(0.0, 0.0, 0.0, frame_id="base_link")
            self._set_state(MissionState.SET_LAND)
            return

        descent_speed = 0.0
        if horizontal_error <= fine_tolerance:
            descent_speed = -float(
                self.get_parameter("dock_central_descent_speed_m_s").value
            )
        self._publish_velocity(
            velocity_x, velocity_y, descent_speed, frame_id="base_link"
        )

    def _fly_circle(self, dt: float) -> None:
        angular_speed = (
            self._mission.circle_speed_m_s / self._mission.circle_radius_m
        )
        self._circle_progress += angular_speed * dt
        total_angle = 2.0 * math.pi * self._mission.circle_laps
        if self._circle_progress >= total_angle:
            self._set_state(MissionState.RETURN_TO_DOCK)
            return

        direction = -1.0 if self._mission.circle_clockwise else 1.0
        angle = direction * self._circle_progress
        radius = self._mission.circle_radius_m
        center_x, center_y = self._mission_xy(
            self._mission.circle_center_x_m,
            self._mission.circle_center_y_m,
        )
        target_x = center_x + radius * math.cos(angle)
        target_y = center_y + radius * math.sin(angle)
        feedforward_x = -direction * self._mission.circle_speed_m_s * math.sin(angle)
        feedforward_y = direction * self._mission.circle_speed_m_s * math.cos(angle)
        correction = self._position_velocity(
            (target_x, target_y, self._mission.takeoff_altitude_m)
        )
        velocity = self._limit_velocity(
            feedforward_x + correction[0],
            feedforward_y + correction[1],
            correction[2],
        )
        self._publish_velocity(*velocity)

    def _mission_xy(self, x_m: float, y_m: float) -> Tuple[float, float]:
        """Resolve horizontal coordinates against the start pose."""
        if self._mission_origin is None:
            raise RuntimeError("mission origin was not captured before flight")
        return self._mission.resolve_xy(x_m, y_m, *self._mission_origin)

    def _fly_to(self, target: Tuple[float, float, float]) -> bool:
        velocity = self._position_velocity(target)
        self._publish_velocity(*velocity)
        position = self._pose.pose.position
        horizontal_error = math.hypot(target[0] - position.x, target[1] - position.y)
        vertical_error = abs(target[2] - position.z)
        return (
            horizontal_error
            <= float(self.get_parameter("position_tolerance_m").value)
            and vertical_error
            <= float(self.get_parameter("altitude_tolerance_m").value)
        )

    def _position_velocity(
        self, target: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        position = self._pose.pose.position
        gain = float(self.get_parameter("position_gain").value)
        velocity_x = gain * (target[0] - position.x)
        velocity_y = gain * (target[1] - position.y)
        velocity_z = gain * (target[2] - position.z)
        return self._limit_velocity(velocity_x, velocity_y, velocity_z)

    def _limit_velocity(
        self, velocity_x: float, velocity_y: float, velocity_z: float
    ) -> Tuple[float, float, float]:
        max_horizontal = float(
            self.get_parameter("max_horizontal_speed_m_s").value
        )
        horizontal_speed = math.hypot(velocity_x, velocity_y)
        if horizontal_speed > max_horizontal:
            scale = max_horizontal / horizontal_speed
            velocity_x *= scale
            velocity_y *= scale
        max_vertical = float(self.get_parameter("max_vertical_speed_m_s").value)
        velocity_z = max(-max_vertical, min(max_vertical, velocity_z))
        return velocity_x, velocity_y, velocity_z

    def _publish_velocity(
        self,
        velocity_x: float,
        velocity_y: float,
        velocity_z: float,
        frame_id: str = "map",
    ) -> None:
        frame = (
            VelocityFrame.BODY_FLU
            if frame_id == "base_link"
            else VelocityFrame.MAP_ENU
        )
        self._vehicle.send_velocity(
            frame,
            velocity_x,
            velocity_y,
            velocity_z,
            now=time.monotonic(),
        )

    def destroy_node(self):
        """Stop direct commands and close the pymavlink connection."""
        if getattr(self, "_vehicle", None) is not None:
            self._vehicle.close()
            self._vehicle = None
        return super().destroy_node()


def main(args=None) -> None:
    """Run the mission manager."""
    rclpy.init(args=args)
    node = None
    try:
        node = MissionManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as exception:
        if node is not None and not isinstance(exception, KeyboardInterrupt):
            node.get_logger().fatal(str(exception))
        elif not isinstance(exception, KeyboardInterrupt):
            print(f"mission_manager: {exception}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass

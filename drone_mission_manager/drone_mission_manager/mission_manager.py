"""Non-blocking circle mission and camera-guided precision landing manager."""

import math
import time
from enum import Enum, auto
from typing import Callable, Optional, Tuple

import rclpy
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .control import camera_offset_to_body_velocity
from .landing_target import (
    MAV_CMD_CONDITION_YAW,
    MAV_CMD_DO_AUX_FUNCTION,
    MavlinkLandingTargetSender,
    camera_dock_orientation_to_body_yaw_error,
    camera_target_to_body_frd,
    measurement_time_usec,
    ros_enu_quaternion_to_mavlink_heading_deg,
)
from .landing_quality import (
    CircularYawWindow,
    DescendingCheckpoint,
    FreshMeasurementWindow,
    StableCondition,
    surface_relative_height,
    target_loss_requires_retry,
    yaw_turn_can_finish_above,
)
from .landing_wait import (
    LandingWaitAction,
    disarm_wait_action,
    touchdown_wait_action,
)
from .mission import CircleMission, load_mission


GUIDED_MODE = 4
LOITER_MODE = 5
LAND_MODE = 9
PRECISION_LOITER_AUX_FUNCTION = 39
AUX_SWITCH_LOW = 0
AUX_SWITCH_HIGH = 2
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5


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
    ALIGN_DOCK_YAW = auto()
    SET_PRECISION_LOITER = auto()
    ENABLE_PRECISION_LOITER = auto()
    PRECISION_LOITER_ALIGN = auto()
    RECOVERY_DISABLE_PRECISION_LOITER = auto()
    RECOVERY_SET_LOITER = auto()
    RECOVERY_ACQUIRE = auto()
    SET_GUIDED_DOCK_ALIGN = auto()
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

    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("position_gain", 0.7)
        self.declare_parameter("max_horizontal_speed_m_s", 1.5)
        self.declare_parameter("max_vertical_speed_m_s", 0.7)
        self.declare_parameter("position_tolerance_m", 0.35)
        self.declare_parameter("altitude_tolerance_m", 0.25)
        self.declare_parameter("pose_timeout_s", 1.0)
        self.declare_parameter("service_timeout_s", 10.0)
        self.declare_parameter("prearm_timeout_s", 60.0)
        self.declare_parameter("prearm_retry_period_s", 1.0)
        self.declare_parameter("dock_pose_timeout_s", 0.35)
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
        self.declare_parameter("landing_target_min_rate_hz", 5.0)
        self.declare_parameter("landing_target_max_gap_s", 0.25)
        self.declare_parameter("landing_coarse_horizontal_tolerance_m", 0.5)
        self.declare_parameter("landing_yaw_window_s", 1.0)
        self.declare_parameter("landing_yaw_jitter_max_deg", 3.0)
        self.declare_parameter("landing_initial_yaw_rate_deg_s", 7.5)
        self.declare_parameter("landing_initial_yaw_tolerance_deg", 5.0)
        self.declare_parameter("landing_initial_yaw_stable_time_s", 1.0)
        self.declare_parameter(
            "precision_loiter_horizontal_tolerance_m", 0.15
        )
        self.declare_parameter("precision_loiter_stable_time_s", 1.5)
        self.declare_parameter("precision_loiter_yaw_realign_time_s", 0.5)
        self.declare_parameter("landing_central_stable_time_s", 0.75)
        self.declare_parameter("landing_reacquisition_quiet_time_s", 1.0)
        self.declare_parameter("landing_checkpoint_confirm_time_s", 0.15)
        self.declare_parameter("landing_yaw_3m_threshold_deg", 8.0)
        self.declare_parameter("landing_yaw_3m_max_correction_deg", 15.0)
        self.declare_parameter("landing_yaw_3m_rate_deg_s", 5.0)
        self.declare_parameter("landing_yaw_3m_horizontal_max_m", 0.20)
        self.declare_parameter("landing_yaw_2m_threshold_deg", 6.0)
        self.declare_parameter("landing_yaw_2m_max_correction_deg", 10.0)
        self.declare_parameter("landing_yaw_2m_rate_deg_s", 5.0)
        self.declare_parameter("landing_yaw_2m_horizontal_max_m", 0.15)
        self.declare_parameter("landing_yaw_hard_limit_deg", 15.0)
        self.declare_parameter("landing_validation_yaw_max_deg", 8.0)
        self.declare_parameter("landing_validation_horizontal_max_m", 0.15)
        self.declare_parameter("landing_height_timeout_s", 0.35)
        self.declare_parameter("landing_descent_speed_m_s", 0.25)
        self.declare_parameter("landing_final_yaw_clearance_margin_m", 0.20)
        self.declare_parameter("landing_final_target_loss_action", "continue")
        self.declare_parameter("mavlink_command_timeout_s", 3.0)
        self.declare_parameter("mavlink_command_retry_s", 0.5)
        self.declare_parameter("landing_target_bind_address", "0.0.0.0")
        self.declare_parameter("landing_target_udp_port", 14551)
        self.declare_parameter("landing_target_source_system", 254)
        self.declare_parameter("landing_target_source_component", 191)
        self.declare_parameter("landing_target_camera_forward_m", 0.0)
        self.declare_parameter("landing_target_camera_right_m", 0.0)
        self.declare_parameter("landing_target_camera_down_m", 0.19)
        self.declare_parameter("prearm_service", "/ap/v1/prearm_check")
        self.declare_parameter("mode_service", "/ap/v1/mode_switch")
        self.declare_parameter("arm_service", "/ap/v1/arm_motors")
        self.declare_parameter(
            "takeoff_service", "/ap/v1/experimental/takeoff"
        )
        self.declare_parameter(
            "get_parameters_service", "/ap/v1/get_parameters"
        )
        self.declare_parameter("pose_topic", "/ap/v1/pose/filtered")
        self.declare_parameter("status_topic", "/ap/v1/status")
        self.declare_parameter("velocity_topic", "/ap/v1/cmd_vel")
        self.declare_parameter("dock_pose_topic", "/dock/pose")
        self.declare_parameter("dock_mode_topic", "/dock/mode")
        self.declare_parameter(
            "dock_filter_status_topic", "/dock/filter_status"
        )

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
        self._pose_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._pose_callback,
            sensor_qos,
        )
        self._status_subscription = self.create_subscription(
            Status,
            str(self.get_parameter("status_topic").value),
            self._status_callback,
            sensor_qos,
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
        self._dock_filter_status_subscription = self.create_subscription(
            String,
            str(self.get_parameter("dock_filter_status_topic").value),
            self._dock_filter_status_callback,
            sensor_qos,
        )
        self._velocity_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("velocity_topic").value),
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

        self._prearm_client = self.create_client(
            Trigger, str(self.get_parameter("prearm_service").value)
        )
        self._mode_client = self.create_client(
            ModeSwitch, str(self.get_parameter("mode_service").value)
        )
        self._arm_client = self.create_client(
            ArmMotors, str(self.get_parameter("arm_service").value)
        )
        self._takeoff_client = self.create_client(
            Takeoff, str(self.get_parameter("takeoff_service").value)
        )
        self._get_parameters_client = self.create_client(
            GetParameters,
            str(self.get_parameter("get_parameters_service").value),
        )

        self._landing_target_sender = None
        if self._landing_backend == "ardupilot":
            try:
                self._landing_target_sender = MavlinkLandingTargetSender(
                    bind_address=str(
                        self.get_parameter(
                            "landing_target_bind_address"
                        ).value
                    ),
                    port=int(
                        self.get_parameter("landing_target_udp_port").value
                    ),
                    source_system=int(
                        self.get_parameter(
                            "landing_target_source_system"
                        ).value
                    ),
                    source_component=int(
                        self.get_parameter(
                            "landing_target_source_component"
                        ).value
                    ),
                )
            except OSError as exception:
                raise ValueError(
                    "could not open MAVLink LANDING_TARGET UDP endpoint: "
                    f"{exception}"
                ) from exception

        self._pose: Optional[PoseStamped] = None
        self._pose_received_at = 0.0
        self._mission_origin: Optional[Tuple[float, float]] = None
        self._status: Optional[Status] = None
        self._has_seen_armed = False
        self._dock_pose: Optional[PoseStamped] = None
        self._dock_pose_received_at = 0.0
        self._dock_pose_generation = 0
        self._last_streamed_dock_pose_generation = -1
        self._last_streamed_source_stamp_ns: Optional[int] = None
        self._dock_mode = "NO_TARGET"
        self._dock_mode_received_at = 0.0
        self._dock_mode_since_at = 0.0
        self._last_filter_reacquisition_at = 0.0
        self._pending = None
        self._pending_name = ""
        self._pending_deadline = 0.0
        self._next_recovery_mode_at = 0.0
        self._next_prearm_at = 0.0
        self._circle_progress = 0.0
        self._iteration = 1
        self._repeat_at = 0.0
        self._alignment_started_at: Optional[float] = None
        self._landing_stream = FreshMeasurementWindow(
            required_duration_s=float(
                self.get_parameter("landing_target_warmup_s").value
            ),
            minimum_rate_hz=float(
                self.get_parameter("landing_target_min_rate_hz").value
            ),
            maximum_gap_s=float(
                self.get_parameter("landing_target_max_gap_s").value
            ),
            maximum_age_s=float(
                self.get_parameter("dock_pose_timeout_s").value
            ),
        )
        self._dock_yaw_window = CircularYawWindow(
            maximum_gap_s=float(
                self.get_parameter("landing_target_max_gap_s").value
            ),
            maximum_age_s=float(
                self.get_parameter("dock_pose_timeout_s").value
            ),
        )
        self._initial_yaw_stable = StableCondition(
            required_duration_s=float(
                self.get_parameter("landing_initial_yaw_stable_time_s").value
            )
        )
        self._precision_loiter_stable = StableCondition(
            required_duration_s=float(
                self.get_parameter("precision_loiter_stable_time_s").value
            )
        )
        self._precision_loiter_yaw_bad = StableCondition(
            required_duration_s=float(
                self.get_parameter("precision_loiter_yaw_realign_time_s").value
            )
        )
        checkpoint_confirmation = float(
            self.get_parameter("landing_checkpoint_confirm_time_s").value
        )
        self._landing_3m_checkpoint = DescendingCheckpoint(
            threshold_m=3.0,
            confirmation_duration_s=checkpoint_confirmation,
        )
        self._landing_2m_checkpoint = DescendingCheckpoint(
            threshold_m=2.0,
            confirmation_duration_s=checkpoint_confirmation,
        )
        self._landing_1m_checkpoint = DescendingCheckpoint(
            threshold_m=1.0,
            confirmation_duration_s=checkpoint_confirmation,
        )
        self._alignment_yaw_command_sent = False
        self._landing_retrying = False
        self._final_yaw_locked = False
        self._mavlink_command_id: Optional[int] = None
        self._mavlink_command_label = ""
        self._mavlink_command_sender: Optional[Callable[[], bool]] = None
        self._mavlink_command_deadline = 0.0
        self._mavlink_command_next_send_at = 0.0
        self._mavlink_command_fatal = True
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
            "landing_target_min_rate_hz",
            "landing_target_max_gap_s",
            "landing_coarse_horizontal_tolerance_m",
            "landing_yaw_window_s",
            "landing_yaw_jitter_max_deg",
            "landing_initial_yaw_rate_deg_s",
            "landing_initial_yaw_tolerance_deg",
            "landing_initial_yaw_stable_time_s",
            "precision_loiter_horizontal_tolerance_m",
            "precision_loiter_stable_time_s",
            "precision_loiter_yaw_realign_time_s",
            "landing_central_stable_time_s",
            "landing_reacquisition_quiet_time_s",
            "landing_checkpoint_confirm_time_s",
            "landing_yaw_3m_threshold_deg",
            "landing_yaw_3m_max_correction_deg",
            "landing_yaw_3m_rate_deg_s",
            "landing_yaw_3m_horizontal_max_m",
            "landing_yaw_2m_threshold_deg",
            "landing_yaw_2m_max_correction_deg",
            "landing_yaw_2m_rate_deg_s",
            "landing_yaw_2m_horizontal_max_m",
            "landing_yaw_hard_limit_deg",
            "landing_validation_yaw_max_deg",
            "landing_validation_horizontal_max_m",
            "landing_height_timeout_s",
            "landing_descent_speed_m_s",
            "landing_final_yaw_clearance_margin_m",
            "mavlink_command_timeout_s",
            "mavlink_command_retry_s",
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
        port = int(self.get_parameter("landing_target_udp_port").value)
        if not 1 <= port <= 65535:
            raise ValueError("landing_target_udp_port must be in [1, 65535]")
        for name in (
            "landing_target_source_system",
            "landing_target_source_component",
        ):
            value = int(self.get_parameter(name).value)
            if not 1 <= value <= 255:
                raise ValueError(f"{name} must be in [1, 255]")
        target_gap = float(
            self.get_parameter("landing_target_max_gap_s").value
        )
        pose_timeout = float(self.get_parameter("dock_pose_timeout_s").value)
        if target_gap > pose_timeout:
            raise ValueError(
                "landing_target_max_gap_s must not exceed "
                "dock_pose_timeout_s"
            )
        final_loss_action = str(
            self.get_parameter("landing_final_target_loss_action").value
        ).lower()
        if final_loss_action not in ("continue", "retry"):
            raise ValueError(
                "landing_final_target_loss_action must be 'continue' or "
                "'retry'"
            )

    def _pose_callback(self, message: PoseStamped) -> None:
        self._pose = message
        self._pose_received_at = time.monotonic()

    def _status_callback(self, message: Status) -> None:
        self._status = message
        self._has_seen_armed = self._has_seen_armed or message.armed

    def _dock_pose_callback(self, message: PoseStamped) -> None:
        now = time.monotonic()
        self._dock_pose = message
        self._dock_pose_received_at = now
        self._dock_pose_generation += 1
        orientation = message.pose.orientation
        try:
            yaw_error = camera_dock_orientation_to_body_yaw_error(
                quaternion_x=orientation.x,
                quaternion_y=orientation.y,
                quaternion_z=orientation.z,
                quaternion_w=orientation.w,
            )
        except ValueError as exception:
            self.get_logger().warning(
                str(exception), throttle_duration_sec=1.0
            )
            self._dock_yaw_window.reset()
        else:
            self._dock_yaw_window.observe(now, yaw_error)

    def _dock_mode_callback(self, message: String) -> None:
        now = time.monotonic()
        if message.data != self._dock_mode:
            self._dock_mode = message.data
            self._dock_mode_since_at = now
            self._reset_landing_measurement_evidence()
        self._dock_mode_received_at = now

    def _dock_filter_status_callback(self, message: String) -> None:
        if message.data not in (
            "initialized",
            "reset_after_gap",
            "reacquired",
        ):
            return
        self._last_filter_reacquisition_at = time.monotonic()
        self._reset_landing_measurement_evidence()

    def _reset_landing_measurement_evidence(self) -> None:
        """Clear evidence that cannot span a source/filter discontinuity."""
        self._landing_stream.reset()
        self._dock_yaw_window.reset()
        self._initial_yaw_stable.reset()
        self._precision_loiter_stable.reset()
        self._precision_loiter_yaw_bad.reset()

    def _set_state(self, state: MissionState) -> None:
        if state == self._state:
            return
        self._state = state
        self._state_started_at = time.monotonic()
        self._alignment_started_at = None
        if state == MissionState.ACQUIRE_DOCK:
            self._landing_retrying = False
            self._reset_landing_measurement_evidence()
        elif state == MissionState.ALIGN_DOCK_YAW:
            self._alignment_yaw_command_sent = False
            self._initial_yaw_stable.reset()
        elif state == MissionState.PRECISION_LOITER_ALIGN:
            self._precision_loiter_stable.reset()
            self._precision_loiter_yaw_bad.reset()
        if state == MissionState.WAIT_LANDED:
            self._landing_3m_checkpoint.reset()
            self._landing_2m_checkpoint.reset()
            self._landing_1m_checkpoint.reset()
            self._final_yaw_locked = False
            self._landing_timeout_warned = False
        elif state == MissionState.WAIT_DISARM:
            self._disarm_timeout_warned = False
        self._publish_state()
        self.get_logger().info(f"Mission state: {state.name}")

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.name
        self._state_publisher.publish(message)

    def _services_ready(self) -> bool:
        clients = [
            self._prearm_client,
            self._mode_client,
            self._arm_client,
            self._takeoff_client,
        ]
        if self._landing_backend == "ardupilot":
            clients.append(self._get_parameters_client)
        return all(
            client.service_is_ready()
            for client in clients
        )

    def _pose_is_fresh(self, now: float) -> bool:
        timeout = float(self.get_parameter("pose_timeout_s").value)
        return (
            self._pose is not None
            and now - self._pose_received_at <= timeout
        )

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
        timeout = float(self.get_parameter("service_timeout_s").value)
        self._pending_deadline = time.monotonic() + timeout

    def _begin_mavlink_command(
        self,
        *,
        command_id: int,
        label: str,
        send: Callable[[], bool],
        now: float,
        fatal: bool,
    ) -> None:
        """Start an ACK-tracked, idempotent MAVLink command."""
        sender = self._landing_target_sender
        if sender is None:
            self._fail(f"cannot send {label}: MAVLink endpoint is disabled")
            return
        while sender.take_command_ack(command_id) is not None:
            pass
        self._mavlink_command_id = command_id
        self._mavlink_command_label = label
        self._mavlink_command_sender = send
        self._mavlink_command_deadline = now + float(
            self.get_parameter("mavlink_command_timeout_s").value
        )
        self._mavlink_command_next_send_at = now
        self._mavlink_command_fatal = fatal

    def _clear_mavlink_command(self) -> None:
        self._mavlink_command_id = None
        self._mavlink_command_label = ""
        self._mavlink_command_sender = None
        self._mavlink_command_deadline = 0.0
        self._mavlink_command_next_send_at = 0.0

    def _update_mavlink_command(self, now: float) -> str:
        """Drive the active command and return idle/pending/accepted/failed."""
        command_id = self._mavlink_command_id
        sender = self._landing_target_sender
        if command_id is None or sender is None:
            return "idle"

        acknowledgement = sender.take_command_ack(command_id)
        if acknowledgement == MAV_RESULT_ACCEPTED:
            label = self._mavlink_command_label
            self._clear_mavlink_command()
            self.get_logger().info(f"ArduPilot accepted {label}")
            return "accepted"
        if acknowledgement not in (None, MAV_RESULT_IN_PROGRESS):
            label = self._mavlink_command_label
            fatal = self._mavlink_command_fatal
            self._clear_mavlink_command()
            message = (
                f"ArduPilot rejected {label} with MAV_RESULT="
                f"{acknowledgement}"
            )
            if fatal:
                self._fail(message)
            else:
                self.get_logger().warning(message)
            return "failed"

        if now > self._mavlink_command_deadline:
            label = self._mavlink_command_label
            fatal = self._mavlink_command_fatal
            self._clear_mavlink_command()
            message = f"timed out waiting for ArduPilot ACK for {label}"
            if fatal:
                self._fail(message)
            else:
                self.get_logger().warning(message)
            return "failed"

        if acknowledgement != MAV_RESULT_IN_PROGRESS:
            if now >= self._mavlink_command_next_send_at:
                send = self._mavlink_command_sender
                if send is not None:
                    send()
                self._mavlink_command_next_send_at = now + float(
                    self.get_parameter("mavlink_command_retry_s").value
                )
        return "pending"

    def _fail(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._pending = None
        self._clear_mavlink_command()
        self._publish_guided_hold()
        self._set_state(MissionState.FAILED)

    def _publish_guided_hold(self) -> None:
        """Publish a zero velocity only when GUIDED will consume it."""
        if self._status is not None and self._status.mode == GUIDED_MODE:
            self._publish_velocity(0.0, 0.0, 0.0, frame_id="base_link")

    def _handle_pending_call(self, now: float) -> bool:
        if self._pending is None:
            return False
        if now > self._pending_deadline:
            self._fail(f"{self._pending_name} service call timed out")
            return True
        if not self._pending.done():
            return True

        try:
            result = self._pending.result()
        except Exception as exception:
            self._fail(
                f"{self._pending_name} service call failed: {exception}"
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
            if result.curr_mode != GUIDED_MODE:
                self._fail("ArduPilot rejected GUIDED mode")
            else:
                self._set_state(MissionState.ARM)
        elif name == "arm":
            if not result.result:
                self._fail("ArduPilot rejected the arm request")
            else:
                self._set_state(MissionState.TAKEOFF)
        elif name == "takeoff":
            if not result.status:
                self._fail("ArduPilot rejected the takeoff request")
            else:
                self._set_state(MissionState.WAIT_TAKEOFF)
        elif name == "land_mode":
            if result.curr_mode != LAND_MODE:
                if result.curr_mode == LOITER_MODE:
                    self.get_logger().warning(
                        "ArduPilot rejected LAND; remaining in Precision "
                        "Loiter and revalidating X/Y and yaw"
                    )
                    self._set_state(MissionState.PRECISION_LOITER_ALIGN)
                else:
                    self._fail("ArduPilot rejected LAND mode")
            else:
                sender = self._landing_target_sender
                if sender is not None:
                    sender.send_aux_function(
                        function=PRECISION_LOITER_AUX_FUNCTION,
                        switch_position=AUX_SWITCH_LOW,
                    )
                self._set_state(MissionState.WAIT_LANDED)
        elif name == "precision_loiter_mode":
            if result.curr_mode != LOITER_MODE:
                if result.curr_mode == GUIDED_MODE:
                    self.get_logger().warning(
                        "ArduPilot rejected LOITER pre-alignment; "
                        "revalidating GUIDED yaw before retry"
                    )
                    self._set_state(MissionState.ALIGN_DOCK_YAW)
                else:
                    self._fail(
                        "ArduPilot rejected LOITER pre-alignment mode"
                    )
            else:
                self._set_state(MissionState.ENABLE_PRECISION_LOITER)
        elif name == "recovery_loiter_mode":
            if result.curr_mode != LOITER_MODE:
                self._next_recovery_mode_at = now + float(
                    self.get_parameter("mavlink_command_retry_s").value
                )
                self.get_logger().warning(
                    "ArduPilot has not entered LOITER recovery mode; "
                    "retrying the mode request"
                )
            else:
                self._reset_landing_measurement_evidence()
                self._set_state(MissionState.RECOVERY_ACQUIRE)
        elif name == "guided_dock_align_mode":
            if result.curr_mode != GUIDED_MODE:
                if result.curr_mode == LOITER_MODE:
                    self.get_logger().warning(
                        "ArduPilot rejected GUIDED yaw alignment; remaining "
                        "in LOITER and reacquiring before retry"
                    )
                    self._reset_landing_measurement_evidence()
                    self._set_state(MissionState.RECOVERY_ACQUIRE)
                else:
                    self._fail(
                        "ArduPilot rejected GUIDED yaw-alignment mode"
                    )
            else:
                self._set_state(MissionState.ALIGN_DOCK_YAW)
        elif name == "landing_config":
            values = result.values
            parameter_names = (
                "PLND_ENABLED",
                "PLND_TYPE",
                "PLND_EST_TYPE",
                "PLND_XY_DIST_MAX",
                "LAND_SPD_MS",
                "PLND_ALT_MIN",
                "PLND_ALT_MAX",
            )
            configured = {
                parameter_name: (
                    self._parameter_value_as_number(values[index])
                    if index < len(values)
                    else None
                )
                for index, parameter_name in enumerate(parameter_names)
            }
            invalid = {
                name: configured[name]
                for name, expected in (
                    ("PLND_ENABLED", 1.0),
                    ("PLND_TYPE", 1.0),
                    ("PLND_EST_TYPE", 0.0),
                    ("PLND_ALT_MIN", 0.0),
                    ("PLND_ALT_MAX", 0.0),
                )
                if configured[name] != expected
            }
            if invalid:
                self._fail(
                    "ArduPilot native precision landing is not configured: "
                    f"invalid parameters={invalid}; restart the simulation "
                    "with the updated no-rangefinder X500 defaults"
                )
            else:
                xy_max = configured["PLND_XY_DIST_MAX"]
                land_speed = configured["LAND_SPD_MS"]
                if xy_max is None or xy_max > 0.25:
                    self.get_logger().warning(
                        "Recommended PLND_XY_DIST_MAX is 0.25 m or less; "
                        f"current value is {xy_max}"
                    )
                if (
                    land_speed is None
                    or land_speed < 0.20
                    or land_speed > 0.25
                ):
                    self.get_logger().warning(
                        "Recommended LAND_SPD_MS is 0.20-0.25 m/s; current "
                        f"value is {land_speed}"
                    )
                self._set_state(MissionState.PREARM)
        return True

    @staticmethod
    def _parameter_value_as_number(value) -> Optional[float]:
        if value.type == ParameterType.PARAMETER_INTEGER:
            return float(value.integer_value)
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return float(value.double_value)
        return None

    def _update(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self._last_update_at, 0.0), 0.5)
        self._last_update_at = now

        # USED TO SEND DOCK POSE TO PRECISION LANDING VIA MAVLINK
        self._stream_native_landing_target(now)

        if self._handle_pending_call(now):
            return
        if self._state == MissionState.IDLE:
            return
        if self._state == MissionState.FAILED:
            return
        if self._state == MissionState.REPEAT_DELAY:
            if now >= self._repeat_at:
                self._start_next_iteration(now)
            return
        if self._state == MissionState.WAITING_FOR_AP:
            if self._services_ready() and self._pose_is_fresh(now):
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
            request = GetParameters.Request()
            request.names = [
                "PLND_ENABLED",
                "PLND_TYPE",
                "PLND_EST_TYPE",
                "PLND_XY_DIST_MAX",
                "LAND_SPD_MS",
                "PLND_ALT_MIN",
                "PLND_ALT_MAX",
            ]
            self._begin_call(
                "landing_config",
                self._get_parameters_client.call_async(request),
            )
            return
        if self._state == MissionState.PREARM:
            if now >= self._next_prearm_at:
                self._begin_call(
                    "prearm", self._prearm_client.call_async(Trigger.Request())
                )
            return
        if self._state == MissionState.SET_GUIDED:
            request = ModeSwitch.Request()
            request.mode = GUIDED_MODE
            self._begin_call("mode", self._mode_client.call_async(request))
            return
        if self._state == MissionState.ARM:
            request = ArmMotors.Request()
            request.arm = True
            self._begin_call("arm", self._arm_client.call_async(request))
            return
        if self._state == MissionState.TAKEOFF:
            request = Takeoff.Request()
            request.alt = self._mission.takeoff_altitude_m
            self._begin_call(
                "takeoff", self._takeoff_client.call_async(request)
            )
            return
        if self._state == MissionState.SET_PRECISION_LOITER:
            request = ModeSwitch.Request()
            request.mode = LOITER_MODE
            self._begin_call(
                "precision_loiter_mode",
                self._mode_client.call_async(request),
            )
            return
        if self._state == MissionState.ENABLE_PRECISION_LOITER:
            if self._mavlink_command_id is None:
                sender = self._landing_target_sender
                self._begin_mavlink_command(
                    command_id=MAV_CMD_DO_AUX_FUNCTION,
                    label="Precision Loiter auxiliary function 39 HIGH",
                    send=lambda: sender.send_aux_function(
                        function=PRECISION_LOITER_AUX_FUNCTION,
                        switch_position=AUX_SWITCH_HIGH,
                    ),
                    now=now,
                    fatal=False,
                )
            command_status = self._update_mavlink_command(now)
            if command_status == "accepted":
                self._set_state(MissionState.PRECISION_LOITER_ALIGN)
            elif command_status == "failed":
                self._begin_native_recovery(
                    "Precision Loiter enable command was not acknowledged"
                )
            return
        if self._state == MissionState.RECOVERY_DISABLE_PRECISION_LOITER:
            self._clear_mavlink_command()
            sender = self._landing_target_sender
            if sender is not None and not sender.send_aux_function(
                function=PRECISION_LOITER_AUX_FUNCTION,
                switch_position=AUX_SWITCH_LOW,
            ):
                self.get_logger().warning(
                    "Could not confirm a MAVLink peer while disabling "
                    "Precision Loiter; requesting LOITER recovery anyway"
                )
            self._set_state(MissionState.RECOVERY_SET_LOITER)
            return
        if self._state == MissionState.RECOVERY_SET_LOITER:
            if now < self._next_recovery_mode_at:
                return
            request = ModeSwitch.Request()
            request.mode = LOITER_MODE
            self._begin_call(
                "recovery_loiter_mode", self._mode_client.call_async(request)
            )
            return
        if self._state == MissionState.SET_GUIDED_DOCK_ALIGN:
            request = ModeSwitch.Request()
            request.mode = GUIDED_MODE
            self._begin_call(
                "guided_dock_align_mode",
                self._mode_client.call_async(request),
            )
            return
        if self._state == MissionState.SET_LAND:
            request = ModeSwitch.Request()
            request.mode = LAND_MODE
            self._begin_call(
                "land_mode", self._mode_client.call_async(request)
            )
            return
        if self._state == MissionState.WAIT_LANDED:
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
                return
            elif action == LandingWaitAction.TOUCHDOWN:
                self._set_state(MissionState.WAIT_DISARM)
                self.get_logger().info(
                    "ArduPilot reports touchdown; waiting for automatic disarm"
                )
                return
            elif action == LandingWaitAction.WARN:
                self._landing_timeout_warned = True
                timeout = float(
                    self.get_parameter("dock_land_timeout_s").value
                )
                self.get_logger().warning(
                    f"LAND has not reported touchdown after {timeout:.1f}s; "
                    "continuing precision-landing target stream and waiting"
                )
            if self._landing_backend == "ardupilot":
                self._monitor_native_land(now)
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
                self._mission.dock_surface_z_m
                + self._mission.hover_altitude_m,
            )
            if self._fly_to(target):
                if self._mission.precision_landing_enabled:
                    self._set_state(MissionState.ACQUIRE_DOCK)
                    self.get_logger().info(
                        "Coarse dock approach complete; acquiring visual "
                        "target"
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
                    self._mission.dock_surface_z_m
                    + self._mission.hover_altitude_m,
                )
            )
            return
        if self._state == MissionState.ACQUIRE_DOCK:
            self._publish_guided_hold()
            if self._dock_target_is_fresh(now):
                if self._landing_backend == "ros":
                    self._set_state(MissionState.ALIGN_DOCK)
                elif self._native_coarse_target_is_ready(now):
                    self._set_state(MissionState.ALIGN_DOCK_YAW)
                    self.get_logger().info(
                        "Fresh OUTER LANDING_TARGET stream acquired; "
                        "starting GUIDED dock-yaw alignment"
                    )
            if now - self._state_started_at > float(
                self.get_parameter("dock_acquire_timeout_s").value
            ):
                if self._landing_backend == "ardupilot":
                    self.get_logger().warning(
                        "Holding the GUIDED coarse hover until a continuous "
                        "fresh OUTER target stream is available",
                        throttle_duration_sec=10.0,
                    )
                else:
                    self._fail("dock target was not acquired before timeout")
            return
        if self._state == MissionState.ALIGN_DOCK_YAW:
            if not self._alignment_yaw_command_sent:
                self._publish_guided_hold()
            self._update_native_yaw_alignment(now)
            return
        if self._state == MissionState.PRECISION_LOITER_ALIGN:
            self._update_precision_loiter_alignment(now)
            return
        if self._state == MissionState.RECOVERY_ACQUIRE:
            if self._native_coarse_target_is_ready(now):
                self._set_state(MissionState.SET_GUIDED_DOCK_ALIGN)
                self.get_logger().info(
                    "Dock target reacquired in LOITER; returning to GUIDED "
                    "for yaw alignment"
                )
            else:
                self.get_logger().warning(
                    "Holding LOITER while waiting for a fresh dock target",
                    throttle_duration_sec=10.0,
                )
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
            "Precision landing complete after "
            f"{self._iteration} iteration(s); "
            "vehicle is disarmed on the dock"
        )

    def _start_next_iteration(self, now: float) -> None:
        """Reset transient flight state and begin the next configured cycle."""
        self._pending = None
        self._pending_name = ""
        self._next_prearm_at = 0.0
        self._circle_progress = 0.0
        self._alignment_started_at = None
        self._reset_landing_measurement_evidence()
        self._clear_mavlink_command()
        self._alignment_yaw_command_sent = False
        self._landing_retrying = False
        self._final_yaw_locked = False
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
        sender = self._landing_target_sender
        if sender is None:
            return
        sender.poll()
        active_states = (
            MissionState.ACQUIRE_DOCK,
            MissionState.ALIGN_DOCK_YAW,
            MissionState.SET_PRECISION_LOITER,
            MissionState.ENABLE_PRECISION_LOITER,
            MissionState.PRECISION_LOITER_ALIGN,
            MissionState.RECOVERY_DISABLE_PRECISION_LOITER,
            MissionState.RECOVERY_SET_LOITER,
            MissionState.RECOVERY_ACQUIRE,
            MissionState.SET_GUIDED_DOCK_ALIGN,
            MissionState.SET_LAND,
            MissionState.WAIT_LANDED,
            MissionState.WAIT_DISARM,
        )
        source_stable = self._dock_source_is_stable(
            now, accepted_modes=("OUTER", "CENTRAL")
        )
        if (
            self._state not in active_states
            or not self._dock_target_is_fresh(now)
            or not source_stable
        ):
            if self._state in active_states:
                self._landing_stream.reset()
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
        source_stamp_ns = (
            int(self._dock_pose.header.stamp.sec) * 1_000_000_000
            + int(self._dock_pose.header.stamp.nanosec)
        )
        if (
            source_stamp_ns > 0
            and self._last_streamed_source_stamp_ns is not None
            and source_stamp_ns <= self._last_streamed_source_stamp_ns
        ):
            return
        measurement_usec = measurement_time_usec(
            stamp_sec=self._dock_pose.header.stamp.sec,
            stamp_nanosec=self._dock_pose.header.stamp.nanosec,
            fallback_usec=int(now * 1_000_000),
        )
        sent = sender.send(
            time_usec=measurement_usec,
            forward_m=forward_m,
            right_m=right_m,
            down_m=down_m,
        )
        if sent:
            self._last_streamed_dock_pose_generation = (
                self._dock_pose_generation
            )
            if source_stamp_ns > 0:
                self._last_streamed_source_stamp_ns = source_stamp_ns
            self._landing_stream.observe(now)

    def _native_target_is_ready(self, now: float) -> bool:
        return self._landing_stream.ready(now_s=now)

    def _dock_source_is_stable(
        self, now: float, *, accepted_modes: Tuple[str, ...]
    ) -> bool:
        return (
            self._dock_mode in accepted_modes
            and now - self._dock_mode_since_at
            >= float(
                self.get_parameter("landing_central_stable_time_s").value
            )
        )

    def _filter_is_quiet(self, now: float) -> bool:
        return now - self._last_filter_reacquisition_at >= float(
            self.get_parameter("landing_reacquisition_quiet_time_s").value
        )

    def _stable_yaw_estimate(self, now: float):
        estimate = self._dock_yaw_window.estimate(
            now_s=now,
            required_duration_s=float(
                self.get_parameter("landing_yaw_window_s").value
            ),
            minimum_rate_hz=float(
                self.get_parameter("landing_target_min_rate_hz").value
            ),
        )
        if estimate is None:
            return None
        maximum_jitter = math.radians(
            float(self.get_parameter("landing_yaw_jitter_max_deg").value)
        )
        return estimate if estimate.jitter_rad <= maximum_jitter else None

    def _current_yaw_error(self) -> Optional[float]:
        if self._dock_pose is None:
            return None
        orientation = self._dock_pose.pose.orientation
        try:
            return camera_dock_orientation_to_body_yaw_error(
                quaternion_x=orientation.x,
                quaternion_y=orientation.y,
                quaternion_z=orientation.z,
                quaternion_w=orientation.w,
            )
        except ValueError:
            return None

    def _native_coarse_target_is_ready(self, now: float) -> bool:
        accepted_modes = (
            ("OUTER", "CENTRAL") if self._landing_retrying else ("OUTER",)
        )
        return (
            self._dock_target_is_fresh(now)
            and self._native_target_is_ready(now)
            and self._dock_source_is_stable(
                now, accepted_modes=accepted_modes
            )
            and self._dock_horizontal_error()
            < float(
                self.get_parameter(
                    "landing_coarse_horizontal_tolerance_m"
                ).value
            )
        )

    def _absolute_heading_for_yaw_error(self, yaw_error_rad: float) -> float:
        orientation = self._pose.pose.orientation
        current_heading = ros_enu_quaternion_to_mavlink_heading_deg(
            quaternion_x=orientation.x,
            quaternion_y=orientation.y,
            quaternion_z=orientation.z,
            quaternion_w=orientation.w,
        )
        return (current_heading + math.degrees(yaw_error_rad)) % 360.0

    def _update_native_yaw_alignment(self, now: float) -> None:
        if self._status is None or self._status.mode != GUIDED_MODE:
            if now - self._state_started_at > 1.0:
                self._fail(
                    "vehicle is not in GUIDED during dock-yaw alignment"
                )
            return
        accepted_modes = (
            ("OUTER", "CENTRAL") if self._landing_retrying else ("OUTER",)
        )
        target_valid = (
            self._dock_target_is_fresh(now)
            and self._native_target_is_ready(now)
            and self._dock_source_is_stable(
                now, accepted_modes=accepted_modes
            )
        )
        if not target_valid:
            self._clear_mavlink_command()
            if self._landing_retrying:
                self._begin_native_recovery(
                    "dock target lost during recovery yaw alignment"
                )
            else:
                self._set_state(MissionState.ACQUIRE_DOCK)
            return

        tolerance_deg = float(
            self.get_parameter("landing_initial_yaw_tolerance_deg").value
        )
        if not self._alignment_yaw_command_sent:
            estimate = self._stable_yaw_estimate(now)
            if estimate is None:
                return
            if (
                abs(math.degrees(estimate.mean_rad)) > tolerance_deg
                and bool(
                    self.get_parameter(
                        "landing_yaw_correction_enabled"
                    ).value
                )
            ):
                sender = self._landing_target_sender
                heading_deg = self._absolute_heading_for_yaw_error(
                    estimate.mean_rad
                )
                yaw_rate = float(
                    self.get_parameter(
                        "landing_initial_yaw_rate_deg_s"
                    ).value
                )
                self._begin_mavlink_command(
                    command_id=MAV_CMD_CONDITION_YAW,
                    label="initial dock yaw alignment",
                    send=lambda: sender.send_condition_yaw_absolute(
                        heading_deg=heading_deg,
                        yaw_rate_deg_s=yaw_rate,
                    ),
                    now=now,
                    fatal=False,
                )
                self._alignment_yaw_command_sent = True

        if self._mavlink_command_id is not None:
            self._update_mavlink_command(now)
            if self._state == MissionState.FAILED:
                return

        yaw_error = self._current_yaw_error()
        stable_estimate = self._stable_yaw_estimate(now)
        accepted = (
            yaw_error is not None
            and abs(math.degrees(yaw_error)) < tolerance_deg
            and stable_estimate is not None
            and abs(math.degrees(stable_estimate.mean_rad)) < tolerance_deg
            and self._dock_target_is_fresh(now)
        )
        if self._initial_yaw_stable.update(now_s=now, accepted=accepted):
            self._clear_mavlink_command()
            self._set_state(MissionState.SET_PRECISION_LOITER)
            self.get_logger().info(
                "Dock yaw remained within tolerance; entering LOITER "
                "pre-alignment"
            )
            return
        if now - self._state_started_at > float(
            self.get_parameter("dock_acquire_timeout_s").value
        ):
            self._fail(
                "dock yaw did not converge before the alignment timeout"
            )

    def _update_precision_loiter_alignment(self, now: float) -> None:
        if not self._dock_target_is_fresh(now):
            self._begin_native_recovery(
                "dock target became stale during Precision Loiter"
            )
            return
        if self._status is None or self._status.mode != LOITER_MODE:
            if now - self._state_started_at > 1.0:
                self._begin_native_recovery(
                    "vehicle left LOITER during precision pre-alignment"
                )
            return

        horizontal_error = self._dock_horizontal_error()
        yaw_error = self._current_yaw_error()
        stable_estimate = self._stable_yaw_estimate(now)
        yaw_tolerance = float(
            self.get_parameter("landing_initial_yaw_tolerance_deg").value
        )
        horizontal_tolerance = float(
            self.get_parameter(
                "precision_loiter_horizontal_tolerance_m"
            ).value
        )
        yaw_bad_while_centered = (
            horizontal_error < horizontal_tolerance
            and yaw_error is not None
            and abs(math.degrees(yaw_error)) >= yaw_tolerance
            and stable_estimate is not None
        )
        if self._precision_loiter_yaw_bad.update(
            now_s=now, accepted=yaw_bad_while_centered
        ):
            self._begin_native_recovery(
                "yaw left tolerance after Precision Loiter centered X/Y"
            )
            return

        accepted = (
            self._native_target_is_ready(now)
            and (
                self._dock_source_is_stable(
                    now, accepted_modes=("OUTER",)
                )
                or self._dock_source_is_stable(
                    now, accepted_modes=("CENTRAL",)
                )
            )
            and horizontal_error < horizontal_tolerance
            and yaw_error is not None
            and abs(math.degrees(yaw_error)) < yaw_tolerance
            and stable_estimate is not None
            and abs(math.degrees(stable_estimate.mean_rad)) < yaw_tolerance
            and self._filter_is_quiet(now)
        )
        if self._precision_loiter_stable.update(
            now_s=now, accepted=accepted
        ):
            self._set_state(MissionState.SET_LAND)
            self.get_logger().info(
                "Precision Loiter X/Y and yaw are stable; switching directly "
                "to native precision LAND"
            )

    def _begin_native_recovery(self, reason: str) -> None:
        if self._state in (
            MissionState.RECOVERY_DISABLE_PRECISION_LOITER,
            MissionState.RECOVERY_SET_LOITER,
            MissionState.RECOVERY_ACQUIRE,
            MissionState.SET_GUIDED_DOCK_ALIGN,
        ):
            return
        self._landing_retrying = True
        self._reset_landing_measurement_evidence()
        self.get_logger().warning(
            f"Native landing retry: {reason}; disabling Precision Loiter "
            "before requesting LOITER"
        )
        self._set_state(MissionState.RECOVERY_DISABLE_PRECISION_LOITER)

    def _height_above_dock(self, now: float) -> Optional[float]:
        if (
            self._pose is None
            or now - self._pose_received_at
            > float(
                self.get_parameter("landing_height_timeout_s").value
            )
        ):
            return None
        return surface_relative_height(
            self._pose.pose.position.z,
            self._mission.dock_surface_z_m,
        )

    def _monitor_native_land(self, now: float) -> None:
        """Apply one-shot yaw/validation checkpoints during native LAND."""
        height_m = self._height_above_dock(now)
        target_fresh = self._dock_target_is_fresh(now)
        if not target_fresh:
            self._landing_stream.reset()
            final_action = str(
                self.get_parameter(
                    "landing_final_target_loss_action"
                ).value
            ).lower()
            clearance_height = 1.0 + float(
                self.get_parameter(
                    "landing_final_yaw_clearance_margin_m"
                ).value
            )
            if not target_loss_requires_retry(
                height_m=height_m,
                clearance_height_m=clearance_height,
                policy=final_action,
            ):
                self.get_logger().warning(
                    "Dock pose lost in the final low-clearance region; "
                    "continuing LAND without stale target data or new yaw "
                    "commands",
                    throttle_duration_sec=2.0,
                )
                return
            self._begin_native_recovery(
                "dock pose exceeded the short ROS freshness timeout"
            )
            return
        if height_m is None:
            self._begin_native_recovery(
                "flight-controller height estimate became stale"
            )
            return

        if self._mavlink_command_id is not None:
            self._update_mavlink_command(now)

        valid_height = math.isfinite(height_m)
        crossed_3m = self._landing_3m_checkpoint.update(
            now_s=now, height_m=height_m, valid=valid_height
        )
        crossed_2m = self._landing_2m_checkpoint.update(
            now_s=now, height_m=height_m, valid=valid_height
        )
        crossed_1m = self._landing_1m_checkpoint.update(
            now_s=now, height_m=height_m, valid=valid_height
        )

        if crossed_1m:
            self._validate_final_metre(now)
            return
        if self._final_yaw_locked:
            return
        if crossed_2m:
            self._evaluate_landing_yaw_checkpoint(
                now=now, height_m=height_m, checkpoint="2m"
            )
            return
        if crossed_3m:
            self._evaluate_landing_yaw_checkpoint(
                now=now, height_m=height_m, checkpoint="3m"
            )

    def _evaluate_landing_yaw_checkpoint(
        self, *, now: float, height_m: float, checkpoint: str
    ) -> None:
        if not bool(
            self.get_parameter("landing_yaw_correction_enabled").value
        ):
            return
        if self._mavlink_command_id is not None:
            self.get_logger().warning(
                f"Skipping {checkpoint} yaw checkpoint because another "
                "MAVLink command is unresolved"
            )
            return

        estimate = self._stable_yaw_estimate(now)
        if estimate is None:
            self.get_logger().warning(
                f"Skipping {checkpoint} yaw correction: target yaw is not "
                "stable"
            )
            return
        yaw_error_deg = math.degrees(estimate.mean_rad)
        horizontal_error = self._dock_horizontal_error()

        if checkpoint == "3m":
            source_ready = self._dock_source_is_stable(
                now, accepted_modes=("OUTER",)
            )
            threshold_deg = float(
                self.get_parameter("landing_yaw_3m_threshold_deg").value
            )
            maximum_correction_deg = float(
                self.get_parameter(
                    "landing_yaw_3m_max_correction_deg"
                ).value
            )
            yaw_rate_deg_s = float(
                self.get_parameter("landing_yaw_3m_rate_deg_s").value
            )
            horizontal_limit_m = float(
                self.get_parameter(
                    "landing_yaw_3m_horizontal_max_m"
                ).value
            )
        else:
            central_ready = self._dock_source_is_stable(
                now, accepted_modes=("CENTRAL",)
            )
            source_ready = central_ready or self._dock_source_is_stable(
                now, accepted_modes=("OUTER",)
            )
            threshold_deg = float(
                self.get_parameter("landing_yaw_2m_threshold_deg").value
            )
            maximum_correction_deg = float(
                self.get_parameter(
                    "landing_yaw_2m_max_correction_deg"
                ).value
            )
            yaw_rate_deg_s = float(
                self.get_parameter("landing_yaw_2m_rate_deg_s").value
            )
            horizontal_limit_m = float(
                self.get_parameter(
                    "landing_yaw_2m_horizontal_max_m"
                ).value
            )
            if not self._filter_is_quiet(now):
                self.get_logger().warning(
                    "Skipping 2m yaw correction after recent filter "
                    "reacquisition"
                )
                return
            hard_limit_deg = float(
                self.get_parameter("landing_yaw_hard_limit_deg").value
            )
            if source_ready and abs(yaw_error_deg) > hard_limit_deg:
                self._begin_native_recovery(
                    f"2m yaw error {yaw_error_deg:.1f} deg exceeds the hard "
                    f"limit of {hard_limit_deg:.1f} deg"
                )
                return

        if (
            not source_ready
            or horizontal_error >= horizontal_limit_m
            or abs(yaw_error_deg) <= threshold_deg
        ):
            return

        correction_deg = max(
            -maximum_correction_deg,
            min(maximum_correction_deg, yaw_error_deg),
        )
        if not yaw_turn_can_finish_above(
            height_m=height_m,
            floor_height_m=1.0,
            correction_deg=correction_deg,
            yaw_rate_deg_s=yaw_rate_deg_s,
            descent_speed_m_s=float(
                self.get_parameter("landing_descent_speed_m_s").value
            ),
            clearance_margin_m=float(
                self.get_parameter(
                    "landing_final_yaw_clearance_margin_m"
                ).value
            ),
        ):
            self.get_logger().warning(
                f"Skipping {checkpoint} yaw correction because the turn "
                "cannot finish above the final yaw-lock region"
            )
            return

        sender = self._landing_target_sender
        heading_deg = self._absolute_heading_for_yaw_error(
            math.radians(correction_deg)
        )
        self._begin_mavlink_command(
            command_id=MAV_CMD_CONDITION_YAW,
            label=f"{checkpoint} dock yaw checkpoint",
            send=lambda: sender.send_condition_yaw_absolute(
                heading_deg=heading_deg,
                yaw_rate_deg_s=yaw_rate_deg_s,
            ),
            now=now,
            fatal=False,
        )
        self._update_mavlink_command(now)
        self.get_logger().info(
            f"Started {checkpoint} yaw correction of {correction_deg:.1f} "
            f"deg at {yaw_rate_deg_s:.1f} deg/s"
        )

    def _validate_final_metre(self, now: float) -> None:
        yaw_error = self._current_yaw_error()
        stable_estimate = self._stable_yaw_estimate(now)
        yaw_limit_deg = float(
            self.get_parameter("landing_validation_yaw_max_deg").value
        )
        accepted = (
            self._dock_target_is_fresh(now)
            and self._dock_source_is_stable(
                now, accepted_modes=("CENTRAL",)
            )
            and stable_estimate is not None
            and yaw_error is not None
            and abs(math.degrees(yaw_error)) < yaw_limit_deg
            and abs(math.degrees(stable_estimate.mean_rad)) < yaw_limit_deg
            and self._dock_horizontal_error()
            < float(
                self.get_parameter(
                    "landing_validation_horizontal_max_m"
                ).value
            )
        )
        if accepted:
            self._final_yaw_locked = True
            self.get_logger().info(
                "Final-metre validation passed; yaw is locked and only "
                "ArduPilot precision-landing X/Y correction remains active"
            )
            return
        self._begin_native_recovery(
            "1m CENTRAL yaw/X/Y validation failed"
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
        feedforward_x = (
            -direction * self._mission.circle_speed_m_s * math.sin(angle)
        )
        feedforward_y = (
            direction * self._mission.circle_speed_m_s * math.cos(angle)
        )
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
            raise RuntimeError(
                "mission origin was not captured before flight"
            )
        return self._mission.resolve_xy(x_m, y_m, *self._mission_origin)

    def _fly_to(self, target: Tuple[float, float, float]) -> bool:
        velocity = self._position_velocity(target)
        self._publish_velocity(*velocity)
        position = self._pose.pose.position
        horizontal_error = math.hypot(
            target[0] - position.x, target[1] - position.y
        )
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
        max_vertical = float(
            self.get_parameter("max_vertical_speed_m_s").value
        )
        velocity_z = max(-max_vertical, min(max_vertical, velocity_z))
        return velocity_x, velocity_y, velocity_z

    def _publish_velocity(
        self,
        velocity_x: float,
        velocity_y: float,
        velocity_z: float,
        frame_id: str = "map",
    ) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = frame_id
        command.twist.linear.x = velocity_x
        command.twist.linear.y = velocity_y
        command.twist.linear.z = velocity_z
        self._velocity_publisher.publish(command)

    def destroy_node(self):
        """Release the optional MAVLink socket before destroying the node."""
        if self._landing_target_sender is not None:
            self._landing_target_sender.close()
            self._landing_target_sender = None
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

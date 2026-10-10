"""Non-blocking validated flight-cycle start, docking handoff, and fallback."""

from __future__ import annotations

import math
import json
import time
from datetime import datetime
from dataclasses import dataclass, replace
from enum import Enum, auto
from typing import Any, Mapping, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .dock_search import (
    DockSearchController,
    SearchConfig,
    SearchObservation,
    SearchOutcome,
)
from .landing_backend import (
    DockObservation,
    DockTargetMode,
    LandingBackendOutcome,
    VehicleObservation,
    VelocityFrame,
)
from .landing_target import (
    camera_dock_orientation_to_body_yaw_error,
    camera_target_to_body_frd,
    measurement_time_usec,
)
from .landing_attempt_log import LandingAttemptLog
from .touchdown_trace import TouchdownTrace
from .mission_contract import (
    MissionContract,
    MissionContractError,
    MissionValidationConfig,
    validate_docking_mission,
)
from .pid import PIDConfig
from .pid_landing import PID_HOLD_STATES, PidLandingConfig, PidLandingState
from .pid_landing_backend import PidLandingBackend
from .precise_landing_backend import (
    PreciseLandingBackend,
    PreciseLandingConfig,
)
from .pymavlink_vehicle import MavlinkVehicle, VehicleCommandResult, VehicleState


# ArduCopter custom modes used by the lifecycle.
AUTO_MODE = 3
AUTO_RTL_MODE = 27
GUIDED_MODE = 4
LAND_MODE = 9
MISSION_STATE_ACTIVE = 3
MISSION_TYPE_MISSION = 0


class CycleState(Enum):
    """All lifecycle states, including the later docking/authority slices."""

    READY = auto()
    WAITING_LAND_TRIGGER = auto()
    VALIDATING_MISSION = auto()
    INVALID_ONBOARD_MISSION = auto()
    PREARM = auto()
    SET_GUIDED = auto()
    ARM = auto()
    TAKEOFF = auto()
    WAIT_TAKEOFF_STABLE = auto()
    RESET_ONBOARD_MISSION = auto()
    START_AUTO = auto()
    AUTO_MISSION = auto()
    WAIT_FOR_HANDOFF = auto()
    GUIDED_TAKEOVER = auto()
    SEARCH_DOCK = auto()
    LANDING_BACKEND = auto()
    RETURN_TO_FC = auto()
    FC_FALLBACK_LAND = auto()
    WAIT_LANDED = auto()
    WAIT_DISARM = auto()
    DOCKED = auto()
    PILOT_OVERRIDE = auto()
    LINK_LOSS = auto()
    FAILED_DISARMED = auto()


_AIRBORNE_AUTHORITY_STATES = frozenset(
    {
        CycleState.SET_GUIDED,
        CycleState.ARM,
        CycleState.TAKEOFF,
        CycleState.WAIT_TAKEOFF_STABLE,
        CycleState.RESET_ONBOARD_MISSION,
        CycleState.START_AUTO,
        CycleState.AUTO_MISSION,
        CycleState.WAIT_FOR_HANDOFF,
        CycleState.GUIDED_TAKEOVER,
        CycleState.SEARCH_DOCK,
        CycleState.LANDING_BACKEND,
        CycleState.RETURN_TO_FC,
        CycleState.FC_FALLBACK_LAND,
        CycleState.WAIT_LANDED,
    }
)


@dataclass
class _PendingOperation:
    """One correlated MAVLink transaction owned by the timer state machine."""

    name: str
    future: Any
    deadline: float


class FlightCycleManager(Node):
    """Own one explicit takeoff-to-stored-mission transition at a time.

    MAVLink transactions are initiated in ``tick`` and completed by later
    ticks. The service callback only admits a new request and begins a read-
    only mission download; it neither waits nor sends flight commands.
    """

    def __init__(
        self,
        *,
        vehicle: Optional[MavlinkVehicle] = None,
        parameter_values: Optional[Mapping[str, object]] = None,
    ) -> None:
        overrides = [
            Parameter(name=name, value=value)
            for name, value in (parameter_values or {}).items()
        ]
        super().__init__("flight_cycle_manager", parameter_overrides=overrides)
        self._declare_parameters()

        self._landing_backend = str(
            self.get_parameter("landing_backend").value
        ).strip().lower()
        if self._landing_backend not in {"pid", "precise_landing"}:
            self.destroy_node()
            raise ValueError("landing_backend must be 'pid' or 'precise_landing'")
        self._simulation_mode = bool(
            self.get_parameter("simulation_mode").value
        )
        self._auto_start = bool(self.get_parameter("auto_start").value)
        if self._auto_start and not self._simulation_mode:
            self.destroy_node()
            raise ValueError("auto_start=true requires simulation_mode=true")
        self._operation_mode = str(
            self.get_parameter("operation_mode").value
        ).strip().lower()
        if self._operation_mode not in {
            "flight_cycle",
            "pid_landing_debug",
            "pid_align_debug",
        }:
            self.destroy_node()
            raise ValueError(
                "operation_mode must be 'flight_cycle', 'pid_landing_debug', "
                "or 'pid_align_debug'"
            )
        if (
            self._operation_mode in {"pid_landing_debug", "pid_align_debug"}
            and self._landing_backend != "pid"
        ):
            self.destroy_node()
            raise ValueError("PID debug modes require landing_backend=pid")
        hold_name = str(self.get_parameter("pid_hold_at_state").value).strip().upper()
        allowed_holds = {state.name for state in PID_HOLD_STATES}
        if hold_name != "NONE" and hold_name not in allowed_holds:
            self.destroy_node()
            raise ValueError("pid_hold_at_state must be none or one of " + ", ".join(sorted(allowed_holds)))
        if hold_name != "NONE" and self._landing_backend != "pid":
            self.destroy_node()
            raise ValueError("pid_hold_at_state requires landing_backend=pid")
        if self._operation_mode == "pid_align_debug" and hold_name not in {"NONE", "PID_ALIGN"}:
            self.destroy_node()
            raise ValueError("pid_align_debug can only hold at PID_ALIGN")
        fixed_mode = str(self.get_parameter("pid_fixed_central_reference_mode").value).strip().lower()
        if fixed_mode not in ("off", "descend_central", "hold_central"):
            self.destroy_node()
            raise ValueError("invalid pid_fixed_central_reference_mode")
        if fixed_mode != "off" and self._landing_backend != "pid":
            self.destroy_node()
            raise ValueError("fixed central reference requires landing_backend=pid")
        self._pid_hold_at_state = (
            None if hold_name == "NONE" else PidLandingState[hold_name]
        )
        self._pid_align_auto_start = bool(
            self.get_parameter("pid_align_auto_start").value
        )

        self._vehicle = vehicle or MavlinkVehicle(
            connection=str(self.get_parameter("mavlink_connection").value),
            baud=int(self.get_parameter("mavlink_baud").value),
            target_system=int(
                self.get_parameter("mavlink_target_system").value
            ),
            target_component=int(
                self.get_parameter("mavlink_target_component").value
            ),
            source_system=int(
                self.get_parameter("mavlink_source_system").value
            ),
            source_component=int(
                self.get_parameter("mavlink_source_component").value
            ),
        )
        update_rate_hz = self._positive_parameter("update_rate_hz")
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
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
        self._state_publisher = self.create_publisher(
            String,
            "~/state",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._reason_publisher = self.create_publisher(
            String,
            "~/reason",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._pid_debug_publisher = self.create_publisher(
            String, "~/pid_debug", 10
        )
        self._parameter_callback = self.add_on_set_parameters_callback(
            self._on_parameters_changed
        )
        self._start_service = self.create_service(
            Trigger, "~/start_cycle", self._start_cycle_callback
        )
        self._request_dock_service = self.create_service(
            Trigger, "~/request_dock", self._request_dock_callback
        )
        self._timer = self.create_timer(1.0 / update_rate_hz, self._timer_callback)
        self._state = CycleState.READY
        self._reason = "initializing"
        self._pending: Optional[_PendingOperation] = None
        self._contract: Optional[MissionContract] = None
        self._start_requested = False
        self._stable_takeoff_since: Optional[float] = None
        self._takeoff_reference_started_at: Optional[float] = None
        self._takeoff_target_local_z_m: Optional[float] = None
        self._takeoff_airborne_deadline: Optional[float] = None
        self._mission_confirmation_deadline: Optional[float] = None
        self._mission_confirmation_expected_seq: Optional[int] = None
        self._auto_heartbeat_baseline: Optional[float] = None
        self._mission_current_baseline: Optional[float] = None
        self._validated_mission_total: Optional[int] = None
        self._validated_mission_id: Optional[int] = None
        self._dock_request_queued = False
        self._dock_request_acknowledged = False
        self._guided_handoff_retry_count = 0
        self._guided_handoff_origin_mode: Optional[int] = None
        self._guided_search_telemetry_deadline: Optional[float] = None
        self._search_controller: Optional[Any] = None
        self._landing_controller: Optional[Any] = None
        self._landing_expected_modes: frozenset[int] = frozenset()
        self._fc_fallback_expected_modes: frozenset[int] = frozenset()
        self._deferred_command: Optional[str] = None
        self._deferred_command_reason = ""
        self._dock_pose: Optional[PoseStamped] = None
        self._dock_pose_received_at = 0.0
        self._dock_pose_generation = 0
        self._last_streamed_dock_pose_generation = -1
        self._dock_mode = "NO_TARGET"
        self._dock_mode_received_at = 0.0
        self._pending_pid_disarm_request_id: Optional[int] = None
        self._pending_pid_disarm_timeout_notified = False
        self._debug_land_trigger_armed = False
        self._pid_align_auto_start_pending = (
            self._operation_mode == "pid_align_debug"
            and self._pid_align_auto_start
        )
        self._landing_attempt_log = LandingAttemptLog(
            enabled=bool(
                self.get_parameter("landing_attempt_log_enabled").value
            ),
            directory=str(
                self.get_parameter("landing_attempt_log_directory").value
            ),
            warn=self.get_logger().warning,
        )
        self._touchdown_trace = TouchdownTrace(
            enabled=bool(self.get_parameter("touchdown_trace_enabled").value),
            directory=str(self.get_parameter("landing_attempt_log_directory").value),
            warn=self.get_logger().warning,
        )
        self._last_landing_debug_event: Optional[tuple[object, ...]] = None
        self._closed = False

        # Startup validation is deliberately read-only. A later cycle request
        # always starts another download, so it never trusts this result.
        if self._operation_mode in {"pid_landing_debug", "pid_align_debug"}:
            reason = (
                "waiting for fresh airborne telemetry to start PID alignment"
                if self._pid_align_auto_start_pending
                else "waiting for a fresh airborne LAND mode transition"
            )
            self._transition(
                CycleState.WAITING_LAND_TRIGGER,
                reason,
            )
        else:
            self._begin_validation(time.monotonic(), start_requested=False)

    @property
    def vehicle(self) -> MavlinkVehicle:
        """Expose the boundary for inspection and deterministic unit tests."""
        return self._vehicle

    @property
    def state(self) -> CycleState:
        """Current authoritative lifecycle state."""
        return self._state

    @property
    def reason(self) -> str:
        """Reason associated with the latest material lifecycle transition."""
        return self._reason

    @property
    def contract(self) -> Optional[MissionContract]:
        """Expose the validated mission indices for deterministic tests."""
        return self._contract

    @property
    def landing_backend(self) -> str:
        """Configured landing backend selected for this node instance."""
        return self._landing_backend

    @property
    def search_controller(self) -> Optional[Any]:
        """Expose the shared search controller while docking is active."""
        return self._search_controller

    @property
    def backend(self) -> Optional[Any]:
        """Expose the active landing backend for deterministic tests."""
        return self._landing_controller

    def _declare_parameters(self) -> None:
        """Declare lifecycle, shared-search, and landing backend defaults."""
        defaults = {
            "operation_mode": "flight_cycle",
            "pid_hold_at_state": "none",
            "pid_fixed_central_reference_mode": "off",
            "pid_align_auto_start": False,
            # Placeholder until the alignment controller captures its first dock pose.
            "pid_align_distance_setpoint_m": 0.0,
            "simulation_mode": False,
            "auto_start": False,
            "landing_backend": "pid",
            "update_rate_hz": 20.0,
            "takeoff_altitude_m": 4.0,
            "takeoff_altitude_tolerance_m": 0.25,
            "takeoff_vertical_speed_tolerance_m_s": 0.15,
            "takeoff_stable_time_s": 1.0,
            "takeoff_airborne_timeout_s": 5.0,
            "mission_handoff_min_loiter_s": 5.0,
            "mission_landing_location_tolerance_m": 2.0,
            "mission_download_timeout_s": 10.0,
            "mavlink_operation_timeout_s": 10.0,
            "guided_handoff_retry_attempts": 1,
            "mission_start_confirmation_timeout_s": 5.0,
            "mavlink_heartbeat_timeout_s": 2.5,
            "pose_timeout_s": 1.0,
            "landed_state_timeout_s": 1.0,
            "dock_pose_timeout_s": 0.5,
            "dock_pose_topic": "/dock/pose",
            "dock_mode_topic": "/dock/mode",
            "search_radius_m": 1.0,
            "search_tangential_speed_m_s": 0.30,
            "search_radial_gain": 0.70,
            "search_altitude_gain": 0.70,
            "search_max_horizontal_speed_m_s": 0.50,
            "search_max_vertical_speed_m_s": 0.50,
            "search_target_stable_time_s": 0.50,
            "search_timeout_s": 30.0,
            "search_max_displacement_m": 2.0,
            "landing_yaw_correction_enabled": True,
            "landing_target_warmup_s": 1.0,
            "landing_target_camera_forward_m": 0.0,
            "landing_target_camera_right_m": 0.0,
            "landing_target_camera_down_m": 0.19,
            "landing_touchdown_warn_after_s": 30.0,
            "landing_disarm_warn_after_s": 20.0,
            "landing_attempt_log_enabled": False,
            "touchdown_trace_enabled": False,
            "landing_attempt_log_directory": (
                "~/.ros/flight_cycle_manager/landing_attempts"
            ),
            "pid_tag_stable_time_s": 0.5,
            "pid_central_transition_distance_m": 1.0,
            "pid_central_acquire_timeout_s": 3.0,
            "pid_central_loss_grace_s": 0.3,
            "pid_recovery_climb_m": 0.3,
            "pid_recovery_altitude_tolerance_m": 0.05,
            "pid_max_recovery_attempts": 3,
            "pid_touchdown_commit_height_m": 0.20,
            "pid_touchdown_speed_m_s": 0.05,
            "pid_touchdown_timeout_s": 15.0,
            "pid_landed_state_timeout_s": 1.0,
            "pid_landed_stable_time_s": 0.5,
            "pid_disarm_retry_period_s": 1.0,
            "pid_alignment_stable_time_s": 1.0,
            "pid_outer_horizontal_tolerance_m": 0.2,
            "pid_central_horizontal_tolerance_m": 0.07,
            "pid_outer_yaw_tolerance_rad": 0.2,
            "pid_central_yaw_tolerance_rad": 0.1,
            "pid_touchdown_horizontal_tolerance_m": 0.06,
            "pid_touchdown_yaw_tolerance_rad": 0.08,
            "pid_touchdown_horizontal_speed_tolerance_m_s": 0.05,
            "pid_touchdown_horizontal_speed_stable_time_s": 1.0,
            "pid_max_horizontal_speed_m_s": 0.6,
            "pid_descend_outer_max_horizontal_speed_m_s": 0.6,
            "pid_descend_central_max_horizontal_speed_m_s": 0.30,
            "pid_touchdown_max_horizontal_speed_m_s": 0.12,
            "pid_outer_descent_speed_m_s": 0.3,
            "pid_central_descent_speed_m_s": 0.12,
            "pid_recovery_speed_m_s": 0.2,
            "pid_max_yaw_rate_rad_s": 0.35,
            "pid_target_loss_grace_s" : 0.5,
            "pid_gain_near_distance_m": 0.2,
            "pid_gain_far_distance_m": 6.0,
            "pid_x_k_v": 0.0,
            "pid_y_k_v": 0.0,
            "pid_z_k_v": 0.0,
            "pid_x_kp": 1.0,
            "pid_x_ki": 0.0,
            "pid_x_kd": 0.0,
            "pid_x_integral_limit": 0.5,
            "pid_x_derivative_filter_time_constant_s": 0.1,
            "pid_y_kp": 1.0,
            "pid_y_ki": 0.0,
            "pid_y_kd": 0.0,
            "pid_y_integral_limit": 0.5,
            "pid_y_derivative_filter_time_constant_s": 0.1,
            "pid_z_kp": 1.0,
            "pid_z_ki": 0.0,
            "pid_z_kd": 0.0,
            "pid_z_integral_limit": 0.5,
            "pid_z_derivative_filter_time_constant_s": 0.1,
            "pid_yaw_kp": 1.0,
            "pid_yaw_ki": 0.0,
            "pid_yaw_kd": 0.0,
            "pid_yaw_integral_limit": 0.5,
            "pid_yaw_derivative_filter_time_constant_s": 0.1,
            "mavlink_connection": "tcp:127.0.0.1:5760",
            "mavlink_baud": 921600,
            "mavlink_target_system": 1,
            "mavlink_target_component": 1,
            "mavlink_source_system": 254,
            "mavlink_source_component": 191,
        }
        for side in ("near", "far"):
            for axis in ("x", "y", "z", "yaw"):
                for gain in ("kp", "ki", "kd"):
                    defaults[f"pid_gain_{side}_{axis}_{gain}"] = -1.0
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _start_cycle_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Queue only a fresh mission validation for one explicit cycle."""
        del request
        if self._operation_mode in {"pid_landing_debug", "pid_align_debug"}:
            return self._reject_start(
                response,
                "cycle request rejected: mission lifecycle is disabled in "
                f"{self._operation_mode} mode",
            )
        if self._vehicle.state.armed is True:
            return self._reject_start(response, "cycle request rejected: vehicle is armed")
        if self._pending is not None and self._pending.future.done():
            self._pending = None
        if self._state not in {
            CycleState.READY,
            CycleState.INVALID_ONBOARD_MISSION,
            CycleState.DOCKED,
        }:
            return self._reject_start(
                response,
                f"cycle request rejected: lifecycle is {self._state.name}",
            )
        if self._pending is not None:
            return self._reject_start(
                response,
                "cycle request rejected: previous MAVLink operation is still pending",
            )
        self._begin_validation(time.monotonic(), start_requested=True)
        response.success = True
        response.message = "cycle request accepted for validation"
        return response

    def _request_dock_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Queue one DO_LAND_START request or acknowledge active docking."""
        del request
        if self._state is CycleState.AUTO_MISSION:
            self._dock_request_queued = True
            self._transition(
                CycleState.WAIT_FOR_HANDOFF,
                "dock request accepted; awaiting DO_LAND_START acknowledgement",
            )
            response.success = True
            response.message = "dock request accepted"
            return response
        if self._state in {
            CycleState.WAIT_FOR_HANDOFF,
            CycleState.GUIDED_TAKEOVER,
            CycleState.SEARCH_DOCK,
            CycleState.LANDING_BACKEND,
            CycleState.RETURN_TO_FC,
            CycleState.FC_FALLBACK_LAND,
            CycleState.WAIT_LANDED,
            CycleState.WAIT_DISARM,
        }:
            response.success = True
            response.message = "dock request already in progress"
            return response
        self._transition(
            self._state,
            f"dock request rejected: lifecycle is {self._state.name}; "
            "AUTO_MISSION is required",
            force=True,
        )
        response.success = False
        response.message = self._reason
        return response

    def tick(self, now: float) -> None:
        """Advance at most one acknowledged transaction or lifecycle action."""
        self._vehicle.receive_telemetry(now)
        if self._closed:
            return
        if self._operation_mode in {"pid_landing_debug", "pid_align_debug"}:
            if self._advance_pid_landing_debug_lifecycle(now):
                return
        if self._state is CycleState.DOCKED:
            self._advance_passive_pending()
            return
        if self._state in {CycleState.PILOT_OVERRIDE, CycleState.LINK_LOSS}:
            self._advance_passive_state(now)
            return
        if (
            self._operation_mode == "flight_cycle"
            and self._handle_airborne_authority(now)
        ):
            return
        self._vehicle.service_pending(now)
        if self._advance_pending(now):
            return
        if self._advance_deferred_command(now):
            return
        if self._state is CycleState.PREARM:
            if not self._fresh_ground_telemetry(now):
                self._route_failure(
                    "fresh heartbeat, position, and landed state telemetry "
                    "are required before arming",
                    grounded_state=CycleState.FAILED_DISARMED,
                    now=now,
                )
                return
            self._begin_operation("prearm", self._vehicle.prearm_async(now), now)
            return
        if self._state is CycleState.SET_GUIDED:
            self._begin_operation(
                "set_guided",
                self._vehicle.set_mode_async(GUIDED_MODE, now),
                now,
            )
            return
        if self._state is CycleState.ARM:
            if not self._fresh_ground_telemetry(now):
                self._route_failure(
                    "fresh heartbeat, position, and landed state telemetry "
                    "are required before arming",
                    grounded_state=CycleState.FAILED_DISARMED,
                    now=now,
                )
                return
            self._begin_operation("arm", self._vehicle.arm_async(True, now), now)
            return
        if self._state is CycleState.TAKEOFF:
            self._begin_takeoff(now)
            return
        if self._state is CycleState.WAIT_TAKEOFF_STABLE:
            self._update_takeoff_stability(now)
            return
        if self._state is CycleState.RESET_ONBOARD_MISSION:
            self._begin_operation(
                "reset_mission",
                self._vehicle.set_mission_current_async(0, reset=True, now=now),
                now,
            )
            return
        if self._state is CycleState.START_AUTO:
            if self._mission_confirmation_deadline is None:
                self._begin_operation(
                    "start_mission", self._vehicle.start_mission_async(now), now
                )
                return
            if self._mission_progress_confirmed(now):
                self._transition(
                    CycleState.AUTO_MISSION,
                    "AUTO heartbeat and active onboard mission confirmed",
                )
                return
            if now > self._mission_confirmation_deadline:
                state = self._vehicle.state
                self.get_logger().error(
                    "Mission confirmation timeout: "
                    f"mode={state.mode}, "
                    f"seq={state.mission_seq}, "
                    f"expected_seq={self._mission_confirmation_expected_seq}, "
                    f"mission_state={state.mission_state}, "
                    f"mission_mode={state.mission_mode}, "
                    f"heartbeat_at={state.heartbeat_at}, "
                    f"heartbeat_baseline={self._auto_heartbeat_baseline}, "
                    f"mission_current_at={state.mission_current_at}, "
                    f"mission_current_baseline={self._mission_current_baseline}"
                )
                self._command_direct_land("mission confirmation timed out")
            return
        if self._state is CycleState.AUTO_MISSION:
            self._advance_auto_mission(now)
            return
        if self._state is CycleState.WAIT_FOR_HANDOFF:
            self._advance_wait_for_handoff(now)
            return
        if self._state is CycleState.GUIDED_TAKEOVER:
            if self._guided_search_telemetry_deadline is not None:
                self._advance_guided_search_start(now)
                return
            self._begin_guided_handoff(now, reset_retries=True)
            return
        if self._state is CycleState.SEARCH_DOCK:
            self._advance_search(now)
            return
        if self._state is CycleState.LANDING_BACKEND:
            self._advance_landing_backend(now)
            return
        if self._state is CycleState.RETURN_TO_FC:
            self._advance_return_to_fc(now)
            return
        if self._state is CycleState.FC_FALLBACK_LAND:
            self._advance_fc_fallback_land(now)
            return
        if self._state is CycleState.WAIT_LANDED:
            self._advance_wait_landed(now)
            return
        if self._state is CycleState.WAIT_DISARM:
            self._advance_wait_disarm(now)
            return

    def _advance_pid_landing_debug_lifecycle(self, now: float) -> bool:
        """Start, cancel, and re-arm mission-free docking from flight-mode edges."""
        state = self._vehicle.state

        if self._state is CycleState.DOCKED:
            if (
                self._fresh_pid_landing_debug_trigger_state(now)
                and state.mode != LAND_MODE
            ):
                self._debug_land_trigger_armed = True
                self._transition(
                    CycleState.WAITING_LAND_TRIGGER,
                    "vehicle is airborne again; waiting for a LAND mode transition",
                )
            return True

        if self._state is CycleState.WAITING_LAND_TRIGGER:
            if not self._fresh_pid_landing_debug_trigger_state(now):
                self._debug_land_trigger_armed = False
                return True
            if self._pid_align_auto_start_pending:
                self._pid_align_auto_start_pending = False
                self._debug_land_trigger_armed = False
                self._transition(
                    CycleState.GUIDED_TAKEOVER,
                    "fresh airborne telemetry received; requesting GUIDED "
                    "for PID alignment",
                )
                self._begin_guided_handoff(now, reset_retries=True)
                return True
            if state.mode != LAND_MODE:
                self._debug_land_trigger_armed = True
                return True
            if self._debug_land_trigger_armed:
                self._debug_land_trigger_armed = False
                self._transition(
                    CycleState.GUIDED_TAKEOVER,
                    "airborne LAND trigger received; requesting GUIDED for PID docking",
                )
                self._begin_guided_handoff(now, reset_retries=True)
            return True

        if self._state in {
            CycleState.GUIDED_TAKEOVER,
            CycleState.SEARCH_DOCK,
            CycleState.LANDING_BACKEND,
        }:
            if not self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            ):
                self._debug_land_trigger_armed = False
                self._cancel_pid_landing_debug_attempt(
                    "heartbeat became stale; docking attempt cancelled"
                )
                return True
            guided_transition_pending = (
                self._state is CycleState.GUIDED_TAKEOVER
                and (
                    self._operation_mode == "pid_align_debug"
                    or state.mode in {LAND_MODE, GUIDED_MODE}
                )
                and (
                    self._pending_mode() == GUIDED_MODE
                    or self._guided_search_telemetry_deadline is not None
                )
            )
            if (
                state.flying is True
                and state.mode != GUIDED_MODE
                and not guided_transition_pending
            ):
                self._debug_land_trigger_armed = (
                    state.mode != LAND_MODE
                    and self._fresh_pid_landing_debug_trigger_state(now)
                )
                self._cancel_pid_landing_debug_attempt(
                    "flight mode changed away from GUIDED; docking attempt cancelled"
                )
                return True
        return False

    def _cancel_pid_landing_debug_attempt(self, reason: str) -> None:
        self._reset_active_guidance()
        self._cancel_active_operation()
        self._deferred_command = None
        self._deferred_command_reason = ""
        self._transition(CycleState.WAITING_LAND_TRIGGER, reason, force=True)

    def _fresh_pid_landing_debug_trigger_state(self, now: float) -> bool:
        state = self._vehicle.state
        return self._fresh_airborne_telemetry(now) and self._fresh_landed_state(
            now, state
        )

    def close(self) -> None:
        """Cancel timer, release MAVLink once, then destroy the ROS node."""
        if self._closed:
            return
        self._closed = True
        self._finish_landing_attempt_log(
            "SHUTDOWN", self._state.name, "manager closed during landing attempt"
        )
        try:
            self._touchdown_trace.finish("SHUTDOWN", "manager closed during landing attempt")
        except Exception as error:
            self.get_logger().warning(f"Could not finish touchdown trace: {error}")
            self._touchdown_trace.disable()
        self._timer.cancel()
        if self._state in {CycleState.PILOT_OVERRIDE, CycleState.LINK_LOSS}:
            self._vehicle.close_without_output()
        else:
            self._vehicle.close()
        self.destroy_node()

    def _timer_callback(self) -> None:
        self.tick(time.monotonic())

    def _begin_validation(self, now: float, *, start_requested: bool) -> None:
        self._start_requested = start_requested
        self._contract = None
        self._stable_takeoff_since = None
        self._takeoff_reference_started_at = None
        self._takeoff_target_local_z_m = None
        self._takeoff_airborne_deadline = None
        self._mission_confirmation_deadline = None
        self._mission_confirmation_expected_seq = None
        self._auto_heartbeat_baseline = None
        self._mission_current_baseline = None
        self._validated_mission_total = None
        self._validated_mission_id = None
        self._dock_request_queued = False
        self._dock_request_acknowledged = False
        self._search_controller = None
        self._landing_controller = None
        self._landing_expected_modes = frozenset()
        self._fc_fallback_expected_modes = frozenset()
        self._deferred_command = None
        self._deferred_command_reason = ""
        self._last_streamed_dock_pose_generation = -1
        self._pending_pid_disarm_request_id = None
        self._pending_pid_disarm_timeout_notified = False
        self._transition(
            CycleState.VALIDATING_MISSION,
            "downloading onboard mission for validation",
        )
        try:
            self._begin_operation(
                "mission_download",
                self._vehicle.download_mission_async(now),
                now,
                timeout_name="mission_download_timeout_s",
            )
        except Exception as error:
            self._invalid_mission(
                f"onboard mission download could not start: {error}", now
            )

    def _begin_operation(
        self,
        name: str,
        future: Any,
        now: float,
        *,
        timeout_name: str = "mavlink_operation_timeout_s",
    ) -> None:
        if self._pending is not None:
            raise RuntimeError("manager attempted to overlap MAVLink operations")
        timeout = self._positive_parameter(timeout_name)
        self._pending = _PendingOperation(name, future, now + timeout)

    def _advance_pending(self, now: float) -> bool:
        pending = self._pending
        if pending is None:
            return False
        if pending.name == "pid_disarm":
            return self._advance_pid_disarm(now)
        if pending.name == "native_land":
            return self._advance_native_land(now)
        if not pending.future.done():
            if now > pending.deadline:
                self._pending = None
                self._operation_failed(
                    pending.name, "MAVLink operation timed out", now
                )
            return True
        self._pending = None
        try:
            result = pending.future.result()
        except Exception as error:
            self._operation_failed(
                pending.name, f"MAVLink operation failed: {error}", now
            )
            return True
        if not getattr(result, "success", False):
            self._operation_failed(
                pending.name,
                getattr(result, "message", "MAVLink operation was rejected"),
                now,
            )
            return True
        self._operation_succeeded(pending.name, result, now)
        return True

    def _operation_succeeded(self, name: str, result: Any, now: float) -> None:
        if name == "mission_download":
            self._validate_download(result, now)
        elif name == "native_landing_parameters":
            self._validate_native_landing_parameters(result, now)
        elif name == "prearm":
            self._transition(CycleState.SET_GUIDED, "pre-arm checks passed")
        elif name == "set_guided":
            command = self._command_result(result)
            if command.success or command.current_mode == GUIDED_MODE:
                self._transition(CycleState.ARM, "GUIDED mode acknowledged")
            else:
                self._operation_failed("set_guided", command.message, now)
        elif name == "arm":
            self._takeoff_reference_started_at = now
            self._transition(CycleState.TAKEOFF, "arm command acknowledged")
        elif name == "takeoff":
            self._stable_takeoff_since = None
            self._takeoff_airborne_deadline = now + self._positive_parameter(
                "takeoff_airborne_timeout_s"
            )
            self._transition(
                CycleState.WAIT_TAKEOFF_STABLE,
                "vertical takeoff acknowledged; waiting for stable altitude",
            )
        elif name == "reset_mission":
            self._transition(
                CycleState.START_AUTO,
                "onboard mission reset to sequence zero",
            )
        elif name == "request_dock":
            self._dock_request_acknowledged = True
            self._transition(
                CycleState.WAIT_FOR_HANDOFF,
                "DO_LAND_START acknowledged; waiting for landing handoff",
                force=True,
            )
        elif name == "handoff_guided":
            if self._operation_mode == "pid_align_debug":
                self._start_pid_alignment(now, result)
            else:
                self._start_shared_search(now, result)
        elif name == "start_mission":
            state = self._vehicle.state
            self._auto_heartbeat_baseline = state.heartbeat_at
            self._mission_current_baseline = state.mission_current_at
            self._mission_confirmation_expected_seq = 0
            self._mission_confirmation_deadline = now + self._positive_parameter(
                "mission_start_confirmation_timeout_s"
            )
            self._transition(
                CycleState.START_AUTO,
                "onboard mission start acknowledged; awaiting AUTO confirmation",
                force=True,
            )
        elif name == "fallback_select":
            self._transition(
                CycleState.RETURN_TO_FC,
                "validated NAV_LAND selected; awaiting mission restart",
                force=True,
            )
        elif name == "fallback_start":
            state = self._vehicle.state
            self._auto_heartbeat_baseline = state.heartbeat_at
            self._mission_current_baseline = state.mission_current_at
            assert self._contract is not None
            self._mission_confirmation_expected_seq = self._contract.fallback_land_seq
            self._mission_confirmation_deadline = now + self._positive_parameter(
                "mission_start_confirmation_timeout_s"
            )
            self._transition(
                CycleState.RETURN_TO_FC,
                "fallback mission start acknowledged; awaiting AUTO NAV_LAND confirmation",
                force=True,
            )
        elif name == "mission_integrity_rtl":
            self._fc_fallback_expected_modes = frozenset(
                {AUTO_RTL_MODE, LAND_MODE}
            )
            self._transition(
                CycleState.FC_FALLBACK_LAND,
                "mission authority invalidated; RTL acknowledged; waiting for flight controller landing",
                force=True,
            )
        elif name == "direct_land":
            self._fc_fallback_expected_modes = frozenset({LAND_MODE})
            self._transition(
                CycleState.FC_FALLBACK_LAND,
                "direct LAND command acknowledged; waiting for flight controller landing",
                force=True,
            )

    def _operation_failed(self, name: str, message: str, now: float) -> None:
        reason = f"{name.replace('_', ' ')} failed: {message or 'rejected'}"
        if name == "handoff_guided" and self._retry_guided_handoff_if_safe(
            message, now
        ):
            return
        if (
            self._operation_mode == "pid_align_debug"
            and name == "handoff_guided"
        ):
            self._cancel_pid_landing_debug_attempt(reason)
            return
        if name in {"mission_download", "native_landing_parameters"}:
            self._invalid_mission(reason, now)
            return
        if name == "mission_integrity_rtl":
            self._command_direct_land(reason)
            return
        if name in {"fallback_select", "fallback_start"}:
            self._command_direct_land(reason)
            return
        if name == "direct_land":
            self._transition(CycleState.FC_FALLBACK_LAND, reason, force=True)
            return
        self._route_failure(
            reason,
            grounded_state=CycleState.FAILED_DISARMED,
            now=now,
        )

    def _validate_download(self, result: Any, now: float) -> None:
        try:
            contract = validate_docking_mission(
                result.items,
                MissionValidationConfig(
                    min_handoff_loiter_s=self._positive_parameter(
                        "mission_handoff_min_loiter_s", allow_zero=True
                    ),
                    landing_location_tolerance_m=self._positive_parameter(
                        "mission_landing_location_tolerance_m", allow_zero=True
                    ),
                ),
            )
        except (MissionContractError, TypeError, ValueError) as error:
            self._invalid_mission(
                f"onboard mission contract is invalid: {error}", now
            )
            return
        self._contract = contract
        # ArduPilot's streamed MISSION_CURRENT.total follows the mission's last
        # sequence index, not the raw MISSION_COUNT item count.
        self._validated_mission_total = (
            max(int(item.seq) for item in result.items) if result.items else None
        )
        opaque_id = getattr(result, "opaque_id", None)
        self._validated_mission_id = (
            opaque_id
            if isinstance(opaque_id, int)
            and not isinstance(opaque_id, bool)
            and opaque_id != 0
            else None
        )
        if self._landing_backend == "precise_landing":
            self._transition(
                CycleState.VALIDATING_MISSION,
                "validating native precision landing parameters",
                force=True,
            )
            try:
                self._begin_operation(
                    "native_landing_parameters",
                    self._vehicle.get_parameters_async(
                        ("PLND_ENABLED", "PLND_TYPE"), now
                    ),
                    now,
                )
            except Exception as error:
                self._invalid_mission(
                    f"native precision landing parameter read could not start: {error}",
                    now,
                )
            return
        self._validation_succeeded(now)

    def _validate_native_landing_parameters(self, result: Any, now: float) -> None:
        parameters = self._command_result(result).parameters
        enabled = parameters.get("PLND_ENABLED")
        landing_type = parameters.get("PLND_TYPE")
        if enabled != 1.0 or landing_type != 1.0:
            self._invalid_mission(
                "ArduPilot native precision landing is not configured: "
                f"PLND_ENABLED={enabled}, PLND_TYPE={landing_type}",
                now,
            )
            return
        self._validation_succeeded(now)

    def _validation_succeeded(self, now: float) -> None:
        if self._start_requested:
            self._transition(
                CycleState.PREARM,
                "onboard mission validation passed; waiting for fresh pre-arm telemetry",
            )
            return
        if self._auto_start:
            self._auto_start = False
            self._begin_validation(now, start_requested=True)
            return
        self._transition(CycleState.READY, "onboard mission contract is valid")

    def _dock_pose_callback(self, message: PoseStamped) -> None:
        self._dock_pose = message
        self._dock_pose_received_at = time.monotonic()
        self._dock_pose_generation += 1

    def _dock_mode_callback(self, message: String) -> None:
        self._dock_mode = message.data
        self._dock_mode_received_at = time.monotonic()

    def _begin_takeoff(self, now: float) -> None:
        """Translate a climb above the launch position into FC altitude frames."""
        if self._takeoff_reference_started_at is None:
            self._takeoff_reference_started_at = now
        state = self._vehicle.state
        reference_is_fresh = (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter("mavlink_heartbeat_timeout_s"),
            )
            and state.armed is True
            and state.flying is False
            and self._fresh_landed_state(now, state)
            and self._fresh_position(now, state)
            and all(math.isfinite(value) for value in state.position_enu_m)
            and state.position_at > self._takeoff_reference_started_at
            and state.relative_altitude_m is not None
            and math.isfinite(state.relative_altitude_m)
            and state.relative_altitude_at > self._takeoff_reference_started_at
            and 0.0 <= now - state.relative_altitude_at
            <= self._positive_parameter("pose_timeout_s")
        )
        if not reference_is_fresh:
            if now - self._takeoff_reference_started_at >= self._positive_parameter(
                "mavlink_operation_timeout_s"
            ):
                self._command_direct_land("fresh post-arm takeoff altitude reference timed out")
            return

        height_m = self._positive_parameter("takeoff_altitude_m")
        # NAV_TAKEOFF uses height above FC home, whereas LOCAL_POSITION_NED
        # uses the EKF origin. Apply the same climb to each starting altitude.
        self._takeoff_target_local_z_m = state.position_enu_m[2] + height_m
        self._begin_operation(
            "takeoff",
            self._vehicle.takeoff_async(state.relative_altitude_m + height_m, now),
            now,
        )

    def _update_takeoff_stability(self, now: float) -> None:
        state = self._vehicle.state
        if not self._fresh_airborne_telemetry(now):
            if (
                self._takeoff_airborne_deadline is not None
                and now <= self._takeoff_airborne_deadline
            ):
                return
            self._command_direct_land(
                "takeoff telemetry became stale before stable altitude confirmation"
            )
            return
        self._takeoff_airborne_deadline = None
        assert state.position_enu_m is not None
        assert state.velocity_enu_m_s is not None
        if self._takeoff_target_local_z_m is None:
            self._command_direct_land("takeoff altitude reference is unavailable")
            return
        altitude_error = abs(
            state.position_enu_m[2] - self._takeoff_target_local_z_m
        )
        vertical_speed = abs(state.velocity_enu_m_s[2])
        is_stable = (
            altitude_error <= self._positive_parameter(
                "takeoff_altitude_tolerance_m", allow_zero=True
            )
            and vertical_speed <= self._positive_parameter(
                "takeoff_vertical_speed_tolerance_m_s", allow_zero=True
            )
        )
        if not is_stable:
            self._stable_takeoff_since = None
            return
        if self._stable_takeoff_since is None:
            self._stable_takeoff_since = now
            return
        if now - self._stable_takeoff_since < self._positive_parameter(
            "takeoff_stable_time_s", allow_zero=True
        ):
            return
        self._transition(
            CycleState.RESET_ONBOARD_MISSION,
            "takeoff altitude and vertical speed are stable",
        )
        self._begin_operation(
            "reset_mission",
            self._vehicle.set_mission_current_async(0, reset=True, now=now),
            now,
        )

    def _advance_auto_mission(self, now: float) -> None:
        if self._handoff_sequence_reached():
            self._begin_guided_takeover(now)
            return
        if self._landing_route_is_active():
            self._transition(
                CycleState.WAIT_FOR_HANDOFF,
                "landing route active; waiting for validated handoff",
            )

    def _advance_wait_for_handoff(self, now: float) -> None:
        if self._dock_request_queued and not self._dock_request_acknowledged:
            self._begin_operation(
                "request_dock",
                self._vehicle.request_dock_async(now),
                now,
            )
            return
        if self._handoff_sequence_reached():
            self._begin_guided_takeover(now)
            return

    def _begin_guided_takeover(self, now: float) -> None:
        self._dock_request_queued = True
        self._dock_request_acknowledged = True
        self._transition(
            CycleState.GUIDED_TAKEOVER,
            "landing handoff reached; requesting GUIDED takeover",
        )
        self._begin_guided_handoff(now, reset_retries=True)

    def _begin_guided_handoff(self, now: float, *, reset_retries: bool) -> None:
        """Request GUIDED and retain the pre-handoff mode for safe retries."""
        if reset_retries:
            self._guided_handoff_retry_count = 0
            self._guided_handoff_origin_mode = self._vehicle.state.mode
        self._begin_operation(
            "handoff_guided",
            self._vehicle.set_mode_async(GUIDED_MODE, now),
            now,
        )

    def _retry_guided_handoff_if_safe(self, message: str, now: float) -> bool:
        """Retry only an ACKed GUIDED command whose mode confirmation timed out."""
        if not message.startswith(
            "MAVLink mode change acknowledged but timed out waiting for mode "
        ):
            return False
        retry_limit = self._nonnegative_integer_parameter(
            "guided_handoff_retry_attempts"
        )
        state = self._vehicle.state
        if (
            self._guided_handoff_retry_count >= retry_limit
            or self._guided_handoff_origin_mode is None
            or state.mode != self._guided_handoff_origin_mode
            or not self._fresh_airborne_telemetry(now)
            or not self._fresh_landed_state(now, state)
        ):
            return False
        self._guided_handoff_retry_count += 1
        self._transition(
            CycleState.GUIDED_TAKEOVER,
            "GUIDED handoff mode confirmation timed out after acknowledgement; "
            f"retrying GUIDED ({self._guided_handoff_retry_count}/{retry_limit})",
            force=True,
        )
        self._begin_guided_handoff(now, reset_retries=False)
        return True

    def _start_shared_search(self, now: float, result: Any) -> None:
        command = self._command_result(result)
        if not (
            command.success
            or command.current_mode == GUIDED_MODE
            or self._vehicle.state.mode == GUIDED_MODE
        ):
            self._operation_failed("handoff_guided", command.message, now)
            return
        self._guided_search_telemetry_deadline = now + max(
            self._positive_parameter("pose_timeout_s"),
            self._positive_parameter("landed_state_timeout_s"),
        )
        self._advance_guided_search_start(now)

    def _start_pid_alignment(self, now: float, result: Any) -> None:
        """Confirm GUIDED and await telemetry without running dock search."""
        command = self._command_result(result)
        if not (
            command.success
            or command.current_mode == GUIDED_MODE
            or self._vehicle.state.mode == GUIDED_MODE
        ):
            self._operation_failed("handoff_guided", command.message, now)
            return
        self._guided_search_telemetry_deadline = now + max(
            self._positive_parameter("pose_timeout_s"),
            self._positive_parameter("landed_state_timeout_s"),
        )
        self._advance_guided_search_start(now)

    def _advance_guided_search_start(self, now: float) -> None:
        deadline = self._guided_search_telemetry_deadline
        if deadline is None:
            return
        if not self._fresh_guided_search_telemetry(now):
            if now >= deadline:
                self._guided_search_telemetry_deadline = None
                self._route_failure(
                    "guided handoff telemetry readiness timed out before shared "
                    "dock search: "
                    + self._guided_search_telemetry_timeout_reason(now),
                    grounded_state=CycleState.FAILED_DISARMED,
                    now=now,
                )
            elif "waiting for fresh telemetry" not in self._reason:
                self._transition(
                    CycleState.GUIDED_TAKEOVER,
                    "GUIDED handoff confirmed; waiting for fresh telemetry "
                    "before shared dock search",
                    force=True,
                )
            return
        self._guided_search_telemetry_deadline = None
        if self._operation_mode == "pid_align_debug":
            self._start_selected_backend(now)
            return
        state = self._vehicle.state
        assert state.position_enu_m is not None
        try:
            controller = DockSearchController(self._build_search_config())
            controller.start(now=now, center_enu_m=state.position_enu_m)
        except Exception as error:
            self._begin_fc_handback(
                now,
                f"shared dock search could not start: {error}",
            )
            return
        self._search_controller = controller
        self._landing_controller = None
        self._last_streamed_dock_pose_generation = -1
        self._transition(
            CycleState.SEARCH_DOCK,
            "GUIDED handoff confirmed; starting shared dock search",
        )

    def _advance_search(self, now: float) -> None:
        controller = self._search_controller
        if controller is None:
            self._begin_fc_handback(now, "shared dock search is unavailable")
            return
        if not self._fresh_guided_search_telemetry(now):
            self._command_direct_land(
                "search telemetry became stale before dock acquisition"
            )
            return
        state = self._vehicle.state
        assert state.position_enu_m is not None
        decision = controller.update(
            SearchObservation(
                now=now,
                position_enu_m=state.position_enu_m,
                target_visible=self._dock_observation(now) is not None,
            )
        )
        if decision.outcome is SearchOutcome.SEARCHING:
            self._vehicle.send_velocity(
                VelocityFrame.MAP_ENU,
                decision.velocity_enu_m_s[0],
                decision.velocity_enu_m_s[1],
                decision.velocity_enu_m_s[2],
                now=now,
            )
            return
        if decision.outcome is SearchOutcome.ACQUIRED:
            self._start_selected_backend(now)
            return
        self._begin_fc_handback(
            now,
            decision.reason or "shared dock search failed",
        )

    def _start_selected_backend(self, now: float) -> None:
        if not self._fresh_landing_backend_telemetry(now):
            self._command_direct_land(
                "landing telemetry became stale before backend start: "
                + self._landing_backend_stale_reason(now)
            )
            return
        try:
            backend = self._build_landing_backend()
            backend.start(
                now=now,
                dock=self._dock_observation(now),
                vehicle=self._vehicle_observation(),
            )
        except Exception as error:
            self._begin_fc_handback(
                now,
                f"landing backend could not start: {error}",
            )
            return
        self._landing_controller = backend
        self._landing_expected_modes = frozenset({GUIDED_MODE})
        self._transition(
            CycleState.LANDING_BACKEND,
            (
                "PID alignment-only backend started"
                if self._operation_mode == "pid_align_debug"
                else f"{self._landing_backend} landing backend started after "
                "shared search acquisition"
            ),
        )

    def _advance_landing_backend(self, now: float) -> None:
        backend = self._landing_controller
        if backend is None:
            self._begin_fc_handback(now, "landing backend is unavailable")
            return
        if not self._fresh_landing_backend_telemetry(now):
            if self._operation_mode == "pid_align_debug":
                self._cancel_pid_landing_debug_attempt(
                    "landing telemetry became stale; PID alignment cancelled"
                )
                return
            self._command_direct_land(
                "landing telemetry became stale during docking: "
                + self._landing_backend_stale_reason(now)
            )
            return
        try:
            dock = self._dock_observation(now)
            vehicle = self._vehicle_observation()
            decision = backend.update(
                now=now,
                dock=dock,
                vehicle=vehicle,
            )

            self._safe_record_landing_backend_decision(decision, now)
            self._publish_pid_debug(now, dock, vehicle, decision)
            self._record_touchdown_sample(now, dock, vehicle, decision)

            self.get_logger().info(
                f"Landing backend update: {decision}"
            )
        except Exception as error:
            self._begin_fc_handback(
                now,
                f"landing backend update failed: {error}",
            )
            return

        # 1) execute velocity requested by backend
        if decision.velocity is not None:
            command = decision.velocity
            self._vehicle.send_velocity(
                command.frame,
                command.velocity_x,
                command.velocity_y,
                command.velocity_z,
                now=now,
                yaw_rate_rad_s=command.yaw_rate_rad_s,
            )

        # 2) execute yaw correction if backend precise landing requested it 
        if decision.yaw_correction_rad is not None:
            self._vehicle.send_condition_yaw(
                yaw_error_rad=decision.yaw_correction_rad
            )
        
        # 3) stream landing target
        if decision.stream_landing_target:
            self._stream_latest_landing_target_once(now)

        # 4) if precise-landing backend request to land
        if decision.request_land:
            try:
                self._landing_expected_modes = frozenset({GUIDED_MODE})
                self._begin_operation(
                    "native_land",
                    self._vehicle.land_async(now),
                    now,
                )
            except Exception:
                self._landing_expected_modes = frozenset({GUIDED_MODE})
                self._notify_native_land_result(False, now)
            return
        
        # 5) If PID request to disarm
        if decision.request_disarm:
            self._begin_pid_disarm(decision.disarm_request_id, now)
            return
            
        # 6) if backend mark complete 
        if decision.outcome is LandingBackendOutcome.COMPLETE:
            self._landing_controller = None

            self._transition(
                CycleState.DOCKED,
                "landing backend completed; vehicle landed and disarmed",
            )
            return

        # 7) Handling for failures
        if decision.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT:
            self._begin_fc_handback(
                now,
                decision.reason or "landing backend failed before commitment",
            )
            return
        if decision.outcome is LandingBackendOutcome.FAIL_AFTER_COMMIT:
            self._command_direct_land(
                decision.reason or "landing backend failed after commitment"
            )

    def _advance_return_to_fc(self, now: float) -> None:
        if self._contract is None:
            self._command_direct_land("validated fallback mission is unavailable")
            return
        if self._mission_confirmation_deadline is None:
            self._begin_operation(
                "fallback_start",
                self._vehicle.start_mission_async(now),
                now,
            )
            return
        if self._mission_progress_confirmed(now):
            self._fc_fallback_expected_modes = frozenset({AUTO_MODE})
            self._transition(
                CycleState.FC_FALLBACK_LAND,
                "AUTO NAV_LAND fallback confirmed; waiting for flight controller landing",
            )
            return
        if now > self._mission_confirmation_deadline:
            self._command_direct_land("fallback mission confirmation timed out")

    def _advance_pid_disarm(self, now: float) -> bool:
        pending = self._pending
        backend = self._landing_controller
        request_id = self._pending_pid_disarm_request_id
        if pending is None:
            return False
        if backend is None or request_id is None:
            self._pending = None
            self._record_pid_disarm_result(
                request_id, False, "disarm transaction lost its controller"
            )
            self._command_direct_land("PID disarm transaction lost its controller")
            return True
        notifier = getattr(backend, "notify_disarm_result", None)
        if (
            now > pending.deadline
            and not self._pending_pid_disarm_timeout_notified
            and callable(notifier)
        ):
            self._record_pid_disarm_result(
                request_id, False, "command timed out"
            )
            notifier(request_id, False, now)
            self._pending_pid_disarm_timeout_notified = True
        if not pending.future.done():
            return False
        timeout_notified = self._pending_pid_disarm_timeout_notified
        try:
            result = pending.future.result()
            success = bool(getattr(result, "success", False))
            result_reason = str(getattr(result, "message", "")).strip()
            if not result_reason:
                result_reason = "acknowledged" if success else "command rejected"
        except Exception as error:
            success = False
            result_reason = f"command raised {error}"
        self._pending = None
        self._pending_pid_disarm_request_id = None
        self._pending_pid_disarm_timeout_notified = False
        if not timeout_notified and callable(notifier):
            self._record_pid_disarm_result(
                request_id, success, result_reason
            )
            notifier(request_id, success, now)
        return False

    def _begin_pid_disarm(self, request_id: Optional[int], now: float) -> None:
        backend = self._landing_controller
        if request_id is None:
            self._record_pid_disarm_result(
                None, False, "request did not include an ID"
            )
            self._command_direct_land(
                "PID disarm request did not include a request ID"
            )
            return
        notifier = getattr(backend, "notify_disarm_result", None)
        if self._pending is None:
            try:
                self._begin_operation(
                    "pid_disarm",
                    self._vehicle.arm_async(False, now),
                    now,
                )
            except Exception as error:
                self._record_pid_disarm_result(
                    request_id, False, f"command could not start: {error}"
                )
                if callable(notifier):
                    notifier(request_id, False, now)
                return
            self._pending_pid_disarm_request_id = request_id
            self._pending_pid_disarm_timeout_notified = False
            return
        if request_id != self._pending_pid_disarm_request_id and callable(notifier):
            self._record_pid_disarm_result(
                request_id,
                False,
                f"request {self._pending_pid_disarm_request_id} is still pending",
            )
            notifier(request_id, False, now)

    def _record_pid_disarm_result(
        self, request_id: Optional[int], success: bool, reason: str
    ) -> None:
        request = "none" if request_id is None else str(request_id)
        self._write_landing_attempt_log(
            f"DISARM_RESULT request={request} success={success} reason={reason}"
        )

    def _advance_native_land(self, now: float) -> bool:
        pending = self._pending
        if pending is None:
            return False
        if not pending.future.done():
            if now > pending.deadline:
                self._pending = None
                self._notify_native_land_result(False, now)
                return False
            return True
        self._pending = None
        success = False
        try:
            result = self._command_result(pending.future.result())
            success = (
                result.success
                or result.current_mode == LAND_MODE
                or self._vehicle.state.mode == LAND_MODE
            )
        except Exception:
            success = False
        self._notify_native_land_result(success, now)
        self._landing_expected_modes = (
            frozenset({LAND_MODE}) if success else frozenset({GUIDED_MODE})
        )
        return success

    def _notify_native_land_result(self, success: bool, now: float) -> None:
        backend = self._landing_controller
        notifier = getattr(backend, "notify_land_result", None)
        if callable(notifier):
            notifier(success, now)

    def _begin_fc_handback(self, now: float, reason: str) -> None:
        contract = self._contract
        if contract is None:
            self._command_direct_land(reason)
            return
        self._reset_active_guidance()
        self._transition(
            CycleState.RETURN_TO_FC,
            f"{reason}; returning control to validated NAV_LAND",
            force=True,
        )
        try:
            self._begin_operation(
                "fallback_select",
                self._vehicle.set_mission_current_async(
                    contract.fallback_land_seq,
                    reset=True,
                    now=now,
                ),
                now,
            )
        except Exception as error:
            self._command_direct_land(
                f"{reason}; fallback mission select could not start: {error}"
            )

    def _mission_progress_confirmed(self, now: float) -> bool:
        state = self._vehicle.state
        expected_seq = self._mission_confirmation_expected_seq
        heartbeat_baseline = self._auto_heartbeat_baseline
        mission_current_baseline = self._mission_current_baseline
        if (
            expected_seq is None
            or heartbeat_baseline is None
            or mission_current_baseline is None
        ):
            return False
        mission_mode_matches = (
            state.mission_mode is None
            or state.mission_mode == MISSION_TYPE_MISSION
        )
        mission_state_matches = (
            state.mission_state is None
            or state.mission_state == MISSION_STATE_ACTIVE
        )
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.mode == AUTO_MODE
            and state.heartbeat_at > heartbeat_baseline
            and state.mission_seq is not None
            and state.mission_seq >= expected_seq # TODO: change here to not have 
            and state.mission_current_at > mission_current_baseline
            and mission_state_matches
            # and mission_mode_matches
            and 0.0 <= now - state.mission_current_at
            <= self._positive_parameter("mavlink_heartbeat_timeout_s")
        )

    def _landing_route_is_active(self) -> bool:
        contract = self._contract
        mission_seq = self._vehicle.state.mission_seq
        return (
            contract is not None
            and mission_seq is not None
            and contract.land_start_seq <= mission_seq < contract.handoff_seq
        )

    def _handoff_sequence_reached(self) -> bool:
        contract = self._contract
        return (
            contract is not None
            and self._vehicle.state.mission_seq == contract.handoff_seq
        )

    def _dock_observation(self, now: float) -> Optional[DockObservation]:
        if not self._dock_target_is_fresh(now):
            return None
        assert self._dock_pose is not None
        mode = {
            "OUTER": DockTargetMode.OUTER,
            "CENTRAL_ACQUIRING": DockTargetMode.CENTRAL_ACQUIRING,
            "CENTRAL": DockTargetMode.CENTRAL,
        }.get(self._dock_mode)
        if mode is None:
            return None
        position = self._dock_pose.pose.position
        orientation = self._dock_pose.pose.orientation
        forward_m, right_m, distance_m = camera_target_to_body_frd(
            camera_x_m=position.x,
            camera_y_m=position.y,
            camera_z_m=position.z,
        )
        try:
            yaw_error_rad = -camera_dock_orientation_to_body_yaw_error(
                quaternion_x=orientation.x,
                quaternion_y=orientation.y,
                quaternion_z=orientation.z,
                quaternion_w=orientation.w,
            )
        except ValueError:
            return None

        return DockObservation(
            forward_m=forward_m,
            left_m=-right_m,
            distance_m=distance_m,
            yaw_error_rad=yaw_error_rad,
            mode=mode,
            source_stamp_s=(
                float(self._dock_pose.header.stamp.sec)
                + float(self._dock_pose.header.stamp.nanosec) * 1.0e-9
            ),
            received_at=self._dock_pose_received_at,
        )

    def _dock_target_is_fresh(self, now: float) -> bool:
        timeout = self._positive_parameter("dock_pose_timeout_s")
        return (
            self._dock_pose is not None
            and self._dock_mode != "NO_TARGET"
            and 0.0 <= now - self._dock_pose_received_at <= timeout
            and 0.0 <= now - self._dock_mode_received_at <= timeout
        )

    def _stream_latest_landing_target_once(self, now: float) -> None:
        if not self._dock_target_is_fresh(now):
            return
        if self._dock_pose_generation == self._last_streamed_dock_pose_generation:
            return
        assert self._dock_pose is not None
        position = self._dock_pose.pose.position
        forward_m, right_m, down_m = camera_target_to_body_frd(
            camera_x_m=position.x,
            camera_y_m=position.y,
            camera_z_m=position.z,
            camera_forward_m=self._positive_parameter(
                "landing_target_camera_forward_m",
                allow_zero=True,
            ),
            camera_right_m=self._positive_parameter(
                "landing_target_camera_right_m",
                allow_zero=True,
            ),
            camera_down_m=self._positive_parameter(
                "landing_target_camera_down_m",
                allow_zero=True,
            ),
        )
        if self._vehicle.send_landing_target(
            time_usec=measurement_time_usec(
                stamp_sec=self._dock_pose.header.stamp.sec,
                stamp_nanosec=self._dock_pose.header.stamp.nanosec,
                fallback_usec=int(now * 1_000_000),
            ),
            forward_m=forward_m,
            right_m=right_m,
            down_m=down_m,
        ):
            self._last_streamed_dock_pose_generation = self._dock_pose_generation

    def _fresh_guided_search_telemetry(self, now: float) -> bool:
        state = self._vehicle.state
        freshness = self._guided_search_telemetry_freshness(now)
        return (
            freshness["heartbeat"]
            and state.armed is True
            and state.flying is True
            and state.mode == GUIDED_MODE
            and state.position_enu_m is not None
            and state.velocity_enu_m_s is not None
            and freshness["position"]
            and freshness["landed_state"]
        )

    def _guided_search_telemetry_freshness(self, now: float) -> dict[str, bool]:
        """Return the individual telemetry freshness gates for GUIDED handoff."""
        state = self._vehicle.state
        return {
            "heartbeat": self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            ),
            "position": self._fresh_position(now, state),
            "landed_state": self._fresh_landed_state(now, state),
        }

    def _guided_search_telemetry_timeout_reason(self, now: float) -> str:
        """Describe every failed GUIDED-handoff telemetry readiness gate."""
        state = self._vehicle.state
        freshness = self._guided_search_telemetry_freshness(now)
        limits = {
            "heartbeat": self._positive_parameter("mavlink_heartbeat_timeout_s"),
            "position": self._positive_parameter("pose_timeout_s"),
            "landed_state": self._positive_parameter("landed_state_timeout_s"),
        }
        received_at = {
            "heartbeat": state.heartbeat_at,
            "position": state.position_at,
            "landed_state": state.landed_at,
        }
        details = []
        for name in ("heartbeat", "position", "landed_state"):
            age = max(0.0, now - received_at[name])
            status = "OK" if freshness[name] else "STALE"
            details.append(
                f"{name}={status} age={age:.2f}s limit={limits[name]:.2f}s"
            )
        details.append(
            f"connected={state.connected!r} system_status={state.system_status!r} "
            f"armed={state.armed!r} flying={state.flying!r} "
            f"mode={self._flight_mode_name(state.mode)} "
            f"position_value={state.position_enu_m is not None} "
            f"velocity_value={state.velocity_enu_m_s is not None}"
        )
        return "; ".join(details)

    def _fresh_landing_backend_telemetry(self, now: float) -> bool:
        state = self._vehicle.state
        freshness = self._landing_backend_telemetry_freshness(now)
        return (
            freshness["heartbeat"]
            and state.armed is not None
            and state.flying is not None
            and state.position_enu_m is not None
            and state.velocity_enu_m_s is not None
            and freshness["position"]
            and freshness["landed_state"]
        )

    def _landing_backend_telemetry_freshness(self, now: float) -> dict[str, bool]:
        """Return the individual freshness gates used during landing."""
        state = self._vehicle.state
        return {
            "heartbeat": self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            ),
            "position": self._fresh_position(now, state),
            "landed_state": self._fresh_landed_state(now, state),
            "dock_target": self._dock_target_is_fresh(now),
        }

    def _landing_backend_stale_reason(self, now: float) -> str:
        """Explain which landing telemetry freshness gates are failing."""
        state = self._vehicle.state
        freshness = self._landing_backend_telemetry_freshness(now)
        limits = {
            "heartbeat": self._positive_parameter("mavlink_heartbeat_timeout_s"),
            "position": self._positive_parameter("pose_timeout_s"),
            "landed_state": self._positive_parameter("landed_state_timeout_s"),
            "dock_target": self._positive_parameter("dock_pose_timeout_s"),
        }
        received_at = {
            "heartbeat": state.heartbeat_at,
            "position": state.position_at,
            "landed_state": state.landed_at,
            "dock_target": min(
                self._dock_pose_received_at,
                self._dock_mode_received_at,
            ),
        }
        details = []
        for name in ("heartbeat", "position", "landed_state", "dock_target"):
            age = max(0.0, now - received_at[name])
            status = "OK" if freshness[name] else "STALE"
            details.append(
                f"{name}={status} age={age:.2f}s limit={limits[name]:.2f}s"
            )
        details.append(
            f"armed={state.armed!r} flying={state.flying!r} "
            f"position_value={state.position_enu_m is not None} "
            f"velocity_value={state.velocity_enu_m_s is not None}"
        )
        return "; ".join(details)

    def _vehicle_observation(self) -> VehicleObservation:
        state = self._vehicle.state
        if state.position_enu_m is None or state.velocity_enu_m_s is None:
            raise RuntimeError("vehicle observation is unavailable")
        return VehicleObservation(
            position_enu_m=state.position_enu_m,
            velocity_enu_m_s=state.velocity_enu_m_s,
            armed=state.armed,
            flying=state.flying,
            mode=state.mode,
            landed_at=state.landed_at,
            heading_rad=state.heading_rad,
        )

    def _build_search_config(self) -> SearchConfig:
        return SearchConfig(
            radius_m=self._positive_parameter("search_radius_m"),
            tangential_speed_m_s=self._positive_parameter(
                "search_tangential_speed_m_s",
                allow_zero=True,
            ),
            radial_gain=self._positive_parameter(
                "search_radial_gain",
                allow_zero=True,
            ),
            altitude_gain=self._positive_parameter(
                "search_altitude_gain",
                allow_zero=True,
            ),
            max_horizontal_speed_m_s=self._positive_parameter(
                "search_max_horizontal_speed_m_s"
            ),
            max_vertical_speed_m_s=self._positive_parameter(
                "search_max_vertical_speed_m_s"
            ),
            target_stable_time_s=self._positive_parameter(
                "search_target_stable_time_s",
                allow_zero=True,
            ),
            timeout_s=self._positive_parameter("search_timeout_s"),
            max_displacement_m=self._positive_parameter("search_max_displacement_m"),
        )

    def _build_landing_backend(self) -> Any:
        if self._landing_backend == "pid":
            config = self._build_pid_landing_config()
            return PidLandingBackend(
                config,
                align_only=self._operation_mode == "pid_align_debug",
                hold_at_state=self._pid_hold_at_state,
            )
        return PreciseLandingBackend(self._build_precise_landing_config())

    def _build_pid_landing_config(self) -> PidLandingConfig:
        max_horizontal = self._positive_parameter("pid_max_horizontal_speed_m_s")
        horizontal_pid_limit = max(
            max_horizontal,
            self._positive_parameter("pid_descend_outer_max_horizontal_speed_m_s"),
            self._positive_parameter("pid_descend_central_max_horizontal_speed_m_s"),
            self._positive_parameter("pid_touchdown_max_horizontal_speed_m_s"),
        )
        max_yaw = self._positive_parameter("pid_max_yaw_rate_rad_s")
        base_pids = tuple(self._build_pid_axis_config(f"pid_{axis}", limit)
                          for axis, limit in zip(
                              ("x", "y", "z", "yaw"),
                              (horizontal_pid_limit, horizontal_pid_limit,
                               self._positive_parameter("search_max_vertical_speed_m_s"),
                               max_yaw)))
        return PidLandingConfig(
            tag_stable_time_s=self._positive_parameter("pid_tag_stable_time_s"),
            central_transition_distance_m=self._positive_parameter(
                "pid_central_transition_distance_m"
            ),
            central_acquire_timeout_s=self._positive_parameter(
                "pid_central_acquire_timeout_s"
            ),
            central_loss_grace_s=self._positive_parameter(
                "pid_central_loss_grace_s"
            ),
            recovery_climb_m=self._positive_parameter("pid_recovery_climb_m"),
            recovery_altitude_tolerance_m=self._positive_parameter(
                "pid_recovery_altitude_tolerance_m"
            ),
            max_recovery_attempts=int(
                self.get_parameter("pid_max_recovery_attempts").value
            ),
            touchdown_commit_height_m=self._positive_parameter(
                "pid_touchdown_commit_height_m"
            ),
            touchdown_speed_m_s=self._positive_parameter(
                "pid_touchdown_speed_m_s"
            ),
            touchdown_timeout_s=self._positive_parameter("pid_touchdown_timeout_s"),
            landed_state_timeout_s=self._positive_parameter(
                "pid_landed_state_timeout_s"
            ),
            landed_stable_time_s=self._positive_parameter(
                "pid_landed_stable_time_s"
            ),
            disarm_retry_period_s=self._positive_parameter(
                "pid_disarm_retry_period_s"
            ),
            alignment_stable_time_s=self._positive_parameter(
                "pid_alignment_stable_time_s"
            ),
            outer_horizontal_tolerance_m=self._positive_parameter(
                "pid_outer_horizontal_tolerance_m"
            ),
            central_horizontal_tolerance_m=self._positive_parameter(
                "pid_central_horizontal_tolerance_m"
            ),
            outer_yaw_tolerance_rad=self._positive_parameter(
                "pid_outer_yaw_tolerance_rad"
            ),
            central_yaw_tolerance_rad=self._positive_parameter(
                "pid_central_yaw_tolerance_rad"
            ),
            touchdown_horizontal_tolerance_m=self._positive_parameter(
                "pid_touchdown_horizontal_tolerance_m"
            ),
            touchdown_yaw_tolerance_rad=self._positive_parameter(
                "pid_touchdown_yaw_tolerance_rad"
            ),
            touchdown_horizontal_speed_tolerance_m_s=self._positive_parameter(
                "pid_touchdown_horizontal_speed_tolerance_m_s"
            ),
            touchdown_horizontal_speed_stable_time_s=self._positive_parameter(
                "pid_touchdown_horizontal_speed_stable_time_s"
            ),
            max_horizontal_speed_m_s=max_horizontal,
            descend_outer_max_horizontal_speed_m_s=self._positive_parameter(
                "pid_descend_outer_max_horizontal_speed_m_s"
            ),
            descend_central_max_horizontal_speed_m_s=self._positive_parameter(
                "pid_descend_central_max_horizontal_speed_m_s"
            ),
            touchdown_max_horizontal_speed_m_s=self._positive_parameter(
                "pid_touchdown_max_horizontal_speed_m_s"
            ),
            outer_descent_speed_m_s=self._positive_parameter(
                "pid_outer_descent_speed_m_s"
            ),
            central_descent_speed_m_s=self._positive_parameter(
                "pid_central_descent_speed_m_s"
            ),
            recovery_speed_m_s=self._positive_parameter("pid_recovery_speed_m_s"),
            max_yaw_rate_rad_s=max_yaw,
            x_pid=base_pids[0], y_pid=base_pids[1],
            z_pid=base_pids[2], yaw_pid=base_pids[3],
            x_k_v=self._positive_parameter("pid_x_k_v", allow_zero=True),
            y_k_v=self._positive_parameter("pid_y_k_v", allow_zero=True),
            z_k_v=self._positive_parameter("pid_z_k_v", allow_zero=True),
            gain_near_distance_m=self._positive_parameter("pid_gain_near_distance_m"),
            gain_far_distance_m=self._positive_parameter("pid_gain_far_distance_m"),
            near_pids=self._build_distance_pid_configs("near", base_pids),
            far_pids=self._build_distance_pid_configs("far", base_pids),
            target_loss_grace_s=self._positive_parameter(
                "pid_target_loss_grace_s"
            ),
            fixed_central_reference_mode=str(
                self.get_parameter("pid_fixed_central_reference_mode").value
            ).strip().lower(),
        )

    def _build_distance_pid_configs(
        self, side: str, base: tuple[PIDConfig, ...],
        updates: Optional[Mapping[str, object]] = None,
    ) -> tuple[PIDConfig, ...]:
        updates = updates or {}
        result = []
        for axis, config in zip(("x", "y", "z", "yaw"), base):
            gains = {}
            for gain in ("kp", "ki", "kd"):
                name = f"pid_gain_{side}_{axis}_{gain}"
                value = float(updates.get(name, self.get_parameter(name).value))
                if not math.isfinite(value) or (value < 0.0 and value != -1.0):
                    raise ValueError(f"{name} must be -1 (inherit) or nonnegative")
                gains[gain] = getattr(config, gain) if value == -1.0 else value
            result.append(replace(config, **gains))
        return tuple(result)

    def _build_pid_axis_config(self, prefix: str, output_limit: float) -> PIDConfig:
        return PIDConfig(
            kp=self._positive_parameter(f"{prefix}_kp", allow_zero=True),
            ki=self._positive_parameter(f"{prefix}_ki", allow_zero=True),
            kd=self._positive_parameter(f"{prefix}_kd", allow_zero=True),
            integral_limit=self._positive_parameter(
                f"{prefix}_integral_limit",
                allow_zero=True,
            ),
            output_limit=output_limit,
            derivative_filter_time_constant_s=self._positive_parameter(
                f"{prefix}_derivative_filter_time_constant_s",
                allow_zero=True,
            ),
        )

    def _on_parameters_changed(self, parameters: list[Parameter]) -> SetParametersResult:
        """Validate and atomically apply live PID tuning to an active landing."""
        tunable_suffixes = {
            "kp", "ki", "kd", "integral_limit",
            "derivative_filter_time_constant_s",
        }
        updates = {parameter.name: parameter.value for parameter in parameters}
        tuning_updates = {
            name: value for name, value in updates.items()
            if any(name == f"pid_{axis}_{suffix}"
                   for axis in ("x", "y", "z", "yaw")
                   for suffix in tunable_suffixes)
        }
        velocity_updates = {name: updates[name] for name in
                            ("pid_x_k_v", "pid_y_k_v", "pid_z_k_v") if name in updates}
        has_distance_update = "pid_align_distance_setpoint_m" in updates
        distance_update = updates.get("pid_align_distance_setpoint_m")
        schedule_updates = {
            name: value for name, value in updates.items()
            if name in ("pid_gain_near_distance_m", "pid_gain_far_distance_m")
            or any(name == f"pid_gain_{side}_{axis}_{gain}"
                   for side in ("near", "far")
                   for axis in ("x", "y", "z", "yaw")
                   for gain in ("kp", "ki", "kd"))
        }
        has_fixed_mode_update = "pid_fixed_central_reference_mode" in updates
        has_hold_update = "pid_hold_at_state" in updates
        if not (tuning_updates or velocity_updates or schedule_updates or has_distance_update
                or has_hold_update or has_fixed_mode_update):
            return SetParametersResult(successful=True)
        try:
            backend = self._landing_controller
            hold_state = None
            fixed_mode = None
            if has_fixed_mode_update:
                fixed_mode = updates["pid_fixed_central_reference_mode"]
                if not isinstance(fixed_mode, str) or fixed_mode not in (
                        "off", "descend_central", "hold_central"):
                    raise ValueError("invalid pid_fixed_central_reference_mode")
                if self._landing_backend != "pid":
                    raise ValueError("fixed central reference requires landing_backend=pid")
                if self._state is CycleState.LANDING_BACKEND and backend is not None:
                    backend.validate_fixed_central_reference_mode(fixed_mode)
            if has_hold_update:
                hold_value = updates["pid_hold_at_state"]
                if not isinstance(hold_value, str):
                    raise ValueError("pid_hold_at_state must be a string")
                hold_name = hold_value.strip().upper()
                if hold_name != "NONE" and hold_name not in {s.name for s in PID_HOLD_STATES}:
                    raise ValueError("invalid pid_hold_at_state")
                if hold_name != "NONE" and self._landing_backend != "pid":
                    raise ValueError("pid_hold_at_state requires landing_backend=pid")
                if self._operation_mode == "pid_align_debug" and hold_name not in {"NONE", "PID_ALIGN"}:
                    raise ValueError("pid_align_debug can only hold at PID_ALIGN")
                hold_state = None if hold_name == "NONE" else PidLandingState[hold_name]
                if self._state is CycleState.LANDING_BACKEND and backend is not None:
                    backend.validate_hold_at_state(hold_state)
            if has_distance_update:
                if (self._operation_mode != "pid_align_debug"
                        or self._state is not CycleState.LANDING_BACKEND
                        or backend is None):
                    raise ValueError("dock distance setpoint requires active PID alignment debug")
                if (isinstance(distance_update, bool)
                        or not isinstance(distance_update, (int, float))
                        or not math.isfinite(distance_update)
                        or not 0.3 <= distance_update <= 20.0):
                    raise ValueError("dock distance setpoint must be between 0.3 and 20 m")
                diagnostics = backend.diagnostics()
                if not diagnostics["align_dock_visible"]:
                    raise ValueError("wait for a fresh dock observation before setting Z")
            for name, value in velocity_updates.items():
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or value < 0.0):
                    raise ValueError(f"{name} must be finite and nonnegative")
            if velocity_updates and not tuning_updates and not schedule_updates:
                if backend is not None:
                    backend.reconfigure_velocity_feedback(**{
                        axis: float(updates.get(f"pid_{axis}_k_v",
                                                self.get_parameter(f"pid_{axis}_k_v").value))
                        for axis in ("x", "y", "z")
                    })
                if has_distance_update:
                    backend.set_align_distance_setpoint(distance_update)
                if has_hold_update:
                    if self._state is CycleState.LANDING_BACKEND and backend is not None:
                        backend.set_hold_at_state(hold_state)
                    self._pid_hold_at_state = hold_state
                if has_fixed_mode_update and self._state is CycleState.LANDING_BACKEND and backend is not None:
                    backend.set_fixed_central_reference_mode(fixed_mode)
                return SetParametersResult(successful=True)
            if not tuning_updates and not schedule_updates and not velocity_updates:
                if has_distance_update:
                    backend.set_align_distance_setpoint(distance_update)
                if has_hold_update:
                    if self._state is CycleState.LANDING_BACKEND and backend is not None:
                        backend.set_hold_at_state(hold_state)
                    self._pid_hold_at_state = hold_state
                if has_fixed_mode_update and self._state is CycleState.LANDING_BACKEND and backend is not None:
                    backend.set_fixed_central_reference_mode(fixed_mode)
                return SetParametersResult(successful=True)
            for name, value in tuning_updates.items():
                numeric = float(value)
                if not math.isfinite(numeric) or numeric < 0.0:
                    raise ValueError(f"{name} must be finite and nonnegative")
            near_distance = float(updates.get(
                "pid_gain_near_distance_m",
                self.get_parameter("pid_gain_near_distance_m").value))
            far_distance = float(updates.get(
                "pid_gain_far_distance_m",
                self.get_parameter("pid_gain_far_distance_m").value))
            if (not math.isfinite(near_distance) or not math.isfinite(far_distance)
                    or near_distance <= 0 or far_distance <= near_distance):
                raise ValueError("gain distances must be finite, positive and near < far")
            max_horizontal = self._positive_parameter("pid_max_horizontal_speed_m_s")
            horizontal_pid_limit = max(
                max_horizontal,
                self._positive_parameter("pid_descend_outer_max_horizontal_speed_m_s"),
                self._positive_parameter("pid_descend_central_max_horizontal_speed_m_s"),
                self._positive_parameter("pid_touchdown_max_horizontal_speed_m_s"),
            )
            max_yaw = self._positive_parameter("pid_max_yaw_rate_rad_s")
            limits = (horizontal_pid_limit, horizontal_pid_limit,
                      self._positive_parameter("search_max_vertical_speed_m_s"), max_yaw)
            configs = tuple(
                self._build_live_pid_axis_config(f"pid_{axis}", limit, tuning_updates)
                for axis, limit in zip(("x", "y", "z", "yaw"), limits)
            )
            near = self._build_distance_pid_configs("near", configs, schedule_updates)
            far = self._build_distance_pid_configs("far", configs, schedule_updates)
            reconfigure = getattr(backend, "reconfigure_pids", None)
            if callable(reconfigure):
                reconfigure(configs, near, far, near_distance, far_distance)
            if velocity_updates and backend is not None:
                backend.reconfigure_velocity_feedback(**{
                    axis: float(updates.get(f"pid_{axis}_k_v",
                                            self.get_parameter(f"pid_{axis}_k_v").value))
                    for axis in ("x", "y", "z")
                })
            if has_distance_update:
                backend.set_align_distance_setpoint(distance_update)
            if has_hold_update:
                if self._state is CycleState.LANDING_BACKEND and backend is not None:
                    backend.set_hold_at_state(hold_state)
                self._pid_hold_at_state = hold_state
            if has_fixed_mode_update and self._state is CycleState.LANDING_BACKEND and backend is not None:
                backend.set_fixed_central_reference_mode(fixed_mode)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _build_live_pid_axis_config(
        self, prefix: str, output_limit: float, updates: Mapping[str, object],
    ) -> PIDConfig:
        def value(suffix: str) -> float:
            name = f"{prefix}_{suffix}"
            return float(updates.get(name, self.get_parameter(name).value))

        return PIDConfig(
            kp=value("kp"), ki=value("ki"), kd=value("kd"),
            integral_limit=value("integral_limit"),
            output_limit=output_limit,
            derivative_filter_time_constant_s=value(
                "derivative_filter_time_constant_s"
            ),
        )

    def _publish_pid_debug(self, now: float, dock: Any, vehicle: Any, decision: Any) -> None:
        diagnostics = getattr(self._landing_controller, "diagnostics", None)
        if not callable(diagnostics) or decision.velocity is None:
            return
        command = decision.velocity
        data = diagnostics()
        data.update({
            "time_s": now,
            "state": decision.substate,
            "command_body_flu": [command.velocity_x, command.velocity_y,
                                 command.velocity_z, command.yaw_rate_rad_s],
            "measured_velocity_enu": list(vehicle.velocity_enu_m_s),
        })
        data["dock_distance_m"] = dock.distance_m if dock is not None else None
        # Keep the measured camera offsets separate from the PID error, which
        # can refer to a latched target or a distance setpoint instead.
        data["measured_dock_position"] = ({
            "forward_m": dock.forward_m,
            "left_m": dock.left_m,
            "distance_m": dock.distance_m,
        } if dock is not None else None)
        heading = vehicle.heading_rad
        if heading is not None and math.isfinite(heading):
            east, north, up = vehicle.velocity_enu_m_s
            data["measured_velocity_body_flu"] = [
                east * math.sin(heading) + north * math.cos(heading),
                -east * math.cos(heading) + north * math.sin(heading),
                up,
            ]
        else:
            data["measured_velocity_body_flu"] = None
        if dock is not None:
            control_error = data.get("control_error") or {}
            data["error"] = {
                "x": control_error.get("x", dock.forward_m),
                "y": control_error.get("y", dock.left_m),
                "z": (data.get("distance_setpoint_m") - dock.distance_m)
                if data.get("distance_setpoint_m") is not None else None,
                "yaw": control_error.get("yaw", dock.yaw_error_rad),
            }
            data["dock_age_s"] = max(0.0, now - dock.received_at)
        message = String()
        message.data = json.dumps(data, separators=(",", ":"), allow_nan=False)
        self._pid_debug_publisher.publish(message)

    def _record_touchdown_sample(self, now: float, dock: Any, vehicle: Any,
                                 decision: Any) -> None:
        """Record only final PID stages; recording errors cannot interrupt guidance."""
        try:
            self._write_touchdown_sample(now, dock, vehicle, decision)
        except Exception as error:
            self.get_logger().warning(f"Touchdown trace sample skipped: {error}")
            self._touchdown_trace.disable()

    def _write_touchdown_sample(self, now: float, dock: Any, vehicle: Any,
                                decision: Any) -> None:
        if not isinstance(self._landing_controller, PidLandingBackend):
            return
        if (decision.substate in ("DESCEND_CENTRAL", "HOLD_CENTRAL", "TOUCHDOWN_APPROACH")
                and not self._touchdown_trace.active):
            self._touchdown_trace.start()
        if not self._touchdown_trace.active:
            return
        diagnostics = self._landing_controller.diagnostics()
        command = decision.velocity
        frozen_target = diagnostics["touchdown_reference_enu_m"]
        position_error_to_frozen_target = (
            [frozen_target[axis] - vehicle.position_enu_m[axis] for axis in (0, 1)]
            if frozen_target is not None else None
        )
        horizontal_speed_m_s = math.hypot(
            vehicle.velocity_enu_m_s[0],
            vehicle.velocity_enu_m_s[1],
        )
        frozen_error_norm_m = (
            math.hypot(
                position_error_to_frozen_target[0],
                position_error_to_frozen_target[1],
            )
            if position_error_to_frozen_target is not None
            else None
        )
        data = {
            "event": "SAMPLE", "time_monotonic_s": now,
            "wall_time": datetime.now().astimezone().isoformat(),
            "state": decision.substate,
            "position_enu_m": list(vehicle.position_enu_m),
            "velocity_enu_m_s": list(vehicle.velocity_enu_m_s),
            "horizontal_speed_m_s": horizontal_speed_m_s,
            "heading_ned_rad": vehicle.heading_rad,
            "position_age_s": max(0.0, now - self._vehicle.state.position_at),
            "dock_mode": dock.mode.name if dock is not None else None,
            "dock_offset_body_flu_m": [dock.forward_m, dock.left_m] if dock is not None else None,
            "dock_distance_m": dock.distance_m if dock is not None else None,
            "dock_age_s": max(0.0, now - dock.received_at) if dock is not None else None,
            "last_dock_pose_age_s": (max(0.0, now - self._dock_pose_received_at)
                                     if self._dock_pose is not None else None),
            "last_dock_mode": self._dock_mode,
            "frozen_target_enu_m": frozen_target,
            "position_error_to_frozen_target_m": position_error_to_frozen_target,
            "frozen_error_norm_m": frozen_error_norm_m,
            "frozen_heading_ned_rad": diagnostics["touchdown_reference_heading_rad"],
            "fixed_reference_enu_m": diagnostics["fixed_reference_enu_m"],
            "command_frame": command.frame.name if command is not None else None,
            "command_body_flu_m_s": [command.velocity_x, command.velocity_y, command.velocity_z]
            if command is not None else None,
            "command_yaw_rate_rad_s": command.yaw_rate_rad_s if command is not None else None,
            "armed": vehicle.armed, "flying": vehicle.flying,
        }
        self._touchdown_trace.write(data)

    def _build_precise_landing_config(self) -> PreciseLandingConfig:
        yaw_threshold = (
            0.0
            if bool(self.get_parameter("landing_yaw_correction_enabled").value)
            else None
        )
        return PreciseLandingConfig(
            warmup_s=self._positive_parameter("landing_target_warmup_s"),
            dock_fresh_timeout_s=self._positive_parameter("dock_pose_timeout_s"),
            landed_state_timeout_s=self._positive_parameter("landed_state_timeout_s"),
            touchdown_warn_after_s=self._positive_parameter(
                "landing_touchdown_warn_after_s"
            ),
            disarm_warn_after_s=self._positive_parameter(
                "landing_disarm_warn_after_s"
            ),
            yaw_correction_threshold_rad=yaw_threshold,
            land_mode=LAND_MODE,
        )

    def _fresh_ground_telemetry(self, now: float) -> bool:
        state = self._vehicle.state
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.armed is False
            and state.flying is False
            and self._fresh_position(now, state)
            and self._fresh_landed_state(now, state)
        )

    def _fresh_airborne_telemetry(self, now: float) -> bool:
        state = self._vehicle.state
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.armed is True
            and state.flying is True
            and state.position_enu_m is not None
            and state.velocity_enu_m_s is not None
            and self._fresh_position(now, state)
        )

    def _handle_airborne_authority(self, now: float) -> bool:
        if self._state not in _AIRBORNE_AUTHORITY_STATES:
            return False
        if self._vehicle.state.flying is not True:
            return False
        if not self._vehicle.ready(
            now,
            heartbeat_timeout_s=self._positive_parameter("mavlink_heartbeat_timeout_s"),
        ):
            self._enter_passive_state(
                CycleState.LINK_LOSS,
                "heartbeat became stale while airborne; manager is now passive",
            )
            return True
        unexpected_mode_reason = self._unexpected_mode_reason(now)
        if unexpected_mode_reason is not None:
            self._enter_passive_state(
                CycleState.PILOT_OVERRIDE, unexpected_mode_reason
            )
            return True
        mission_integrity_reason = self._mission_integrity_reason(now)
        if mission_integrity_reason is not None:
            self._request_mission_integrity_fallback(now, mission_integrity_reason)
            return True
        return False

    def _unexpected_mode_reason(self, now: float) -> Optional[str]:
        state = self._vehicle.state
        if self._state not in _AIRBORNE_AUTHORITY_STATES:
            return None
        allowed_modes = self._expected_modes()
        if not allowed_modes:
            return None
        pending_mode = self._pending_mode()
        if pending_mode is not None and state.mode == pending_mode:
            return None
        if (
            self._pending is not None
            and self._pending.name == "handoff_guided"
            and state.mode == self._guided_handoff_origin_mode
        ):
            return None
        if state.mode in allowed_modes:
            return None
        if self._state is CycleState.WAIT_FOR_HANDOFF:
            return (
                "unexpected approach mode "
                f"{self._flight_mode_name(state.mode)}; pilot now owns approach authority"
            )
        return (
            "unexpected flight mode "
            f"{self._flight_mode_name(state.mode)}; pilot now owns flight authority"
        )

    def _flight_mode_name(self, mode: Optional[int]) -> str:
        return {
            AUTO_MODE: "AUTO",
            AUTO_RTL_MODE: "AUTO_RTL",
            GUIDED_MODE: "GUIDED",
            LAND_MODE: "LAND",
            5: "LOITER",
            None: "UNKNOWN",
        }.get(mode, str(mode))

    def _fresh_position(self, now: float, state: VehicleState) -> bool:
        return (
            state.position_enu_m is not None
            and 0.0 <= now - state.position_at
            <= self._positive_parameter("pose_timeout_s")
        )

    def _fresh_landed_state(self, now: float, state: VehicleState) -> bool:
        return (
            state.flying is not None
            and 0.0 <= now - state.landed_at
            <= self._positive_parameter("landed_state_timeout_s")
        )

    def _vehicle_is_confirmed_disarmed_on_ground(self, now: float) -> bool:
        state = self._vehicle.state
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.armed is False
            and state.flying is False
            and self._fresh_landed_state(now, state)
        )

    def _advance_passive_pending(self) -> bool:
        pending = self._pending
        if pending is None or not pending.future.done():
            return False
        self._pending = None
        self._pending_pid_disarm_request_id = None
        self._pending_pid_disarm_timeout_notified = False
        return True

    def _advance_passive_state(self, now: float) -> None:
        self._advance_passive_pending()
        self._advance_touchdown_completion(now)

    def _advance_deferred_command(self, now: float) -> bool:
        if self._deferred_command is None:
            return False
        if self._pending is not None:
            return True
        command = self._deferred_command
        reason = self._deferred_command_reason
        self._deferred_command = None
        self._deferred_command_reason = ""
        if command == "direct_land":
            return self._begin_priority_command(
                "direct_land",
                reason,
                now,
                start_command=lambda: self._vehicle.land_async(now),
            )
        if command == "mission_integrity_rtl":
            return self._begin_priority_command(
                "mission_integrity_rtl",
                reason,
                now,
                start_command=lambda: self._vehicle.rtl_async(now),
            )
        raise RuntimeError(f"unsupported deferred command: {command}")

    def _advance_fc_fallback_land(self, now: float) -> None:
        if self._advance_touchdown_completion(now):
            return
        state = self._vehicle.state
        if (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter("mavlink_heartbeat_timeout_s"),
            )
            and state.flying is True
            and state.mode == LAND_MODE
        ):
            self._transition(
                CycleState.WAIT_LANDED,
                "LAND mode confirmed; waiting for touchdown",
            )

    def _advance_wait_landed(self, now: float) -> None:
        self._advance_touchdown_completion(now)

    def _advance_wait_disarm(self, now: float) -> None:
        self._advance_touchdown_completion(now)

    def _advance_touchdown_completion(self, now: float) -> bool:
        state = self._vehicle.state
        if not self._fresh_on_ground(now, state):
            return False
        if state.armed is False:
            self._start_requested = False
            self._transition(CycleState.DOCKED, "landing complete and vehicle disarmed")
            return True
        self._transition(CycleState.WAIT_DISARM, "touchdown confirmed; waiting for disarm")
        return True

    def _fresh_on_ground(self, now: float, state: VehicleState) -> bool:
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter("mavlink_heartbeat_timeout_s"),
            )
            and state.flying is False
            and self._fresh_landed_state(now, state)
        )

    def _expected_modes(self) -> frozenset[int]:
        if self._state in {
            CycleState.SET_GUIDED,
            CycleState.ARM,
            CycleState.TAKEOFF,
            CycleState.WAIT_TAKEOFF_STABLE,
            CycleState.RESET_ONBOARD_MISSION,
        }:
            return frozenset({GUIDED_MODE})
        if self._state is CycleState.START_AUTO:
            return (
                frozenset({GUIDED_MODE})
                if self._mission_confirmation_deadline is None
                else frozenset({GUIDED_MODE, AUTO_MODE})
            )
        if self._state is CycleState.AUTO_MISSION:
            return (
                frozenset({AUTO_MODE, AUTO_RTL_MODE})
                if self._landing_route_is_active()
                else frozenset({AUTO_MODE})
            )
        if self._state is CycleState.WAIT_FOR_HANDOFF:
            return frozenset({AUTO_MODE, AUTO_RTL_MODE})
        if self._state in {CycleState.GUIDED_TAKEOVER, CycleState.SEARCH_DOCK}:
            return frozenset({GUIDED_MODE})
        if self._state is CycleState.LANDING_BACKEND:
            return self._landing_expected_modes or frozenset({GUIDED_MODE})
        if self._state is CycleState.RETURN_TO_FC:
            return (
                frozenset({GUIDED_MODE})
                if self._mission_confirmation_deadline is None
                else frozenset({GUIDED_MODE, AUTO_MODE})
            )
        if self._state is CycleState.FC_FALLBACK_LAND:
            return self._fc_fallback_expected_modes
        if self._state is CycleState.WAIT_LANDED:
            return frozenset({LAND_MODE})
        return frozenset()

    def _pending_mode(self) -> Optional[int]:
        pending = self._pending
        if pending is None:
            return None
        if pending.name in {"set_guided", "handoff_guided"}:
            return GUIDED_MODE
        if pending.name in {"start_mission", "fallback_start"}:
            return AUTO_MODE
        if pending.name == "mission_integrity_rtl":
            return AUTO_RTL_MODE
        if pending.name in {"direct_land", "native_land"}:
            return LAND_MODE
        return None

    def _mission_integrity_reason(self, now: float) -> Optional[str]:
        state = self._vehicle.state
        if (
            self._validated_mission_total is None
            or state.mission_current_at <= 0.0
            or now - state.mission_current_at
            > self._positive_parameter("mavlink_heartbeat_timeout_s")
        ):
            return None
        if (
            state.mission_total is not None
            and state.mission_total != self._validated_mission_total
        ):
            return (
                "validated mission item count changed from "
                f"{self._validated_mission_total} to {state.mission_total}"
            )
        if (
            self._validated_mission_id is not None
            and state.mission_id is not None
            and state.mission_id != 0
            and state.mission_id != self._validated_mission_id
        ):
            return (
                "validated mission ID changed from "
                f"{self._validated_mission_id} to {state.mission_id}"
            )
        return None

    def _request_mission_integrity_fallback(self, now: float, reason: str) -> None:
        expected_modes = self._expected_modes()
        self._contract = None
        self._validated_mission_total = None
        self._validated_mission_id = None
        self._reset_active_guidance()
        self._cancel_active_operation()
        self._deferred_command = None
        self._deferred_command_reason = ""
        self._fc_fallback_expected_modes = self._fallback_transition_modes(
            expected_modes,
            LAND_MODE,
        )
        self._transition(
            CycleState.FC_FALLBACK_LAND,
            f"{reason}; requesting RTL and ceding landing to the flight controller",
            force=True,
        )
        self._begin_priority_command(
            "mission_integrity_rtl",
            reason,
            now,
            start_command=lambda: self._vehicle.rtl_async(now),
        )

    def _enter_passive_state(self, state: CycleState, reason: str) -> None:
        self._reset_active_guidance()
        self._cancel_active_operation()
        self._fc_fallback_expected_modes = frozenset()
        self._deferred_command = None
        self._deferred_command_reason = ""
        self._transition(state, reason, force=True)

    def _reset_active_guidance(self) -> None:
        self._dock_request_queued = False
        self._dock_request_acknowledged = False
        self._guided_handoff_retry_count = 0
        self._guided_handoff_origin_mode = None
        self._guided_search_telemetry_deadline = None
        self._search_controller = None
        self._landing_controller = None
        self._landing_expected_modes = frozenset()
        self._mission_confirmation_deadline = None
        self._mission_confirmation_expected_seq = None
        self._auto_heartbeat_baseline = None
        self._mission_current_baseline = None
        self._last_streamed_dock_pose_generation = -1
        self._pending_pid_disarm_request_id = None
        self._pending_pid_disarm_timeout_notified = False

    def _cancel_active_operation(self) -> None:
        self._pending = None
        self._vehicle.cancel_pending_operation()

    @staticmethod
    def _fallback_transition_modes(
        expected_modes: frozenset[int], *additional_modes: int
    ) -> frozenset[int]:
        modes = set(expected_modes)
        modes.update(additional_modes)
        return frozenset(modes)

    def _begin_priority_command(
        self,
        name: str,
        reason: str,
        now: float,
        *,
        start_command,
    ) -> bool:
        if self._pending is not None:
            self._defer_priority_command(name, reason)
            return True
        try:
            self._begin_operation(name, start_command(), now)
        except Exception as error:
            if self._transport_operation_is_still_active(error):
                self._defer_priority_command(name, reason)
                return True
            if name == "mission_integrity_rtl":
                self._command_direct_land(f"{reason}; RTL could not start: {error}")
                return True
            self._transition(
                CycleState.FC_FALLBACK_LAND,
                f"{reason}; direct LAND could not start: {error}",
                force=True,
            )
            return True
        return True

    def _defer_priority_command(self, name: str, reason: str) -> None:
        self._deferred_command = name
        self._deferred_command_reason = reason

    @staticmethod
    def _transport_operation_is_still_active(error: Exception) -> bool:
        return "already active" in str(error).lower()

    def _command_direct_land(self, reason: str) -> None:
        self._fc_fallback_expected_modes = self._expected_modes()
        self._transition(CycleState.FC_FALLBACK_LAND, reason)
        now = time.monotonic()
        self._begin_priority_command(
            "direct_land",
            reason,
            now,
            start_command=lambda: self._vehicle.land_async(now),
        )

    def _invalid_mission(self, reason: str, now: float) -> None:
        self._contract = None
        self._route_failure(
            reason,
            grounded_state=CycleState.INVALID_ONBOARD_MISSION,
            now=now,
        )

    def _route_failure(
        self,
        reason: str,
        *,
        grounded_state: CycleState,
        now: float,
    ) -> None:
        """Land unless live telemetry confirms a disarmed vehicle on ground."""
        self._start_requested = False
        if self._vehicle_is_confirmed_disarmed_on_ground(now):
            if grounded_state is CycleState.FAILED_DISARMED:
                self._fail_disarmed(reason)
            else:
                self._transition(grounded_state, reason)
            return
        self._command_direct_land(reason)

    def _fail_disarmed(self, reason: str) -> None:
        self._start_requested = False
        self._transition(CycleState.FAILED_DISARMED, reason)

    def _reject_start(self, response: Trigger.Response, reason: str) -> Trigger.Response:
        self._transition(self._state, reason, force=True)
        response.success = False
        response.message = reason
        return response

    def _transition(self, state: CycleState, reason: str, *, force: bool = False) -> None:
        if not force and state is self._state and reason == self._reason:
            return
        if state is CycleState.GUIDED_TAKEOVER:
            self._last_landing_debug_event = None
            self._start_landing_attempt_log(state.name, reason)
        self._state = state
        self._reason = reason
        self._write_landing_attempt_log(
            f"LIFECYCLE state={state.name} reason={reason}"
        )
        state_message = String()
        state_message.data = state.name
        reason_message = String()
        reason_message.data = reason
        self._state_publisher.publish(state_message)
        self._reason_publisher.publish(reason_message)
        try:
            self._vehicle.send_status_text(f"{state.name}: {reason}")
        except Exception as error:
            self.get_logger().warning(
                f"Could not send flight cycle status over MAVLink: {error}"
            )
        self.get_logger().info(f"Flight cycle state: {state.name}: {reason}")
        terminal_outcome = {
            CycleState.DOCKED: "COMPLETED",
            CycleState.WAITING_LAND_TRIGGER: "CANCELLED",
            CycleState.PILOT_OVERRIDE: "CANCELLED",
            CycleState.LINK_LOSS: "CANCELLED",
            CycleState.RETURN_TO_FC: "FAILED",
            CycleState.FC_FALLBACK_LAND: "FAILED",
            CycleState.FAILED_DISARMED: "FAILED",
        }.get(state)
        if terminal_outcome is not None:
            self._finish_landing_attempt_log(
                terminal_outcome, state.name, reason
            )
            try:
                self._touchdown_trace.finish(terminal_outcome, reason)
            except Exception as error:
                self.get_logger().warning(f"Could not finish touchdown trace: {error}")
                self._touchdown_trace.disable()

    def _start_landing_attempt_log(self, state: str, reason: str) -> None:
        try:
            if not self._landing_attempt_log.active:
                self._landing_attempt_log.start(state, reason)
        except Exception as error:
            self._warn_landing_attempt_log(error)

    def _write_landing_attempt_log(self, message: str) -> None:
        try:
            self._landing_attempt_log.write(message)
        except Exception as error:
            self._warn_landing_attempt_log(error)

    def _finish_landing_attempt_log(
        self, outcome: str, state: str, reason: str
    ) -> None:
        try:
            self._landing_attempt_log.finish(outcome, state, reason)
        except Exception as error:
            self._warn_landing_attempt_log(error)

    def _warn_landing_attempt_log(self, error: Exception) -> None:
        try:
            self.get_logger().warning(
                f"Landing attempt log disabled after unexpected error: {error}"
            )
        except Exception:
            pass

    def _safe_record_landing_backend_decision(
        self, decision: Any, now: float
    ) -> None:
        try:
            self._record_landing_backend_decision(decision, now)
        except Exception as error:
            self._warn_landing_attempt_log(error)

    def _record_landing_backend_decision(self, decision: Any, now: float) -> None:
        signature = (
            decision.substate,
            decision.recovery_attempts,
            decision.reason,
            decision.request_disarm,
            decision.disarm_request_id,
            decision.outcome,
        )
        if signature == self._last_landing_debug_event:
            return
        self._last_landing_debug_event = signature
        state = self._vehicle.state
        dock = self._dock_observation(now)
        if dock is None:
            dock_summary = "dock=none"
        else:
            dock_summary = (
                f"dock={dock.mode.name} forward_m={dock.forward_m:.3f} "
                f"left_m={dock.left_m:.3f} distance_m={dock.distance_m:.3f} "
                f"yaw_error_rad={dock.yaw_error_rad:.3f}"
            )
        message = (
            f"PID state={decision.substate} "
            f"recovery={decision.recovery_attempts}/"
            f"{int(self.get_parameter('pid_max_recovery_attempts').value)} "
            f"vehicle_mode={state.mode} armed={state.armed} flying={state.flying} "
            f"{dock_summary}"
        )
        if decision.request_disarm:
            message += f" disarm_request={decision.disarm_request_id}"
        if decision.reason:
            message += f" reason={decision.reason}"
        self._write_landing_attempt_log(message)

    def _positive_parameter(self, name: str, *, allow_zero: bool = False) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0 or (not allow_zero and value == 0.0):
            comparator = "non-negative" if allow_zero else "positive"
            raise ValueError(f"{name} must be finite and {comparator}")
        return value

    def _nonnegative_integer_parameter(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or int(value) != value or int(value) < 0:
            raise ValueError(f"{name} must be a non-negative integer")
        return int(value)

    @staticmethod
    def _command_result(result: Any) -> VehicleCommandResult:
        if not isinstance(result, VehicleCommandResult):
            raise TypeError("vehicle returned an invalid command result")
        return result


def main(args: Optional[list[str]] = None) -> None:
    """Construct, spin, and always close the MAVLink-owning ROS node."""
    rclpy.init(args=args)
    node: Optional[FlightCycleManager] = None
    try:
        node = FlightCycleManager()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.close()
        rclpy.shutdown()

"""Non-blocking validated flight-cycle start, docking handoff, and fallback."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Mapping, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
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
from .mission_contract import (
    MissionContract,
    MissionContractError,
    MissionValidationConfig,
    validate_docking_mission,
)
from .pid import PIDConfig
from .pid_landing import PidLandingConfig
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
        self._mission_confirmation_deadline: Optional[float] = None
        self._mission_confirmation_expected_seq: Optional[int] = None
        self._auto_heartbeat_baseline: Optional[float] = None
        self._mission_current_baseline: Optional[float] = None
        self._validated_mission_total: Optional[int] = None
        self._validated_mission_id: Optional[int] = None
        self._dock_request_queued = False
        self._dock_request_acknowledged = False
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
        self._closed = False

        # Startup validation is deliberately read-only. A later cycle request
        # always starts another download, so it never trusts this result.
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
            "simulation_mode": False,
            "auto_start": False,
            "landing_backend": "pid",
            "update_rate_hz": 20.0,
            "takeoff_altitude_m": 4.0,
            "takeoff_altitude_tolerance_m": 0.25,
            "takeoff_vertical_speed_tolerance_m_s": 0.15,
            "takeoff_stable_time_s": 1.0,
            "mission_handoff_min_loiter_s": 5.0,
            "mission_landing_location_tolerance_m": 2.0,
            "mission_download_timeout_s": 10.0,
            "mavlink_operation_timeout_s": 10.0,
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
            "pid_coarse_horizontal_tolerance_m": 0.2,
            "pid_fine_horizontal_tolerance_m": 0.07,
            "pid_coarse_yaw_tolerance_rad": 0.2,
            "pid_fine_yaw_tolerance_rad": 0.1,
            "pid_max_horizontal_speed_m_s": 0.5,
            "pid_outer_descent_speed_m_s": 0.3,
            "pid_central_descent_speed_m_s": 0.12,
            "pid_recovery_speed_m_s": 0.2,
            "pid_max_yaw_rate_rad_s": 0.35,
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
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _start_cycle_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Queue only a fresh mission validation for one explicit cycle."""
        del request
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
        if self._state is CycleState.DOCKED:
            self._advance_passive_pending()
            return
        if self._state in {CycleState.PILOT_OVERRIDE, CycleState.LINK_LOSS}:
            self._advance_passive_state(now)
            return
        if self._handle_airborne_authority(now):
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
            self._begin_operation(
                "takeoff",
                self._vehicle.takeoff_async(
                    self._positive_parameter("takeoff_altitude_m"), now
                ),
                now,
            )
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
                self._command_direct_land("mission confirmation timed out")
            return
        if self._state is CycleState.AUTO_MISSION:
            self._advance_auto_mission(now)
            return
        if self._state is CycleState.WAIT_FOR_HANDOFF:
            self._advance_wait_for_handoff(now)
            return
        if self._state is CycleState.GUIDED_TAKEOVER:
            self._begin_operation(
                "handoff_guided",
                self._vehicle.set_mode_async(GUIDED_MODE, now),
                now,
            )
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

    def close(self) -> None:
        """Cancel timer, release MAVLink once, then destroy the ROS node."""
        if self._closed:
            return
        self._closed = True
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
            self._transition(CycleState.TAKEOFF, "arm command acknowledged")
        elif name == "takeoff":
            self._stable_takeoff_since = None
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
        self._validated_mission_total = len(result.items)
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

    def _update_takeoff_stability(self, now: float) -> None:
        state = self._vehicle.state
        if not self._fresh_airborne_telemetry(now):
            self._command_direct_land(
                "takeoff telemetry became stale before stable altitude confirmation"
            )
            return
        assert state.position_enu_m is not None
        assert state.velocity_enu_m_s is not None
        altitude_error = abs(
            state.position_enu_m[2]
            - self._positive_parameter("takeoff_altitude_m")
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
        self._begin_operation(
            "handoff_guided",
            self._vehicle.set_mode_async(GUIDED_MODE, now),
            now,
        )

    def _start_shared_search(self, now: float, result: Any) -> None:
        command = self._command_result(result)
        if not (
            command.success
            or command.current_mode == GUIDED_MODE
            or self._vehicle.state.mode == GUIDED_MODE
        ):
            self._operation_failed("handoff_guided", command.message, now)
            return
        if not self._fresh_guided_search_telemetry(now):
            self._route_failure(
                "fresh heartbeat, position, and landed state telemetry "
                "are required before shared dock search",
                grounded_state=CycleState.FAILED_DISARMED,
                now=now,
            )
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
                "landing telemetry became stale before backend start"
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
            f"{self._landing_backend} landing backend started after shared search acquisition",
        )

    def _advance_landing_backend(self, now: float) -> None:
        backend = self._landing_controller
        if backend is None:
            self._begin_fc_handback(now, "landing backend is unavailable")
            return
        if not self._fresh_landing_backend_telemetry(now):
            self._command_direct_land("landing telemetry became stale during docking")
            return
        try:
            decision = backend.update(
                now=now,
                dock=self._dock_observation(now),
                vehicle=self._vehicle_observation(),
            )
        except Exception as error:
            self._begin_fc_handback(
                now,
                f"landing backend update failed: {error}",
            )
            return
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
        if decision.yaw_correction_rad is not None:
            self._vehicle.send_condition_yaw(
                yaw_error_rad=decision.yaw_correction_rad
            )
        if decision.stream_landing_target:
            self._stream_latest_landing_target_once(now)
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
        if decision.request_disarm:
            self._begin_pid_disarm(decision.disarm_request_id, now)
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
            self._command_direct_land("PID disarm transaction lost its controller")
            return True
        notifier = getattr(backend, "notify_disarm_result", None)
        if (
            now > pending.deadline
            and not self._pending_pid_disarm_timeout_notified
            and callable(notifier)
        ):
            notifier(request_id, False, now)
            self._pending_pid_disarm_timeout_notified = True
        if not pending.future.done():
            return False
        timeout_notified = self._pending_pid_disarm_timeout_notified
        try:
            result = pending.future.result()
            success = bool(getattr(result, "success", False))
        except Exception:
            success = False
        self._pending = None
        self._pending_pid_disarm_request_id = None
        self._pending_pid_disarm_timeout_notified = False
        if not timeout_notified and callable(notifier):
            notifier(request_id, success, now)
        return False

    def _begin_pid_disarm(self, request_id: Optional[int], now: float) -> None:
        backend = self._landing_controller
        if request_id is None:
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
            except Exception:
                if callable(notifier):
                    notifier(request_id, False, now)
                return
            self._pending_pid_disarm_request_id = request_id
            self._pending_pid_disarm_timeout_notified = False
            return
        if request_id != self._pending_pid_disarm_request_id and callable(notifier):
            notifier(request_id, False, now)

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
            and state.mission_seq == expected_seq
            and state.mission_current_at > mission_current_baseline
            and mission_state_matches
            and mission_mode_matches
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
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.armed is True
            and state.flying is True
            and state.mode == GUIDED_MODE
            and state.position_enu_m is not None
            and state.velocity_enu_m_s is not None
            and self._fresh_position(now, state)
            and self._fresh_landed_state(now, state)
        )

    def _fresh_landing_backend_telemetry(self, now: float) -> bool:
        state = self._vehicle.state
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.armed is not None
            and state.flying is not None
            and state.position_enu_m is not None
            and state.velocity_enu_m_s is not None
            and self._fresh_position(now, state)
            and self._fresh_landed_state(now, state)
        )

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
            return PidLandingBackend(self._build_pid_landing_config())
        return PreciseLandingBackend(self._build_precise_landing_config())

    def _build_pid_landing_config(self) -> PidLandingConfig:
        max_horizontal = self._positive_parameter("pid_max_horizontal_speed_m_s")
        max_yaw = self._positive_parameter("pid_max_yaw_rate_rad_s")
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
            coarse_horizontal_tolerance_m=self._positive_parameter(
                "pid_coarse_horizontal_tolerance_m"
            ),
            fine_horizontal_tolerance_m=self._positive_parameter(
                "pid_fine_horizontal_tolerance_m"
            ),
            coarse_yaw_tolerance_rad=self._positive_parameter(
                "pid_coarse_yaw_tolerance_rad"
            ),
            fine_yaw_tolerance_rad=self._positive_parameter(
                "pid_fine_yaw_tolerance_rad"
            ),
            max_horizontal_speed_m_s=max_horizontal,
            outer_descent_speed_m_s=self._positive_parameter(
                "pid_outer_descent_speed_m_s"
            ),
            central_descent_speed_m_s=self._positive_parameter(
                "pid_central_descent_speed_m_s"
            ),
            recovery_speed_m_s=self._positive_parameter("pid_recovery_speed_m_s"),
            max_yaw_rate_rad_s=max_yaw,
            x_pid=self._build_pid_axis_config("pid_x", max_horizontal),
            y_pid=self._build_pid_axis_config("pid_y", max_horizontal),
            z_pid=self._build_pid_axis_config(
                "pid_z",
                self._positive_parameter("search_max_vertical_speed_m_s"),
            ),
            yaw_pid=self._build_pid_axis_config("pid_yaw", max_yaw),
        )

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
        self._state = state
        self._reason = reason
        state_message = String()
        state_message.data = state.name
        reason_message = String()
        reason_message.data = reason
        self._state_publisher.publish(state_message)
        self._reason_publisher.publish(reason_message)
        self.get_logger().info(f"Flight cycle state: {state.name}: {reason}")

    def _positive_parameter(self, name: str, *, allow_zero: bool = False) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0 or (not allow_zero and value == 0.0):
            comparator = "non-negative" if allow_zero else "positive"
            raise ValueError(f"{name} must be finite and {comparator}")
        return value

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

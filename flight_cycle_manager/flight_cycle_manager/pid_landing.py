"""Pure state machine for guided PID landing after shared acquisition."""

from dataclasses import dataclass, replace
from enum import Enum, auto
import math
from typing import Optional, Tuple

from .landing_backend import (
    DockObservation,
    DockTargetMode,
    VehicleObservation,
    VelocityCommand,
    VelocityFrame,
)
from .pid import PID, PIDConfig


class PidLandingState(Enum):
    """States in the post-acquisition guided PID landing lifecycle."""

    PID_ALIGN = auto()
    DESCEND_OUTER = auto()
    ACQUIRE_CENTRAL = auto()
    DESCEND_CENTRAL = auto()
    HOLD_CENTRAL = auto()
    RECOVERY_CLIMB = auto()
    REACQUIRE_DOCK = auto()
    TOUCHDOWN_APPROACH = auto()
    DISARM = auto()
    COMPLETE = auto()
    FAILED = auto()


# Debug holds are intentionally limited to pre-touchdown states. Recovery and
# terminal states must retain their normal safety and completion behavior.
PID_HOLD_STATES = frozenset({
    PidLandingState.PID_ALIGN,
    PidLandingState.DESCEND_OUTER,
    PidLandingState.ACQUIRE_CENTRAL,
    PidLandingState.DESCEND_CENTRAL,
    PidLandingState.HOLD_CENTRAL,
})


# Only these transition settings may be changed without rebuilding the backend.
TOUCHDOWN_TRANSITION_FIELDS = (
    "touchdown_commit_height_m", "approach_height_tolerance_m", "touchdown_speed_m_s",
    "touchdown_horizontal_tolerance_m", "touchdown_yaw_tolerance_rad",
    "alignment_stable_time_s", "touchdown_horizontal_speed_tolerance_m_s",
    "touchdown_horizontal_speed_stable_time_s",
    "central_horizontal_tolerance_m", "central_yaw_tolerance_rad",
)


class PidLandingOutcome(Enum):
    """Terminal status carried by a PID landing decision."""

    RUNNING = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass(frozen=True)
class PidLandingDecision:
    state: PidLandingState
    command: VelocityCommand
    outcome: PidLandingOutcome = PidLandingOutcome.RUNNING
    request_disarm: bool = False
    disarm_request_id: Optional[int] = None
    reason: str = ""


@dataclass(frozen=True)
class PidLandingConfig:
    """Tuning and safety bounds for post-acquisition PID landing."""

    tag_stable_time_s: float
    central_transition_distance_m: float
    central_acquire_timeout_s: float
    central_loss_grace_s: float
    recovery_climb_m: float
    recovery_altitude_tolerance_m: float
    max_recovery_attempts: int
    touchdown_commit_height_m: float
    touchdown_speed_m_s: float
    touchdown_timeout_s: float
    landed_state_timeout_s: float
    landed_stable_time_s: float
    disarm_retry_period_s: float
    alignment_stable_time_s: float
    outer_horizontal_tolerance_m: float
    central_horizontal_tolerance_m: float
    outer_yaw_tolerance_rad: float
    central_yaw_tolerance_rad: float
    touchdown_horizontal_tolerance_m: float
    touchdown_yaw_tolerance_rad: float
    touchdown_horizontal_speed_tolerance_m_s: float
    touchdown_horizontal_speed_stable_time_s: float
    max_horizontal_speed_m_s: float
    descend_outer_max_horizontal_speed_m_s: float
    descend_central_max_horizontal_speed_m_s: float
    touchdown_max_horizontal_speed_m_s: float
    outer_descent_speed_m_s: float
    central_descent_speed_m_s: float
    recovery_speed_m_s: float
    max_yaw_rate_rad_s: float
    x_pid: PIDConfig
    y_pid: PIDConfig
    z_pid: PIDConfig
    yaw_pid: PIDConfig
    target_loss_grace_s: float
    fixed_central_reference_mode: str = "off"
    gain_near_distance_m: float = 0.2
    gain_far_distance_m: float = 6.0
    near_pids: Optional[Tuple[PIDConfig, ...]] = None
    far_pids: Optional[Tuple[PIDConfig, ...]] = None
    x_k_v: float = 0.0
    y_k_v: float = 0.0
    z_k_v: float = 0.0
    approach_vertical_response_time_s: float = 0.5
    approach_braking_accel_m_s2: float = 0.5
    approach_height_tolerance_m: float = 0.01

    def __post_init__(self) -> None:
        for axis in ("x", "y", "z"):
            value = getattr(self, f"{axis}_k_v")
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{axis}_k_v must be finite and nonnegative")
        if self.fixed_central_reference_mode not in ("off", "descend_central", "hold_central"):
            raise ValueError("fixed_central_reference_mode must be off, descend_central or hold_central")
        numeric_fields = (
            "tag_stable_time_s",
            "central_transition_distance_m",
            "central_acquire_timeout_s",
            "central_loss_grace_s",
            "recovery_climb_m",
            "recovery_altitude_tolerance_m",
            "touchdown_commit_height_m",
            "touchdown_speed_m_s",
            "touchdown_timeout_s",
            "landed_state_timeout_s",
            "landed_stable_time_s",
            "disarm_retry_period_s",
            "alignment_stable_time_s",
            "outer_horizontal_tolerance_m",
            "central_horizontal_tolerance_m",
            "outer_yaw_tolerance_rad",
            "central_yaw_tolerance_rad",
            "touchdown_horizontal_tolerance_m",
            "touchdown_yaw_tolerance_rad",
            "touchdown_horizontal_speed_tolerance_m_s",
            "touchdown_horizontal_speed_stable_time_s",
            "max_horizontal_speed_m_s",
            "descend_outer_max_horizontal_speed_m_s",
            "descend_central_max_horizontal_speed_m_s",
            "touchdown_max_horizontal_speed_m_s",
            "outer_descent_speed_m_s",
            "central_descent_speed_m_s",
            "recovery_speed_m_s",
            "max_yaw_rate_rad_s",
            "target_loss_grace_s",
            "approach_vertical_response_time_s",
            "approach_braking_accel_m_s2",
            "approach_height_tolerance_m",
        )
        for field in numeric_fields:
            if not math.isfinite(getattr(self, field)):
                raise ValueError(f"{field} must be finite")
        for field in numeric_fields:
            if getattr(self, field) <= 0.0:
                raise ValueError(f"{field} must be positive")
        if (
            not isinstance(self.max_recovery_attempts, int)
            or isinstance(self.max_recovery_attempts, bool)
            or self.max_recovery_attempts < 1
        ):
            raise ValueError("max_recovery_attempts must be at least one")
        if self.recovery_altitude_tolerance_m >= self.recovery_climb_m:
            raise ValueError(
                "recovery_altitude_tolerance_m must be smaller than "
                "recovery_climb_m"
            )
        if self.touchdown_commit_height_m >= self.central_transition_distance_m:
            raise ValueError(
                "touchdown_commit_height_m must be below "
                "central_transition_distance_m"
            )
        if (self.touchdown_commit_height_m + self.approach_height_tolerance_m
                >= self.central_transition_distance_m):
            raise ValueError("approach_height_tolerance_m must keep the arrival band "
                             "below central_transition_distance_m")
        if self.central_horizontal_tolerance_m >= self.outer_horizontal_tolerance_m:
            raise ValueError(
                "central_horizontal_tolerance_m must be smaller than "
                "outer_horizontal_tolerance_m"
            )
        if self.central_yaw_tolerance_rad >= self.outer_yaw_tolerance_rad:
            raise ValueError(
                "central_yaw_tolerance_rad must be smaller than "
                "outer_yaw_tolerance_rad"
            )
        if self.touchdown_horizontal_tolerance_m > self.central_horizontal_tolerance_m:
            raise ValueError(
                "touchdown_horizontal_tolerance_m must not exceed "
                "central_horizontal_tolerance_m"
            )
        if self.touchdown_yaw_tolerance_rad > self.central_yaw_tolerance_rad:
            raise ValueError(
                "touchdown_yaw_tolerance_rad must not exceed "
                "central_yaw_tolerance_rad"
            )
        if (not math.isfinite(self.gain_near_distance_m)
                or not math.isfinite(self.gain_far_distance_m)
                or self.gain_near_distance_m <= 0
                or self.gain_far_distance_m <= self.gain_near_distance_m):
            raise ValueError("gain distances must be finite, positive and near < far")
        for name in ("near_pids", "far_pids"):
            configs = getattr(self, name)
            if configs is not None and (len(configs) != 4 or not all(
                    isinstance(config, PIDConfig) for config in configs)):
                raise ValueError(f"{name} must contain X, Y, Z and yaw PIDConfig")


def _zero_body_command() -> VelocityCommand:
    return VelocityCommand(VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0)


class PidLandingController:
    """Pure landing controller; callers supply all time and observations."""

    def __init__(
        self, config: PidLandingConfig, *, align_only: bool = False,
        hold_at_state: Optional[PidLandingState] = None,
    ):
        if hold_at_state is not None and hold_at_state not in PID_HOLD_STATES:
            raise ValueError("hold_at_state must be a supported pre-touchdown PID state")
        if align_only and hold_at_state not in (None, PidLandingState.PID_ALIGN):
            raise ValueError("alignment-only mode cannot hold at a later state")
        self._config = config
        self._alignment_only_mode = align_only
        self._align_only = align_only or hold_at_state is PidLandingState.PID_ALIGN
        self._hold_at_state = hold_at_state
        horizontal_pid_limit = max(
            config.max_horizontal_speed_m_s,
            config.descend_outer_max_horizontal_speed_m_s,
            config.descend_central_max_horizontal_speed_m_s,
            config.touchdown_max_horizontal_speed_m_s,
            config.x_pid.output_limit, config.y_pid.output_limit,
        )
        self._x_pid = PID(replace(config.x_pid, output_limit=horizontal_pid_limit))
        self._y_pid = PID(replace(config.y_pid, output_limit=horizontal_pid_limit))
        self._z_pid = PID(config.z_pid)
        self._yaw_pid = PID(config.yaw_pid)
        self._near_pids = config.near_pids or self._base_pids()
        self._far_pids = config.far_pids or self._base_pids()
        self._gain_distance_m: Optional[float] = None
        self._started = False
        self._state = PidLandingState.PID_ALIGN
        self._state_started_at = 0.0
        self._alignment_stable_started_at: Optional[float] = None
        self._touchdown_speed_stable_started_at: Optional[float] = None
        self._central_stable_started_at: Optional[float] = None
        self._central_loss_started_at: Optional[float] = None
        self._distance_setpoint_m: Optional[float] = None
        self._align_distance_setpoint_m: Optional[float] = None
        self._align_dock_visible = False
        self._descent_last_updated_at: Optional[float] = None
        self._descent_paused = False
        self._recovery_attempts = 0
        self._recovery_target_altitude_m: Optional[float] = None
        self._dock_stable_started_at: Optional[float] = None
        self._landed_stable_started_at: Optional[float] = None
        self._disarm_request_in_flight = False
        self._disarm_acknowledged = False
        self._disarm_next_retry_at = 0.0
        self._next_disarm_request_id = 0
        self._active_disarm_request_id: Optional[int] = None
        self._failure_reason = ""
        self._target_loss_started_at: Optional[float] = None
        self._touchdown_reference_enu_m: Optional[Tuple[float, float]] = None
        self._touchdown_reference_heading_rad: Optional[float] = None
        self._fixed_reference_enu_m: Optional[Tuple[float, float]] = None
        self._fixed_reference_heading_rad: Optional[float] = None
        self._fixed_reference_pending = False
        self._control_error: Optional[dict] = None
        self._velocity_feedback_terms: dict = {}
        self._applied_horizontal_limit_m_s: Optional[float] = None
        self._approach_height_guard: Optional[dict] = None
        self._touchdown_transition: Optional[dict] = None
        self._touchdown_dwell_reset: Optional[dict] = None

    def start_after_acquisition(self, now: float) -> None:
        """Begin at PID alignment after the shared search acquired the dock."""
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        self._started = True
        self._state = PidLandingState.PID_ALIGN
        self._state_started_at = now
        self._alignment_stable_started_at = None
        self._touchdown_speed_stable_started_at = None
        self._touchdown_transition = None
        self._touchdown_dwell_reset = None
        self._central_stable_started_at = None
        self._central_loss_started_at = None
        self._distance_setpoint_m = None
        self._align_distance_setpoint_m = None
        self._align_dock_visible = False
        self._descent_last_updated_at = None
        self._recovery_attempts = 0
        self._recovery_target_altitude_m = None
        self._dock_stable_started_at = None
        self._target_loss_started_at = None
        self._touchdown_reference_enu_m = None
        self._touchdown_reference_heading_rad = None
        self._fixed_reference_enu_m = None
        self._fixed_reference_heading_rad = None
        self._fixed_reference_pending = False
        self._control_error = None
        self._approach_height_guard = None
        self._landed_stable_started_at = None
        self._disarm_request_in_flight = False
        self._disarm_acknowledged = False
        self._disarm_next_retry_at = now
        self._active_disarm_request_id = None
        self._failure_reason = ""
        self._reset_pids()
        self._gain_distance_m = None
        # A new attempt must not inherit the previous attempt's last tune.
        for pid, config in zip(self._pids(), self._base_pids()):
            pid.reconfigure(config)

    @property
    def recovery_attempts(self) -> int:
        """Number of recovery climbs used by the current landing attempt."""
        return min(self._recovery_attempts, self._config.max_recovery_attempts)

    def set_align_distance_setpoint(self, distance_m: float) -> None:
        """Set the dock-distance target for an active alignment-only attempt."""
        if (not self._align_only or not self._started or not self._align_dock_visible
                or self._align_distance_setpoint_m is None):
            raise ValueError("alignment needs a fresh dock observation before setting Z")
        if isinstance(distance_m, bool) or not isinstance(distance_m, (int, float)) or not math.isfinite(distance_m):
            raise ValueError("dock distance setpoint must be finite")
        if not 0.3 <= distance_m <= 20.0:
            raise ValueError("dock distance setpoint must be between 0.3 and 20 m")
        if distance_m < self._config.touchdown_commit_height_m:
            raise ValueError("dock distance setpoint must not be below the approach height")
        self._align_distance_setpoint_m = float(distance_m)
        self._z_pid.reset()

    def set_hold_at_state(self, state: Optional[PidLandingState]) -> None:
        """Change the hold for this attempt without rewinding or bypassing safety."""
        self.validate_hold_at_state(state)
        if state is self._hold_at_state:
            return
        if self._started and state is self._state and state in (
                PidLandingState.DESCEND_OUTER, PidLandingState.DESCEND_CENTRAL):
            # Drop the advancing descent target; the next visual sample latches
            # its measured distance instead of continuing the old descent.
            self._distance_setpoint_m = None
            self._descent_last_updated_at = None
            self._z_pid.reset()
        if self._started and self._hold_at_state is self._state and state is not self._state:
            self._descent_last_updated_at = None
            if self._state is not PidLandingState.HOLD_CENTRAL:
                self._distance_setpoint_m = None
            self._z_pid.reset()
        if self._state is PidLandingState.HOLD_CENTRAL:
            self._alignment_stable_started_at = None
            self._touchdown_speed_stable_started_at = None
        self._hold_at_state = state
        self._align_only = self._alignment_only_mode or state is PidLandingState.PID_ALIGN

    def validate_hold_at_state(self, state: Optional[PidLandingState]) -> None:
        """Check a proposed live hold before applying other parameter changes."""
        if state is not None and state not in PID_HOLD_STATES:
            raise ValueError("hold_at_state must be a supported pre-touchdown PID state")
        if self._alignment_only_mode and state not in (None, PidLandingState.PID_ALIGN):
            raise ValueError("alignment-only mode cannot hold at a later state")
        if self._started:
            if self._state in (PidLandingState.TOUCHDOWN_APPROACH, PidLandingState.DISARM,
                               PidLandingState.COMPLETE, PidLandingState.FAILED):
                raise ValueError("cannot change PID hold after touchdown commitment or completion")
            stages = (PidLandingState.PID_ALIGN, PidLandingState.DESCEND_OUTER,
                      PidLandingState.ACQUIRE_CENTRAL, PidLandingState.DESCEND_CENTRAL,
                      PidLandingState.HOLD_CENTRAL)
            if self._state is PidLandingState.HOLD_CENTRAL and state not in (
                    None, PidLandingState.HOLD_CENTRAL):
                raise ValueError("release HOLD_CENTRAL with none before selecting another hold")
            if self._state in stages and state in stages and (
                    stages.index(state) < stages.index(self._state)):
                raise ValueError("PID hold state has already been passed")

    def validate_fixed_central_reference_mode(self, mode: str) -> None:
        if not isinstance(mode, str) or mode not in (
                "off", "descend_central", "hold_central"):
            raise ValueError("invalid pid_fixed_central_reference_mode")
        if self._started and self._state in (
                PidLandingState.TOUCHDOWN_APPROACH, PidLandingState.DISARM,
                PidLandingState.COMPLETE, PidLandingState.FAILED):
            raise ValueError("cannot change fixed reference after touchdown commitment or completion")

    def set_fixed_central_reference_mode(self, mode: str) -> None:
        """Change the target source; latch at the next fresh central update if needed."""
        self.validate_fixed_central_reference_mode(mode)
        if mode == self._config.fixed_central_reference_mode:
            return
        self._config = replace(self._config, fixed_central_reference_mode=mode)
        self._fixed_reference_enu_m = None
        self._fixed_reference_heading_rad = None
        self._fixed_reference_pending = (
            self._started and mode != "off" and
            (self._state is PidLandingState.HOLD_CENTRAL or
             (self._state is PidLandingState.DESCEND_CENTRAL and
              mode == "descend_central"))
        )
        self._alignment_stable_started_at = None
        self._touchdown_speed_stable_started_at = None
        self._control_error = None
        self._x_pid.reset()
        self._y_pid.reset()
        self._yaw_pid.reset()

    def update(
        self,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        self._approach_height_guard = None
        self._touchdown_transition = None
        previous_state = self._state
        decision = self._update_decision(now, dock, vehicle)
        if (self._touchdown_transition is None and previous_state in (
                PidLandingState.DESCEND_CENTRAL, PidLandingState.HOLD_CENTRAL)):
            usable, _ = self._usable_dock(dock)
            control = self._control_dock(dock, vehicle) if usable else None
            self._capture_touchdown_transition(now, dock, control, vehicle, evaluated=False)
            self._touchdown_transition["reason"] = decision.reason
        usable, _ = self._usable_dock(dock)
        if (decision.outcome is PidLandingOutcome.RUNNING
                and decision.state in PID_HOLD_STATES and usable):
            limit = (self._config.outer_descent_speed_m_s
                     if decision.state is PidLandingState.DESCEND_OUTER
                     else self._config.central_descent_speed_m_s)
            limit = min(limit, self._z_pid.config.output_limit)
            z = self._protect_approach_height(
                now, dock, vehicle, decision.command.velocity_z, limit,
            )
            if self._control_error is not None and self._control_error.get("z") is not None:
                self._z_pid.track_applied_output(z)
            decision = replace(decision, command=replace(decision.command, velocity_z=z))
        return decision

    def _update_decision(
        self,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        if not self._started:
            raise RuntimeError(
                "start_after_acquisition() must be called before update()"
            )
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if (
            vehicle.armed is False
            and self._state not in (
                PidLandingState.DISARM,
                PidLandingState.COMPLETE,
                PidLandingState.FAILED,
            )
        ):
            return self._fail(now, "vehicle disarmed while mission was active")

        self._control_error = None
        self._velocity_feedback_terms = {}
        self._applied_horizontal_limit_m_s = None
        if self._fixed_reference_pending:
            usable, reason = self._usable_dock(dock)
            if not usable or dock.mode is not DockTargetMode.CENTRAL:
                return self._handle_target_loss(
                    now, vehicle, reason if not usable else "central tag required for fixed reference")
            if not self._latch_fixed_reference(dock, vehicle):
                return self._handle_target_loss(
                    now, vehicle, "vehicle heading unavailable for fixed reference")
            self._fixed_reference_pending = False
            self._target_loss_started_at = None
            # Latching and changing error source cannot satisfy an old alignment dwell.
            self._alignment_stable_started_at = None
            self._touchdown_speed_stable_started_at = None
            self._x_pid.reset()
            self._y_pid.reset()
            self._yaw_pid.reset()
            return PidLandingDecision(self._state, _zero_body_command(),
                                      reason="fixed central reference latched")
        # Select gains from a usable camera distance, not from the landing state.
        # Touchdown retains the last selection because its pose is latched.
        usable, _ = self._usable_dock(dock)
        if usable and self._state not in (
                PidLandingState.TOUCHDOWN_APPROACH, PidLandingState.DISARM,
                PidLandingState.COMPLETE, PidLandingState.FAILED):
            self._apply_distance_gains(dock.distance_m)

        if self._hold_at_state is self._state and self._state is not PidLandingState.PID_ALIGN:
            return self._update_debug_hold(now, dock, vehicle)

        if self._state is PidLandingState.HOLD_CENTRAL:
            return self._release_final_central_hold(now, dock, vehicle)

        if self._state is PidLandingState.PID_ALIGN:
            return self._update_pid_align(now, dock, vehicle)
        if self._state is PidLandingState.DESCEND_OUTER:
            return self._update_descend_outer(now, dock, vehicle)
        if self._state is PidLandingState.ACQUIRE_CENTRAL:
            return self._update_acquire_central(now, dock, vehicle)
        if self._state is PidLandingState.DESCEND_CENTRAL:
            return self._update_descend_central(now, dock, vehicle)
        if self._state is PidLandingState.RECOVERY_CLIMB:
            return self._update_recovery_climb(now, vehicle)
        if self._state is PidLandingState.REACQUIRE_DOCK:
            return self._update_reacquire_dock(now, dock, vehicle)
        if self._state is PidLandingState.TOUCHDOWN_APPROACH:
            return self._update_touchdown_approach(now, vehicle)
        if self._state is PidLandingState.DISARM:
            return self._update_disarm(now, vehicle)
        if self._state is PidLandingState.COMPLETE:
            return PidLandingDecision(
                self._state,
                _zero_body_command(),
                outcome=PidLandingOutcome.COMPLETE,
            )
        if self._state is PidLandingState.FAILED:
            return PidLandingDecision(
                self._state,
                _zero_body_command(),
                outcome=PidLandingOutcome.FAILED,
                reason=self._failure_reason,
            )
        raise RuntimeError(f"state {self._state.name} is not implemented")

    def notify_disarm_result(
        self, request_id: int, success: bool, now: float,
    ) -> None:
        if (
            not isinstance(request_id, int)
            or isinstance(request_id, bool)
            or request_id < 1
        ):
            raise ValueError("request_id must be a positive int")
        if not isinstance(success, bool):
            raise ValueError("success must be a bool")
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if (
            self._state is not PidLandingState.DISARM
            or not self._disarm_request_in_flight
            or request_id != self._active_disarm_request_id
        ):
            return
        self._disarm_request_in_flight = False
        self._active_disarm_request_id = None
        if success:
            self._disarm_acknowledged = True
            return
        self._disarm_next_retry_at = now + self._config.disarm_retry_period_s

    # ---------------------------- State update methods ----------------------------
    def _update_pid_align(
        self, now: float, dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        """
        Update the PID_ALIGN state, which is the first state after acquisition.
        The goal is to align the vehicle with the dock before descending.
        """
        usable, reason = self._usable_dock(dock)
        if not usable:
            if self._align_only:
                self._align_dock_visible = False
                self._alignment_stable_started_at = None
                self._touchdown_speed_stable_started_at = None
                self._target_loss_started_at = None
                self._reset_pids()
                return PidLandingDecision(
                    self._state,
                    _zero_body_command(),
                    reason=f"holding for dock target: {reason}",
                )
            return self._handle_target_loss(now, vehicle, reason)
        self._target_loss_started_at = None

        tolerance = self._tolerance_for(dock.distance_m)
        if self._align_only:
            self._align_dock_visible = True
            if self._align_distance_setpoint_m is None:
                self._align_distance_setpoint_m = max(
                    self._config.touchdown_commit_height_m, dock.distance_m,
                )
                self._z_pid.reset()
            command = self._visual_command(now, dock, vehicle, z_enabled=False)
            velocity_z = self._z_pid.update(
                self._align_distance_setpoint_m - dock.distance_m, now
            )
            self._control_error["z"] = self._align_distance_setpoint_m - dock.distance_m
            velocity_z = self._feedback_vertical(velocity_z, vehicle)
            return PidLandingDecision(
                self._state,
                VelocityCommand(command.frame, command.velocity_x, command.velocity_y,
                                velocity_z, command.yaw_rate_rad_s),
            )
        command = self._visual_command(now, dock, vehicle, z_enabled=False)
        if not self._alignment_is_stable(now, dock, tolerance):
            return PidLandingDecision(self._state, command)

        if dock.distance_m > self._config.central_transition_distance_m:
            self._enter_state(PidLandingState.DESCEND_OUTER, now)
        elif dock.mode is DockTargetMode.CENTRAL:
            if not self._latch_fixed_reference_if_needed("descend_central", dock, vehicle):
                return self._handle_target_loss(now, vehicle, "vehicle heading unavailable for fixed reference")
            self._enter_state(PidLandingState.DESCEND_CENTRAL, now)
        else:
            self._enter_state(PidLandingState.ACQUIRE_CENTRAL, now)
        return PidLandingDecision(self._state, _zero_body_command())

    def _update_debug_hold(
        self, now: float, dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        """Latch at a reached state without advancing descent or disabling safety."""
        usable, reason = self._usable_dock(dock)
        if not usable or (
            self._state in (PidLandingState.DESCEND_CENTRAL, PidLandingState.HOLD_CENTRAL)
            and dock.mode is not DockTargetMode.CENTRAL
        ):
            missing = reason if not usable else "central tag required"
            return self._handle_target_loss(now, vehicle, missing)
        self._target_loss_started_at = None
        if self._state is PidLandingState.ACQUIRE_CENTRAL:
            return PidLandingDecision(self._state, _zero_body_command())
        if self._distance_setpoint_m is None:
            self._distance_setpoint_m = (
                self._config.touchdown_commit_height_m
                if self._state is PidLandingState.HOLD_CENTRAL else max(
                    self._config.touchdown_commit_height_m, dock.distance_m,
                )
            )
            self._z_pid.reset()
        control_dock = self._control_dock(dock, vehicle)
        if control_dock is None:
            return self._handle_target_loss(now, vehicle, "vehicle heading unavailable for fixed reference")
        return PidLandingDecision(
            self._state,
            self._visual_command(
                now, control_dock, vehicle, z_enabled=True,
                vertical_limit_m_s=(
                    self._config.outer_descent_speed_m_s
                    if self._state is PidLandingState.DESCEND_OUTER
                    else self._config.central_descent_speed_m_s
                ),
            ),
        )

    def _release_final_central_hold(
        self, now: float, dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        """Resume touchdown only on fresh central vision, alignment and low lateral speed."""
        usable, reason = self._usable_dock(dock)
        if not usable or dock.mode is not DockTargetMode.CENTRAL:
            return self._handle_target_loss(
                now, vehicle, reason if not usable else "central tag required",
            )
        self._target_loss_started_at = None
        control_dock = self._control_dock(dock, vehicle)
        if control_dock is None:
            return self._handle_target_loss(now, vehicle, "vehicle heading unavailable for fixed reference")
        if self._touchdown_is_ready(now, dock, control_dock, vehicle):
            self._enter_state(PidLandingState.TOUCHDOWN_APPROACH, now)
            self._latch_touchdown_or_fixed_reference(dock, vehicle)
            return PidLandingDecision(self._state, _zero_body_command())
        return self._update_debug_hold(now, dock, vehicle)

    def _update_descend_outer(
        self, now: float, dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        """
        Update the DESCEND_OUTER state, which is the state where the vehicle
        descends towards the dock while maintaining alignment. If the dock is
        lost or the vehicle is not aligned, it will enter recovery or reacquire the dock.
        """
        usable, reason = self._usable_dock(dock)
        if not usable:
            return self._handle_target_loss(now, vehicle, reason)
        self._target_loss_started_at = None

        central_is_stable = self._central_is_stable(now, dock)
        if dock.distance_m <= self._config.central_transition_distance_m:
            if central_is_stable:
                if not self._latch_fixed_reference_if_needed("descend_central", dock, vehicle):
                    return self._handle_target_loss(now, vehicle, "vehicle heading unavailable for fixed reference")
                self._enter_state(PidLandingState.DESCEND_CENTRAL, now)
            else:
                self._enter_state(PidLandingState.ACQUIRE_CENTRAL, now)
            return PidLandingDecision(self._state, _zero_body_command())

        self._advance_distance_setpoint(
            now,
            dock,
            self._config.outer_descent_speed_m_s,
            self._config.outer_horizontal_tolerance_m,
            self._config.outer_yaw_tolerance_rad,
        )
        # update with PID command to align with the dock while descending
        return PidLandingDecision(
            self._state,
            self._visual_command(
                now,
                dock,
                vehicle,
                z_enabled=not self._descent_paused,
                vertical_limit_m_s=self._config.outer_descent_speed_m_s,
            ),
        )

    def _handle_target_loss(
        self, now: float, vehicle: VehicleObservation, reason: str,
    ) -> PidLandingDecision:
        """
        Handle the case where the dock target is lost. If the target has been lost for longer 
        than the grace period, enter recovery. Otherwise, hold position and wait for reacquisition.
        """
        if self._target_loss_started_at is None:
            self._target_loss_started_at = now
        # An observation gap breaks all visual dwell and PID history. Rebase
        # descent on the recovered measurement rather than an old setpoint.
        self._alignment_stable_started_at = None
        self._touchdown_speed_stable_started_at = None
        self._central_stable_started_at = None
        self._distance_setpoint_m = None
        self._descent_last_updated_at = None
        self._reset_pids()
        if now - self._target_loss_started_at >= self._config.target_loss_grace_s:
            return self._enter_recovery(now, vehicle)
        return PidLandingDecision(
            self._state, _zero_body_command(),
            reason=f"holding for dock target: {reason}",
        )

    def _update_reacquire_dock(
        self, now: float, dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        """
        Update the REACQUIRE_DOCK state, which is the state where the vehicle attempts to reacquire 
        the dock after losing it. If the dock is usable and stable, it will transition to PID_ALIGN.
        """
        usable, _ = self._usable_dock(dock)
        if not usable:
            self._dock_stable_started_at = None
        else:
            if self._dock_stable_started_at is None:
                self._dock_stable_started_at = now
            # when the dock is reacquired, enter PID_ALIGN state
            if now - self._dock_stable_started_at >= self._config.tag_stable_time_s:
                self._enter_state(PidLandingState.PID_ALIGN, now)
                return PidLandingDecision(self._state, _zero_body_command())
        if now - self._state_started_at >= self._config.central_acquire_timeout_s:
            return self._enter_recovery(now, vehicle)
        return PidLandingDecision(self._state, _zero_body_command())

    def _update_acquire_central(
        self,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        if self._central_is_stable(now, dock):
            if not self._latch_fixed_reference_if_needed("descend_central", dock, vehicle):
                return self._handle_target_loss(now, vehicle, "vehicle heading unavailable for fixed reference")
            self._enter_state(PidLandingState.DESCEND_CENTRAL, now)
            return PidLandingDecision(self._state, _zero_body_command())
        if now - self._state_started_at >= self._config.central_acquire_timeout_s:
            return self._enter_recovery(now, vehicle)
        return PidLandingDecision(self._state, _zero_body_command())

    def _update_descend_central(
        self,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        usable, reason = self._usable_dock(dock)
        if not usable or dock.mode is not DockTargetMode.CENTRAL:
            if self._central_loss_started_at is None:
                self._central_loss_started_at = now
            self._descent_last_updated_at = None
            self._alignment_stable_started_at = None
            self._touchdown_speed_stable_started_at = None
            self._reset_pids()
            # CHECK IF the central is lost for more than the treshold
            if (
                now - self._central_loss_started_at
                >= self._config.central_loss_grace_s
            ):
                return self._enter_recovery(now, vehicle)
            return PidLandingDecision(self._state, _zero_body_command())

        self._central_loss_started_at = None
        control_dock = self._control_dock(dock, vehicle)
        if control_dock is None:
            return self._handle_target_loss(now, vehicle, "vehicle heading unavailable for fixed reference")
        if self._touchdown_is_ready(now, dock, control_dock, vehicle):
            # used only for debug hold, not for normal landing
            if self._hold_at_state is PidLandingState.HOLD_CENTRAL:
                # Hold the approach height, including after a small undershoot.
                # This is still pre-commit: retain vision and target-loss recovery.
                if not self._latch_fixed_reference_if_needed("hold_central", dock, vehicle):
                    return self._handle_target_loss(now, vehicle, "vehicle heading unavailable for fixed reference")
                self._enter_state(PidLandingState.HOLD_CENTRAL, now)
                self._distance_setpoint_m = self._config.touchdown_commit_height_m
                self._z_pid.reset()
                return PidLandingDecision(self._state, _zero_body_command())

            self._enter_state(PidLandingState.TOUCHDOWN_APPROACH, now)
            # The tag is about to fill the camera, so the last aligned
            # observation becomes the aperture reference for touchdown.
            self._latch_touchdown_or_fixed_reference(dock, vehicle)
            return PidLandingDecision(self._state, _zero_body_command())

        self._advance_distance_setpoint(
            now,
            control_dock,
            self._config.central_descent_speed_m_s,
            self._config.central_horizontal_tolerance_m,
            self._config.central_yaw_tolerance_rad,
        )
        return PidLandingDecision(
            self._state,
            self._visual_command(
                now,
                control_dock,
                vehicle,
                z_enabled=not self._descent_paused,
                vertical_limit_m_s=self._config.central_descent_speed_m_s,
            ),
        )

    def _update_touchdown_approach(
        self, now: float, vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        if now - self._state_started_at >= self._config.touchdown_timeout_s:
            return self._fail(
                now, "touchdown was not confirmed before timeout"
            )

        if self._fresh_on_ground(now, vehicle):
            if self._landed_stable_started_at is None:
                self._landed_stable_started_at = now
            if (
                now - self._landed_stable_started_at
                >= self._config.landed_stable_time_s
            ):
                self._enter_state(PidLandingState.DISARM, now)
                return self._update_disarm(now, vehicle)
        else:
            self._landed_stable_started_at = None

        return PidLandingDecision(
            self._state,
            self._touchdown_descent_command(now, vehicle),
        )

    def _latch_fixed_reference_if_needed(
        self, stage: str, dock: DockObservation, vehicle: VehicleObservation,
    ) -> bool:
        if self._config.fixed_central_reference_mode != stage:
            return True
        return self._latch_fixed_reference(dock, vehicle)

    def _latch_fixed_reference(
        self, dock: DockObservation, vehicle: VehicleObservation,
    ) -> bool:
        heading = self._finite_heading(vehicle)
        if heading is None:
            return False
        east, north, _ = self._finite_position(vehicle)
        self._fixed_reference_enu_m = (
            east + dock.forward_m * math.sin(heading) - dock.left_m * math.cos(heading),
            north + dock.forward_m * math.cos(heading) + dock.left_m * math.sin(heading),
        )
        self._fixed_reference_heading_rad = heading - self._wrap_yaw(dock.yaw_error_rad)
        return True

    def _control_dock(
        self, dock: DockObservation, vehicle: VehicleObservation,
    ) -> Optional[DockObservation]:
        reference = self._fixed_reference_enu_m
        if reference is None:
            return dock
        heading = self._finite_heading(vehicle)
        if heading is None:
            return None
        east, north, _ = self._finite_position(vehicle)
        error_east = reference[0] - east
        error_north = reference[1] - north
        return replace(
            dock,
            forward_m=error_east * math.sin(heading) + error_north * math.cos(heading),
            left_m=-error_east * math.cos(heading) + error_north * math.sin(heading),
            yaw_error_rad=self._wrap_yaw(heading - self._fixed_reference_heading_rad),
        )

    def _latch_touchdown_or_fixed_reference(
        self, dock: DockObservation, vehicle: VehicleObservation,
    ) -> None:
        """
            Freeze the last aligned tag observation as the touchdown reference.
            If a fixed reference is configured, use that instead of the tag.
        """
        if self._fixed_reference_enu_m is not None:
            self._touchdown_reference_enu_m = self._fixed_reference_enu_m
            self._touchdown_reference_heading_rad = self._fixed_reference_heading_rad
        else:
            self._latch_touchdown_reference(dock, vehicle)

    def _latch_touchdown_reference(
        self, dock: DockObservation, vehicle: VehicleObservation,
    ) -> None:
        """Freeze the last aligned tag observation as the touchdown reference.

        Literally treating the *tag* as the reference keeps the vehicle on the
        cube aperture axis instead of wherever the residual alignment offset
        left it at commit.
        """
        position_east_m, position_north_m, _ = self._finite_position(vehicle)
        heading_rad = self._finite_heading(vehicle)
        if heading_rad is None:
            # Without attitude the tag offset cannot be rotated into the local
            # frame, so the sampled position is the best available reference.
            self._touchdown_reference_enu_m = (
                position_east_m, position_north_m,
            )
            self._touchdown_reference_heading_rad = None
            return
        offset_east_m = (
            dock.forward_m * math.sin(heading_rad)
            - dock.left_m * math.cos(heading_rad)
        )
        offset_north_m = (
            dock.forward_m * math.cos(heading_rad)
            + dock.left_m * math.sin(heading_rad)
        )
        self._touchdown_reference_enu_m = (
            position_east_m + offset_east_m,
            position_north_m + offset_north_m,
        )
        # DockObservation.yaw_error_rad is counter-clockwise positive while the
        # attitude yaw is NED clockwise positive, so aligning subtracts it.
        self._touchdown_reference_heading_rad = heading_rad - self._wrap_yaw(
            dock.yaw_error_rad
        )

    def _touchdown_descent_command(
        self, now: float, vehicle: VehicleObservation,
    ) -> VelocityCommand:
        """Descend while holding the latched aperture position and orientation.

        The X, Y and yaw PIDs run exactly as in the visual states, only against
        the latched reference instead of the camera, so ground effect and wall
        torque cannot push the vehicle off the entry axis. Missing attitude
        telemetry falls back to the previous passive descent.
        """
        position_east_m, position_north_m, _ = self._finite_position(vehicle)
        heading_rad = self._finite_heading(vehicle)
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0
        if heading_rad is not None:
            reference_enu_m = self._touchdown_reference_enu_m
            if reference_enu_m is None:
                reference_enu_m = (position_east_m, position_north_m)
                self._touchdown_reference_enu_m = reference_enu_m
            error_east_m = reference_enu_m[0] - position_east_m
            error_north_m = reference_enu_m[1] - position_north_m
            self._control_error = {
                "x": error_east_m * math.sin(heading_rad) + error_north_m * math.cos(heading_rad),
                "y": -error_east_m * math.cos(heading_rad) + error_north_m * math.sin(heading_rad),
                "z": None,
                "yaw": self._wrap_yaw(heading_rad - self._touchdown_reference_heading_rad)
                if self._touchdown_reference_heading_rad is not None else None,
            }
            # NED yaw is clockwise positive, so the forward axis is
            # (sin, cos) and the left axis is (-cos, sin) in East/North.
            velocity_x = self._x_pid.update(
                error_east_m * math.sin(heading_rad)
                + error_north_m * math.cos(heading_rad),
                now,
            )
            velocity_y = self._y_pid.update(
                -error_east_m * math.cos(heading_rad)
                + error_north_m * math.sin(heading_rad),
                now,
            )
            velocity_x, velocity_y = self._feedback_horizontal(
                velocity_x, velocity_y, vehicle,
                self._config.touchdown_max_horizontal_speed_m_s,
            )
            if self._touchdown_reference_heading_rad is not None:
                yaw_rate = self._limit_symmetric(
                    self._yaw_pid.update(
                        self._wrap_yaw(
                            heading_rad - self._touchdown_reference_heading_rad
                        ),
                        now,
                    ),
                    self._config.max_yaw_rate_rad_s,
                )
        return VelocityCommand(
            VelocityFrame.BODY_FLU,
            velocity_x,
            velocity_y,
            -self._config.touchdown_speed_m_s,
            yaw_rate,
        )

    @staticmethod
    def _finite_heading(vehicle: VehicleObservation) -> Optional[float]:
        heading_rad = vehicle.heading_rad
        if heading_rad is None or not math.isfinite(heading_rad):
            return None
        return heading_rad

    def _update_disarm(
        self, now: float, vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        if not self._fresh_on_ground(now, vehicle):
            if vehicle.armed is False:
                return self._fail(
                    now, "vehicle disarmed without fresh ground telemetry"
                )
            self._enter_state(PidLandingState.TOUCHDOWN_APPROACH, now)
            return PidLandingDecision(
                self._state,
                self._touchdown_descent_command(now, vehicle),
            )

        if vehicle.armed is False:
            self._enter_state(PidLandingState.COMPLETE, now)
            return PidLandingDecision(
                self._state,
                _zero_body_command(),
                outcome=PidLandingOutcome.COMPLETE,
            )

        if self._disarm_acknowledged:
            return PidLandingDecision(self._state, _zero_body_command())
        if self._disarm_request_in_flight:
            return PidLandingDecision(self._state, _zero_body_command())
        if now < self._disarm_next_retry_at:
            return PidLandingDecision(self._state, _zero_body_command())

        self._disarm_request_in_flight = True
        self._next_disarm_request_id += 1
        self._active_disarm_request_id = self._next_disarm_request_id
        self._disarm_next_retry_at = now + self._config.disarm_retry_period_s
        return PidLandingDecision(
            self._state,
            _zero_body_command(),
            request_disarm=True,
            disarm_request_id=self._active_disarm_request_id,
        )

    def _update_recovery_climb(
        self, now: float, vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        """
        Update the RECOVERY_CLIMB state, which is the state where the vehicle climbs to a safe altitude in order to reacquire the dock. 
        """
        _, _, altitude_m = self._finite_position(vehicle)
        assert self._recovery_target_altitude_m is not None
        if altitude_m >= (
            self._recovery_target_altitude_m
            - self._config.recovery_altitude_tolerance_m
        ):
            self._enter_state(PidLandingState.REACQUIRE_DOCK, now)
            return PidLandingDecision(self._state, _zero_body_command())
        # Allow the nominal climb duration plus one acquisition window for
        # settling, but never keep commanding a climb indefinitely.
        climb_timeout_s = (
            self._config.recovery_climb_m / self._config.recovery_speed_m_s
            + self._config.central_acquire_timeout_s
        )

        # TEMPORAL FIX
        climb_timeout_s = max(climb_timeout_s, 10.0)

        if now - self._state_started_at >= climb_timeout_s:
            return self._fail(now, "recovery climb timed out")
        return PidLandingDecision(
            self._state,
            VelocityCommand(
                VelocityFrame.BODY_FLU,
                0.0,
                0.0,
                self._config.recovery_speed_m_s,
            ),
        )

    def _enter_recovery(
        self, now: float, vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        self._recovery_attempts += 1
        if self._recovery_attempts > self._config.max_recovery_attempts:
            return self._fail(
                now, "dock target recovery attempts exhausted",
            )
        _, _, altitude_m = self._finite_position(vehicle)
        self._enter_state(PidLandingState.RECOVERY_CLIMB, now)
        self._recovery_target_altitude_m = (
            altitude_m + self._config.recovery_climb_m
        )
        return PidLandingDecision(self._state, _zero_body_command())

    def _visual_command(
        self,
        now: float,
        dock: DockObservation,
        vehicle: VehicleObservation,
        *,
        z_enabled: bool,
        vertical_limit_m_s: Optional[float] = None,
    ) -> VelocityCommand:
        self._control_error = {"x": dock.forward_m, "y": dock.left_m,
                               "yaw": self._wrap_yaw(dock.yaw_error_rad),
                               "z": self._distance_setpoint_m - dock.distance_m
                               if z_enabled else None}
        velocity_x = self._x_pid.update(dock.forward_m, now)
        velocity_y = self._y_pid.update(dock.left_m, now)
        yaw_rate = self._yaw_pid.update(
            self._wrap_yaw(dock.yaw_error_rad), now
        )
        velocity_x, velocity_y = self._feedback_horizontal(
            velocity_x, velocity_y, vehicle, self._horizontal_speed_limit(),
        )
        yaw_rate = max(
            -self._config.max_yaw_rate_rad_s,
            min(self._config.max_yaw_rate_rad_s, yaw_rate),
        )
        velocity_z = 0.0
        if z_enabled:
            assert self._distance_setpoint_m is not None
            velocity_z = self._z_pid.update(
                self._distance_setpoint_m - dock.distance_m,
                now,
            )
            velocity_z = self._feedback_vertical(
                velocity_z, vehicle, vertical_limit_m_s,
            )
        return VelocityCommand(
            VelocityFrame.BODY_FLU,
            velocity_x,
            velocity_y,
            velocity_z,
            yaw_rate,
        )

    def _feedback_horizontal(
        self, x: float, y: float, vehicle: VehicleObservation, limit: float,
    ) -> Tuple[float, float]:
        heading = self._finite_heading(vehicle)
        east, north = vehicle.velocity_enu_m_s[:2]
        if (heading is not None and math.isfinite(east) and math.isfinite(north)):
            measured_x = east * math.sin(heading) + north * math.cos(heading)
            measured_y = -east * math.cos(heading) + north * math.sin(heading)
            self._velocity_feedback_terms.update({
                "x": -self._config.x_k_v * measured_x,
                "y": -self._config.y_k_v * measured_y,
            })
            x -= self._config.x_k_v * measured_x
            y -= self._config.y_k_v * measured_y
        return self._limit_horizontal(x, y, limit)

    def _feedback_vertical(
        self, z: float, vehicle: VehicleObservation, limit: Optional[float] = None,
    ) -> float:
        measured_z = vehicle.velocity_enu_m_s[2]
        if math.isfinite(measured_z):
            self._velocity_feedback_terms["z"] = -self._config.z_k_v * measured_z
            z -= self._config.z_k_v * measured_z
        if limit is not None:
            z = self._limit_symmetric(z, limit)
        else:
            z = self._limit_symmetric(z, self._z_pid.config.output_limit)
        self._z_pid.track_applied_output(z)
        return z

    def reconfigure_velocity_feedback(self, *, x: float, y: float, z: float) -> None:
        self._config = replace(self._config, x_k_v=x, y_k_v=y, z_k_v=z)

    def validate_touchdown_transition(self, config: PidLandingConfig) -> None:
        changed = any(getattr(config, name) != getattr(self._config, name)
                      for name in TOUCHDOWN_TRANSITION_FIELDS)
        if changed and self._state in (
                PidLandingState.TOUCHDOWN_APPROACH, PidLandingState.DISARM,
                PidLandingState.COMPLETE, PidLandingState.FAILED):
            raise ValueError("cannot change touchdown transition after commitment or completion")

    def reconfigure_touchdown_transition(self, config: PidLandingConfig) -> None:
        """Apply validated thresholds; require new dwell evidence, not a new attempt."""
        self.validate_touchdown_transition(config)
        updates = {name: getattr(config, name) for name in TOUCHDOWN_TRANSITION_FIELDS}
        if all(value == getattr(self._config, name) for name, value in updates.items()):
            return
        if config.touchdown_commit_height_m != self._config.touchdown_commit_height_m:
            self._distance_setpoint_m = None
            self._descent_last_updated_at = None
            self._z_pid.reset()
        self._config = replace(self._config, **updates)
        self._alignment_stable_started_at = None
        self._touchdown_speed_stable_started_at = None
        self._touchdown_transition = None
        self._touchdown_dwell_reset = None

    def _touchdown_is_ready(self, now, dock, control_dock, vehicle) -> bool:
        fine_alignment_stable = self._touchdown_alignment_is_stable(now, control_dock, vehicle)
        height_ready = self._touchdown_height_is_ready(dock, vehicle)
        self._capture_touchdown_transition(now, dock, control_dock, vehicle, evaluated=True)
        self._touchdown_transition["fine_alignment_stable"] = fine_alignment_stable
        return height_ready and fine_alignment_stable

    def _capture_touchdown_transition(self, now, dock, control_dock, vehicle, *, evaluated):
        """Snapshot the actual gate inputs/timers without advancing or resetting them."""
        config = self._config
        usable, reason = self._usable_dock(dock)
        central = usable and dock.mode is DockTargetMode.CENTRAL
        control = control_dock if central else None
        checks = {}

        def numeric(key, label, value, limit, unit, field, *, dwell=False):
            value = value if value is not None and math.isfinite(value) else None
            checks[key] = dict(label=label, value=value, limit=limit, unit=unit,
                               parameter=f"pid_{field}", comparison=">=" if dwell else "<=",
                               passed=value is not None and (value >= limit if dwell else value <= limit))

        def elapsed(start):
            return max(0.0, now - start) if evaluated and start is not None else 0.0

        checks["central_vision"] = dict(label="Usable central vision", passed=central,
                                        detail=("central" if central else
                                                "central tag required" if usable else reason))
        checks["control_reference"] = dict(label="Control reference available",
                                           passed=control is not None,
                                           detail="fixed reference" if self._fixed_reference_enu_m is not None
                                           else "camera pose")
        numeric("x", "|X error|", abs(control.forward_m) if control else None,
                config.touchdown_horizontal_tolerance_m, "m", "touchdown_horizontal_tolerance_m")
        numeric("y", "|Y error|", abs(control.left_m) if control else None,
                config.touchdown_horizontal_tolerance_m, "m", "touchdown_horizontal_tolerance_m")
        numeric("yaw", "|Wrapped yaw error|", abs(self._wrap_yaw(control.yaw_error_rad)) if control else None,
                config.touchdown_yaw_tolerance_rad, "rad", "touchdown_yaw_tolerance_rad")
        vx, vy, vz = vehicle.velocity_enu_m_s
        numeric("horizontal_speed", "Measured horizontal speed", math.hypot(vx, vy),
                config.touchdown_horizontal_speed_tolerance_m_s, "m/s",
                "touchdown_horizontal_speed_tolerance_m_s")
        numeric("alignment_dwell", "Alignment dwell", elapsed(self._alignment_stable_started_at),
                config.alignment_stable_time_s, "s", "alignment_stable_time_s", dwell=True)
        numeric("speed_dwell", "Low-speed dwell", elapsed(self._touchdown_speed_stable_started_at),
                config.touchdown_horizontal_speed_stable_time_s, "s",
                "touchdown_horizontal_speed_stable_time_s", dwell=True)
        numeric("height", "|Distance − approach height|", abs(dock.distance_m - config.touchdown_commit_height_m)
                if usable else None, config.approach_height_tolerance_m, "m", "approach_height_tolerance_m")
        numeric("vertical_speed", "|Measured vertical speed|", abs(vz),
                config.touchdown_speed_m_s, "m/s", "touchdown_speed_m_s")
        checks["debug_hold"] = dict(label="Debug hold released", passed=self._hold_at_state not in (
            PidLandingState.DESCEND_CENTRAL, PidLandingState.HOLD_CENTRAL),
            detail=self._hold_at_state.name if self._hold_at_state else "none")
        fine = evaluated and all(checks[key]["passed"] for key in (
            "x", "y", "yaw", "horizontal_speed", "alignment_dwell", "speed_dwell"))
        interrupted = [key for key in ("central_vision", "control_reference", "x", "y", "yaw",
                                       "horizontal_speed") if not checks[key]["passed"]]
        if interrupted:
            self._touchdown_dwell_reset = dict(
                time_s=now, checks={key: dict(checks[key]) for key in interrupted},
            )
        self._touchdown_transition = dict(
            evaluated=evaluated, fine_alignment_stable=fine,
            height_ready=checks["height"]["passed"] and checks["vertical_speed"]["passed"],
            ready=evaluated and all(check["passed"] for check in checks.values()),
            blockers=[key for key, check in checks.items() if not check["passed"]],
            checks=checks, distance_m=dock.distance_m if usable else None,
            approach_height_m=config.touchdown_commit_height_m,
            last_dwell_interruption=self._touchdown_dwell_reset,
        )

    def _touchdown_height_is_ready(
        self, dock: DockObservation, vehicle: VehicleObservation,
    ) -> bool:
        # A bounded arrival band avoids waiting forever for an exact height.
        # A substantial undershoot must first be corrected; rapid vertical
        # motion must first be braked, even if horizontal alignment is good.
        return (abs(dock.distance_m - self._config.touchdown_commit_height_m)
                <= self._config.approach_height_tolerance_m
                and math.isfinite(vehicle.velocity_enu_m_s[2])
                and abs(vehicle.velocity_enu_m_s[2]) <= self._config.touchdown_speed_m_s)

    def _protect_approach_height(
        self, now: float, dock: DockObservation, vehicle: VehicleObservation,
        requested_z: float, vertical_limit: float,
    ) -> float:
        """Bound pre-commit descent with an assumed stopping-response model.

        Camera-to-controller delay is covered by the configured response time;
        receipt age adds time spent waiting after that observation arrived.
        The model is a command guard, not a guaranteed physical clearance.
        """
        height = self._config.touchdown_commit_height_m
        clearance = max(0.0, dock.distance_m - height)
        response = (self._config.approach_vertical_response_time_s
                    + max(0.0, now - dock.received_at))
        accel = self._config.approach_braking_accel_m_s2
        # Invert clearance = speed * response + speed**2 / (2 * accel).
        # This form avoids cancellation when clearance is very small.
        maximum_down = (2.0 * accel * clearance
                        / (math.sqrt((accel * response)**2 + 2.0 * accel * clearance)
                           + accel * response))
        measured_z = vehicle.velocity_enu_m_s[2]
        known_velocity = math.isfinite(measured_z)
        down_speed = max(0.0, -measured_z) if known_velocity else None
        z = max(requested_z, -maximum_down) if known_velocity else max(requested_z, 0.0)
        if known_velocity and down_speed > maximum_down:
            z = max(z, min(vertical_limit, down_speed - maximum_down))
        if dock.distance_m < height:
            # Correct upward even while lateral misalignment has paused Z PID.
            z = max(z, min(vertical_limit, (height - dock.distance_m) / response))
        self._approach_height_guard = {
            "height_m": height,
            "arrival_tolerance_m": self._config.approach_height_tolerance_m,
            "response_time_s": response,
            "braking_accel_m_s2": accel,
            "maximum_descent_speed_m_s": maximum_down,
            "measured_descent_speed_m_s": down_speed,
            "stopping_distance_m": (down_speed * response + down_speed**2 / (2.0 * accel)
                                    if known_velocity else None),
            "requested_vertical_speed_m_s": requested_z,
            "applied_vertical_speed_m_s": z,
            "active": not math.isclose(z, requested_z, rel_tol=1e-9, abs_tol=1e-12),
        }
        return z

    def _advance_distance_setpoint(
        self,
        now: float,
        dock: DockObservation,
        speed_m_s: float,
        horizontal_tolerance_m: float,
        yaw_tolerance_rad: float,
    ) -> None:
        """
            Function used to advance the distance setpoint for the descent while
            ensuring that the vehicle is aligned with the dock within the specified tolerances.
            If the vehicle is not aligned, the descent is paused until alignment is regained.
            This prevents the vehicle from descending too quickly and potentially losing alignment with the dock.
        """
        # Pause immediately at the existing tolerance; resume inside 80% of
        # it. Rebase while paused so an old height error cannot keep descending.
        resume_scale = 0.8 if self._descent_paused else 1.0
        aligned = self._is_aligned(
            dock, horizontal_tolerance_m * resume_scale,
            yaw_tolerance_rad * resume_scale,
        )
        was_paused = self._descent_paused
        self._descent_paused = not aligned
        if self._descent_paused or was_paused:
            self._distance_setpoint_m = max(
                self._config.touchdown_commit_height_m, dock.distance_m,
            )
            self._descent_last_updated_at = now
            self._z_pid.reset()
            return
        if self._distance_setpoint_m is None:
            self._distance_setpoint_m = max(
                self._config.touchdown_commit_height_m, dock.distance_m,
            )
            self._descent_last_updated_at = now
            return
        if self._descent_last_updated_at is None:
            self._descent_last_updated_at = now
            return
        dt = max(0.0, now - self._descent_last_updated_at)
        if self._is_aligned(
            dock, horizontal_tolerance_m, yaw_tolerance_rad
        ):
            self._distance_setpoint_m = max(
                self._config.touchdown_commit_height_m,
                self._distance_setpoint_m - speed_m_s * dt,
            )
        self._descent_last_updated_at = now

    def _touchdown_alignment_is_stable(
        self, now: float, dock: DockObservation, vehicle: VehicleObservation,
    ) -> bool:
        """
        Check if the vehicle is aligned with the dock and has a stable lateral speed for touchdown.
        """
        # Vertical descent is intentional; only measured lateral speed gates
        # touchdown. Both alignment and speed must stay good for their own dwell.
        vx, vy = vehicle.velocity_enu_m_s[:2]
        if (not math.isfinite(vx) or not math.isfinite(vy)
                or math.hypot(vx, vy) > self._config.touchdown_horizontal_speed_tolerance_m_s):
            self._alignment_stable_started_at = None
            self._touchdown_speed_stable_started_at = None
            return False
        aligned = self._alignment_is_stable(now, dock, (
            self._config.touchdown_horizontal_tolerance_m,
            self._config.touchdown_yaw_tolerance_rad,
        ))
        if self._alignment_stable_started_at is None:
            return False
        if self._touchdown_speed_stable_started_at is None:
            self._touchdown_speed_stable_started_at = now
        return (aligned and now - self._touchdown_speed_stable_started_at
                >= self._config.touchdown_horizontal_speed_stable_time_s)

    def _alignment_is_stable(
        self,
        now: float,
        dock: DockObservation,
        tolerance: Tuple[float, float],
    ) -> bool:
        if not self._is_aligned(dock, *tolerance):
            self._alignment_stable_started_at = None
            self._touchdown_speed_stable_started_at = None
            return False
        if self._alignment_stable_started_at is None:
            self._alignment_stable_started_at = now
        return (
            now - self._alignment_stable_started_at
            >= self._config.alignment_stable_time_s
        )

    def _central_is_stable(
        self, now: float, dock: Optional[DockObservation],
    ) -> bool:
        usable, reason = self._usable_dock(dock)
        if not usable or dock.mode is not DockTargetMode.CENTRAL:
            self._central_stable_started_at = None
            return False
        if self._central_stable_started_at is None:
            self._central_stable_started_at = now
        return (
            now - self._central_stable_started_at
            >= self._config.tag_stable_time_s
        )

    def _tolerance_for(self, distance_m: float) -> Tuple[float, float]:
        if distance_m > self._config.central_transition_distance_m:
            return (
                self._config.outer_horizontal_tolerance_m,
                self._config.outer_yaw_tolerance_rad,
            )
        return (
            self._config.central_horizontal_tolerance_m,
            self._config.central_yaw_tolerance_rad,
        )

    @staticmethod
    def _wrap_yaw(yaw_error_rad: float) -> float:
        return math.atan2(math.sin(yaw_error_rad), math.cos(yaw_error_rad))

    def _is_aligned(
        self,
        dock: DockObservation,
        horizontal_tolerance_m: float,
        yaw_tolerance_rad: float,
    ) -> bool:
        return (
            abs(dock.forward_m) <= horizontal_tolerance_m
            and abs(dock.left_m) <= horizontal_tolerance_m
            and abs(self._wrap_yaw(dock.yaw_error_rad)) <= yaw_tolerance_rad
        )

    def _enter_state(self, state: PidLandingState, now: float) -> None:
        if state not in (PidLandingState.DESCEND_CENTRAL, PidLandingState.HOLD_CENTRAL,
                         PidLandingState.TOUCHDOWN_APPROACH, PidLandingState.DISARM):
            self._fixed_reference_enu_m = None
            self._fixed_reference_heading_rad = None
            self._fixed_reference_pending = False
        self._state = state
        self._state_started_at = now
        self._alignment_stable_started_at = None
        self._touchdown_speed_stable_started_at = None
        self._central_stable_started_at = None
        self._distance_setpoint_m = None
        self._descent_last_updated_at = None
        self._descent_paused = False
        self._central_loss_started_at = None
        self._target_loss_started_at = None
        self._dock_stable_started_at = None
        self._landed_stable_started_at = None
        self._disarm_request_in_flight = False
        self._disarm_acknowledged = False
        self._disarm_next_retry_at = now
        self._active_disarm_request_id = None
        self._reset_pids()

    def _fail(self, now: float, reason: str) -> PidLandingDecision:
        self._enter_state(PidLandingState.FAILED, now)
        self._failure_reason = reason
        return PidLandingDecision(
            self._state,
            _zero_body_command(),
            outcome=PidLandingOutcome.FAILED,
            reason=reason,
        )

    def _fresh_on_ground(
        self, now: float, vehicle: VehicleObservation,
    ) -> bool:
        return (
            vehicle.flying is False
            and math.isfinite(vehicle.landed_at)
            and 0.0 <= now - vehicle.landed_at
            <= self._config.landed_state_timeout_s
        )

    @staticmethod
    def _usable_dock(
        dock: Optional[DockObservation],
    ) -> tuple[bool, str]:
        if dock is None:
            return False, "dock observation is None"

        if not isinstance(dock.mode, DockTargetMode):
            return False, f"invalid dock mode: {dock.mode!r}"

        if not math.isfinite(dock.distance_m):
            return False, f"invalid distance_m: {dock.distance_m}"

        if dock.distance_m <= 0.0:
            return False, f"distance_m must be > 0, got {dock.distance_m}"

        values = {
            "forward_m": dock.forward_m,
            "left_m": dock.left_m,
            "distance_m": dock.distance_m,
            "yaw_error_rad": dock.yaw_error_rad,
            "source_stamp_s": dock.source_stamp_s,
            "received_at": dock.received_at,
        }

        for name, value in values.items():
            if not math.isfinite(value):
                return False, f"{name} is not finite: {value}"

        return True, "dock observation is usable"

    @staticmethod
    def _finite_position(
        vehicle: VehicleObservation,
    ) -> Tuple[float, float, float]:
        if len(vehicle.position_enu_m) != 3 or not all(
            math.isfinite(value) for value in vehicle.position_enu_m
        ):
            raise ValueError(
                "vehicle position_enu_m must contain finite XYZ values"
            )
        return vehicle.position_enu_m

    def _horizontal_speed_limit(self) -> float:
        if self._state is PidLandingState.DESCEND_OUTER:
            return self._config.descend_outer_max_horizontal_speed_m_s
        if self._state in (PidLandingState.DESCEND_CENTRAL, PidLandingState.HOLD_CENTRAL):
            return self._config.descend_central_max_horizontal_speed_m_s
        return self._config.max_horizontal_speed_m_s

    def _limit_horizontal(
        self, velocity_x: float, velocity_y: float, limit_m_s: float,
    ) -> Tuple[float, float]:
        self._applied_horizontal_limit_m_s = limit_m_s
        speed = math.hypot(velocity_x, velocity_y)
        if speed > limit_m_s:
            scale = limit_m_s / speed
            velocity_x, velocity_y = velocity_x * scale, velocity_y * scale
        self._x_pid.track_applied_output(velocity_x)
        self._y_pid.track_applied_output(velocity_y)
        return velocity_x, velocity_y

    def _base_pids(self) -> Tuple[PIDConfig, ...]:
        return (self._config.x_pid, self._config.y_pid,
                self._config.z_pid, self._config.yaw_pid)

    def _apply_distance_gains(self, distance_m: Optional[float]) -> None:
        if distance_m is not None:
            self._gain_distance_m = distance_m
        distance_m = self._gain_distance_m
        if distance_m is not None:
            near = self._config.gain_near_distance_m
            far = self._config.gain_far_distance_m
            fraction = max(0.0, min(1.0, (distance_m - near) / (far - near)))
            configs = tuple(replace(near_config, **{
                gain: getattr(near_config, gain) + fraction * (
                    getattr(far_config, gain) - getattr(near_config, gain))
                for gain in ("kp", "ki", "kd")
            }) for near_config, far_config in zip(self._near_pids, self._far_pids))
            configs = self._expand_horizontal_pid_limits(configs)
            for pid, config in zip(self._pids(), configs):
                pid.reconfigure(config)

    def reconfigure_pids(
        self, configs: Tuple[PIDConfig, ...],
        near: Optional[Tuple[PIDConfig, ...]] = None,
        far: Optional[Tuple[PIDConfig, ...]] = None,
        near_distance_m: Optional[float] = None,
        far_distance_m: Optional[float] = None,
    ) -> None:
        if len(configs) != 4 or any(group is not None and len(group) != 4
                                     for group in (near, far)):
            raise ValueError("PID gain groups must contain four axes")
        near_distance_m = (self._config.gain_near_distance_m if near_distance_m is None
                           else near_distance_m)
        far_distance_m = (self._config.gain_far_distance_m if far_distance_m is None
                          else far_distance_m)
        if (not math.isfinite(near_distance_m) or not math.isfinite(far_distance_m)
                or near_distance_m <= 0 or near_distance_m >= far_distance_m):
            raise ValueError("gain distances must be finite, positive and near < far")
        configs = self._expand_horizontal_pid_limits(configs)
        near = self._expand_horizontal_pid_limits(near) if near is not None else None
        far = self._expand_horizontal_pid_limits(far) if far is not None else None
        self._config = replace(self._config, x_pid=configs[0], y_pid=configs[1],
                               z_pid=configs[2], yaw_pid=configs[3],
                               gain_near_distance_m=near_distance_m,
                               gain_far_distance_m=far_distance_m)
        self._near_pids = near or configs
        self._far_pids = far or configs
        if self._gain_distance_m is None:
            for pid, config in zip(self._pids(), configs):
                pid.reconfigure(config)
        else:
            self._apply_distance_gains(None)

    def _expand_horizontal_pid_limits(
        self, configs: Tuple[PIDConfig, ...],
    ) -> Tuple[PIDConfig, ...]:
        limit = max(
            self._config.max_horizontal_speed_m_s,
            self._config.descend_outer_max_horizontal_speed_m_s,
            self._config.descend_central_max_horizontal_speed_m_s,
            self._config.touchdown_max_horizontal_speed_m_s,
            configs[0].output_limit, configs[1].output_limit,
        )
        return (replace(configs[0], output_limit=limit),
                replace(configs[1], output_limit=limit), *configs[2:])

    def _pids(self) -> Tuple[PID, ...]:
        return self._x_pid, self._y_pid, self._z_pid, self._yaw_pid

    def diagnostics(self) -> dict:
        return {
            "touchdown_transition": self._touchdown_transition,
            "approach_height_guard": self._approach_height_guard,
            "approach_height_m": self._config.touchdown_commit_height_m,
            "velocity_feedback_terms": dict(self._velocity_feedback_terms),
            "applied_horizontal_limit_m_s": self._applied_horizontal_limit_m_s,
            "descent_paused": self._descent_paused,
            "fixed_central_reference_mode": self._config.fixed_central_reference_mode,
            "fixed_reference_active": self._fixed_reference_enu_m is not None,
            "fixed_reference_pending": self._fixed_reference_pending,
            "fixed_reference_enu_m": self._fixed_reference_enu_m,
            "fixed_reference_heading_rad": self._fixed_reference_heading_rad,
            "touchdown_reference_enu_m": self._touchdown_reference_enu_m,
            "touchdown_reference_heading_rad": self._touchdown_reference_heading_rad,
            "control_error": self._control_error,
            "pid_hold_at_state": (
                self._hold_at_state.name if self._hold_at_state is not None else "none"
            ),
            "distance_setpoint_m": (
                self._align_distance_setpoint_m if self._align_only
                else self._distance_setpoint_m
            ),
            "alignment_only": self._align_only,
            "align_dock_visible": self._align_dock_visible,
            "gain_distance_m": self._gain_distance_m,
            "gain_near_distance_m": self._config.gain_near_distance_m,
            "gain_far_distance_m": self._config.gain_far_distance_m,
            "velocity_feedback": {axis: getattr(self._config, f"{axis}_k_v")
                                  for axis in ("x", "y", "z")},
            "axes": {axis: pid.diagnostics()
                     for axis, pid in zip(("x", "y", "z", "yaw"), self._pids())},
        }

    @staticmethod
    def _limit_symmetric(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _reset_pids(self) -> None:
        self._x_pid.reset()
        self._y_pid.reset()
        self._z_pid.reset()
        self._yaw_pid.reset()

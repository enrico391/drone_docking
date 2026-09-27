"""Pure state machine for guided PID landing after shared acquisition."""

from dataclasses import dataclass
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
    RECOVERY_CLIMB = auto()
    REACQUIRE_DOCK = auto()
    TOUCHDOWN_APPROACH = auto()
    DISARM = auto()
    COMPLETE = auto()
    FAILED = auto()


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
    coarse_horizontal_tolerance_m: float
    fine_horizontal_tolerance_m: float
    coarse_yaw_tolerance_rad: float
    fine_yaw_tolerance_rad: float
    max_horizontal_speed_m_s: float
    outer_descent_speed_m_s: float
    central_descent_speed_m_s: float
    recovery_speed_m_s: float
    max_yaw_rate_rad_s: float
    x_pid: PIDConfig
    y_pid: PIDConfig
    z_pid: PIDConfig
    yaw_pid: PIDConfig
    target_loss_grace_s: float

    def __post_init__(self) -> None:
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
            "coarse_horizontal_tolerance_m",
            "fine_horizontal_tolerance_m",
            "coarse_yaw_tolerance_rad",
            "fine_yaw_tolerance_rad",
            "max_horizontal_speed_m_s",
            "outer_descent_speed_m_s",
            "central_descent_speed_m_s",
            "recovery_speed_m_s",
            "max_yaw_rate_rad_s",
            "target_loss_grace_s"
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
        if self.fine_horizontal_tolerance_m >= self.coarse_horizontal_tolerance_m:
            raise ValueError(
                "fine_horizontal_tolerance_m must be smaller than "
                "coarse_horizontal_tolerance_m"
            )
        if self.fine_yaw_tolerance_rad >= self.coarse_yaw_tolerance_rad:
            raise ValueError(
                "fine_yaw_tolerance_rad must be smaller than "
                "coarse_yaw_tolerance_rad"
            )


def _zero_body_command() -> VelocityCommand:
    return VelocityCommand(VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0)


class PidLandingController:
    """Pure landing controller; callers supply all time and observations."""

    def __init__(self, config: PidLandingConfig, *, align_only: bool = False):
        self._config = config
        self._align_only = align_only
        self._x_pid = PID(config.x_pid)
        self._y_pid = PID(config.y_pid)
        self._z_pid = PID(config.z_pid)
        self._yaw_pid = PID(config.yaw_pid)
        self._started = False
        self._state = PidLandingState.PID_ALIGN
        self._state_started_at = 0.0
        self._alignment_stable_started_at: Optional[float] = None
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

    def start_after_acquisition(self, now: float) -> None:
        """Begin at PID alignment after the shared search acquired the dock."""
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        self._started = True
        self._state = PidLandingState.PID_ALIGN
        self._state_started_at = now
        self._alignment_stable_started_at = None
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
        self._landed_stable_started_at = None
        self._disarm_request_in_flight = False
        self._disarm_acknowledged = False
        self._disarm_next_retry_at = now
        self._active_disarm_request_id = None
        self._failure_reason = ""
        self._reset_pids()

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
        self._align_distance_setpoint_m = float(distance_m)
        self._z_pid.reset()

    def update(
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
                self._align_distance_setpoint_m = dock.distance_m
                self._z_pid.reset()
            command = self._visual_command(now, dock, z_enabled=False)
            velocity_z = self._z_pid.update(
                self._align_distance_setpoint_m - dock.distance_m, now
            )
            self._z_pid.track_applied_output(velocity_z)
            return PidLandingDecision(
                self._state,
                VelocityCommand(command.frame, command.velocity_x, command.velocity_y,
                                velocity_z, command.yaw_rate_rad_s),
            )
        command = self._visual_command(now, dock, z_enabled=False)
        if not self._alignment_is_stable(now, dock, tolerance):
            return PidLandingDecision(self._state, command)

        if dock.distance_m > self._config.central_transition_distance_m:
            self._enter_state(PidLandingState.DESCEND_OUTER, now)
        elif dock.mode is DockTargetMode.CENTRAL:
            self._enter_state(PidLandingState.DESCEND_CENTRAL, now)
        else:
            self._enter_state(PidLandingState.ACQUIRE_CENTRAL, now)
        return PidLandingDecision(self._state, _zero_body_command())

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
                self._enter_state(PidLandingState.DESCEND_CENTRAL, now)
            else:
                self._enter_state(PidLandingState.ACQUIRE_CENTRAL, now)
            return PidLandingDecision(self._state, _zero_body_command())

        self._advance_distance_setpoint(
            now,
            dock,
            self._config.outer_descent_speed_m_s,
            self._config.coarse_horizontal_tolerance_m,
            self._config.coarse_yaw_tolerance_rad,
        )
        # update with PID command to align with the dock while descending
        return PidLandingDecision(
            self._state,
            self._visual_command(
                now,
                dock,
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
            self._reset_pids()
            # CHECK IF the central is lost for more than the treshold
            if (
                now - self._central_loss_started_at
                >= self._config.central_loss_grace_s
            ):
                return self._enter_recovery(now, vehicle)
            return PidLandingDecision(self._state, _zero_body_command())

        self._central_loss_started_at = None
        fine_alignment_stable = self._alignment_is_stable(
            now,
            dock,
            (
                self._config.fine_horizontal_tolerance_m,
                self._config.fine_yaw_tolerance_rad,
            ),
        )
        if (
            dock.distance_m <= self._config.touchdown_commit_height_m
            and fine_alignment_stable
        ):
            self._enter_state(PidLandingState.TOUCHDOWN_APPROACH, now)
            # The tag is about to fill the camera, so the last aligned
            # observation becomes the aperture reference for touchdown.
            self._latch_touchdown_reference(dock, vehicle)
            return PidLandingDecision(self._state, _zero_body_command())

        self._advance_distance_setpoint(
            now,
            dock,
            self._config.central_descent_speed_m_s,
            self._config.fine_horizontal_tolerance_m,
            self._config.fine_yaw_tolerance_rad,
        )
        return PidLandingDecision(
            self._state,
            self._visual_command(
                now,
                dock,
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
            velocity_x, velocity_y = self._limit_horizontal(
                velocity_x, velocity_y
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
        *,
        z_enabled: bool,
        vertical_limit_m_s: Optional[float] = None,
    ) -> VelocityCommand:
        velocity_x = self._x_pid.update(dock.forward_m, now)
        velocity_y = self._y_pid.update(dock.left_m, now)
        yaw_rate = self._yaw_pid.update(
            self._wrap_yaw(dock.yaw_error_rad), now
        )
        velocity_x, velocity_y = self._limit_horizontal(velocity_x, velocity_y)
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
            if vertical_limit_m_s is not None:
                velocity_z = self._limit_symmetric(
                    velocity_z, vertical_limit_m_s
                )
            self._z_pid.track_applied_output(velocity_z)
        return VelocityCommand(
            VelocityFrame.BODY_FLU,
            velocity_x,
            velocity_y,
            velocity_z,
            yaw_rate,
        )

    def _advance_distance_setpoint(
        self,
        now: float,
        dock: DockObservation,
        speed_m_s: float,
        horizontal_tolerance_m: float,
        yaw_tolerance_rad: float,
    ) -> None:
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
            self._distance_setpoint_m = dock.distance_m
            self._descent_last_updated_at = now
            self._z_pid.reset()
            return
        if self._distance_setpoint_m is None:
            self._distance_setpoint_m = dock.distance_m
            self._descent_last_updated_at = now
            return
        if self._descent_last_updated_at is None:
            self._descent_last_updated_at = now
            return
        dt = max(0.0, now - self._descent_last_updated_at)
        if self._is_aligned(
            dock, horizontal_tolerance_m, yaw_tolerance_rad
        ):
            self._distance_setpoint_m -= speed_m_s * dt
        self._descent_last_updated_at = now

    def _alignment_is_stable(
        self,
        now: float,
        dock: DockObservation,
        tolerance: Tuple[float, float],
    ) -> bool:
        if not self._is_aligned(dock, *tolerance):
            self._alignment_stable_started_at = None
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
                self._config.coarse_horizontal_tolerance_m,
                self._config.coarse_yaw_tolerance_rad,
            )
        return (
            self._config.fine_horizontal_tolerance_m,
            self._config.fine_yaw_tolerance_rad,
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
        self._state = state
        self._state_started_at = now
        self._alignment_stable_started_at = None
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

    def _limit_horizontal(
        self, velocity_x: float, velocity_y: float,
    ) -> Tuple[float, float]:
        speed = math.hypot(velocity_x, velocity_y)
        if speed > self._config.max_horizontal_speed_m_s:
            scale = self._config.max_horizontal_speed_m_s / speed
            velocity_x, velocity_y = velocity_x * scale, velocity_y * scale
        self._x_pid.track_applied_output(velocity_x)
        self._y_pid.track_applied_output(velocity_y)
        return velocity_x, velocity_y

    def reconfigure_pids(self, configs: Tuple[PIDConfig, ...]) -> None:
        for pid, config in zip(self._pids(), configs):
            pid.reconfigure(config)

    def _pids(self) -> Tuple[PID, ...]:
        return self._x_pid, self._y_pid, self._z_pid, self._yaw_pid

    def diagnostics(self) -> dict:
        return {
            "descent_paused": self._descent_paused,
            "distance_setpoint_m": (
                self._align_distance_setpoint_m if self._align_only
                else self._distance_setpoint_m
            ),
            "alignment_only": self._align_only,
            "align_dock_visible": self._align_dock_visible,
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

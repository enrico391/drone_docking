"""Pure state machine for guided PID landing decisions."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Optional, Tuple

from .pid import PID, PIDConfig
from .pymavlink_vehicle import VelocityFrame


_SEARCH_DIRECTION = 1.0


class DockTargetMode(Enum):
    """AprilTag layout currently observed by the camera."""

    OUTER = auto()
    CENTRAL = auto()


class PidLandingState(Enum):
    """All states in the guided PID landing lifecycle."""

    ACQUIRE_DOCK = auto()
    SEARCH_CIRCLE = auto()
    PID_ALIGN = auto()
    DESCEND_OUTER = auto()
    ACQUIRE_CENTRAL = auto()
    DESCEND_CENTRAL = auto()
    RECOVERY_CLIMB = auto()
    TOUCHDOWN_APPROACH = auto()
    DISARM = auto()
    COMPLETE = auto()
    FAILED = auto()


class PidLandingOutcome(Enum):
    """Terminal status carried by a landing decision."""

    RUNNING = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass(frozen=True)
class DockObservation:
    forward_m: float
    left_m: float
    distance_m: float
    yaw_error_rad: float
    mode: DockTargetMode
    source_stamp_s: float
    received_at: float


@dataclass(frozen=True)
class VehicleObservation:
    position_enu_m: Tuple[float, float, float]
    velocity_enu_m_s: Tuple[float, float, float]
    armed: Optional[bool]
    flying: Optional[bool]
    mode: Optional[int]
    landed_at: float


@dataclass(frozen=True)
class VelocityCommand:
    frame: VelocityFrame
    velocity_x: float
    velocity_y: float
    velocity_z: float
    yaw_rate_rad_s: float = 0.0


@dataclass(frozen=True)
class PidLandingDecision:
    state: PidLandingState
    command: VelocityCommand
    outcome: PidLandingOutcome = PidLandingOutcome.RUNNING
    request_disarm: bool = False
    reason: str = ""


@dataclass(frozen=True)
class PidLandingConfig:
    """Tuning and safety bounds for the complete PID landing lifecycle."""

    initial_acquire_time_s: float
    tag_stable_time_s: float
    search_radius_m: float
    search_speed_m_s: float
    search_timeout_s: float
    central_transition_distance_m: float
    central_acquire_timeout_s: float
    central_loss_grace_s: float
    recovery_climb_m: float
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
    search_position_gain: float
    search_position_tolerance_m: float
    search_max_vertical_speed_m_s: float
    x_pid: PIDConfig
    y_pid: PIDConfig
    z_pid: PIDConfig
    yaw_pid: PIDConfig

    def __post_init__(self) -> None:
        numeric_fields = (
            "initial_acquire_time_s", "tag_stable_time_s", "search_radius_m",
            "search_speed_m_s", "search_timeout_s",
            "central_transition_distance_m", "central_acquire_timeout_s",
            "central_loss_grace_s", "recovery_climb_m",
            "touchdown_commit_height_m", "touchdown_speed_m_s",
            "touchdown_timeout_s", "landed_state_timeout_s",
            "landed_stable_time_s", "disarm_retry_period_s",
            "alignment_stable_time_s", "coarse_horizontal_tolerance_m",
            "fine_horizontal_tolerance_m", "coarse_yaw_tolerance_rad",
            "fine_yaw_tolerance_rad", "max_horizontal_speed_m_s",
            "outer_descent_speed_m_s", "central_descent_speed_m_s",
            "recovery_speed_m_s", "max_yaw_rate_rad_s",
            "search_position_gain", "search_position_tolerance_m",
            "search_max_vertical_speed_m_s",
        )
        for field in numeric_fields:
            if not math.isfinite(getattr(self, field)):
                raise ValueError(f"{field} must be finite")
        positive_fields = (
            "initial_acquire_time_s", "tag_stable_time_s", "search_radius_m",
            "search_speed_m_s", "search_timeout_s",
            "central_transition_distance_m", "central_acquire_timeout_s",
            "central_loss_grace_s", "recovery_climb_m",
            "touchdown_commit_height_m", "touchdown_speed_m_s",
            "touchdown_timeout_s", "landed_state_timeout_s",
            "landed_stable_time_s", "disarm_retry_period_s",
            "alignment_stable_time_s", "coarse_horizontal_tolerance_m",
            "fine_horizontal_tolerance_m", "coarse_yaw_tolerance_rad",
            "fine_yaw_tolerance_rad", "max_horizontal_speed_m_s",
            "outer_descent_speed_m_s", "central_descent_speed_m_s",
            "recovery_speed_m_s", "max_yaw_rate_rad_s",
            "search_position_gain", "search_position_tolerance_m",
            "search_max_vertical_speed_m_s",
        )
        for field in positive_fields:
            if getattr(self, field) <= 0.0:
                raise ValueError(f"{field} must be positive")
        if not isinstance(self.max_recovery_attempts, int) or isinstance(
            self.max_recovery_attempts, bool
        ) or self.max_recovery_attempts < 1:
            raise ValueError("max_recovery_attempts must be at least one")
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

    def __init__(self, config: PidLandingConfig):
        self._config = config
        self._x_pid = PID(config.x_pid)
        self._y_pid = PID(config.y_pid)
        self._z_pid = PID(config.z_pid)
        self._yaw_pid = PID(config.yaw_pid)
        self._started = False
        self._state = PidLandingState.ACQUIRE_DOCK
        self._state_started_at = 0.0
        self._dock_x_m = 0.0
        self._dock_y_m = 0.0
        self._hover_altitude_m = 0.0
        self._tag_stable_started_at: Optional[float] = None
        self._search_perimeter_reached = False
        self._search_orbit_started_at: Optional[float] = None
        self._failure_reason = ""

    def start(
        self, now: float, dock_x_m: float, dock_y_m: float,
        hover_altitude_m: float,
    ) -> None:
        if not all(math.isfinite(value) for value in (
            now, dock_x_m, dock_y_m, hover_altitude_m,
        )):
            raise ValueError("start values must be finite")
        self._started = True
        self._state = PidLandingState.ACQUIRE_DOCK
        self._state_started_at = now
        self._dock_x_m = dock_x_m
        self._dock_y_m = dock_y_m
        self._hover_altitude_m = hover_altitude_m
        self._tag_stable_started_at = None
        self._search_perimeter_reached = False
        self._search_orbit_started_at = None
        self._failure_reason = ""
        self._reset_pids()

    def update(
        self, now: float, dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        if not self._started:
            raise RuntimeError("start() must be called before update()")
        if not math.isfinite(now):
            raise ValueError("now must be finite")

        if self._state is PidLandingState.ACQUIRE_DOCK:
            return self._update_acquire_dock(now, dock)
        if self._state is PidLandingState.SEARCH_CIRCLE:
            return self._update_search_circle(now, dock, vehicle)
        if self._state is PidLandingState.FAILED:
            return PidLandingDecision(
                self._state,
                _zero_body_command(),
                outcome=PidLandingOutcome.FAILED,
                reason=self._failure_reason,
            )
        raise RuntimeError(f"state {self._state.name} is not implemented")

    def notify_disarm_result(self, success: bool, now: float) -> None:
        if not isinstance(success, bool):
            raise ValueError("success must be a bool")
        if not math.isfinite(now):
            raise ValueError("now must be finite")

    def _update_acquire_dock(
        self, now: float, dock: Optional[DockObservation],
    ) -> PidLandingDecision:
        if self._usable_dock(dock):
            if self._tag_stable_started_at is None:
                self._tag_stable_started_at = now
            if now - self._tag_stable_started_at >= self._config.tag_stable_time_s:
                self._state = PidLandingState.PID_ALIGN
                self._state_started_at = now
                return PidLandingDecision(self._state, _zero_body_command())
        else:
            self._tag_stable_started_at = None
        if now - self._state_started_at >= self._config.initial_acquire_time_s:
            self._state = PidLandingState.SEARCH_CIRCLE
            self._state_started_at = now
            self._tag_stable_started_at = None
            self._search_perimeter_reached = False
            self._search_orbit_started_at = None
            return PidLandingDecision(self._state, _zero_body_command())
        return PidLandingDecision(self._state, _zero_body_command())

    def _update_search_circle(
        self, now: float, dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> PidLandingDecision:
        if self._usable_dock(dock):
            if self._tag_stable_started_at is None:
                self._tag_stable_started_at = now
            if now - self._tag_stable_started_at >= self._config.tag_stable_time_s:
                self._state = PidLandingState.PID_ALIGN
                self._state_started_at = now
                return PidLandingDecision(self._state, _zero_body_command())
        else:
            self._tag_stable_started_at = None

        if now - self._state_started_at >= self._config.search_timeout_s:
            self._state = PidLandingState.FAILED
            self._failure_reason = "dock tag was not acquired during search"
            return PidLandingDecision(
                self._state,
                _zero_body_command(),
                outcome=PidLandingOutcome.FAILED,
                reason=self._failure_reason,
            )

        position_x, position_y, position_z = self._finite_position(vehicle)
        perimeter_x = self._dock_x_m + self._config.search_radius_m
        perimeter_y = self._dock_y_m
        if not self._search_perimeter_reached:
            if math.hypot(perimeter_x - position_x, perimeter_y - position_y) <= (
                self._config.search_position_tolerance_m
            ):
                self._search_perimeter_reached = True
                self._search_orbit_started_at = now

        if self._search_perimeter_reached:
            assert self._search_orbit_started_at is not None
            elapsed = now - self._search_orbit_started_at
            angular_speed = (
                self._config.search_speed_m_s / self._config.search_radius_m
            )
            angle = _SEARCH_DIRECTION * angular_speed * elapsed
            target_x = self._dock_x_m + self._config.search_radius_m * math.cos(angle)
            target_y = self._dock_y_m + self._config.search_radius_m * math.sin(angle)
            feedforward_x = (
                -_SEARCH_DIRECTION * self._config.search_speed_m_s
                * math.sin(angle)
            )
            feedforward_y = (
                _SEARCH_DIRECTION * self._config.search_speed_m_s
                * math.cos(angle)
            )
        else:
            target_x = perimeter_x
            target_y = perimeter_y
            feedforward_x = 0.0
            feedforward_y = 0.0

        correction_x = self._config.search_position_gain * (target_x - position_x)
        correction_y = self._config.search_position_gain * (target_y - position_y)
        velocity_x, velocity_y = self._limit_horizontal(
            feedforward_x + correction_x,
            feedforward_y + correction_y,
        )
        velocity_z = self._limit_vertical(
            self._config.search_position_gain * (
                self._hover_altitude_m - position_z
            )
        )
        return PidLandingDecision(
            self._state,
            VelocityCommand(
                VelocityFrame.MAP_ENU, velocity_x, velocity_y, velocity_z
            ),
        )

    @staticmethod
    def _usable_dock(dock: Optional[DockObservation]) -> bool:
        return (
            dock is not None
            and isinstance(dock.mode, DockTargetMode)
            and dock.distance_m > 0.0
            and all(math.isfinite(value) for value in (
                dock.forward_m, dock.left_m, dock.distance_m, dock.yaw_error_rad,
                dock.source_stamp_s, dock.received_at,
            ))
        )

    @staticmethod
    def _finite_position(vehicle: VehicleObservation) -> Tuple[float, float, float]:
        if len(vehicle.position_enu_m) != 3 or not all(
            math.isfinite(value) for value in vehicle.position_enu_m
        ):
            raise ValueError("vehicle position_enu_m must contain finite XYZ values")
        return vehicle.position_enu_m

    def _limit_horizontal(self, velocity_x: float, velocity_y: float) -> Tuple[float, float]:
        speed = math.hypot(velocity_x, velocity_y)
        if speed > self._config.max_horizontal_speed_m_s:
            scale = self._config.max_horizontal_speed_m_s / speed
            return velocity_x * scale, velocity_y * scale
        return velocity_x, velocity_y

    def _limit_vertical(self, velocity_z: float) -> float:
        limit = self._config.search_max_vertical_speed_m_s
        return max(-limit, min(limit, velocity_z))

    def _reset_pids(self) -> None:
        self._x_pid.reset()
        self._y_pid.reset()
        self._z_pid.reset()
        self._yaw_pid.reset()

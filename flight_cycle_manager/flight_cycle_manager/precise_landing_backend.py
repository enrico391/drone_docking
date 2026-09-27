"""Pure backend for ArduPilot native precision landing after dock acquisition."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Optional

from .landing_backend import (
    DockObservation,
    DockTargetMode,
    LandingBackendDecision,
    LandingBackendOutcome,
    VehicleObservation,
)
from .landing_wait import (
    LandingWaitAction,
    disarm_wait_action,
    touchdown_wait_action,
)


class PreciseLandingState(Enum):
    """States after the shared dock search has acquired a target."""

    WARMUP = auto()
    REQUEST_LAND = auto()
    WAIT_LANDED = auto()
    WAIT_DISARM = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass(frozen=True)
class PreciseLandingConfig:
    """Timing and optional yaw settings for native precision landing."""

    warmup_s: float
    dock_fresh_timeout_s: float = 0.5
    landed_state_timeout_s: float = 1.0
    touchdown_warn_after_s: float = 30.0
    disarm_warn_after_s: float = 20.0
    yaw_correction_threshold_rad: Optional[float] = None
    land_mode: int = 9

    def __post_init__(self) -> None:
        numeric_fields = (
            "warmup_s",
            "dock_fresh_timeout_s",
            "landed_state_timeout_s",
            "touchdown_warn_after_s",
            "disarm_warn_after_s",
        )
        for field in numeric_fields:
            value = getattr(self, field)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field} must be finite and positive")
        if self.yaw_correction_threshold_rad is not None:
            if (
                not math.isfinite(self.yaw_correction_threshold_rad)
                or self.yaw_correction_threshold_rad < 0.0
            ):
                raise ValueError(
                    "yaw_correction_threshold_rad must be finite and "
                    "non-negative"
                )
        if (
            not isinstance(self.land_mode, int)
            or isinstance(self.land_mode, bool)
            or self.land_mode < 0
        ):
            raise ValueError("land_mode must be a non-negative int")


class PreciseLandingBackend:
    """Make native landing decisions without owning transport or mode changes."""

    def __init__(self, config: PreciseLandingConfig):
        self._config = config
        self._started = False
        self._state = PreciseLandingState.WARMUP
        self._state_started_at = 0.0
        self._warmup_started_at: Optional[float] = None
        self._yaw_requested = False
        self._touchdown_warning_emitted = False
        self._disarm_warning_emitted = False
        self._failure_outcome = LandingBackendOutcome.FAIL_BEFORE_COMMIT
        self._failure_reason = ""

    def start(
        self,
        *,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> None:
        """Begin streaming a target after shared dock search acquisition."""
        del vehicle
        self._validate_now(now)
        self._started = True
        self._state = PreciseLandingState.WARMUP
        self._state_started_at = now
        self._warmup_started_at = now if self._fresh_dock(now, dock) else None
        self._yaw_requested = False
        self._touchdown_warning_emitted = False
        self._disarm_warning_emitted = False
        self._failure_outcome = LandingBackendOutcome.FAIL_BEFORE_COMMIT
        self._failure_reason = ""

    def update(
        self,
        *,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> LandingBackendDecision:
        """Return one native-landing decision for the latest observations."""
        if not self._started:
            raise RuntimeError("start() must be called before update()")
        self._validate_now(now)

        if self._state is PreciseLandingState.WARMUP:
            return self._update_warmup(now, dock)
        if self._state is PreciseLandingState.REQUEST_LAND:
            if not self._fresh_dock(now, dock):
                return self._fail_before_commit(
                    now, "dock target lost before LAND was accepted"
                )
            return self._running(stream_landing_target=True)
        if self._state is PreciseLandingState.WAIT_LANDED:
            return self._update_wait_landed(now, dock, vehicle)
        if self._state is PreciseLandingState.WAIT_DISARM:
            return self._update_wait_disarm(now, dock, vehicle)
        if self._state is PreciseLandingState.COMPLETE:
            return LandingBackendDecision(
                LandingBackendOutcome.COMPLETE, self._state.name
            )
        if self._state is PreciseLandingState.FAILED:
            return LandingBackendDecision(
                self._failure_outcome,
                self._state.name,
                reason=self._failure_reason,
            )
        raise RuntimeError(f"state {self._state.name} is not implemented")

    def notify_land_result(self, success: bool, now: float) -> None:
        """Accept the manager's correlated result for the one LAND request."""
        if not isinstance(success, bool):
            raise ValueError("success must be a bool")
        self._validate_now(now)
        if self._state is not PreciseLandingState.REQUEST_LAND:
            return
        if success:
            self._enter_state(PreciseLandingState.WAIT_LANDED, now)
            return
        self._fail_before_commit(now, "LAND command was rejected")

    def _update_warmup(
        self, now: float, dock: Optional[DockObservation]
    ) -> LandingBackendDecision:
        if not self._fresh_dock(now, dock):
            return self._fail_before_commit(
                now, "dock target lost before LAND was accepted"
            )
        if self._warmup_started_at is None:
            self._warmup_started_at = now

        yaw_correction = self._take_yaw_correction(dock)
        if yaw_correction is not None:
            return self._running(
                stream_landing_target=True,
                yaw_correction_rad=yaw_correction,
            )
        if now - self._warmup_started_at >= self._config.warmup_s:
            self._enter_state(PreciseLandingState.REQUEST_LAND, now)
            return self._running(
                stream_landing_target=True,
                request_land=True,
            )
        return self._running(stream_landing_target=True)

    def _update_wait_landed(
        self,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> LandingBackendDecision:
        departure = self._mode_departure(now, vehicle)
        if departure is not None:
            return departure

        action = touchdown_wait_action(
            armed=self._armed_or_true(vehicle),
            flying=not self._fresh_on_ground(now, vehicle),
            elapsed_s=now - self._state_started_at,
            warn_after_s=self._config.touchdown_warn_after_s,
            warning_emitted=self._touchdown_warning_emitted,
        )
        if action is LandingWaitAction.COMPLETE:
            self._enter_state(PreciseLandingState.COMPLETE, now)
            return LandingBackendDecision(
                LandingBackendOutcome.COMPLETE, self._state.name
            )
        if action is LandingWaitAction.TOUCHDOWN:
            self._enter_state(PreciseLandingState.WAIT_DISARM, now)
            return self._running(stream_landing_target=self._fresh_dock(now, dock))
        if action is LandingWaitAction.WARN:
            self._touchdown_warning_emitted = True
            return self._running(
                stream_landing_target=self._fresh_dock(now, dock),
                reason="landing has not completed",
            )
        return self._running(stream_landing_target=self._fresh_dock(now, dock))

    def _update_wait_disarm(
        self,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> LandingBackendDecision:
        departure = self._mode_departure(now, vehicle)
        if departure is not None:
            return departure

        action = disarm_wait_action(
            armed=self._armed_or_true(vehicle),
            flying=not self._fresh_on_ground(now, vehicle),
            elapsed_s=now - self._state_started_at,
            warn_after_s=self._config.disarm_warn_after_s,
            warning_emitted=self._disarm_warning_emitted,
        )
        if action is LandingWaitAction.COMPLETE:
            self._enter_state(PreciseLandingState.COMPLETE, now)
            return LandingBackendDecision(
                LandingBackendOutcome.COMPLETE, self._state.name
            )
        if action is LandingWaitAction.AIRBORNE:
            self._enter_state(PreciseLandingState.WAIT_LANDED, now)
            return self._running(stream_landing_target=self._fresh_dock(now, dock))
        if action is LandingWaitAction.WARN:
            self._disarm_warning_emitted = True
            return self._running(
                stream_landing_target=self._fresh_dock(now, dock),
                reason="automatic disarm has not completed",
            )
        return self._running(stream_landing_target=self._fresh_dock(now, dock))

    def _take_yaw_correction(self, dock: DockObservation) -> Optional[float]:
        if self._yaw_requested:
            return None
        threshold = self._config.yaw_correction_threshold_rad
        if threshold is None or abs(dock.yaw_error_rad) < threshold:
            return None
        self._yaw_requested = True
        return dock.yaw_error_rad

    def _mode_departure(
        self, now: float, vehicle: VehicleObservation
    ) -> Optional[LandingBackendDecision]:
        if vehicle.mode is None or vehicle.mode == self._config.land_mode:
            return None
        return self._fail_after_commit(now, "vehicle left LAND")

    def _fresh_on_ground(self, now: float, vehicle: VehicleObservation) -> bool:
        return (
            vehicle.flying is False
            and self._finite(vehicle.landed_at)
            and 0.0 <= now - vehicle.landed_at
            <= self._config.landed_state_timeout_s
        )

    def _fresh_dock(self, now: float, dock: Optional[DockObservation]) -> bool:
        return (
            dock is not None
            and isinstance(dock.mode, DockTargetMode)
            and dock.distance_m > 0.0
            and all(
                self._finite(value)
                for value in (
                    dock.forward_m,
                    dock.left_m,
                    dock.distance_m,
                    dock.yaw_error_rad,
                    dock.source_stamp_s,
                    dock.received_at,
                )
            )
            and 0.0 <= now - dock.received_at
            <= self._config.dock_fresh_timeout_s
        )

    @staticmethod
    def _armed_or_true(vehicle: VehicleObservation) -> bool:
        return vehicle.armed if isinstance(vehicle.armed, bool) else True

    def _fail_before_commit(
        self, now: float, reason: str
    ) -> LandingBackendDecision:
        self._failure_outcome = LandingBackendOutcome.FAIL_BEFORE_COMMIT
        return self._fail(now, reason)

    def _fail_after_commit(
        self, now: float, reason: str
    ) -> LandingBackendDecision:
        self._failure_outcome = LandingBackendOutcome.FAIL_AFTER_COMMIT
        return self._fail(now, reason)

    def _fail(self, now: float, reason: str) -> LandingBackendDecision:
        self._enter_state(PreciseLandingState.FAILED, now)
        self._failure_reason = reason
        return LandingBackendDecision(
            self._failure_outcome, self._state.name, reason=reason
        )

    def _running(
        self,
        *,
        stream_landing_target: bool = False,
        request_land: bool = False,
        yaw_correction_rad: Optional[float] = None,
        reason: str = "",
    ) -> LandingBackendDecision:
        return LandingBackendDecision(
            LandingBackendOutcome.RUNNING,
            self._state.name,
            stream_landing_target=stream_landing_target,
            request_land=request_land,
            yaw_correction_rad=yaw_correction_rad,
            reason=reason,
        )

    def _enter_state(self, state: PreciseLandingState, now: float) -> None:
        self._state = state
        self._state_started_at = now
        if state is PreciseLandingState.WAIT_LANDED:
            self._touchdown_warning_emitted = False
        if state is PreciseLandingState.WAIT_DISARM:
            self._disarm_warning_emitted = False

    @staticmethod
    def _finite(value: object) -> bool:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    @staticmethod
    def _validate_now(now: float) -> None:
        if not PreciseLandingBackend._finite(now):
            raise ValueError("now must be finite")

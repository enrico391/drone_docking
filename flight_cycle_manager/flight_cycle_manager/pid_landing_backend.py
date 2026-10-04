"""Common landing-backend adapter for the guided PID controller."""

from typing import Optional

from .landing_backend import (
    DockObservation,
    LandingBackendDecision,
    LandingBackendOutcome,
    VehicleObservation,
)
from .pid_landing import (
    PidLandingConfig,
    PidLandingController,
    PidLandingDecision,
    PidLandingOutcome,
    PidLandingState,
)
from .pid import PIDConfig


_COMMITTED_STATES = frozenset(
    (
        PidLandingState.TOUCHDOWN_APPROACH,
        PidLandingState.DISARM,
        PidLandingState.COMPLETE,
    )
)


class PidLandingBackend:
    """Map pure PID decisions into the shared landing-backend contract."""

    def __init__(
        self, config: PidLandingConfig, *, align_only: bool = False,
        hold_at_state: Optional[PidLandingState] = None,
    ):
        self._controller = PidLandingController(
            config, align_only=align_only, hold_at_state=hold_at_state,
        )
        self._touchdown_committed = False

    def reconfigure_pids(
        self, configs: tuple[PIDConfig, ...],
        near: tuple[PIDConfig, ...] | None = None,
        far: tuple[PIDConfig, ...] | None = None,
        near_distance_m: float | None = None,
        far_distance_m: float | None = None,
    ) -> None:
        self._controller.reconfigure_pids(
            configs, near, far, near_distance_m, far_distance_m,
        )

    def diagnostics(self) -> dict:
        return self._controller.diagnostics()

    def set_align_distance_setpoint(self, distance_m: float) -> None:
        self._controller.set_align_distance_setpoint(distance_m)

    def set_hold_at_state(self, state: Optional[PidLandingState]) -> None:
        self._controller.set_hold_at_state(state)

    def validate_hold_at_state(self, state: Optional[PidLandingState]) -> None:
        self._controller.validate_hold_at_state(state)

    def validate_fixed_central_reference_mode(self, mode: str) -> None:
        self._controller.validate_fixed_central_reference_mode(mode)

    def set_fixed_central_reference_mode(self, mode: str) -> None:
        self._controller.set_fixed_central_reference_mode(mode)

    def start(
        self,
        *,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> None:
        del dock, vehicle
        self._touchdown_committed = False
        self._controller.start_after_acquisition(now)

    def update(
        self,
        *,
        now: float,
        dock: Optional[DockObservation],
        vehicle: VehicleObservation,
    ) -> LandingBackendDecision:
        controller_decision = self._controller.update(now, dock, vehicle)
        if controller_decision.state in _COMMITTED_STATES:
            self._touchdown_committed = True
        return self._map_decision(controller_decision)

    def notify_disarm_result(
        self, request_id: int, success: bool, now: float,
    ) -> None:
        self._controller.notify_disarm_result(request_id, success, now)

    def _map_decision(
        self, decision: PidLandingDecision,
    ) -> LandingBackendDecision:
        if decision.outcome is PidLandingOutcome.COMPLETE:
            outcome = LandingBackendOutcome.COMPLETE
        elif decision.outcome is PidLandingOutcome.FAILED:
            outcome = (
                LandingBackendOutcome.FAIL_AFTER_COMMIT
                if self._touchdown_committed
                else LandingBackendOutcome.FAIL_BEFORE_COMMIT
            )
        else:
            outcome = LandingBackendOutcome.RUNNING
        return LandingBackendDecision(
            outcome=outcome,
            substate=decision.state.name,
            velocity=decision.command,
            request_disarm=decision.request_disarm,
            disarm_request_id=decision.disarm_request_id,
            reason=decision.reason,
            recovery_attempts=self._controller.recovery_attempts,
        )

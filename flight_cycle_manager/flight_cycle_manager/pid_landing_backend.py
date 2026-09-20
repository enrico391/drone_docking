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


_COMMITTED_STATES = frozenset(
    (
        PidLandingState.TOUCHDOWN_APPROACH,
        PidLandingState.DISARM,
        PidLandingState.COMPLETE,
    )
)


class PidLandingBackend:
    """Map pure PID decisions into the shared landing-backend contract."""

    def __init__(self, config: PidLandingConfig):
        self._controller = PidLandingController(config)
        self._touchdown_committed = False

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

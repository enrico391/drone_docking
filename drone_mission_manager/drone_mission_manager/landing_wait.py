"""Pure decision helpers for event-driven landing completion waits."""

from enum import Enum, auto


class LandingWaitAction(Enum):
    """Actions produced while waiting for touchdown or automatic disarm."""

    WAIT = auto()
    WARN = auto()
    AIRBORNE = auto()
    TOUCHDOWN = auto()
    COMPLETE = auto()


def touchdown_wait_action(
    *,
    armed: bool,
    flying: bool,
    elapsed_s: float,
    warn_after_s: float,
    warning_emitted: bool,
) -> LandingWaitAction:
    """Choose the next action while ArduPilot is executing LAND."""
    if not armed:
        return LandingWaitAction.COMPLETE
    if not flying:
        return LandingWaitAction.TOUCHDOWN
    if elapsed_s >= warn_after_s and not warning_emitted:
        return LandingWaitAction.WARN
    return LandingWaitAction.WAIT


def disarm_wait_action(
    *,
    armed: bool,
    flying: bool,
    elapsed_s: float,
    warn_after_s: float,
    warning_emitted: bool,
) -> LandingWaitAction:
    """Choose the next action after touchdown while awaiting auto-disarm."""
    if not armed:
        return LandingWaitAction.COMPLETE
    if flying:
        return LandingWaitAction.AIRBORNE
    if elapsed_s >= warn_after_s and not warning_emitted:
        return LandingWaitAction.WARN
    return LandingWaitAction.WAIT

"""Tests for event-driven landing and disarm wait decisions."""

from flight_cycle_manager.landing_wait import (
    LandingWaitAction,
    disarm_wait_action,
    touchdown_wait_action,
)


def test_slow_landing_warns_instead_of_failing():
    """An armed flying vehicle produces one soft-timeout warning."""
    action = touchdown_wait_action(
        armed=True,
        flying=True,
        elapsed_s=31.0,
        warn_after_s=30.0,
        warning_emitted=False,
    )

    assert action == LandingWaitAction.WARN


def test_landing_warning_is_only_emitted_once():
    """After the warning, a slow landing continues to wait normally."""
    action = touchdown_wait_action(
        armed=True,
        flying=True,
        elapsed_s=90.0,
        warn_after_s=30.0,
        warning_emitted=True,
    )

    assert action == LandingWaitAction.WAIT


def test_not_flying_reports_touchdown_while_still_armed():
    """Touchdown and disarm remain distinct events."""
    action = touchdown_wait_action(
        armed=True,
        flying=False,
        elapsed_s=40.0,
        warn_after_s=30.0,
        warning_emitted=True,
    )

    assert action == LandingWaitAction.TOUCHDOWN


def test_disarmed_vehicle_completes_from_either_wait_phase():
    """A definitive disarm completes even if touchdown status was missed."""
    touchdown_action = touchdown_wait_action(
        armed=False,
        flying=False,
        elapsed_s=12.0,
        warn_after_s=30.0,
        warning_emitted=False,
    )
    disarm_action = disarm_wait_action(
        armed=False,
        flying=False,
        elapsed_s=12.0,
        warn_after_s=20.0,
        warning_emitted=False,
    )

    assert touchdown_action == LandingWaitAction.COMPLETE
    assert disarm_action == LandingWaitAction.COMPLETE


def test_delayed_disarm_warns_and_continues_waiting():
    """An armed landed vehicle is not converted into mission failure."""
    first_action = disarm_wait_action(
        armed=True,
        flying=False,
        elapsed_s=21.0,
        warn_after_s=20.0,
        warning_emitted=False,
    )
    later_action = disarm_wait_action(
        armed=True,
        flying=False,
        elapsed_s=60.0,
        warn_after_s=20.0,
        warning_emitted=True,
    )

    assert first_action == LandingWaitAction.WARN
    assert later_action == LandingWaitAction.WAIT


def test_flying_again_returns_to_touchdown_wait():
    """A bounce cannot leave the mission stuck in the disarm phase."""
    action = disarm_wait_action(
        armed=True,
        flying=True,
        elapsed_s=2.0,
        warn_after_s=20.0,
        warning_emitted=False,
    )

    assert action == LandingWaitAction.AIRBORNE

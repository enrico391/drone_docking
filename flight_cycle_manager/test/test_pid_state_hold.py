"""Debug PID hold points must not advance into touchdown or disarm."""

import pytest

from flight_cycle_manager.landing_backend import LandingBackendOutcome
from flight_cycle_manager.pid_landing import PidLandingState
from flight_cycle_manager.pid_landing_backend import PidLandingBackend
from test_pid_landing_backend import (
    central_observation, dock_observation, enter_central_descent,
    pid_config, vehicle_observation,
)


@pytest.mark.parametrize("hold", [
    PidLandingState.PID_ALIGN,
    PidLandingState.DESCEND_OUTER,
    PidLandingState.ACQUIRE_CENTRAL,
    PidLandingState.DESCEND_CENTRAL,
])
def test_hold_keeps_state_running_with_no_disarm_or_forward_descent(hold):
    backend = PidLandingBackend(pid_config(), hold_at_state=hold)
    vehicle = vehicle_observation()
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle)
    states = []
    for i in range(1, 45):
        now = i * 0.2
        # Enter the outer stage, then acquire the central tag near its boundary.
        if i < 10 or hold is PidLandingState.DESCEND_OUTER:
            dock = dock_observation(now, distance_m=2.0)
        elif hold is PidLandingState.ACQUIRE_CENTRAL:
            dock = dock_observation(now, distance_m=0.9)
        else:
            dock = central_observation(now, distance_m=0.9)
        decision = backend.update(now=now, dock=dock, vehicle=vehicle)
        states.append(decision.substate)
        assert decision.outcome is LandingBackendOutcome.RUNNING
        assert not decision.request_disarm
        if decision.substate == hold.name and hold is not PidLandingState.PID_ALIGN:
            assert decision.velocity.velocity_z == 0.0
    assert hold.name in states
    assert states[-1] == hold.name


def test_central_hold_stops_on_target_loss_then_enters_recovery():
    backend = PidLandingBackend(pid_config(), hold_at_state=PidLandingState.DESCEND_CENTRAL)
    vehicle = vehicle_observation()
    backend.start(now=0.0, dock=central_observation(0.0), vehicle=vehicle)
    backend.update(now=0.1, dock=central_observation(0.1), vehicle=vehicle)
    backend.update(now=1.2, dock=central_observation(1.2), vehicle=vehicle)
    lost = backend.update(now=1.3, dock=None, vehicle=vehicle)
    assert lost.substate == "DESCEND_CENTRAL"
    assert lost.velocity.velocity_z == 0.0
    recovery = backend.update(now=5.0, dock=None, vehicle=vehicle)
    assert recovery.substate == "RECOVERY_CLIMB"
    assert recovery.outcome is LandingBackendOutcome.RUNNING
    assert not recovery.request_disarm


def enter_final_central_hold(backend):
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    enter_central_descent(backend)
    before = backend.update(now=1.8, dock=central_observation(1.8, distance_m=0.20),
                            vehicle=vehicle_observation())
    assert before.substate == "DESCEND_CENTRAL"
    assert backend.update(
        now=2.81, dock=central_observation(2.81, distance_m=0.20),
        vehicle=vehicle_observation(),
    ).substate == "HOLD_CENTRAL"


def test_final_central_hold_waits_for_commit_conditions_and_controls_hover():
    backend = PidLandingBackend(pid_config(), hold_at_state=PidLandingState.HOLD_CENTRAL)
    enter_final_central_hold(backend)
    # The entry command is zero; subsequent updates regulate distance and alignment.
    for now, distance in ((3.0, 0.20), (3.2, 0.25), (3.4, 0.15), (20.0, 0.20)):
        decision = backend.update(now=now, dock=central_observation(now, distance_m=distance),
                                  vehicle=vehicle_observation())
        assert decision.substate == "HOLD_CENTRAL"
        assert decision.outcome is LandingBackendOutcome.RUNNING
        assert not decision.request_disarm
        assert decision.velocity.velocity_x != 0.0
        if distance == 0.25:
            assert decision.velocity.velocity_z < 0.0
        if distance == 0.15:
            assert decision.velocity.velocity_z > 0.0


def test_final_central_hold_still_recovers_from_lost_central_tag():
    backend = PidLandingBackend(pid_config(), hold_at_state=PidLandingState.HOLD_CENTRAL)
    enter_final_central_hold(backend)
    lost = backend.update(now=2.9, dock=None, vehicle=vehicle_observation())
    assert lost.substate == "HOLD_CENTRAL"
    assert lost.velocity.velocity_z == 0.0
    recovery = backend.update(now=3.7, dock=None, vehicle=vehicle_observation())
    assert recovery.substate == "RECOVERY_CLIMB"
    assert recovery.outcome is LandingBackendOutcome.RUNNING
    assert not recovery.request_disarm


def test_without_final_hold_central_descent_enters_touchdown():
    backend = PidLandingBackend(pid_config())
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    enter_central_descent(backend)
    backend.update(now=1.8, dock=central_observation(1.8, distance_m=0.20),
                   vehicle=vehicle_observation())
    decision = backend.update(now=2.81, dock=central_observation(2.81, distance_m=0.20),
                              vehicle=vehicle_observation())
    assert decision.substate == "TOUCHDOWN_APPROACH"


def test_live_hold_in_central_descent_and_release_to_final_hold():
    backend = PidLandingBackend(pid_config())
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    enter_central_descent(backend)
    backend.set_hold_at_state(PidLandingState.DESCEND_CENTRAL)
    held = backend.update(now=2.81, dock=central_observation(2.81, distance_m=0.20),
                          vehicle=vehicle_observation())
    assert held.substate == "DESCEND_CENTRAL"
    backend.set_hold_at_state(PidLandingState.HOLD_CENTRAL)
    backend.update(now=3.0, dock=central_observation(3.0, distance_m=0.20),
                   vehicle=vehicle_observation())
    assert backend.update(now=4.1, dock=central_observation(4.1, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "HOLD_CENTRAL"
    backend.set_hold_at_state(None)
    assert backend.update(now=4.2, dock=central_observation(4.2, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "HOLD_CENTRAL"
    assert backend.update(now=5.3, dock=central_observation(5.3, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "TOUCHDOWN_APPROACH"
    with pytest.raises(ValueError, match="commitment"):
        backend.set_hold_at_state(PidLandingState.HOLD_CENTRAL)


def test_live_hold_rejects_passed_stage_without_changing_descent():
    backend = PidLandingBackend(pid_config())
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    enter_central_descent(backend)
    with pytest.raises(ValueError, match="passed"):
        backend.set_hold_at_state(PidLandingState.PID_ALIGN)
    assert backend.update(now=1.8, dock=central_observation(1.8, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "DESCEND_CENTRAL"


def test_live_release_from_central_descent_hold_resumes_progress():
    backend = PidLandingBackend(pid_config(), hold_at_state=PidLandingState.DESCEND_CENTRAL)
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    enter_central_descent(backend)
    assert backend.update(now=1.8, dock=central_observation(1.8, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "DESCEND_CENTRAL"
    backend.set_hold_at_state(None)
    # Releasing a hold must not skip the fine-alignment dwell.
    assert backend.update(now=2.0, dock=central_observation(2.0, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "DESCEND_CENTRAL"
    assert backend.update(now=3.1, dock=central_observation(3.1, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "TOUCHDOWN_APPROACH"


def test_releasing_final_hold_requires_fresh_central_tag_and_stable_alignment():
    backend = PidLandingBackend(pid_config(), hold_at_state=PidLandingState.HOLD_CENTRAL)
    enter_final_central_hold(backend)
    backend.set_hold_at_state(None)
    assert backend.update(now=3.0, dock=None,
                          vehicle=vehicle_observation()).substate == "HOLD_CENTRAL"
    assert backend.update(now=3.1, dock=central_observation(3.1, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "HOLD_CENTRAL"
    # Central-tag loss resets the fine-alignment dwell, so returning at the
    # commit height cannot cause an immediate touchdown.
    assert backend.update(now=4.0, dock=dock_observation(4.0, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "HOLD_CENTRAL"
    assert backend.update(now=4.1, dock=central_observation(4.1, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "HOLD_CENTRAL"
    assert backend.update(now=5.2, dock=central_observation(5.2, distance_m=0.20),
                          vehicle=vehicle_observation()).substate == "TOUCHDOWN_APPROACH"


def test_releasing_final_hold_waits_for_low_horizontal_speed_dwell():
    backend = PidLandingBackend(pid_config(), hold_at_state=PidLandingState.HOLD_CENTRAL)
    enter_final_central_hold(backend)
    backend.set_hold_at_state(None)
    def update(now, velocity):
        return backend.update(
            now=now, dock=central_observation(now, distance_m=0.20),
            vehicle=vehicle_observation(velocity_enu_m_s=velocity),
        ).substate

    assert update(3.0, (0.0, 0.0, 0.0)) == "HOLD_CENTRAL"
    assert update(4.1, (0.2, 0.0, 0.0)) == "HOLD_CENTRAL"
    assert update(4.2, (0.0, 0.0, 0.0)) == "HOLD_CENTRAL"
    assert update(5.3, (0.0, 0.0, 0.0)) == "TOUCHDOWN_APPROACH"


def test_releasing_final_hold_waits_for_configured_speed_dwell():
    backend = PidLandingBackend(
        pid_config(touchdown_horizontal_speed_stable_time_s=2.0),
        hold_at_state=PidLandingState.HOLD_CENTRAL,
    )
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    enter_central_descent(backend)
    for now in (1.8, 2.81, 3.81):
        entered = backend.update(
            now=now, dock=central_observation(now, distance_m=0.20),
            vehicle=vehicle_observation(),
        )
    assert entered.substate == "HOLD_CENTRAL"
    backend.set_hold_at_state(None)
    for now, expected in ((4.0, "HOLD_CENTRAL"),
                          (5.1, "HOLD_CENTRAL"),
                          (5.99, "HOLD_CENTRAL"),
                          (6.01, "TOUCHDOWN_APPROACH")):
        decision = backend.update(
            now=now, dock=central_observation(now, distance_m=0.20),
            vehicle=vehicle_observation(),
        )
        assert decision.substate == expected


@pytest.mark.parametrize("invalid", [PidLandingState.TOUCHDOWN_APPROACH,
                                     PidLandingState.RECOVERY_CLIMB,
                                     PidLandingState.DISARM])
def test_reject_unsafe_hold_states(invalid):
    with pytest.raises(ValueError):
        PidLandingBackend(pid_config(), hold_at_state=invalid)


def test_default_does_not_hold_at_alignment():
    backend = PidLandingBackend(pid_config())
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    backend.update(now=0.1, dock=dock_observation(0.1), vehicle=vehicle_observation())
    progressed = backend.update(now=1.2, dock=dock_observation(1.2), vehicle=vehicle_observation())
    assert progressed.substate == "DESCEND_OUTER"


def test_disarmed_vehicle_fails_even_during_hold():
    backend = PidLandingBackend(pid_config(), hold_at_state=PidLandingState.PID_ALIGN)
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    decision = backend.update(now=0.1, dock=dock_observation(0.1),
                              vehicle=vehicle_observation(armed=False))
    assert decision.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT
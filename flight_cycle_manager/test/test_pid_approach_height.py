"""Pre-touchdown alignment must not drive through the approach height."""

from dataclasses import replace
import math

import pytest

from flight_cycle_manager.pid_landing import PidLandingState
from flight_cycle_manager.pid_landing_backend import PidLandingBackend
from test_pid_landing_backend import (
    central_observation, dock_observation, enter_central_descent,
    controller_pid_config, pid_config, vehicle_observation,
)


def approach_backend(hold=None, **overrides):
    config = pid_config(touchdown_commit_height_m=0.35,
                        touchdown_horizontal_tolerance_m=0.02, **overrides)
    backend = PidLandingBackend(config, hold_at_state=hold)
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    enter_central_descent(backend)
    return backend


def coarse_aligned(now, distance):
    # Good enough for central descent, but not for touchdown commitment.
    return replace(central_observation(now, distance_m=distance),
                   forward_m=0.03, left_m=0.0, yaw_error_rad=0.0)


def fine_aligned(now, distance):
    return replace(coarse_aligned(now, distance), forward_m=0.0)


def test_waiting_for_fine_alignment_clamps_setpoint_and_holds_approach_height():
    backend = approach_backend()
    for step in range(300):
        now = 2.0 + step * 0.05
        distance = max(0.35, 0.9 - step * 0.01)
        decision = backend.update(now=now, dock=coarse_aligned(now, distance),
                                  vehicle=vehicle_observation())
        assert decision.substate == "DESCEND_CENTRAL"
        assert backend.diagnostics()["distance_setpoint_m"] >= 0.35
        if distance == 0.35:
            assert decision.velocity.velocity_z >= 0.0
            assert not decision.request_disarm


@pytest.mark.parametrize("hold", [None, PidLandingState.DESCEND_CENTRAL])
@pytest.mark.parametrize("forward", [0.03, 0.3])
def test_below_height_commands_upward_even_when_descent_is_paused(hold, forward):
    backend = approach_backend(hold)
    dock = replace(coarse_aligned(2.0, 0.20), forward_m=forward)
    decision = backend.update(now=2.0, dock=dock, vehicle=vehicle_observation())
    assert decision.substate == "DESCEND_CENTRAL"
    assert decision.velocity.velocity_z > 0.0
    assert decision.velocity.velocity_z <= 0.12
    assert backend.diagnostics()["distance_setpoint_m"] >= 0.35


def test_measured_descent_triggers_braking_before_reaching_height():
    backend = approach_backend()
    backend.update(now=2.0, dock=coarse_aligned(2.0, 0.4),
                   vehicle=vehicle_observation())
    decision = backend.update(
        now=2.1, dock=coarse_aligned(2.1, 0.4),
        vehicle=vehicle_observation(velocity_enu_m_s=(0.0, 0.0, -0.2)),
    )
    assert decision.velocity.velocity_z > 0.0
    guard = backend.diagnostics()["approach_height_guard"]
    assert guard["active"]
    assert guard["stopping_distance_m"] > 0.4 - 0.35


def test_camera_receipt_age_increases_braking_distance():
    backend = approach_backend()
    vehicle = vehicle_observation(velocity_enu_m_s=(0.0, 0.0, -0.1))
    backend.update(now=2.0, dock=coarse_aligned(2.0, 0.6), vehicle=vehicle)
    fresh = backend.diagnostics()["approach_height_guard"]["stopping_distance_m"]
    backend.update(now=2.1, dock=coarse_aligned(2.0, 0.6), vehicle=vehicle)
    aged = backend.diagnostics()["approach_height_guard"]["stopping_distance_m"]
    assert aged == pytest.approx(fresh + 0.1 * 0.1)


def test_unknown_vertical_velocity_does_not_allow_more_descent():
    backend = approach_backend()
    backend.update(now=2.0, dock=coarse_aligned(2.0, 0.4),
                   vehicle=vehicle_observation())
    decision = backend.update(
        now=2.1, dock=coarse_aligned(2.1, 0.4),
        vehicle=vehicle_observation(velocity_enu_m_s=(0.0, 0.0, math.nan)),
    )
    assert decision.velocity.velocity_z >= 0.0


def test_normal_landing_can_commit_from_just_above_approach_height():
    backend = approach_backend()
    for now in (2.0, 3.1):
        decision = backend.update(now=now, dock=fine_aligned(now, 0.355),
                                  vehicle=vehicle_observation())
    assert decision.substate == "TOUCHDOWN_APPROACH"
    # The approach guard must not block the deliberately committed descent.
    decision = backend.update(now=3.2, dock=None, vehicle=vehicle_observation())
    assert decision.velocity.velocity_z == -0.05
    assert backend.diagnostics()["approach_height_guard"] is None


def test_large_undershoot_or_fast_vertical_motion_cannot_commit_touchdown():
    backend = approach_backend()
    for now in (2.0, 3.1):
        decision = backend.update(now=now, dock=fine_aligned(now, 0.20),
                                  vehicle=vehicle_observation())
        assert decision.substate == "DESCEND_CENTRAL"
        assert decision.velocity.velocity_z > 0.0
    decision = backend.update(
        now=3.2, dock=fine_aligned(3.2, 0.35),
        vehicle=vehicle_observation(velocity_enu_m_s=(0.0, 0.0, -0.2)),
    )
    assert decision.substate == "DESCEND_CENTRAL"
    assert decision.velocity.velocity_z > 0.0
    decision = backend.update(now=3.3, dock=fine_aligned(3.3, 0.35),
                              vehicle=vehicle_observation())
    assert decision.substate == "TOUCHDOWN_APPROACH"


def test_final_debug_hold_targets_configured_height_and_corrects_undershoot():
    backend = approach_backend(PidLandingState.HOLD_CENTRAL)
    for now in (2.0, 3.1):
        decision = backend.update(now=now, dock=fine_aligned(now, 0.355),
                                  vehicle=vehicle_observation())
    assert decision.substate == "HOLD_CENTRAL"
    assert backend.diagnostics()["distance_setpoint_m"] == pytest.approx(0.35)
    decision = backend.update(now=3.2, dock=fine_aligned(3.2, 0.30),
                              vehicle=vehicle_observation())
    assert decision.velocity.velocity_z > 0.0
    backend.set_hold_at_state(None)
    assert backend.update(now=3.3, dock=fine_aligned(3.3, 0.30),
                          vehicle=vehicle_observation()).substate == "HOLD_CENTRAL"


def test_target_loss_retains_existing_zero_command_and_recovery():
    backend = approach_backend()
    lost = backend.update(now=2.0, dock=None, vehicle=vehicle_observation())
    assert lost.velocity.velocity_z == 0.0
    assert backend.diagnostics()["approach_height_guard"] is None
    assert backend.update(now=2.4, dock=None,
                          vehicle=vehicle_observation()).substate == "RECOVERY_CLIMB"


def test_final_hold_reacquisition_retains_configured_height():
    backend = approach_backend(PidLandingState.HOLD_CENTRAL)
    for now in (2.0, 3.1):
        backend.update(now=now, dock=fine_aligned(now, 0.35),
                       vehicle=vehicle_observation())
    backend.update(now=3.2, dock=None, vehicle=vehicle_observation())
    decision = backend.update(now=3.3, dock=fine_aligned(3.3, 0.45),
                              vehicle=vehicle_observation())
    assert decision.substate == "HOLD_CENTRAL"
    assert backend.diagnostics()["distance_setpoint_m"] == pytest.approx(0.35)
    assert decision.velocity.velocity_z < 0.0


def test_align_debug_rejects_manual_setpoint_below_approach_height():
    backend = PidLandingBackend(pid_config(touchdown_commit_height_m=0.35), align_only=True)
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    backend.update(now=1.0, dock=dock_observation(1.0), vehicle=vehicle_observation())
    with pytest.raises(ValueError, match="approach height"):
        backend.set_align_distance_setpoint(0.3)


@pytest.mark.parametrize("hold", [PidLandingState.PID_ALIGN, PidLandingState.ACQUIRE_CENTRAL])
def test_precommit_hold_brakes_at_height_even_on_transition_tick(hold):
    # Independent initial alignment path, before central descent is entered.
    backend = PidLandingBackend(pid_config(touchdown_commit_height_m=0.35),
                                hold_at_state=hold)
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation())
    command = backend.update(
        now=0.1, dock=coarse_aligned(0.1, 0.34),
        vehicle=vehicle_observation(velocity_enu_m_s=(0.0, 0.0, -0.2)),
    ).velocity
    assert command.velocity_z > 0.0


def test_upward_guard_respects_z_pid_output_limit():
    backend = approach_backend(z_pid=controller_pid_config(output_limit=0.02))
    decision = backend.update(
        now=2.0, dock=coarse_aligned(2.0, 0.2),
        vehicle=vehicle_observation(velocity_enu_m_s=(0.0, 0.0, -0.2)),
    )
    assert 0.0 < decision.velocity.velocity_z <= 0.02


@pytest.mark.parametrize("field", ["approach_vertical_response_time_s",
                                  "approach_braking_accel_m_s2",
                                  "approach_height_tolerance_m"])
@pytest.mark.parametrize("value", [0.0, -0.1, math.nan, math.inf])
def test_braking_configuration_rejects_invalid_values(field, value):
    with pytest.raises(ValueError, match=field):
        pid_config(**{field: value})

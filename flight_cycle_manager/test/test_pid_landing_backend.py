import math
from dataclasses import fields, replace

import pytest

from flight_cycle_manager.landing_backend import (
    DockObservation,
    DockTargetMode,
    LandingBackendOutcome,
    VehicleObservation,
    VelocityCommand,
    VelocityFrame,
)
from flight_cycle_manager.pid import PIDConfig
from flight_cycle_manager.pid_landing import (
    PidLandingConfig,
    PidLandingState,
)
from flight_cycle_manager.pid_landing_backend import PidLandingBackend


def controller_pid_config(output_limit=1.0):
    return PIDConfig(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        integral_limit=0.5,
        output_limit=output_limit,
        derivative_filter_time_constant_s=0.1,
    )


def pid_config(**overrides):
    values = dict(
        tag_stable_time_s=0.5,
        central_transition_distance_m=1.0,
        central_acquire_timeout_s=3.0,
        central_loss_grace_s=0.3,
        target_loss_grace_s=0.7,
        recovery_climb_m=0.3,
        recovery_altitude_tolerance_m=0.05,
        max_recovery_attempts=3,
        touchdown_commit_height_m=0.20,
        touchdown_speed_m_s=0.05,
        touchdown_timeout_s=15.0,
        landed_state_timeout_s=1.0,
        landed_stable_time_s=0.5,
        disarm_retry_period_s=1.0,
        alignment_stable_time_s=1.0,
        coarse_horizontal_tolerance_m=0.2,
        fine_horizontal_tolerance_m=0.07,
        coarse_yaw_tolerance_rad=0.2,
        fine_yaw_tolerance_rad=0.1,
        touchdown_horizontal_speed_tolerance_m_s=0.05,
        touchdown_horizontal_speed_stable_time_s=1.0,
        max_horizontal_speed_m_s=0.5,
        outer_descent_speed_m_s=0.3,
        central_descent_speed_m_s=0.12,
        recovery_speed_m_s=0.2,
        max_yaw_rate_rad_s=0.35,
        x_pid=controller_pid_config(output_limit=0.5),
        y_pid=controller_pid_config(output_limit=0.5),
        z_pid=controller_pid_config(output_limit=0.7),
        yaw_pid=controller_pid_config(output_limit=0.35),
    )
    values.update(overrides)
    return PidLandingConfig(**values)


def dock_observation(
    received_at=1.0,
    *,
    forward_m=0.1,
    left_m=-0.1,
    distance_m=2.0,
    yaw_error_rad=0.05,
    mode=DockTargetMode.OUTER,
):
    return DockObservation(
        forward_m=forward_m,
        left_m=left_m,
        distance_m=distance_m,
        yaw_error_rad=yaw_error_rad,
        mode=mode,
        source_stamp_s=received_at,
        received_at=received_at,
    )


def central_observation(received_at, *, distance_m=0.9):
    return dock_observation(
        received_at,
        forward_m=0.05,
        left_m=-0.05,
        distance_m=distance_m,
        yaw_error_rad=0.05,
        mode=DockTargetMode.CENTRAL,
    )


def vehicle_observation(
    *,
    altitude_m=4.0,
    armed=True,
    flying=True,
    landed_at=0.0,
    heading_rad=None,
    position_enu_m=None,
    velocity_enu_m_s=(0.0, 0.0, 0.0),
):
    return VehicleObservation(
        position_enu_m=(
            (4.0, -2.0, altitude_m)
            if position_enu_m is None
            else position_enu_m
        ),
        velocity_enu_m_s=velocity_enu_m_s,
        armed=armed,
        flying=flying,
        mode=4,
        landed_at=landed_at,
        heading_rad=heading_rad,
    )


def grounded_vehicle(now, *, armed=True, landed_at=None, heading_rad=None):
    return vehicle_observation(
        altitude_m=0.0,
        armed=armed,
        flying=False,
        landed_at=now if landed_at is None else landed_at,
        heading_rad=heading_rad,
    )


def start_backend(config=None):
    backend = PidLandingBackend(config or pid_config())
    backend.start(
        now=0.0,
        dock=dock_observation(0.0),
        vehicle=vehicle_observation(),
    )
    return backend


def test_gains_follow_dock_distance_independent_of_state_and_hold_on_loss():
    base = controller_pid_config(output_limit=0.5)
    near = replace(base, kp=0.5)
    far = replace(base, kp=1.5)
    config = pid_config(
        gain_near_distance_m=0.3, gain_far_distance_m=2.3,
        near_pids=(near,) * 4, far_pids=(far,) * 4,
    )
    backend = PidLandingBackend(config, align_only=True)
    backend.start(now=0.0, dock=None, vehicle=vehicle_observation())
    vehicle = vehicle_observation()

    def sample(now, distance, mode=DockTargetMode.OUTER):
        return backend.update(
            now=now,
            dock=dock_observation(now, distance_m=distance, mode=mode),
            vehicle=vehicle,
        )

    sample(0.1, 3.0)
    for step in range(1, 12):
        far_command = sample(0.1 + 0.05 * step, 3.0)
    assert backend.diagnostics()["axes"]["x"]["kp"] == pytest.approx(1.5)
    assert far_command.velocity.velocity_x == pytest.approx(0.15)
    assert backend.diagnostics()["alignment_only"]
    sample(0.7, 1.3)
    for step in range(1, 12):
        middle_command = sample(0.7 + 0.05 * step, 1.3)
    assert backend.diagnostics()["axes"]["x"]["kp"] == pytest.approx(1.0)
    assert middle_command.velocity.velocity_x == pytest.approx(0.1)
    sample(1.3, 0.2)
    for step in range(1, 12):
        near_command = sample(1.3 + 0.05 * step, 0.2)
    assert backend.diagnostics()["axes"]["x"]["kp"] == pytest.approx(0.5)
    assert near_command.velocity.velocity_x == pytest.approx(0.05)
    last_distance = backend.diagnostics()["gain_distance_m"]
    backend.update(now=1.9, dock=None, vehicle=vehicle)
    assert backend.diagnostics()["gain_distance_m"] == last_distance
    assert backend.diagnostics()["axes"]["x"]["kp"] == pytest.approx(0.5)


def test_distance_gains_remain_latched_during_touchdown():
    base = controller_pid_config(output_limit=0.5)
    near = replace(base, kp=0.3)
    far = replace(base, kp=0.9)
    backend = start_backend(pid_config(
        gain_near_distance_m=0.3, gain_far_distance_m=2.0,
        near_pids=(near,) * 4, far_pids=(far,) * 4,
    ))
    enter_touchdown_commitment(backend)
    distance = backend.diagnostics()["gain_distance_m"]
    backend.update(now=3.0, dock=None, vehicle=vehicle_observation(heading_rad=0.0))
    assert backend.diagnostics()["gain_distance_m"] == distance
    assert backend.diagnostics()["axes"]["x"]["kp"] == pytest.approx(0.3)


def test_central_descent_uses_dock_distance_not_state():
    base = controller_pid_config(output_limit=0.5)
    near = replace(base, kp=0.4)
    far = replace(base, kp=1.4)
    backend = start_backend(pid_config(
        gain_near_distance_m=0.3, gain_far_distance_m=2.3,
        near_pids=(near,) * 4, far_pids=(far,) * 4,
    ))
    assert enter_central_descent(backend).substate == "DESCEND_CENTRAL"
    for step in range(1, 12):
        now = 1.71 + step * 0.05
        backend.update(now=now, dock=central_observation(now, distance_m=0.9),
                       vehicle=vehicle_observation())
    assert backend.diagnostics()["gain_distance_m"] == pytest.approx(0.9)
    assert backend.diagnostics()["axes"]["x"]["kp"] == pytest.approx(0.7)


@pytest.mark.parametrize("near, far", [(0, 2), (2, 2), (2, 1), (math.nan, 2)])
def test_invalid_gain_reference_distances_are_rejected(near, far):
    with pytest.raises(ValueError, match="gain distances"):
        pid_config(gain_near_distance_m=near, gain_far_distance_m=far)


def enter_acquire_central(backend):
    low_outer = dock_observation(
        0.1,
        forward_m=0.05,
        left_m=-0.05,
        distance_m=1.0,
        yaw_error_rad=0.05,
    )
    backend.update(now=0.1, dock=low_outer, vehicle=vehicle_observation())
    return backend.update(
        now=1.11,
        dock=replace(low_outer, source_stamp_s=1.11, received_at=1.11),
        vehicle=vehicle_observation(),
    )


def enter_central_descent(backend):
    enter_acquire_central(backend)
    backend.update(
        now=1.2,
        dock=central_observation(1.2),
        vehicle=vehicle_observation(),
    )
    return backend.update(
        now=1.71,
        dock=central_observation(1.71),
        vehicle=vehicle_observation(),
    )


def enter_touchdown_commitment(backend):
    enter_central_descent(backend)
    backend.update(
        now=1.8,
        dock=central_observation(1.8, distance_m=0.20),
        vehicle=vehicle_observation(),
    )
    return backend.update(
        now=2.81,
        dock=central_observation(2.81, distance_m=0.20),
        vehicle=vehicle_observation(),
    )


@pytest.mark.parametrize("velocity", [
    (0.11, 0.0, 0.0), (0.08, 0.08, 0.0), (float("nan"), 0.0, 0.0),
])
def test_central_descent_waits_for_low_horizontal_speed_throughout_dwell(velocity):
    backend = start_backend(pid_config())
    enter_central_descent(backend)
    dock = lambda now: central_observation(now, distance_m=0.20)
    backend.update(now=1.8, dock=dock(1.8), vehicle=vehicle_observation())
    assert backend.update(
        now=2.81, dock=dock(2.81),
        vehicle=vehicle_observation(velocity_enu_m_s=velocity),
    ).substate == "DESCEND_CENTRAL"
    assert backend.update(
        now=3.0, dock=dock(3.0), vehicle=vehicle_observation(),
    ).substate == "DESCEND_CENTRAL"
    assert backend.update(
        now=4.01, dock=dock(4.01), vehicle=vehicle_observation(),
    ).substate == "TOUCHDOWN_APPROACH"


def test_central_descent_accepts_speed_at_limit_and_ignores_vertical_descent():
    backend = start_backend(pid_config())
    enter_central_descent(backend)
    for now in (1.8, 2.81):
        decision = backend.update(
            now=now, dock=central_observation(now, distance_m=0.20),
            vehicle=vehicle_observation(velocity_enu_m_s=(0.03, 0.04, -0.20)),
        )
    assert decision.substate == "TOUCHDOWN_APPROACH"


def test_touchdown_waits_for_configured_speed_dwell_longer_than_alignment():
    backend = start_backend(pid_config(touchdown_horizontal_speed_stable_time_s=2.0))
    enter_central_descent(backend)
    for now, expected in ((1.8, "DESCEND_CENTRAL"),
                          (2.81, "DESCEND_CENTRAL"),
                          (3.79, "DESCEND_CENTRAL"),
                          (3.81, "TOUCHDOWN_APPROACH")):
        decision = backend.update(
            now=now, dock=central_observation(now, distance_m=0.20),
            vehicle=vehicle_observation(),
        )
        assert decision.substate == expected


def test_speed_dwell_resets_on_misalignment_and_central_tag_loss():
    backend = start_backend(pid_config(touchdown_horizontal_speed_stable_time_s=2.0))
    enter_central_descent(backend)
    def update(now, dock):
        return backend.update(
            now=now, dock=dock, vehicle=vehicle_observation(),
        ).substate

    assert update(1.8, central_observation(1.8, distance_m=0.20)) == "DESCEND_CENTRAL"
    assert update(2.9, central_observation(2.9, distance_m=0.20)) == "DESCEND_CENTRAL"
    misaligned = replace(central_observation(3.0, distance_m=0.20), forward_m=0.1)
    assert update(3.0, misaligned) == "DESCEND_CENTRAL"
    assert update(3.1, central_observation(3.1, distance_m=0.20)) == "DESCEND_CENTRAL"
    assert update(4.2, central_observation(4.2, distance_m=0.20)) == "DESCEND_CENTRAL"
    assert update(4.3, None) == "DESCEND_CENTRAL"
    assert update(4.4, central_observation(4.4, distance_m=0.20)) == "DESCEND_CENTRAL"
    assert update(6.39, central_observation(6.39, distance_m=0.20)) == "DESCEND_CENTRAL"
    assert update(6.4, central_observation(6.4, distance_m=0.20)) == "TOUCHDOWN_APPROACH"


def test_alignment_dwell_remains_required_with_shorter_speed_dwell():
    backend = start_backend(pid_config(touchdown_horizontal_speed_stable_time_s=0.2))
    enter_central_descent(backend)
    for now, expected in ((1.8, "DESCEND_CENTRAL"),
                          (2.1, "DESCEND_CENTRAL"),
                          (2.81, "TOUCHDOWN_APPROACH")):
        decision = backend.update(
            now=now, dock=central_observation(now, distance_m=0.20),
            vehicle=vehicle_observation(),
        )
        assert decision.substate == expected


@pytest.mark.parametrize("duration", [0.0, -0.1, math.nan, math.inf])
def test_invalid_touchdown_horizontal_speed_stable_time_is_rejected(duration):
    with pytest.raises(ValueError, match="touchdown_horizontal_speed_stable_time_s"):
        pid_config(touchdown_horizontal_speed_stable_time_s=duration)


@pytest.mark.parametrize("limit", [0.0, -0.1, math.nan, math.inf])
def test_invalid_touchdown_horizontal_speed_tolerance_is_rejected(limit):
    with pytest.raises(ValueError, match="touchdown_horizontal_speed_tolerance_m_s"):
        pid_config(touchdown_horizontal_speed_tolerance_m_s=limit)


def test_pid_backend_starts_at_alignment_after_shared_search():
    backend = PidLandingBackend(pid_config())
    backend.start(
        now=1.0,
        dock=dock_observation(1.0),
        vehicle=vehicle_observation(),
    )

    decision = backend.update(
        now=1.1,
        dock=dock_observation(1.1),
        vehicle=vehicle_observation(),
    )

    assert decision.substate == "PID_ALIGN"
    assert decision.outcome is LandingBackendOutcome.RUNNING


def test_alignment_only_backend_holds_initial_distance_and_accepts_z_setpoint():
    """Break caught: tuning resumes landing after stable alignment."""
    backend = PidLandingBackend(pid_config(), align_only=True)
    backend.start(
        now=0.0,
        dock=dock_observation(0.0),
        vehicle=vehicle_observation(),
    )

    backend.update(
        now=0.1,
        dock=dock_observation(0.1),
        vehicle=vehicle_observation(),
    )
    decision = backend.update(
        now=2.0,
        dock=dock_observation(2.0),
        vehicle=vehicle_observation(),
    )

    assert decision.substate == "PID_ALIGN"
    assert decision.outcome is LandingBackendOutcome.RUNNING
    assert decision.velocity is not None
    assert decision.velocity.velocity_z == 0.0
    assert decision.velocity.velocity_x != 0.0
    assert backend.diagnostics()["distance_setpoint_m"] == pytest.approx(2.0)

    backend.set_align_distance_setpoint(2.5)
    climb = backend.update(now=2.1, dock=dock_observation(2.1),
                           vehicle=vehicle_observation())
    assert climb.substate == "PID_ALIGN"
    assert climb.velocity.velocity_z == pytest.approx(0.5)
    backend.set_align_distance_setpoint(1.5)
    descend = backend.update(now=2.2, dock=dock_observation(2.2),
                             vehicle=vehicle_observation())
    assert descend.velocity.velocity_z == pytest.approx(-0.5)
    assert backend.diagnostics()["axes"]["z"]["p"] == pytest.approx(-0.5)


def test_alignment_setpoint_requires_observation_and_valid_distance():
    backend = PidLandingBackend(pid_config(), align_only=True)
    backend.start(now=0.0, dock=None, vehicle=vehicle_observation())
    with pytest.raises(ValueError):
        backend.set_align_distance_setpoint(2.0)
    backend.update(now=0.1, dock=dock_observation(0.1),
                   vehicle=vehicle_observation())
    for bad in (float("nan"), float("inf"), True, 0.2, 21.0):
        with pytest.raises(ValueError):
            backend.set_align_distance_setpoint(bad)
    normal_backend = PidLandingBackend(pid_config())
    normal_backend.start(now=0.0, dock=dock_observation(0.0),
                         vehicle=vehicle_observation())
    with pytest.raises(ValueError):
        normal_backend.set_align_distance_setpoint(2.0)


def test_alignment_target_loss_stops_vertical_motion_and_resumes_setpoint():
    backend = PidLandingBackend(pid_config(), align_only=True)
    backend.start(now=0.0, dock=None, vehicle=vehicle_observation())
    backend.update(now=0.1, dock=dock_observation(0.1),
                   vehicle=vehicle_observation())
    backend.set_align_distance_setpoint(2.5)
    assert backend.update(now=0.2, dock=dock_observation(0.2),
                          vehicle=vehicle_observation()).velocity.velocity_z > 0
    lost = backend.update(now=0.3, dock=None, vehicle=vehicle_observation())
    assert lost.velocity == VelocityCommand(VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0)
    assert backend.diagnostics()["distance_setpoint_m"] == 2.5
    with pytest.raises(ValueError):
        backend.set_align_distance_setpoint(1.5)
    recovered = backend.update(now=0.4, dock=dock_observation(0.4),
                               vehicle=vehicle_observation())
    assert recovered.velocity.velocity_z > 0


def test_alignment_only_backend_waits_without_recovery_when_target_is_lost():
    """Break caught: target loss makes tuning mode climb or hand back."""
    backend = PidLandingBackend(
        pid_config(target_loss_grace_s=0.1), align_only=True
    )
    backend.start(
        now=0.0,
        dock=dock_observation(0.0),
        vehicle=vehicle_observation(),
    )

    decision = backend.update(
        now=10.0, dock=None, vehicle=vehicle_observation()
    )

    assert decision.substate == "PID_ALIGN"
    assert decision.outcome is LandingBackendOutcome.RUNNING
    assert decision.velocity == VelocityCommand(
        VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0
    )


def test_removed_initial_search_is_absent_from_the_pid_public_path():
    assert "ACQUIRE_DOCK" not in PidLandingState.__members__
    assert "SEARCH_CIRCLE" not in PidLandingState.__members__
    config_fields = {field.name for field in fields(PidLandingConfig)}
    assert config_fields.isdisjoint(
        {
            "initial_acquire_time_s",
            "search_radius_m",
            "search_speed_m_s",
            "search_timeout_s",
            "search_position_gain",
            "search_position_tolerance_m",
            "search_max_vertical_speed_m_s",
        }
    )


@pytest.mark.parametrize("outer_descent", [False, True])
@pytest.mark.parametrize("lost_dock", [
    None,
    dock_observation(distance_m=0.0),
    dock_observation(forward_m=float("nan")),
    dock_observation(mode="LOST"),
])
def test_target_loss_holds_then_retries_before_handback(outer_descent, lost_dock):
    backend = start_backend()
    if outer_descent:
        enter_outer_descent(backend)

    decision = backend.update(
        now=2.0,
        dock=lost_dock,
        vehicle=vehicle_observation(),
    )

    assert decision.substate == ("DESCEND_OUTER" if outer_descent else "PID_ALIGN")
    assert decision.outcome is LandingBackendOutcome.RUNNING
    assert decision.velocity == VelocityCommand(VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0)
    recovery = backend.update(now=2.71, dock=lost_dock, vehicle=vehicle_observation())
    assert recovery.substate == "RECOVERY_CLIMB"
    assert recovery.outcome is LandingBackendOutcome.RUNNING


def enter_outer_descent(backend):
    backend.update(now=0.1, dock=dock_observation(0.1), vehicle=vehicle_observation())
    decision = backend.update(
        now=1.11, dock=dock_observation(1.11), vehicle=vehicle_observation(),
    )
    assert decision.substate == "DESCEND_OUTER"


@pytest.mark.parametrize("central", [False, True])
@pytest.mark.parametrize("axis", ["forward_m", "yaw_error_rad"])
def test_misalignment_pauses_descent_and_resumes_without_old_vertical_error(central, axis):
    backend = start_backend()
    (enter_central_descent if central else enter_outer_descent)(backend)
    dock = central_observation(2.0) if central else dock_observation(2.0)
    vehicle = vehicle_observation()
    backend.update(now=2.0, dock=dock, vehicle=vehicle)
    descending = backend.update(now=2.1, dock=dock, vehicle=vehicle)
    assert descending.velocity.velocity_z < 0.0
    paused = backend.update(now=2.2, dock=replace(dock, **{axis: 0.3}), vehicle=vehicle)
    assert paused.velocity.velocity_z == 0.0
    assert paused.velocity.velocity_x != 0.0
    tolerance = (0.1 if axis == "yaw_error_rad" else 0.07) if central else 0.2
    near_boundary = replace(dock, **{axis: tolerance * 0.9})
    assert backend.update(now=2.3, dock=near_boundary, vehicle=vehicle).velocity.velocity_z == 0.0
    # Rebase on the current height after drift during the pause.
    recovered = replace(dock, distance_m=dock.distance_m - 0.1)
    assert backend.update(now=2.4, dock=recovered, vehicle=vehicle).velocity.velocity_z == 0.0
    assert backend.update(now=2.5, dock=recovered, vehicle=vehicle).velocity.velocity_z < 0.0


def test_brief_alignment_loss_restarts_stability_dwell():
    backend = start_backend()
    backend.update(now=0.1, dock=dock_observation(0.1), vehicle=vehicle_observation())
    backend.update(now=0.9, dock=None, vehicle=vehicle_observation())
    resumed = backend.update(now=1.2, dock=dock_observation(1.2), vehicle=vehicle_observation())
    assert resumed.substate == "PID_ALIGN"
    aligned = backend.update(now=2.21, dock=dock_observation(2.21), vehicle=vehicle_observation())
    assert aligned.substate == "DESCEND_OUTER"


def test_outer_recovery_reacquires_stable_outer_tag_and_realigns():
    backend = start_backend()
    enter_outer_descent(backend)
    backend.update(now=2.0, dock=None, vehicle=vehicle_observation())
    backend.update(now=2.71, dock=None, vehicle=vehicle_observation())
    climbing = backend.update(now=2.8, dock=None, vehicle=vehicle_observation())
    assert climbing.velocity.velocity_z == 0.2
    waiting = backend.update(now=4.3, dock=None, vehicle=vehicle_observation(altitude_m=4.3))
    assert waiting.substate == "REACQUIRE_DOCK"
    backend.update(now=4.4, dock=dock_observation(4.4), vehicle=vehicle_observation())
    backend.update(now=4.8, dock=None, vehicle=vehicle_observation())
    unstable = backend.update(now=4.9, dock=dock_observation(4.9), vehicle=vehicle_observation())
    assert unstable.substate == "REACQUIRE_DOCK"
    recovered = backend.update(now=5.41, dock=dock_observation(5.41), vehicle=vehicle_observation())
    assert recovered.substate == "PID_ALIGN"
    assert recovered.velocity.velocity_z == 0.0
    backend.update(now=5.5, dock=dock_observation(5.5), vehicle=vehicle_observation())
    aligned = backend.update(now=6.51, dock=dock_observation(6.51), vehicle=vehicle_observation())
    assert aligned.substate == "DESCEND_OUTER"


def test_dock_recovery_exhausts_shared_attempt_budget():
    backend = start_backend(pid_config(max_recovery_attempts=2))
    backend.update(now=0.1, dock=None, vehicle=vehicle_observation())
    backend.update(now=0.81, dock=None, vehicle=vehicle_observation())
    backend.update(now=2.0, dock=None, vehicle=vehicle_observation(altitude_m=4.3))
    retry = backend.update(now=5.01, dock=None, vehicle=vehicle_observation(altitude_m=4.3))
    assert retry.substate == "RECOVERY_CLIMB"
    assert retry.recovery_attempts == 2
    backend.update(now=6.5, dock=None, vehicle=vehicle_observation(altitude_m=4.6))
    failed = backend.update(now=9.51, dock=None, vehicle=vehicle_observation(altitude_m=4.6))
    assert failed.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT
    assert "recovery attempts exhausted" in failed.reason
    assert failed.velocity.velocity_z == 0.0
    assert failed.recovery_attempts == 2


def test_successful_dock_reacquisition_does_not_reset_central_retry_budget():
    backend = start_backend(pid_config(max_recovery_attempts=1))
    backend.update(now=0.1, dock=None, vehicle=vehicle_observation())
    backend.update(now=0.81, dock=None, vehicle=vehicle_observation())
    backend.update(now=2.0, dock=None, vehicle=vehicle_observation(altitude_m=4.3))
    for now in [2.1, 2.61, 2.7, 3.71]:
        recovered = backend.update(
            now=now, dock=central_observation(now), vehicle=vehicle_observation(),
        )
    assert recovered.substate == "DESCEND_CENTRAL"
    backend.update(now=3.8, dock=None, vehicle=vehicle_observation())
    failed = backend.update(now=4.11, dock=None, vehicle=vehicle_observation())
    assert failed.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT
    assert "recovery attempts exhausted" in failed.reason


def test_backend_restart_clears_an_unfinished_target_loss_grace_period():
    backend = start_backend()
    backend.update(now=0.1, dock=None, vehicle=vehicle_observation())
    backend.start(now=10.0, dock=dock_observation(10.0), vehicle=vehicle_observation())
    holding = backend.update(now=10.1, dock=None, vehicle=vehicle_observation())
    assert holding.substate == "PID_ALIGN"
    recovery = backend.update(now=10.81, dock=None, vehicle=vehicle_observation())
    assert recovery.substate == "RECOVERY_CLIMB"


def test_recovery_climb_times_out_if_altitude_does_not_increase():
    backend = start_backend()
    backend.update(now=0.1, dock=None, vehicle=vehicle_observation())
    backend.update(now=0.81, dock=None, vehicle=vehicle_observation())
    failed = backend.update(now=5.32, dock=None, vehicle=vehicle_observation())
    assert failed.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT
    assert "climb timed out" in failed.reason
    assert failed.velocity.velocity_z == 0.0


def test_brief_outer_loss_resets_descent_setpoint_and_central_dwell():
    backend = start_backend()
    enter_outer_descent(backend)
    backend.update(now=1.2, dock=central_observation(1.2, distance_m=2.0), vehicle=vehicle_observation())
    backend.update(now=1.6, dock=central_observation(1.6, distance_m=2.0), vehicle=vehicle_observation())
    backend.update(now=1.7, dock=None, vehicle=vehicle_observation())
    resumed = backend.update(now=2.0, dock=central_observation(2.0, distance_m=2.0), vehicle=vehicle_observation())
    assert resumed.velocity.velocity_z == 0.0
    low = backend.update(now=2.1, dock=central_observation(2.1), vehicle=vehicle_observation())
    assert low.substate == "ACQUIRE_CENTRAL"


def test_alignment_command_is_mapped_to_common_velocity():
    backend = start_backend(
        pid_config(
            max_horizontal_speed_m_s=0.5,
            max_yaw_rate_rad_s=0.3,
            x_pid=controller_pid_config(output_limit=2.0),
            y_pid=controller_pid_config(output_limit=2.0),
            yaw_pid=controller_pid_config(output_limit=2.0),
        )
    )

    decision = backend.update(
        now=0.1,
        dock=dock_observation(
            0.1,
            forward_m=0.6,
            left_m=-0.8,
            yaw_error_rad=0.4,
        ),
        vehicle=vehicle_observation(),
    )

    assert decision.velocity == VelocityCommand(
        frame=VelocityFrame.BODY_FLU,
        velocity_x=0.3,
        velocity_y=-0.4,
        velocity_z=0.0,
        yaw_rate_rad_s=0.3,
    )
    assert decision.stream_landing_target is False
    assert decision.request_land is False


@pytest.mark.parametrize("during_descent", [False, True])
@pytest.mark.parametrize("reacquired_mode", [DockTargetMode.OUTER, DockTargetMode.CENTRAL])
def test_central_failure_reacquires_any_dock_tag_then_realigns(
    during_descent, reacquired_mode,
):
    backend = start_backend()
    if during_descent:
        enter_central_descent(backend)
        backend.update(now=3.8, dock=None, vehicle=vehicle_observation())
    else:
        enter_acquire_central(backend)

    recovery = backend.update(
        now=4.12,
        dock=None,
        vehicle=vehicle_observation(altitude_m=5.0),
    )
    climbing = backend.update(
        now=4.2,
        dock=None,
        vehicle=vehicle_observation(altitude_m=5.20),
    )
    reacquiring = backend.update(
        now=4.3,
        dock=None,
        vehicle=vehicle_observation(altitude_m=5.25),
    )

    assert recovery.substate == "RECOVERY_CLIMB"
    assert recovery.velocity == VelocityCommand(
        VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0
    )
    assert climbing.velocity == VelocityCommand(
        VelocityFrame.BODY_FLU, 0.0, 0.0, 0.2
    )
    assert reacquiring.substate == "REACQUIRE_DOCK"
    assert reacquiring.outcome is LandingBackendOutcome.RUNNING

    for now in [4.4, 4.91]:
        realigning = backend.update(
            now=now,
            dock=replace(central_observation(now), mode=reacquired_mode),
            vehicle=vehicle_observation(altitude_m=5.3),
        )
    assert realigning.substate == "PID_ALIGN"
    assert realigning.velocity.velocity_z == 0.0

    for now in [5.0, 6.01]:
        aligned = backend.update(
            now=now,
            dock=replace(central_observation(now), mode=reacquired_mode),
            vehicle=vehicle_observation(altitude_m=5.3),
        )
    # A recovered outer tag still cannot authorize central descent.
    assert aligned.substate == (
        "ACQUIRE_CENTRAL" if reacquired_mode is DockTargetMode.OUTER
        else "DESCEND_CENTRAL"
    )
    assert aligned.velocity.velocity_z == 0.0


def test_precommit_failure_requests_mission_handback():
    backend = start_backend(pid_config(max_recovery_attempts=1))
    enter_acquire_central(backend)
    backend.update(
        now=4.12,
        dock=None,
        vehicle=vehicle_observation(altitude_m=5.0),
    )
    backend.update(
        now=4.2,
        dock=None,
        vehicle=vehicle_observation(altitude_m=5.25),
    )

    failed = backend.update(
        now=7.21,
        dock=None,
        vehicle=vehicle_observation(altitude_m=5.25),
    )

    assert failed.substate == "FAILED"
    assert failed.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT
    assert "recovery attempts exhausted" in failed.reason


def test_touchdown_commitment_is_reported_as_running():
    backend = start_backend()

    committed = enter_touchdown_commitment(backend)

    assert committed.substate == "TOUCHDOWN_APPROACH"
    assert committed.outcome is LandingBackendOutcome.RUNNING
    assert committed.velocity == VelocityCommand(
        VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0
    )


def enter_touchdown_commitment_with_reference(
    backend, *, heading_rad, dock=None, vehicle=None
):
    dock = dock or central_observation(1.8, distance_m=0.20)
    vehicle = vehicle or {}
    enter_central_descent(backend)
    backend.update(
        now=1.8,
        dock=dock,
        vehicle=vehicle_observation(heading_rad=heading_rad, **vehicle),
    )
    return backend.update(
        now=2.81,
        dock=replace(dock, source_stamp_s=2.81, received_at=2.81),
        vehicle=vehicle_observation(heading_rad=heading_rad, **vehicle),
    )


def test_touchdown_reference_is_the_last_aligned_tag_position():
    """Break caught: touchdown is blind to the aperture centre it must enter."""
    backend = start_backend()
    committed = enter_touchdown_commitment_with_reference(
        backend, heading_rad=0.0
    )
    assert committed.substate == "TOUCHDOWN_APPROACH"

    holding = backend.update(
        now=2.9, dock=None, vehicle=vehicle_observation(heading_rad=0.0)
    )

    # The tag sat 0.05 m ahead and 0.05 m left of the vehicle at commit, so the
    # latched aperture is 0.05 m east and 0.05 m north of it. The residual
    # 0.05 rad yaw error is removed from the reference orientation too.
    assert holding.velocity.velocity_x == pytest.approx(0.05)
    assert holding.velocity.velocity_y == pytest.approx(-0.05)
    assert holding.velocity.velocity_z == -0.05
    assert holding.velocity.yaw_rate_rad_s == pytest.approx(0.05)


def test_touchdown_pulls_the_vehicle_back_onto_the_aperture_axis():
    """Break caught: a disturbance pushes the vehicle off the entry axis."""
    backend = start_backend()
    enter_touchdown_commitment_with_reference(backend, heading_rad=0.0)

    disturbed = backend.update(
        now=2.9,
        dock=None,
        vehicle=vehicle_observation(
            heading_rad=0.0,
            position_enu_m=(4.08, -2.02, 4.0),
        ),
    )

    # 0.03 m east and 0.07 m south of the aperture: fly north and west.
    assert disturbed.velocity.velocity_x == pytest.approx(0.07)
    assert disturbed.velocity.velocity_y == pytest.approx(0.03)


def test_touchdown_reference_rotates_into_the_body_frame():
    """Break caught: the ENU error is not rotated into the body FLU frame."""
    backend = start_backend()
    aligned = replace(
        central_observation(1.8, distance_m=0.20),
        forward_m=0.0,
        left_m=0.0,
        yaw_error_rad=0.0,
    )
    enter_touchdown_commitment_with_reference(
        backend, heading_rad=math.pi / 2.0, dock=aligned
    )

    disturbed = backend.update(
        now=2.9,
        dock=None,
        vehicle=vehicle_observation(
            heading_rad=math.pi / 2.0,
            position_enu_m=(3.90, -2.0, 4.0),
        ),
    )

    # Facing east, a 0.10 m westward push is a pure forward error.
    assert disturbed.velocity.velocity_x == pytest.approx(0.10)
    assert disturbed.velocity.velocity_y == pytest.approx(0.0)
    assert disturbed.velocity.yaw_rate_rad_s == pytest.approx(0.0)


def test_touchdown_heading_reference_wraps_across_the_pi_boundary():
    """Break caught: the heading hold forgets the shortest-path yaw error."""
    backend = start_backend()
    aligned = replace(
        central_observation(1.8, distance_m=0.20),
        forward_m=0.0,
        left_m=0.0,
        yaw_error_rad=0.0,
    )
    enter_touchdown_commitment_with_reference(
        backend, heading_rad=math.pi - 0.05, dock=aligned
    )

    disturbed = backend.update(
        now=2.9,
        dock=None,
        vehicle=vehicle_observation(heading_rad=-math.pi + 0.05),
    )

    assert disturbed.velocity.yaw_rate_rad_s == pytest.approx(0.10)


def test_touchdown_without_attitude_keeps_the_passive_descent():
    """Break caught: missing attitude telemetry must not invent a setpoint."""
    backend = start_backend()
    enter_touchdown_commitment(backend)

    descending = backend.update(
        now=2.9, dock=None, vehicle=vehicle_observation()
    )

    assert descending.velocity == VelocityCommand(
        VelocityFrame.BODY_FLU, 0.0, 0.0, -0.05
    )


def test_disarm_bounce_keeps_the_latched_touchdown_reference():
    """Break caught: a bounced touchdown re-latches a disturbed pose."""
    backend = start_backend()
    enter_touchdown_commitment_with_reference(backend, heading_rad=0.30)
    backend.update(
        now=2.9, dock=None, vehicle=grounded_vehicle(2.9, heading_rad=0.30)
    )
    disarming = backend.update(
        now=3.41,
        dock=None,
        vehicle=grounded_vehicle(3.41, landed_at=2.9, heading_rad=0.30),
    )
    assert disarming.substate == "DISARM"

    bounced = backend.update(
        now=3.5, dock=None, vehicle=vehicle_observation(heading_rad=0.40)
    )

    # The reference is the tag orientation (0.30 - 0.05 rad), not the vehicle
    # orientation sampled at commit.
    assert bounced.substate == "TOUCHDOWN_APPROACH"
    assert bounced.velocity.yaw_rate_rad_s == pytest.approx(0.15)


def test_postcommit_failure_requests_direct_land():
    backend = start_backend(pid_config(touchdown_timeout_s=0.5))
    committed = enter_touchdown_commitment(backend)
    assert committed.substate == "TOUCHDOWN_APPROACH"

    failed = backend.update(
        now=3.32,
        dock=None,
        vehicle=vehicle_observation(),
    )

    assert failed.substate == "FAILED"
    assert failed.outcome is LandingBackendOutcome.FAIL_AFTER_COMMIT
    assert failed.reason == "touchdown was not confirmed before timeout"


def test_disarm_requests_are_mapped_and_correlated_across_retry():
    backend = start_backend()
    enter_touchdown_commitment(backend)
    backend.update(
        now=2.9,
        dock=None,
        vehicle=grounded_vehicle(2.9),
    )
    first = backend.update(
        now=3.41,
        dock=None,
        vehicle=grounded_vehicle(3.41, landed_at=2.9),
    )
    assert first.request_disarm is True
    assert first.disarm_request_id == 1

    backend.notify_disarm_result(request_id=2, success=True, now=3.5)
    wrong_correlation = backend.update(
        now=4.5,
        dock=None,
        vehicle=grounded_vehicle(4.5, landed_at=4.0),
    )
    assert wrong_correlation.request_disarm is False

    backend.notify_disarm_result(request_id=1, success=False, now=4.6)
    before_retry = backend.update(
        now=5.59,
        dock=None,
        vehicle=grounded_vehicle(5.59, landed_at=5.0),
    )
    retry = backend.update(
        now=5.6,
        dock=None,
        vehicle=grounded_vehicle(5.6, landed_at=5.0),
    )

    assert before_retry.request_disarm is False
    assert retry.request_disarm is True
    assert retry.disarm_request_id == 2


def test_disarmed_heartbeat_after_correlated_ack_completes_backend():
    backend = start_backend()
    enter_touchdown_commitment(backend)
    backend.update(
        now=2.9,
        dock=None,
        vehicle=grounded_vehicle(2.9),
    )
    request = backend.update(
        now=3.41,
        dock=None,
        vehicle=grounded_vehicle(3.41, landed_at=2.9),
    )
    backend.notify_disarm_result(
        request_id=request.disarm_request_id,
        success=True,
        now=3.5,
    )

    complete = backend.update(
        now=3.6,
        dock=None,
        vehicle=grounded_vehicle(3.6, armed=False),
    )

    assert complete.substate == "COMPLETE"
    assert complete.outcome is LandingBackendOutcome.COMPLETE
    assert complete.request_disarm is False


@pytest.mark.parametrize("now", [float("nan"), float("inf")])
def test_backend_start_rejects_non_finite_time(now):
    backend = PidLandingBackend(pid_config())

    with pytest.raises(ValueError, match="now"):
        backend.start(
            now=now,
            dock=dock_observation(),
            vehicle=vehicle_observation(),
        )

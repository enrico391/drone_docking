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


def vehicle_observation(*, altitude_m=4.0, armed=True, flying=True, landed_at=0.0):
    return VehicleObservation(
        position_enu_m=(4.0, -2.0, altitude_m),
        velocity_enu_m_s=(0.0, 0.0, 0.0),
        armed=armed,
        flying=flying,
        mode=4,
        landed_at=landed_at,
    )


def grounded_vehicle(now, *, armed=True, landed_at=None):
    return vehicle_observation(
        altitude_m=0.0,
        armed=armed,
        flying=False,
        landed_at=now if landed_at is None else landed_at,
    )


def start_backend(config=None):
    backend = PidLandingBackend(config or pid_config())
    backend.start(
        now=0.0,
        dock=dock_observation(0.0),
        vehicle=vehicle_observation(),
    )
    return backend


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


def test_loss_during_initial_alignment_fails_without_pid_owned_search():
    backend = start_backend()

    decision = backend.update(
        now=0.1,
        dock=None,
        vehicle=vehicle_observation(),
    )

    assert decision.substate == "FAILED"
    assert decision.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT
    assert decision.reason == "dock target lost before central acquisition"


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


def test_recovery_climb_and_central_reacquisition_are_retained():
    backend = start_backend()
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
    assert reacquiring.substate == "ACQUIRE_CENTRAL"
    assert reacquiring.outcome is LandingBackendOutcome.RUNNING


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
    assert failed.reason == "central tag recovery attempts exhausted"


def test_touchdown_commitment_is_reported_as_running():
    backend = start_backend()

    committed = enter_touchdown_commitment(backend)

    assert committed.substate == "TOUCHDOWN_APPROACH"
    assert committed.outcome is LandingBackendOutcome.RUNNING
    assert committed.velocity == VelocityCommand(
        VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0
    )


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

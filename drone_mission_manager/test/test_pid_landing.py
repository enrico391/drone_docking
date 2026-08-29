from dataclasses import replace
import math

import pytest

from drone_mission_manager.pid import PIDConfig
from drone_mission_manager.pid_landing import (
    DockObservation,
    DockTargetMode,
    PidLandingConfig,
    PidLandingController,
    PidLandingOutcome,
    PidLandingState,
    VehicleObservation,
)
from drone_mission_manager.pymavlink_vehicle import VelocityFrame


def pid_config(output_limit=1.0):
    return PIDConfig(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        integral_limit=0.5,
        output_limit=output_limit,
        derivative_filter_time_constant_s=0.1,
    )


def default_config(**overrides):
    values = dict(
        initial_acquire_time_s=2.0,
        tag_stable_time_s=0.5,
        search_radius_m=1.0,
        search_speed_m_s=0.3,
        search_timeout_s=5.0,
        central_transition_distance_m=1.0,
        central_acquire_timeout_s=3.0,
        central_loss_grace_s=0.3,
        recovery_climb_m=0.3,
        max_recovery_attempts=3,
        touchdown_commit_height_m=0.25,
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
        search_position_gain=0.7,
        search_position_tolerance_m=0.35,
        search_max_vertical_speed_m_s=0.7,
        x_pid=pid_config(output_limit=0.5),
        y_pid=pid_config(output_limit=0.5),
        z_pid=pid_config(output_limit=0.7),
        yaw_pid=pid_config(output_limit=0.35),
    )
    values.update(overrides)
    return PidLandingConfig(**values)


@pytest.fixture
def controller():
    return PidLandingController(default_config())


@pytest.fixture
def vehicle():
    return vehicle_at(4.0, -2.0, 4.0)


def vehicle_at(x_m, y_m, z_m):
    return VehicleObservation(
        position_enu_m=(x_m, y_m, z_m),
        velocity_enu_m_s=(0.0, 0.0, 0.0),
        armed=True,
        flying=True,
        mode=4,
        landed_at=0.0,
    )


def outer_observation(received_at):
    return DockObservation(
        forward_m=0.1,
        left_m=-0.1,
        distance_m=2.0,
        yaw_error_rad=0.05,
        mode=DockTargetMode.OUTER,
        source_stamp_s=received_at,
        received_at=received_at,
    )


@pytest.mark.parametrize(
    "field, value, message",
    [
        ("touchdown_commit_height_m", 1.0, "touchdown_commit_height_m"),
        ("recovery_climb_m", 0.0, "recovery_climb_m"),
        ("search_radius_m", 0.0, "search_radius_m"),
        ("initial_acquire_time_s", 0.0, "initial_acquire_time_s"),
        ("max_recovery_attempts", 0, "max_recovery_attempts"),
        ("fine_horizontal_tolerance_m", 0.2, "fine_horizontal_tolerance_m"),
        ("fine_yaw_tolerance_rad", 0.2, "fine_yaw_tolerance_rad"),
        ("search_speed_m_s", math.nan, "search_speed_m_s"),
    ],
)
def test_config_rejects_invalid_geometry_timing_and_tolerances(
    field, value, message
):
    with pytest.raises(ValueError, match=message):
        default_config(**{field: value})


def test_config_rejects_non_finite_value_in_every_float_field():
    config = default_config()
    for field, value in config.__dict__.items():
        if isinstance(value, float):
            with pytest.raises(ValueError, match=field):
                replace(config, **{field: math.nan})


def test_stable_outer_tag_transitions_from_acquire_to_align(controller, vehicle):
    controller.start(now=0.0, dock_x_m=4.0, dock_y_m=-2.0,
                     hover_altitude_m=4.0)

    first = controller.update(0.1, outer_observation(received_at=0.1), vehicle)
    stable = controller.update(0.7, outer_observation(received_at=0.7), vehicle)

    assert first.state is PidLandingState.ACQUIRE_DOCK
    assert stable.state is PidLandingState.PID_ALIGN


@pytest.mark.parametrize(
    "interruption",
    [None, replace(outer_observation(received_at=0.3), distance_m=math.nan)],
)
def test_acquisition_stability_resets_when_observation_is_absent_or_non_finite(
    controller, vehicle, interruption
):
    controller.start(now=0.0, dock_x_m=4.0, dock_y_m=-2.0,
                     hover_altitude_m=4.0)
    controller.update(0.1, outer_observation(received_at=0.1), vehicle)
    controller.update(0.3, interruption, vehicle)

    restarted = controller.update(0.7, outer_observation(received_at=0.7), vehicle)
    stable = controller.update(1.2, outer_observation(received_at=1.2), vehicle)

    assert restarted.state is PidLandingState.ACQUIRE_DOCK
    assert stable.state is PidLandingState.PID_ALIGN


def test_acquisition_timeout_enters_search_with_zero_transition_command(
    controller, vehicle
):
    controller.start(now=0.0, dock_x_m=4.0, dock_y_m=-2.0,
                     hover_altitude_m=4.0)

    decision = controller.update(2.1, None, vehicle)

    assert decision.state is PidLandingState.SEARCH_CIRCLE
    assert decision.command.velocity_x == 0.0
    assert decision.command.velocity_y == 0.0
    assert decision.command.velocity_z == 0.0


def enter_search(controller, vehicle):
    controller.start(now=0.0, dock_x_m=4.0, dock_y_m=-2.0,
                     hover_altitude_m=4.0)
    return controller.update(2.1, None, vehicle)


def test_search_first_approaches_east_perimeter_at_hover_altitude(controller):
    transition = enter_search(controller, vehicle_at(4.0, -2.0, 3.0))
    decision = controller.update(2.2, None, vehicle_at(4.0, -2.0, 3.0))

    assert transition.command.frame is VelocityFrame.BODY_FLU
    assert decision.state is PidLandingState.SEARCH_CIRCLE
    assert decision.command.frame is VelocityFrame.MAP_ENU
    assert decision.command.velocity_x == pytest.approx(0.5)
    assert decision.command.velocity_y == pytest.approx(0.0)
    assert decision.command.velocity_z == pytest.approx(0.7)


def test_search_orbit_uses_elapsed_angle_and_limits_horizontal_velocity(controller):
    enter_search(controller, vehicle_at(4.0, -2.0, 4.0))
    controller.update(2.2, None, vehicle_at(5.0, -2.0, 4.0))

    decision = controller.update(3.2, None, vehicle_at(5.0, -2.0, 4.0))

    angle = 0.3
    expected_x = -0.3 * math.sin(angle) + 0.7 * (math.cos(angle) - 1.0)
    expected_y = 0.3 * math.cos(angle) + 0.7 * math.sin(angle)
    expected_scale = 0.5 / math.hypot(expected_x, expected_y)
    assert decision.command.frame is VelocityFrame.MAP_ENU
    assert decision.command.velocity_x == pytest.approx(expected_x * expected_scale)
    assert decision.command.velocity_y == pytest.approx(expected_y * expected_scale)
    assert math.hypot(
        decision.command.velocity_x, decision.command.velocity_y
    ) == pytest.approx(0.5)
    assert decision.command.velocity_z == pytest.approx(0.0)


def test_stable_tag_during_search_enters_align_with_zero_body_command(
    controller, vehicle
):
    enter_search(controller, vehicle)
    first = controller.update(2.2, outer_observation(received_at=2.2), vehicle)
    stable = controller.update(2.7, outer_observation(received_at=2.7), vehicle)

    assert first.state is PidLandingState.SEARCH_CIRCLE
    assert stable.state is PidLandingState.PID_ALIGN
    assert stable.command.frame is VelocityFrame.BODY_FLU
    assert stable.command.velocity_x == 0.0
    assert stable.command.velocity_y == 0.0
    assert stable.command.velocity_z == 0.0


def test_search_timeout_precedes_tag_stability_at_deadline(controller, vehicle):
    enter_search(controller, vehicle)
    controller.update(6.6, outer_observation(received_at=6.6), vehicle)

    decision = controller.update(7.1, outer_observation(received_at=7.1), vehicle)

    assert decision.state is PidLandingState.FAILED
    assert decision.outcome is PidLandingOutcome.FAILED
    assert decision.reason == "dock tag was not acquired during search"
    assert decision.command.frame is VelocityFrame.BODY_FLU


def test_search_does_not_orbit_until_east_perimeter_altitude_is_reached(
    controller,
):
    enter_search(controller, vehicle_at(5.0, -2.0, 3.0))

    decision = controller.update(2.2, None, vehicle_at(5.0, -2.0, 3.0))

    assert decision.state is PidLandingState.SEARCH_CIRCLE
    assert decision.command.frame is VelocityFrame.MAP_ENU
    assert decision.command.velocity_x == pytest.approx(0.0)
    assert decision.command.velocity_y == pytest.approx(0.0)
    assert decision.command.velocity_z == pytest.approx(0.7)


def test_search_timeout_fails_with_zero_body_command(controller, vehicle):
    enter_search(controller, vehicle)

    decision = controller.update(7.1, None, vehicle)

    assert decision.state is PidLandingState.FAILED
    assert decision.outcome is PidLandingOutcome.FAILED
    assert decision.reason == "dock tag was not acquired during search"
    assert decision.command.frame is VelocityFrame.BODY_FLU
    assert decision.command.velocity_x == 0.0
    assert decision.command.velocity_y == 0.0
    assert decision.command.velocity_z == 0.0


def test_failed_search_continues_to_return_its_zero_command(controller, vehicle):
    enter_search(controller, vehicle)
    controller.update(7.1, None, vehicle)

    decision = controller.update(7.2, None, vehicle)

    assert decision.state is PidLandingState.FAILED
    assert decision.outcome is PidLandingOutcome.FAILED
    assert decision.command.frame is VelocityFrame.BODY_FLU
    assert decision.command.velocity_x == 0.0
    assert decision.command.velocity_y == 0.0
    assert decision.command.velocity_z == 0.0

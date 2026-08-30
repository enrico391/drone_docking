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
    VelocityCommand,
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


def derivative_pid_config():
    return PIDConfig(
        kp=0.0,
        ki=0.0,
        kd=1.0,
        integral_limit=0.5,
        output_limit=10.0,
        derivative_filter_time_constant_s=0.0,
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
        recovery_altitude_tolerance_m=0.05,
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


def dock_observation(
    received_at, *, forward_m=0.1, left_m=-0.1, distance_m=2.0,
    yaw_error_rad=0.05, mode=DockTargetMode.OUTER,
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


@pytest.mark.parametrize(
    "field, value, message",
    [
        ("touchdown_commit_height_m", 1.0, "touchdown_commit_height_m"),
        ("recovery_climb_m", 0.0, "recovery_climb_m"),
        (
            "recovery_altitude_tolerance_m", 0.0,
            "recovery_altitude_tolerance_m",
        ),
        (
            "recovery_altitude_tolerance_m", 0.3,
            "recovery_altitude_tolerance_m",
        ),
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


def test_search_reacquires_central_acquiring_tag_for_alignment(
    controller, vehicle
):
    enter_search(controller, vehicle)

    first = controller.update(
        2.2, central_acquiring_observation(2.2, distance_m=1.4), vehicle
    )
    stable = controller.update(
        2.7, central_acquiring_observation(2.7, distance_m=1.4), vehicle
    )

    assert first.state is PidLandingState.SEARCH_CIRCLE
    assert stable.state is PidLandingState.PID_ALIGN
    assert stable.command == zero_body_command()


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


def enter_align(controller, vehicle, observation=None):
    controller.start(now=0.0, dock_x_m=4.0, dock_y_m=-2.0,
                     hover_altitude_m=4.0)
    observation = observation or outer_observation(received_at=0.1)
    controller.update(0.1, observation, vehicle)
    return controller.update(
        0.6,
        replace(observation, received_at=0.6, source_stamp_s=0.6),
        vehicle,
    )


def test_align_commands_body_flu_toward_dock_with_limited_speed_and_zero_z(
    vehicle,
):
    controller = PidLandingController(default_config(
        max_horizontal_speed_m_s=0.5,
        max_yaw_rate_rad_s=0.3,
        x_pid=pid_config(output_limit=2.0),
        y_pid=pid_config(output_limit=2.0),
        yaw_pid=pid_config(output_limit=2.0),
    ))
    enter_align(controller, vehicle)

    decision = controller.update(0.7, dock_observation(
        0.7, forward_m=0.6, left_m=-0.8, yaw_error_rad=0.4,
    ), vehicle)

    assert decision.state is PidLandingState.PID_ALIGN
    assert decision.command.frame is VelocityFrame.BODY_FLU
    assert decision.command.velocity_x == pytest.approx(0.3)
    assert decision.command.velocity_y == pytest.approx(-0.4)
    assert decision.command.velocity_z == 0.0
    assert decision.command.yaw_rate_rad_s == pytest.approx(0.3)


def test_align_wraps_yaw_error_to_the_shortest_turn_direction(vehicle):
    controller = PidLandingController(default_config(
        max_yaw_rate_rad_s=4.0,
        yaw_pid=pid_config(output_limit=4.0),
    ))
    enter_align(controller, vehicle)

    decision = controller.update(0.7, dock_observation(
        0.7, yaw_error_rad=math.pi + 0.1,
    ), vehicle)

    assert decision.command.yaw_rate_rad_s == pytest.approx(-math.pi + 0.1)


@pytest.mark.parametrize(
    "misaligned",
    [
        {"forward_m": 0.25},
        {"left_m": 0.25},
        {"yaw_error_rad": 0.25},
    ],
)
def test_alignment_stability_restarts_for_each_controlled_error(
    controller, vehicle, misaligned,
):
    enter_align(controller, vehicle)
    aligned = dock_observation(0.7)
    controller.update(0.7, aligned, vehicle)
    controller.update(1.2, dock_observation(1.2, **misaligned), vehicle)
    controller.update(1.3, aligned, vehicle)

    before_stable = controller.update(2.2, dock_observation(2.2), vehicle)
    stable = controller.update(2.31, dock_observation(2.31), vehicle)

    assert before_stable.state is PidLandingState.PID_ALIGN
    assert stable.state is PidLandingState.DESCEND_OUTER


def test_missing_visual_observation_stops_alignment_and_returns_to_acquisition(
    controller, vehicle,
):
    enter_align(controller, vehicle)

    decision = controller.update(0.7, None, vehicle)

    assert decision.state is PidLandingState.ACQUIRE_DOCK
    assert decision.command.frame is VelocityFrame.BODY_FLU
    assert decision.command.velocity_x == 0.0
    assert decision.command.velocity_y == 0.0
    assert decision.command.velocity_z == 0.0


def enter_outer_descent(controller, vehicle):
    enter_align(controller, vehicle)
    controller.update(0.7, dock_observation(0.7), vehicle)
    return controller.update(1.7, dock_observation(1.7), vehicle)


@pytest.mark.parametrize(
    "misaligned",
    [
        {"forward_m": 0.3},
        {"left_m": 0.3},
        {"yaw_error_rad": 0.3},
    ],
)
def test_outer_descent_setpoint_ramps_only_while_aligned(
    vehicle, misaligned,
):
    controller = PidLandingController(default_config(
        z_pid=replace(pid_config(output_limit=0.7), kp=0.1),
    ))
    enter_outer_descent(controller, vehicle)

    initial = controller.update(
        1.8, dock_observation(1.8, distance_m=2.0), vehicle
    )
    descending = controller.update(
        2.8, dock_observation(2.8, distance_m=2.0), vehicle
    )
    held = controller.update(
        3.8,
        dock_observation(3.8, distance_m=2.0, **misaligned),
        vehicle,
    )
    resumed = controller.update(
        4.8, dock_observation(4.8, distance_m=2.0), vehicle
    )

    assert initial.command.velocity_z == pytest.approx(0.0)
    assert descending.command.velocity_z == pytest.approx(-0.03)
    assert held.command.velocity_z == pytest.approx(-0.03)
    assert resumed.command.velocity_z == pytest.approx(-0.06)


def test_outer_descent_visual_z_is_limited_to_outer_descent_speed(
    controller, vehicle,
):
    enter_outer_descent(controller, vehicle)
    controller.update(1.8, dock_observation(1.8, distance_m=2.0), vehicle)

    saturated = controller.update(
        4.8, dock_observation(4.8, distance_m=2.0), vehicle
    )

    assert saturated.state is PidLandingState.DESCEND_OUTER
    assert saturated.command.velocity_z == pytest.approx(-0.3)


@pytest.mark.parametrize(
    "mode, expected_state",
    [
        (DockTargetMode.OUTER, PidLandingState.ACQUIRE_CENTRAL),
        (DockTargetMode.CENTRAL, PidLandingState.DESCEND_CENTRAL),
    ],
)
def test_outer_descent_transitions_by_central_target_stability(
    vehicle, mode, expected_state,
):
    controller = PidLandingController(default_config(tag_stable_time_s=0.5))
    enter_outer_descent(controller, vehicle)
    if mode is DockTargetMode.CENTRAL:
        controller.update(1.8, dock_observation(
            1.8, distance_m=1.2, mode=DockTargetMode.CENTRAL,
        ), vehicle)
        controller.update(2.3, dock_observation(
            2.3, distance_m=1.1, mode=DockTargetMode.CENTRAL,
        ), vehicle)

    decision = controller.update(2.8, dock_observation(
        2.8, distance_m=1.0, mode=mode,
    ), vehicle)

    assert decision.state is expected_state
    assert decision.command.velocity_z == 0.0


def central_observation(received_at, *, distance_m=0.9):
    return dock_observation(
        received_at,
        forward_m=0.05,
        left_m=-0.05,
        distance_m=distance_m,
        yaw_error_rad=0.05,
        mode=DockTargetMode.CENTRAL,
    )


def central_acquiring_observation(received_at, *, distance_m=0.9):
    return dock_observation(
        received_at,
        forward_m=0.05,
        left_m=-0.05,
        distance_m=distance_m,
        yaw_error_rad=0.05,
        mode=DockTargetMode.CENTRAL_ACQUIRING,
    )


def enter_acquire_central(controller, vehicle):
    low_outer = dock_observation(
        0.1,
        forward_m=0.05,
        left_m=-0.05,
        distance_m=1.0,
        yaw_error_rad=0.05,
    )
    enter_align(controller, vehicle, low_outer)
    controller.update(0.7, replace(
        low_outer, received_at=0.7, source_stamp_s=0.7,
    ), vehicle)
    return controller.update(1.71, replace(
        low_outer, received_at=1.71, source_stamp_s=1.71,
    ), vehicle)


def enter_central_descent(controller, vehicle):
    enter_acquire_central(controller, vehicle)
    controller.update(1.8, central_observation(1.8), vehicle)
    return controller.update(2.31, central_observation(2.31), vehicle)


def test_align_uses_fine_tolerance_at_central_transition_distance(vehicle):
    controller = PidLandingController(default_config())
    observation = dock_observation(
        0.1, forward_m=0.1, left_m=0.0, distance_m=1.0,
        yaw_error_rad=0.0,
    )
    enter_align(controller, vehicle, observation)

    decision = controller.update(1.7, replace(
        observation, received_at=1.7, source_stamp_s=1.7,
    ), vehicle)

    assert decision.state is PidLandingState.PID_ALIGN
    assert decision.command.velocity_z == 0.0


def test_central_acquiring_below_transition_waits_for_stable_central(
    controller, vehicle,
):
    acquiring = central_acquiring_observation(0.1, distance_m=0.9)
    enter_align(controller, vehicle, acquiring)
    controller.update(
        0.7,
        replace(acquiring, received_at=0.7, source_stamp_s=0.7),
        vehicle,
    )

    routed = controller.update(
        1.71,
        replace(acquiring, received_at=1.71, source_stamp_s=1.71),
        vehicle,
    )
    first_acquiring = controller.update(
        1.8, central_acquiring_observation(1.8, distance_m=0.9), vehicle
    )
    still_acquiring = controller.update(
        2.31, central_acquiring_observation(2.31, distance_m=0.9), vehicle
    )
    first_central = controller.update(
        2.4, central_observation(2.4, distance_m=0.9), vehicle
    )
    stable_central = controller.update(
        2.91, central_observation(2.91, distance_m=0.9), vehicle
    )

    assert routed.state is PidLandingState.ACQUIRE_CENTRAL
    assert first_acquiring.state is PidLandingState.ACQUIRE_CENTRAL
    assert still_acquiring.state is PidLandingState.ACQUIRE_CENTRAL
    assert first_central.state is PidLandingState.ACQUIRE_CENTRAL
    assert stable_central.state is PidLandingState.DESCEND_CENTRAL


def test_central_acquiring_above_transition_can_enter_outer_descent(
    controller, vehicle,
):
    acquiring = central_acquiring_observation(0.1, distance_m=1.4)
    enter_align(controller, vehicle, acquiring)
    controller.update(
        0.7,
        replace(acquiring, received_at=0.7, source_stamp_s=0.7),
        vehicle,
    )

    decision = controller.update(
        1.71,
        replace(acquiring, received_at=1.71, source_stamp_s=1.71),
        vehicle,
    )

    assert decision.state is PidLandingState.DESCEND_OUTER
    assert decision.command == zero_body_command()


def test_missing_visual_observation_stops_outer_descent_before_transition(
    controller, vehicle,
):
    enter_outer_descent(controller, vehicle)
    controller.update(1.8, dock_observation(1.8), vehicle)

    decision = controller.update(1.9, None, vehicle)

    assert decision.state is PidLandingState.ACQUIRE_DOCK
    assert decision.command.frame is VelocityFrame.BODY_FLU
    assert decision.command.velocity_x == 0.0
    assert decision.command.velocity_y == 0.0
    assert decision.command.velocity_z == 0.0
    assert decision.command.yaw_rate_rad_s == 0.0


def test_missing_outer_descent_requires_fresh_tag_stability_before_align(
    controller, vehicle,
):
    enter_outer_descent(controller, vehicle)
    controller.update(1.8, None, vehicle)

    first = controller.update(1.9, outer_observation(1.9), vehicle)
    almost_stable = controller.update(2.39, outer_observation(2.39), vehicle)
    stable = controller.update(2.41, outer_observation(2.41), vehicle)

    assert first.state is PidLandingState.ACQUIRE_DOCK
    assert almost_stable.state is PidLandingState.ACQUIRE_DOCK
    assert stable.state is PidLandingState.PID_ALIGN


def test_acquire_central_requires_a_stable_central_observation(
    controller, vehicle,
):
    transition = enter_acquire_central(controller, vehicle)
    first = controller.update(1.8, central_observation(1.8), vehicle)
    almost_stable = controller.update(2.29, central_observation(2.29), vehicle)
    stable = controller.update(2.31, central_observation(2.31), vehicle)

    assert transition.state is PidLandingState.ACQUIRE_CENTRAL
    assert first.state is PidLandingState.ACQUIRE_CENTRAL
    assert almost_stable.state is PidLandingState.ACQUIRE_CENTRAL
    assert stable.state is PidLandingState.DESCEND_CENTRAL
    assert stable.command.velocity_z == 0.0


def test_central_loss_within_grace_commands_zero_then_resumes_descent(
    controller, vehicle,
):
    enter_central_descent(controller, vehicle)
    controller.update(2.4, central_observation(2.4), vehicle)
    before_dropout = controller.update(
        2.5, central_observation(2.5), vehicle
    )

    missing = controller.update(2.6, None, vehicle)
    recovered = controller.update(2.79, central_observation(2.79), vehicle)

    assert missing.state is PidLandingState.DESCEND_CENTRAL
    assert missing.command.velocity_z == 0.0
    assert recovered.state is PidLandingState.DESCEND_CENTRAL
    assert recovered.command.velocity_z == pytest.approx(
        before_dropout.command.velocity_z
    )


def test_central_acquiring_does_not_continue_central_descent_command(
    controller, vehicle,
):
    enter_central_descent(controller, vehicle)
    controller.update(2.4, central_observation(2.4), vehicle)
    before_dropout = controller.update(
        2.5, central_observation(2.5), vehicle
    )

    acquiring = controller.update(
        2.6, central_acquiring_observation(2.6), vehicle
    )
    recovery = controller.update(
        2.91, central_acquiring_observation(2.91), vehicle
    )

    assert before_dropout.command.velocity_z != 0.0
    assert acquiring.state is PidLandingState.DESCEND_CENTRAL
    assert acquiring.command == zero_body_command()
    assert recovery.state is PidLandingState.RECOVERY_CLIMB
    assert recovery.command == zero_body_command()


def test_central_dropout_resets_visual_pid_histories(vehicle):
    derivative_only = derivative_pid_config()
    controller = PidLandingController(default_config(
        x_pid=derivative_only,
        y_pid=derivative_only,
        z_pid=derivative_only,
        yaw_pid=derivative_only,
    ))
    enter_central_descent(controller, vehicle)
    controller.update(2.4, central_observation(2.4), vehicle)
    controller.update(2.5, central_observation(2.5), vehicle)
    controller.update(2.6, None, vehicle)

    recovered = controller.update(2.7, dock_observation(
        2.7,
        forward_m=0.06,
        left_m=-0.06,
        distance_m=0.89,
        yaw_error_rad=0.06,
        mode=DockTargetMode.CENTRAL,
    ), vehicle)

    assert recovered.state is PidLandingState.DESCEND_CENTRAL
    assert recovered.command.velocity_x == 0.0
    assert recovered.command.velocity_y == 0.0
    assert recovered.command.velocity_z == 0.0
    assert recovered.command.yaw_rate_rad_s == 0.0


def test_central_loss_beyond_grace_enters_recovery_stopped(
    controller, vehicle,
):
    enter_central_descent(controller, vehicle)
    controller.update(2.4, central_observation(2.4), vehicle)
    first_missing = controller.update(2.5, None, vehicle)

    recovery = controller.update(2.81, None, vehicle)

    assert first_missing.command.velocity_z == 0.0
    assert recovery.state is PidLandingState.RECOVERY_CLIMB
    assert recovery.command.velocity_x == 0.0
    assert recovery.command.velocity_y == 0.0
    assert recovery.command.velocity_z == 0.0
    assert recovery.command.yaw_rate_rad_s == 0.0


def test_recovery_uses_local_altitude_and_ignores_camera_distance(vehicle):
    controller = PidLandingController(default_config())
    enter_acquire_central(controller, vehicle)
    transition = controller.update(
        4.72, None, vehicle_at(4.0, -2.0, 5.0),
    )

    camera_says_near_ground = controller.update(
        4.8,
        central_observation(4.8, distance_m=0.01),
        vehicle_at(4.0, -2.0, 5.24),
    )
    complete = controller.update(
        4.9, None, vehicle_at(4.0, -2.0, 5.25),
    )

    assert transition.state is PidLandingState.RECOVERY_CLIMB
    assert transition.command.velocity_z == 0.0
    assert (
        camera_says_near_ground.state is PidLandingState.RECOVERY_CLIMB
    )
    assert camera_says_near_ground.command.frame is VelocityFrame.BODY_FLU
    assert camera_says_near_ground.command.velocity_x == 0.0
    assert camera_says_near_ground.command.velocity_y == 0.0
    assert camera_says_near_ground.command.velocity_z == pytest.approx(0.2)
    assert camera_says_near_ground.command.yaw_rate_rad_s == 0.0
    assert complete.state is PidLandingState.ACQUIRE_CENTRAL
    assert complete.command.velocity_z == 0.0


def test_central_acquisition_timer_is_fresh_after_recovery(vehicle):
    controller = PidLandingController(default_config())
    enter_central_descent(controller, vehicle)
    controller.update(2.4, None, vehicle)
    controller.update(2.71, None, vehicle)
    controller.update(3.0, None, vehicle_at(4.0, -2.0, 4.25))

    first = controller.update(3.1, central_observation(3.1), vehicle)
    stable = controller.update(3.61, central_observation(3.61), vehicle)

    assert first.state is PidLandingState.ACQUIRE_CENTRAL
    assert stable.state is PidLandingState.DESCEND_CENTRAL


def test_state_entries_reset_all_pid_histories(vehicle):
    derivative_only = derivative_pid_config()
    controller = PidLandingController(default_config(
        tag_stable_time_s=0.1,
        alignment_stable_time_s=0.4,
        x_pid=derivative_only,
        y_pid=derivative_only,
        z_pid=derivative_only,
        yaw_pid=derivative_only,
    ))
    enter_align(controller, vehicle)
    controller.update(0.7, dock_observation(0.7), vehicle)
    controller.update(1.11, dock_observation(1.11), vehicle)

    first_outer = controller.update(1.2, dock_observation(
        1.2, forward_m=0.15, left_m=-0.15, yaw_error_rad=0.1,
    ), vehicle)
    controller.update(1.3, dock_observation(
        1.3, forward_m=0.15, left_m=-0.15, yaw_error_rad=0.1,
    ), vehicle)
    controller.update(1.4, dock_observation(
        1.4, forward_m=0.05, left_m=-0.05, distance_m=1.0,
        yaw_error_rad=0.05,
    ), vehicle)
    controller.update(1.5, central_observation(1.5), vehicle)
    controller.update(1.61, central_observation(1.61), vehicle)

    first_central = controller.update(1.7, central_observation(1.7), vehicle)

    assert first_outer.state is PidLandingState.DESCEND_OUTER
    assert first_outer.command.velocity_x == 0.0
    assert first_outer.command.velocity_y == 0.0
    assert first_outer.command.velocity_z == 0.0
    assert first_outer.command.yaw_rate_rad_s == 0.0
    assert first_central.state is PidLandingState.DESCEND_CENTRAL
    assert first_central.command.velocity_x == 0.0
    assert first_central.command.velocity_y == 0.0
    assert first_central.command.velocity_z == 0.0
    assert first_central.command.yaw_rate_rad_s == 0.0


def test_central_descent_visual_z_is_limited_to_central_descent_speed(
    controller, vehicle,
):
    enter_central_descent(controller, vehicle)
    controller.update(2.4, central_observation(2.4, distance_m=0.9), vehicle)

    saturated = controller.update(
        7.4, central_observation(7.4, distance_m=0.9), vehicle
    )

    assert saturated.state is PidLandingState.DESCEND_CENTRAL
    assert saturated.command.velocity_z == pytest.approx(-0.12)


def test_recovery_attempt_limit_fails_after_the_configured_number_of_climbs(
    vehicle,
):
    controller = PidLandingController(default_config(
        max_recovery_attempts=1,
    ))
    enter_acquire_central(controller, vehicle)
    first_recovery = controller.update(4.72, None, vehicle)
    controller.update(5.0, None, vehicle_at(4.0, -2.0, 4.25))

    failed = controller.update(8.01, None, vehicle_at(4.0, -2.0, 4.25))

    assert first_recovery.state is PidLandingState.RECOVERY_CLIMB
    assert failed.state is PidLandingState.FAILED
    assert failed.outcome is PidLandingOutcome.FAILED
    assert failed.reason == "central tag recovery attempts exhausted"
    assert failed.command.velocity_z == 0.0


def zero_body_command():
    return VelocityCommand(VelocityFrame.BODY_FLU, 0.0, 0.0, 0.0)


def grounded_vehicle(now, *, armed=True, flying=False, landed_at=None):
    return VehicleObservation(
        position_enu_m=(4.0, -2.0, 0.0),
        velocity_enu_m_s=(0.0, 0.0, 0.0),
        armed=armed,
        flying=flying,
        mode=4,
        landed_at=now if landed_at is None else landed_at,
    )


def enter_touchdown_approach(controller, vehicle):
    enter_central_descent(controller, vehicle)
    controller.update(2.4, central_observation(2.4, distance_m=0.25), vehicle)
    return controller.update(
        3.4, central_observation(3.4, distance_m=0.25), vehicle
    )


def advance_to_stable_ground(controller, vehicle):
    enter_touchdown_approach(controller, vehicle)
    controller.update(3.5, None, grounded_vehicle(3.5))
    return controller.update(4.0, None, grounded_vehicle(4.0, landed_at=3.5))


def test_central_descent_commits_only_after_fine_stability_at_commit_height(
    controller, vehicle,
):
    enter_central_descent(controller, vehicle)

    above_height = controller.update(
        2.4, central_observation(2.4, distance_m=0.26), vehicle
    )
    unstable = controller.update(
        3.4,
        dock_observation(
            3.4,
            forward_m=0.08,
            left_m=-0.05,
            distance_m=0.25,
            yaw_error_rad=0.05,
            mode=DockTargetMode.CENTRAL,
        ),
        vehicle,
    )
    stable_start = controller.update(
        3.5, central_observation(3.5, distance_m=0.25), vehicle
    )
    committed = controller.update(
        4.5, central_observation(4.5, distance_m=0.25), vehicle
    )

    assert above_height.state is PidLandingState.DESCEND_CENTRAL
    assert unstable.state is PidLandingState.DESCEND_CENTRAL
    assert stable_start.state is PidLandingState.DESCEND_CENTRAL
    assert committed.state is PidLandingState.TOUCHDOWN_APPROACH
    assert committed.command == zero_body_command()


def test_touchdown_approach_commands_constant_descent_and_ignores_tag_loss(
    controller, vehicle,
):
    transition = enter_touchdown_approach(controller, vehicle)

    decision = controller.update(3.5, None, vehicle)

    assert transition.state is PidLandingState.TOUCHDOWN_APPROACH
    assert transition.command == zero_body_command()
    assert decision.state is PidLandingState.TOUCHDOWN_APPROACH
    assert decision.command == VelocityCommand(
        VelocityFrame.BODY_FLU, 0.0, 0.0, -0.05, 0.0,
    )


def test_central_dropout_resets_fine_stability_before_touchdown_commit(
    controller, vehicle,
):
    enter_central_descent(controller, vehicle)
    controller.update(2.4, central_observation(2.4, distance_m=0.25), vehicle)

    dropout = controller.update(2.8, None, vehicle)
    returned = controller.update(
        2.9, central_observation(2.9, distance_m=0.25), vehicle
    )
    before_stable = controller.update(
        3.89, central_observation(3.89, distance_m=0.25), vehicle
    )
    committed = controller.update(
        3.91, central_observation(3.91, distance_m=0.25), vehicle
    )

    assert dropout.state is PidLandingState.DESCEND_CENTRAL
    assert returned.state is PidLandingState.DESCEND_CENTRAL
    assert before_stable.state is PidLandingState.DESCEND_CENTRAL
    assert committed.state is PidLandingState.TOUCHDOWN_APPROACH


def test_touchdown_timeout_fails_with_zero_command(vehicle):
    controller = PidLandingController(default_config(touchdown_timeout_s=0.5))
    enter_touchdown_approach(controller, vehicle)

    timed_out = controller.update(3.89, None, vehicle)

    assert timed_out.state is PidLandingState.TOUCHDOWN_APPROACH
    failed = controller.update(3.91, None, vehicle)
    assert failed.state is PidLandingState.FAILED
    assert failed.outcome is PidLandingOutcome.FAILED
    assert failed.command == zero_body_command()


def test_unexpected_disarm_before_touchdown_commit_fails_stopped(
    controller, vehicle,
):
    enter_central_descent(controller, vehicle)

    failed = controller.update(
        2.4,
        central_observation(2.4, distance_m=0.3),
        replace(vehicle, armed=False),
    )

    assert failed.state is PidLandingState.FAILED
    assert failed.outcome is PidLandingOutcome.FAILED
    assert failed.reason == "vehicle disarmed while mission was active"
    assert failed.command == zero_body_command()


def test_unexpected_disarm_during_touchdown_approach_fails_stopped(
    controller, vehicle,
):
    enter_touchdown_approach(controller, vehicle)

    failed = controller.update(3.5, None, grounded_vehicle(3.5, armed=False))

    assert failed.state is PidLandingState.FAILED
    assert failed.outcome is PidLandingOutcome.FAILED
    assert failed.reason == "vehicle disarmed while mission was active"
    assert failed.command == zero_body_command()


def test_touchdown_requires_fresh_continuous_false_flying_before_disarm(
    controller, vehicle,
):
    enter_touchdown_approach(controller, vehicle)

    stale = controller.update(3.5, None, grounded_vehicle(3.5, landed_at=2.4))
    first_fresh = controller.update(3.6, None, grounded_vehicle(3.6))
    unknown = controller.update(
        3.8, None, grounded_vehicle(3.8, flying=None, landed_at=3.6)
    )
    restarted = controller.update(3.9, None, grounded_vehicle(3.9))
    disarm = controller.update(4.4, None, grounded_vehicle(4.4, landed_at=3.9))

    assert stale.state is PidLandingState.TOUCHDOWN_APPROACH
    assert first_fresh.state is PidLandingState.TOUCHDOWN_APPROACH
    assert unknown.state is PidLandingState.TOUCHDOWN_APPROACH
    assert restarted.state is PidLandingState.TOUCHDOWN_APPROACH
    assert disarm.state is PidLandingState.DISARM
    assert disarm.request_disarm is True
    assert disarm.command == zero_body_command()


def test_disarm_rejection_retries_after_deadline_without_duplicate_requests(
    controller, vehicle,
):
    initial = advance_to_stable_ground(controller, vehicle)
    assert initial.disarm_request_id is not None
    controller.notify_disarm_result(initial.disarm_request_id, False, now=4.1)

    before_retry = controller.update(5.09, None, grounded_vehicle(5.09, landed_at=4.2))
    retry = controller.update(5.1, None, grounded_vehicle(5.1, landed_at=4.2))

    assert initial.request_disarm is True
    assert before_retry.state is PidLandingState.DISARM
    assert before_retry.request_disarm is False
    assert retry.state is PidLandingState.DISARM
    assert retry.request_disarm is True
    assert retry.command == zero_body_command()


def test_disarm_pending_request_is_not_reemitted_after_retry_deadline(
    controller, vehicle,
):
    initial = advance_to_stable_ground(controller, vehicle)

    pending = controller.update(
        5.1, None, grounded_vehicle(5.1, landed_at=4.2)
    )

    assert initial.request_disarm is True
    assert pending.state is PidLandingState.DISARM
    assert pending.request_disarm is False


def test_late_disarm_result_after_bounce_does_not_affect_later_attempt(
    controller, vehicle,
):
    initial = advance_to_stable_ground(controller, vehicle)
    assert initial.disarm_request_id is not None
    bounced = controller.update(4.1, None, vehicle_at(4.0, -2.0, 0.1))
    controller.notify_disarm_result(initial.disarm_request_id, True, now=4.2)
    controller.update(4.3, None, grounded_vehicle(4.3))

    later_attempt = controller.update(
        4.8, None, grounded_vehicle(4.8, landed_at=4.3)
    )

    assert bounced.state is PidLandingState.TOUCHDOWN_APPROACH
    assert later_attempt.state is PidLandingState.DISARM
    assert later_attempt.request_disarm is True


def test_disarm_acknowledgement_waits_for_disarmed_heartbeat(controller, vehicle):
    initial = advance_to_stable_ground(controller, vehicle)
    assert initial.disarm_request_id is not None
    controller.notify_disarm_result(initial.disarm_request_id, True, now=4.1)

    unknown = controller.update(4.2, None, grounded_vehicle(4.2, armed=None))
    waiting = controller.update(4.3, None, grounded_vehicle(4.3, armed=True))
    complete = controller.update(4.4, None, grounded_vehicle(4.4, armed=False))

    assert unknown.outcome is PidLandingOutcome.RUNNING
    assert unknown.request_disarm is False
    assert waiting.outcome is PidLandingOutcome.RUNNING
    assert waiting.request_disarm is False
    assert complete.state is PidLandingState.COMPLETE
    assert complete.outcome is PidLandingOutcome.COMPLETE
    assert complete.command == zero_body_command()


def test_disarm_ignores_delayed_result_for_an_earlier_retry_request(
    controller, vehicle,
):
    request_a = advance_to_stable_ground(controller, vehicle)
    assert request_a.disarm_request_id is not None
    controller.notify_disarm_result(
        request_a.disarm_request_id, False, now=4.1
    )
    request_b = controller.update(
        5.1, None, grounded_vehicle(5.1, landed_at=4.2)
    )
    assert request_b.disarm_request_id is not None

    controller.notify_disarm_result(request_a.disarm_request_id, True, now=5.2)
    still_pending_b = controller.update(
        5.3, None, grounded_vehicle(5.3, landed_at=4.4)
    )
    controller.notify_disarm_result(
        request_b.disarm_request_id, False, now=5.4
    )
    before_b_retry = controller.update(
        6.39, None, grounded_vehicle(6.39, landed_at=5.4)
    )
    after_b_retry = controller.update(
        6.4, None, grounded_vehicle(6.4, landed_at=5.4)
    )

    assert request_b.disarm_request_id > request_a.disarm_request_id
    assert still_pending_b.request_disarm is False
    assert before_b_retry.request_disarm is False
    assert after_b_retry.request_disarm is True
    assert after_b_retry.disarm_request_id > request_b.disarm_request_id


@pytest.mark.parametrize(
    "bad_vehicle",
    [
        grounded_vehicle(4.1, armed=False, flying=True),
        grounded_vehicle(4.1, armed=False, landed_at=3.0),
        grounded_vehicle(4.1, armed=False, landed_at=math.nan),
    ],
)
def test_disarm_requires_fresh_ground_telemetry_before_armed_false_completes(
    controller, vehicle, bad_vehicle,
):
    advance_to_stable_ground(controller, vehicle)

    decision = controller.update(4.1, None, bad_vehicle)

    assert decision.state is PidLandingState.FAILED
    assert decision.outcome is PidLandingOutcome.FAILED
    assert decision.reason == "vehicle disarmed without fresh ground telemetry"
    assert decision.command == zero_body_command()


def test_disarm_flying_again_returns_to_touchdown_approach(controller, vehicle):
    advance_to_stable_ground(controller, vehicle)

    bounced = controller.update(4.1, None, vehicle_at(4.0, -2.0, 0.1))

    assert bounced.state is PidLandingState.TOUCHDOWN_APPROACH
    assert bounced.request_disarm is False
    assert bounced.command == VelocityCommand(
        VelocityFrame.BODY_FLU, 0.0, 0.0, -0.05, 0.0,
    )

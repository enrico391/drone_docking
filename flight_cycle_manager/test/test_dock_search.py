import math

import pytest

from flight_cycle_manager.dock_search import (
    DockSearchController,
    SearchConfig,
    SearchObservation,
    SearchOutcome,
)


def search_config(**overrides):
    values = dict(
        radius_m=1.0,
        tangential_speed_m_s=0.30,
        radial_gain=0.70,
        altitude_gain=0.70,
        max_horizontal_speed_m_s=0.50,
        max_vertical_speed_m_s=0.50,
        target_stable_time_s=0.50,
        timeout_s=30.0,
        max_displacement_m=2.0,
    )
    values.update(overrides)
    return SearchConfig(**values)


def observation(now, position=(0.0, 0.0, 4.0), target_visible=False):
    return SearchObservation(now, position, target_visible)


def started_controller(**config_overrides):
    controller = DockSearchController(search_config(**config_overrides))
    controller.start(now=0.0, center_enu_m=(0.0, 0.0, 4.0))
    return controller


def test_search_moves_to_radius_then_orbits_with_altitude_hold():
    controller = DockSearchController(search_config())
    controller.start(now=0.0, center_enu_m=(10.0, 20.0, 4.0))

    first = controller.update(observation(0.1, (10.0, 20.0, 3.8)))
    assert first.outcome is SearchOutcome.SEARCHING
    assert first.velocity_enu_m_s[0] > 0.0
    assert first.velocity_enu_m_s[2] > 0.0

    orbit = controller.update(observation(1.0, (11.0, 20.0, 4.0)))
    assert orbit.velocity_enu_m_s[1] > 0.0
    assert math.hypot(*orbit.velocity_enu_m_s[:2]) <= 0.50


def test_target_requires_stable_dwell_before_acquired():
    controller = started_controller()

    assert controller.update(observation(1.0, target_visible=True)).outcome is SearchOutcome.SEARCHING
    acquired = controller.update(observation(1.5, target_visible=True))

    assert acquired.outcome is SearchOutcome.ACQUIRED
    assert acquired.velocity_enu_m_s == (0.0, 0.0, 0.0)


def test_target_dwell_resets_after_one_invisible_observation():
    controller = started_controller()

    controller.update(observation(1.0, target_visible=True))
    controller.update(observation(1.3, target_visible=False))

    assert controller.update(observation(1.6, target_visible=True)).outcome is SearchOutcome.SEARCHING
    assert controller.update(observation(1.9, target_visible=True)).outcome is SearchOutcome.SEARCHING
    assert controller.update(observation(2.1, target_visible=True)).outcome is SearchOutcome.ACQUIRED


def test_timeout_and_spatial_bound_are_terminal():
    assert started_controller().update(observation(30.0)).outcome is SearchOutcome.TIMED_OUT
    assert started_controller().update(observation(1.0, (2.01, 0.0, 4.0))).outcome is SearchOutcome.OUT_OF_BOUNDS


def test_terminal_outcomes_hold_zero_velocity_on_later_updates():
    controller = started_controller()
    assert controller.update(observation(31.0)).velocity_enu_m_s == (0.0, 0.0, 0.0)
    assert controller.update(observation(32.0, target_visible=True)).outcome is SearchOutcome.TIMED_OUT
    assert controller.update(observation(32.0)).velocity_enu_m_s == (0.0, 0.0, 0.0)


def test_radial_correction_points_outward_inside_radius_and_inward_outside():
    inside = started_controller(radial_gain=1.0, tangential_speed_m_s=0.0)
    outside = started_controller(radial_gain=1.0, tangential_speed_m_s=0.0)

    inside_velocity = inside.update(observation(1.0, (0.5, 0.0, 4.0))).velocity_enu_m_s
    outside_velocity = outside.update(observation(1.0, (1.5, 0.0, 4.0))).velocity_enu_m_s

    assert inside_velocity[0] > 0.0
    assert outside_velocity[0] < 0.0


def test_horizontal_speed_is_saturated_without_changing_radial_sign():
    controller = started_controller(radial_gain=10.0, tangential_speed_m_s=10.0)
    decision = controller.update(observation(1.0, (0.5, 0.5, 4.0)))

    assert math.hypot(*decision.velocity_enu_m_s[:2]) == pytest.approx(0.50)
    assert decision.velocity_enu_m_s[0] > 0.0
    assert decision.velocity_enu_m_s[1] > 0.0


def test_vertical_speed_is_saturated_in_both_directions():
    controller = started_controller(altitude_gain=10.0)

    above = controller.update(observation(1.0, (0.0, 0.0, 3.0)))
    below = controller.update(observation(2.0, (0.0, 0.0, 5.0)))

    assert above.velocity_enu_m_s[2] == pytest.approx(0.50)
    assert below.velocity_enu_m_s[2] == pytest.approx(-0.50)


def test_update_before_start_is_rejected():
    controller = DockSearchController(search_config())

    with pytest.raises(RuntimeError, match="start"):
        controller.update(observation(0.0))


@pytest.mark.parametrize(
    "field,value",
    [
        ("radius_m", math.nan),
        ("tangential_speed_m_s", math.inf),
        ("radial_gain", -math.inf),
        ("altitude_gain", math.nan),
        ("max_horizontal_speed_m_s", math.inf),
        ("max_vertical_speed_m_s", math.nan),
        ("target_stable_time_s", math.inf),
        ("timeout_s", math.nan),
        ("max_displacement_m", math.inf),
    ],
)
def test_rejects_non_finite_search_configuration(field, value):
    with pytest.raises(ValueError, match="finite"):
        SearchConfig(**{**search_config().__dict__, field: value})


@pytest.mark.parametrize(
    "bad_observation",
    [
        SearchObservation(math.nan, (0.0, 0.0, 4.0), False),
        SearchObservation(1.0, (math.inf, 0.0, 4.0), False),
        SearchObservation(1.0, (0.0, math.nan, 4.0), False),
        SearchObservation(1.0, (0.0, 0.0, math.inf), False),
        SearchObservation(1.0, (0.0, 0.0), False),
    ],
)
def test_rejects_non_finite_or_malformed_observation(bad_observation):
    controller = started_controller()

    with pytest.raises(ValueError):
        controller.update(bad_observation)


def test_rejects_non_monotonic_caller_time():
    controller = started_controller()
    controller.update(observation(1.0))

    with pytest.raises(ValueError, match="monotonic"):
        controller.update(observation(0.9))


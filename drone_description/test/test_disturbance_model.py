import pytest

from drone_description.disturbance_model import (
    DisturbanceModel,
    DisturbanceSettings,
)


def test_ground_effect_fades_to_zero_at_configured_height():
    settings = DisturbanceSettings(
        ground_effect_enabled=True,
        ground_effect_min_height=0.2,
        ground_effect_max_height=0.8,
        ground_effect_max_force=1.2,
    )
    model = DisturbanceModel(settings)

    near = model.step(dt=0.02, sim_time=1.0, rotor_height=0.2)
    midway = model.step(dt=0.02, sim_time=1.02, rotor_height=0.5)
    far = model.step(dt=0.02, sim_time=1.04, rotor_height=0.8)

    assert near.force == pytest.approx((0.0, 0.0, 1.2))
    assert midway.force == pytest.approx((0.0, 0.0, 0.3))
    assert far.force == pytest.approx((0.0, 0.0, 0.0))


def test_body_frame_constant_wrench_rotates_into_world_frame():
    settings = DisturbanceSettings(
        frame="body",
        constant_force=(1.0, 0.0, 0.0),
        constant_torque=(0.0, 1.0, 0.0),
    )
    model = DisturbanceModel(settings)

    wrench = model.step(
        dt=0.02,
        sim_time=1.0,
        rotor_height=None,
        orientation_xyzw=(0.0, 0.0, 2**-0.5, 2**-0.5),
    )

    assert wrench.force == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    assert wrench.torque == pytest.approx((-1.0, 0.0, 0.0), abs=1e-12)


def test_colored_turbulence_is_repeatable_for_a_seed_and_changes_over_time():
    settings = DisturbanceSettings(
        random_seed=17,
        turbulence_enabled=True,
        turbulence_force_stddev=(0.2, 0.1, 0.05),
        turbulence_torque_stddev=(0.02, 0.01, 0.005),
        turbulence_correlation_time=0.3,
    )
    first = DisturbanceModel(settings)
    second = DisturbanceModel(settings)

    first_samples = [
        first.step(dt=0.02, sim_time=index * 0.02, rotor_height=None)
        for index in range(3)
    ]
    second_samples = [
        second.step(dt=0.02, sim_time=index * 0.02, rotor_height=None)
        for index in range(3)
    ]

    assert first_samples == second_samples
    assert first_samples[0] != first_samples[1]
    assert first_samples[0].force != (0.0, 0.0, 0.0)


def test_gust_is_active_only_during_each_configured_pulse():
    model = DisturbanceModel(
        DisturbanceSettings(
            gust_enabled=True,
            gust_force=(0.8, -0.2, 0.0),
            gust_torque=(0.0, 0.01, 0.0),
            gust_duration=0.25,
            gust_interval=2.0,
        )
    )

    initial = model.step(dt=0.02, sim_time=0.10, rotor_height=None)
    between = model.step(dt=0.02, sim_time=0.30, rotor_height=None)
    repeated = model.step(dt=0.02, sim_time=2.10, rotor_height=None)

    assert initial.force == pytest.approx((0.8, -0.2, 0.0))
    assert initial.torque == pytest.approx((0.0, 0.01, 0.0))
    assert between.force == (0.0, 0.0, 0.0)
    assert repeated == initial


def test_total_wrench_is_clamped_per_axis_after_sources_are_combined():
    model = DisturbanceModel(
        DisturbanceSettings(
            constant_force=(2.0, -3.0, 0.5),
            constant_torque=(0.2, -0.3, 0.04),
            force_limits=(1.0, 1.5, 0.25),
            torque_limits=(0.1, 0.1, 0.02),
        )
    )

    wrench = model.step(dt=0.02, sim_time=0.0, rotor_height=None)

    assert wrench.force == (1.0, -1.5, 0.25)
    assert wrench.torque == (0.1, -0.1, 0.02)


def test_ground_effect_amplifies_seeded_turbulence_near_the_surface():
    common = dict(
        random_seed=9,
        turbulence_enabled=True,
        turbulence_force_stddev=(0.2, 0.2, 0.2),
        turbulence_torque_stddev=(0.01, 0.01, 0.01),
        ground_effect_enabled=True,
        ground_effect_min_height=0.2,
        ground_effect_max_height=0.8,
    )
    normal = DisturbanceModel(DisturbanceSettings(**common))
    amplified = DisturbanceModel(
        DisturbanceSettings(**common, ground_effect_turbulence_gain=2.0)
    )

    normal_wrench = normal.step(dt=0.02, sim_time=0.0, rotor_height=0.2)
    amplified_wrench = amplified.step(dt=0.02, sim_time=0.0, rotor_height=0.2)

    assert amplified_wrench.force == pytest.approx(
        tuple(2.0 * value for value in normal_wrench.force)
    )
    assert amplified_wrench.torque == pytest.approx(
        tuple(2.0 * value for value in normal_wrench.torque)
    )

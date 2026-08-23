"""Tests for native precision-landing quality and checkpoint gates."""

import math

import pytest

from drone_mission_manager.landing_quality import (
    CircularYawWindow,
    DescendingCheckpoint,
    FreshMeasurementWindow,
    StableCondition,
    surface_relative_height,
    target_loss_requires_retry,
    yaw_turn_can_finish_above,
)


def test_fresh_stream_requires_one_second_at_five_hz():
    """A sparse or short stream cannot authorize autonomous alignment."""
    stream = FreshMeasurementWindow(
        required_duration_s=1.0,
        minimum_rate_hz=5.0,
        maximum_gap_s=0.25,
        maximum_age_s=0.35,
    )

    for sample_time in (0.0, 0.2, 0.4, 0.6, 0.8):
        stream.observe(sample_time)
    assert not stream.ready(now_s=0.8)

    stream.observe(1.0)
    assert stream.ready(now_s=1.0)


def test_fresh_stream_resets_after_excessive_gap():
    """A new sample after a dropout starts a new continuity interval."""
    stream = FreshMeasurementWindow(
        required_duration_s=1.0,
        minimum_rate_hz=5.0,
        maximum_gap_s=0.25,
        maximum_age_s=0.35,
    )
    for sample_time in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        stream.observe(sample_time)
    assert stream.ready(now_s=1.0)

    stream.observe(1.4)

    assert not stream.ready(now_s=1.4)


def test_fresh_stream_rejects_an_aged_latest_measurement():
    """Elapsed warmup cannot hide a stale latest measurement."""
    stream = FreshMeasurementWindow(
        required_duration_s=1.0,
        minimum_rate_hz=5.0,
        maximum_gap_s=0.25,
        maximum_age_s=0.35,
    )
    for sample_time in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        stream.observe(sample_time)

    assert not stream.ready(now_s=1.36)


def test_fresh_stream_rejects_four_point_nine_hz():
    """Duration alone cannot authorize a stream below the minimum rate."""
    stream = FreshMeasurementWindow(
        required_duration_s=1.0,
        minimum_rate_hz=5.0,
        maximum_gap_s=0.25,
        maximum_age_s=0.35,
    )
    for index in range(6):
        stream.observe(index / 4.9)

    assert not stream.ready(now_s=5.0 / 4.9)


def test_circular_yaw_window_handles_wraparound():
    """Measurements around plus/minus 180 degrees remain a stable estimate."""
    yaw = CircularYawWindow(maximum_gap_s=0.25, maximum_age_s=0.35)
    values_deg = (179.0, -179.0, 180.0, -178.5, 179.5, -180.0)
    for index, value_deg in enumerate(values_deg):
        yaw.observe(index * 0.2, math.radians(value_deg))

    estimate = yaw.estimate(
        now_s=1.0,
        required_duration_s=1.0,
        minimum_rate_hz=5.0,
    )

    assert estimate is not None
    assert abs(abs(math.degrees(estimate.mean_rad)) - 180.0) < 0.5
    assert math.degrees(estimate.jitter_rad) == pytest.approx(2.5)


def test_continuous_condition_resets_when_any_gate_fails():
    """A stability timer represents continuous simultaneous acceptance."""
    stable = StableCondition(required_duration_s=1.0)

    assert not stable.update(now_s=0.0, accepted=True)
    assert not stable.update(now_s=0.8, accepted=True)
    assert not stable.update(now_s=0.9, accepted=False)
    assert not stable.update(now_s=1.5, accepted=True)
    assert stable.update(now_s=2.5, accepted=True)


def test_surface_relative_height_uses_the_configured_dock_datum():
    """FCU local Z is converted from height above home to height above dock."""
    assert surface_relative_height(4.88, 0.88) == pytest.approx(4.0)


def test_slow_two_metre_yaw_turn_is_rejected_near_final_lock_height():
    """A correction is skipped if descent can carry it into the final metre."""
    assert not yaw_turn_can_finish_above(
        height_m=2.0,
        floor_height_m=1.0,
        correction_deg=10.0,
        yaw_rate_deg_s=3.0,
        descent_speed_m_s=0.25,
        clearance_margin_m=0.2,
    )
    assert yaw_turn_can_finish_above(
        height_m=2.0,
        floor_height_m=1.0,
        correction_deg=10.0,
        yaw_rate_deg_s=5.0,
        descent_speed_m_s=0.25,
        clearance_margin_m=0.2,
    )


def test_altitude_checkpoint_requires_a_debounced_crossing_and_fires_once():
    """FCU height noise cannot trigger a checkpoint on a single timer tick."""
    checkpoint = DescendingCheckpoint(
        threshold_m=3.0, confirmation_duration_s=0.15
    )

    assert not checkpoint.update(now_s=0.0, height_m=3.05, valid=True)
    assert not checkpoint.update(now_s=0.1, height_m=2.98, valid=True)
    assert not checkpoint.update(now_s=0.2, height_m=3.02, valid=True)
    assert not checkpoint.update(now_s=0.3, height_m=2.97, valid=True)
    assert checkpoint.update(now_s=0.45, height_m=2.96, valid=True)
    assert not checkpoint.update(now_s=0.6, height_m=2.90, valid=True)


def test_target_loss_policy_uses_fcu_height_and_low_clearance_boundary():
    """The default retries only while FCU-estimated clearance remains."""
    assert target_loss_requires_retry(
        height_m=None, clearance_height_m=1.2, policy="continue"
    )
    assert target_loss_requires_retry(
        height_m=3.0, clearance_height_m=1.2, policy="continue"
    )
    assert target_loss_requires_retry(
        height_m=1.5, clearance_height_m=1.2, policy="continue"
    )
    assert not target_loss_requires_retry(
        height_m=1.1, clearance_height_m=1.2, policy="continue"
    )
    assert not target_loss_requires_retry(
        height_m=0.8, clearance_height_m=1.2, policy="continue"
    )
    assert target_loss_requires_retry(
        height_m=0.8, clearance_height_m=1.2, policy="retry"
    )

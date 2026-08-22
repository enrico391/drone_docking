"""Tests for temporal dock-pose filtering."""

import math

import numpy as np
import pytest

from dock_apriltag_localizer.pose_filter import TemporalPoseFilter


def make_pose(x=0.0, y=0.0, z=2.0, yaw=0.0):
    pose = np.eye(4)
    pose[0, 0] = math.cos(yaw)
    pose[0, 1] = -math.sin(yaw)
    pose[1, 0] = math.sin(yaw)
    pose[1, 1] = math.cos(yaw)
    pose[:3, 3] = [x, y, z]
    return pose


def test_filter_smooths_position_and_preserves_rotation_matrix():
    pose_filter = TemporalPoseFilter(
        time_constant_s=0.2,
        max_position_jump_m=2.0,
    )
    first, _ = pose_filter.update(make_pose(), 1.0)
    filtered, status = pose_filter.update(
        make_pose(x=1.0, yaw=math.pi / 2.0), 1.1
    )

    expected_alpha = 1.0 - math.exp(-0.1 / 0.2)
    assert status == "accepted"
    assert filtered[0, 3] == pytest.approx(expected_alpha)
    np.testing.assert_allclose(
        filtered[:3, :3].T @ filtered[:3, :3], np.eye(3), atol=1e-9
    )
    assert np.linalg.det(filtered[:3, :3]) == pytest.approx(1.0)
    assert first[0, 3] == pytest.approx(0.0)


def test_filter_rejects_isolated_position_jump():
    pose_filter = TemporalPoseFilter(
        time_constant_s=0.0,
        max_translation_speed_m_s=1.0,
        max_position_jump_m=0.1,
        reacquire_frames=3,
    )
    pose_filter.update(make_pose(), 1.0)

    filtered, status = pose_filter.update(make_pose(x=1.0), 1.1)

    assert filtered is None
    assert status == "outlier_rejected"


def test_filter_reacquires_after_consistent_rejected_measurements():
    pose_filter = TemporalPoseFilter(
        time_constant_s=0.0,
        max_translation_speed_m_s=1.0,
        max_position_jump_m=0.1,
        reacquire_frames=3,
    )
    pose_filter.update(make_pose(), 1.0)
    assert pose_filter.update(make_pose(x=1.00), 1.1)[0] is None
    assert pose_filter.update(make_pose(x=1.02), 1.2)[0] is None

    filtered, status = pose_filter.update(make_pose(x=1.04), 1.3)

    assert status == "reacquired"
    assert filtered[0, 3] == pytest.approx(1.025)


def test_filter_resets_after_input_gap():
    pose_filter = TemporalPoseFilter(reset_timeout_s=0.5)
    pose_filter.update(make_pose(), 1.0)

    filtered, status = pose_filter.update(make_pose(x=2.0), 1.6)

    assert status == "reset_after_gap"
    assert filtered[0, 3] == pytest.approx(2.0)


def test_filter_rejects_repeated_timestamp():
    pose_filter = TemporalPoseFilter()
    pose_filter.update(make_pose(), 1.0)

    filtered, status = pose_filter.update(make_pose(x=0.1), 1.0)

    assert filtered is None
    assert status == "non_monotonic_timestamp"

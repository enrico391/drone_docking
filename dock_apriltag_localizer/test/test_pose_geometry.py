"""Tests for dock/tag pose frame conventions."""

import numpy as np

from dock_apriltag_localizer.dock_localizer import correct_pnp_tag_rotation


def test_pnp_normal_is_flipped_to_physical_up_axis():
    """A raw +Z PnP normal becomes -Z in the downward optical frame."""
    corrected = correct_pnp_tag_rotation(np.eye(3))

    np.testing.assert_allclose(corrected[:, 0], [-1.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(corrected[:, 1], [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(corrected[:, 2], [0.0, 0.0, -1.0], atol=1e-9)


def test_corrected_outer_tag_offset_recovers_common_dock_center():
    """Applying a physical tag offset recovers the same dock origin."""
    raw_rotation = np.array(
        [
            [-0.16, -0.987, 0.0],
            [0.987, -0.16, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    rotation = correct_pnp_tag_rotation(raw_rotation)
    dock_center = np.array([0.13, 0.48, 4.90])
    dock_to_tag = np.array([0.52, 0.52, 0.001])
    measured_tag = dock_center + rotation @ dock_to_tag

    recovered_dock = measured_tag - rotation @ dock_to_tag
    np.testing.assert_allclose(recovered_dock, dock_center, atol=1e-9)

"""Tests for YAML mission loading and validation."""

from pathlib import Path

import pytest

from drone_mission_manager.mission import load_circle_mission


MISSION_FILE = (
    Path(__file__).parents[1]
    / "config"
    / "missions"
    / "simple_circle.yaml"
)


def test_load_simple_circle_mission():
    """The checked-in simulation mission has the expected safe endpoint."""
    mission = load_circle_mission(str(MISSION_FILE))

    assert mission.takeoff_altitude_m == 5.0
    assert mission.circle_radius_m == 3.0
    assert mission.circle_laps == 1.0
    assert (mission.dock_x_m, mission.dock_y_m) == (10.0, 0.0)
    assert mission.hover_altitude_m == 5.0
    assert mission.precision_landing_enabled is True


def test_rejects_non_positive_circle_radius(tmp_path):
    """Invalid geometry is rejected before any ROS flight interfaces start."""
    mission_file = tmp_path / "invalid.yaml"
    mission_file.write_text(
        """
mission:
  name: invalid
  takeoff: {altitude_m: 5.0}
  circle:
    center_x_m: 5.0
    center_y_m: 0.0
    radius_m: 0.0
    speed_m_s: 1.0
    laps: 1.0
    clockwise: false
  return:
    dock_x_m: 10.0
    dock_y_m: 0.0
    hover_altitude_m: 5.0
  precision_landing: {enabled: true}
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="radius_m"):
        load_circle_mission(str(mission_file))

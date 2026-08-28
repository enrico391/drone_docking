"""Tests for YAML mission loading and validation."""

from pathlib import Path

import pytest

from drone_mission_manager.mission import (
    CoordinateReference,
    MissionType,
    load_mission,
)


MISSION_FILE = (
    Path(__file__).parents[1]
    / "config"
    / "missions"
    / "simple_circle.yaml"
)
DOCK_MISSION_FILE = MISSION_FILE.with_name("dock_circle.yaml")


def test_load_simple_circle_mission():
    """The checked-in simulation mission has the expected safe endpoint."""
    mission = load_mission(str(MISSION_FILE))

    assert mission.mission_type == MissionType.CIRCLE
    assert mission.coordinate_reference == CoordinateReference.MAP
    assert mission.takeoff_altitude_m == 5.0
    assert mission.circle_radius_m == 3.0
    assert mission.circle_laps == 1.0
    assert (mission.dock_x_m, mission.dock_y_m) == (10.0, 0.0)
    assert mission.hover_altitude_m == 5.0
    assert mission.precision_landing_enabled is True


def test_load_dock_circle_mission_relative_to_initial_pose():
    """The dock mission circles and returns to its captured start point."""
    mission = load_mission(str(DOCK_MISSION_FILE))

    assert mission.mission_type == MissionType.DOCK_CIRCLE
    assert mission.coordinate_reference == CoordinateReference.INITIAL_POSE
    assert mission.circle_radius_m == 2.0
    assert (mission.circle_center_x_m, mission.circle_center_y_m) == (0.0, 0.0)
    assert mission.resolve_xy(0.0, 0.0, 12.0, -3.0) == (12.0, -3.0)
    assert mission.precision_landing_enabled is True
    assert mission.repeat_count == 3
    assert mission.repeat_delay_s == 3.0
    assert mission.has_next_iteration(1) is True


def test_dock_circle_can_repeat_multiple_complete_cycles(tmp_path):
    """A repeat count represents total takeoff-to-disarm iterations."""
    mission_file = tmp_path / "repeated_dock_circle.yaml"
    mission_file.write_text(
        DOCK_MISSION_FILE.read_text(encoding="utf-8").replace(
            "count: 1", "count: 3"
        ),
        encoding="utf-8",
    )

    mission = load_mission(str(mission_file))

    assert mission.repeat_count == 3
    assert mission.has_next_iteration(1) is True
    assert mission.has_next_iteration(2) is True
    assert mission.has_next_iteration(3) is False


def test_rejects_non_positive_circle_radius(tmp_path):
    """Invalid geometry is rejected before any ROS flight interfaces start."""
    mission_file = tmp_path / "invalid.yaml"
    mission_file.write_text(
        """
mission:
  name: invalid
  type: circle
  coordinates: {reference: map}
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
        load_mission(str(mission_file))


def test_rejects_unknown_mission_type(tmp_path):
    """Unsupported mission executors are rejected with a useful error."""
    mission_file = tmp_path / "invalid_type.yaml"
    mission_file.write_text(
        "mission: {name: invalid, type: survey}", encoding="utf-8"
    )

    with pytest.raises(ValueError, match="mission.type"):
        load_mission(str(mission_file))


def test_rejects_zero_repeat_count(tmp_path):
    """A configured mission must always execute at least once."""
    mission_file = tmp_path / "invalid_repeat.yaml"
    mission_file.write_text(
        DOCK_MISSION_FILE.read_text(encoding="utf-8").replace(
            "count: 3", "count: 0"
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="at least one"):
        load_mission(str(mission_file))

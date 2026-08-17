"""Loading and validation for mission-manager YAML files."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


@dataclass(frozen=True)
class CircleMission:
    """A circle mission with optional visual landing in local ENU coordinates."""

    name: str
    takeoff_altitude_m: float
    circle_center_x_m: float
    circle_center_y_m: float
    circle_radius_m: float
    circle_speed_m_s: float
    circle_laps: float
    circle_clockwise: bool
    dock_x_m: float
    dock_y_m: float
    hover_altitude_m: float
    precision_landing_enabled: bool


def _mapping(parent: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"mission.{key} must be a mapping")
    return value


def _number(parent: Dict[str, Any], key: str, path: str) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path}.{key} must be a number")
    return float(value)


def load_circle_mission(path: str) -> CircleMission:
    """Load and validate a circle mission from *path*."""
    mission_path = Path(path).expanduser()
    if not mission_path.is_file():
        raise ValueError(f"mission file does not exist: {mission_path}")

    with mission_path.open("r", encoding="utf-8") as mission_file:
        document = yaml.safe_load(mission_file)

    if not isinstance(document, dict):
        raise ValueError("mission file must contain a YAML mapping")
    mission = _mapping(document, "mission")
    takeoff = _mapping(mission, "takeoff")
    circle = _mapping(mission, "circle")
    mission_return = _mapping(mission, "return")
    precision_landing = _mapping(mission, "precision_landing")

    name = mission.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("mission.name must be a non-empty string")

    clockwise = circle.get("clockwise", False)
    if not isinstance(clockwise, bool):
        raise ValueError("mission.circle.clockwise must be a boolean")
    precision_enabled = precision_landing.get("enabled")
    if not isinstance(precision_enabled, bool):
        raise ValueError("mission.precision_landing.enabled must be a boolean")

    result = CircleMission(
        name=name,
        takeoff_altitude_m=_number(takeoff, "altitude_m", "mission.takeoff"),
        circle_center_x_m=_number(circle, "center_x_m", "mission.circle"),
        circle_center_y_m=_number(circle, "center_y_m", "mission.circle"),
        circle_radius_m=_number(circle, "radius_m", "mission.circle"),
        circle_speed_m_s=_number(circle, "speed_m_s", "mission.circle"),
        circle_laps=_number(circle, "laps", "mission.circle"),
        circle_clockwise=clockwise,
        dock_x_m=_number(mission_return, "dock_x_m", "mission.return"),
        dock_y_m=_number(mission_return, "dock_y_m", "mission.return"),
        hover_altitude_m=_number(
            mission_return, "hover_altitude_m", "mission.return"
        ),
        precision_landing_enabled=precision_enabled,
    )

    positive_values = {
        "mission.takeoff.altitude_m": result.takeoff_altitude_m,
        "mission.circle.radius_m": result.circle_radius_m,
        "mission.circle.speed_m_s": result.circle_speed_m_s,
        "mission.circle.laps": result.circle_laps,
        "mission.return.hover_altitude_m": result.hover_altitude_m,
    }
    for field, value in positive_values.items():
        if value <= 0.0:
            raise ValueError(f"{field} must be greater than zero")

    return result

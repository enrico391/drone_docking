"""Typed loading and validation for mission-manager YAML files."""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict

import yaml


class MissionType(str, Enum):
    """Mission implementations supported by the manager."""

    CIRCLE = "circle"
    DOCK_CIRCLE = "dock_circle"


class CoordinateReference(str, Enum):
    """Reference used by horizontal mission coordinates."""

    MAP = "map"
    INITIAL_POSE = "initial_pose"


@dataclass(frozen=True)
class CircleMission:
    """A circular flight followed by an optional visual landing."""

    name: str
    mission_type: MissionType
    coordinate_reference: CoordinateReference
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
    repeat_count: int
    repeat_delay_s: float

    def resolve_xy(
        self, x_m: float, y_m: float, initial_x_m: float, initial_y_m: float
    ) -> tuple[float, float]:
        """Resolve mission coordinates into the local ENU map frame."""
        if self.coordinate_reference == CoordinateReference.INITIAL_POSE:
            return initial_x_m + x_m, initial_y_m + y_m
        return x_m, y_m

    def has_next_iteration(self, completed_iterations: int) -> bool:
        """Return whether another complete flight cycle is configured."""
        return completed_iterations < self.repeat_count


def _mapping(
    parent: Dict[str, Any], key: str, path: str = "mission"
) -> Dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        prefix = f"{path}." if path else ""
        raise ValueError(f"{prefix}{key} must be a mapping")
    return value


def _number(parent: Dict[str, Any], key: str, path: str) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path}.{key} must be a number")
    return float(value)


def _repeat_fields(mission: Dict[str, Any]) -> Dict[str, Any]:
    repeat = mission.get("repeat", {})
    if not isinstance(repeat, dict):
        raise ValueError("mission.repeat must be a mapping")

    count = repeat.get("count", 1)
    if isinstance(count, bool) or not isinstance(count, int):
        raise ValueError("mission.repeat.count must be an integer")
    if count < 1:
        raise ValueError("mission.repeat.count must be at least one")

    delay = repeat.get("delay_s", 2.0)
    if isinstance(delay, bool) or not isinstance(delay, (int, float)):
        raise ValueError("mission.repeat.delay_s must be a number")
    if delay < 0.0:
        raise ValueError("mission.repeat.delay_s cannot be negative")
    return {"repeat_count": count, "repeat_delay_s": float(delay)}


def _common_fields(mission: Dict[str, Any]) -> Dict[str, Any]:
    name = mission.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("mission.name must be a non-empty string")

    coordinates = _mapping(mission, "coordinates")
    reference_value = coordinates.get("reference")
    if not isinstance(reference_value, str):
        raise ValueError("mission.coordinates.reference must be a string")
    try:
        reference = CoordinateReference(reference_value)
    except ValueError as exception:
        accepted = ", ".join(value.value for value in CoordinateReference)
        raise ValueError(
            f"mission.coordinates.reference must be one of: {accepted}"
        ) from exception

    takeoff = _mapping(mission, "takeoff")
    mission_return = _mapping(mission, "return")
    precision_landing = _mapping(mission, "precision_landing")
    precision_enabled = precision_landing.get("enabled")
    if not isinstance(precision_enabled, bool):
        raise ValueError("mission.precision_landing.enabled must be a boolean")

    return {
        "name": name,
        "coordinate_reference": reference,
        "takeoff_altitude_m": _number(
            takeoff, "altitude_m", "mission.takeoff"
        ),
        "dock_x_m": _number(mission_return, "dock_x_m", "mission.return"),
        "dock_y_m": _number(mission_return, "dock_y_m", "mission.return"),
        "hover_altitude_m": _number(
            mission_return, "hover_altitude_m", "mission.return"
        ),
        "precision_landing_enabled": precision_enabled,
        **_repeat_fields(mission),
    }


def _circle_fields(circle: Dict[str, Any]) -> Dict[str, Any]:
    clockwise = circle.get("clockwise", False)
    if not isinstance(clockwise, bool):
        raise ValueError("mission.circle.clockwise must be a boolean")
    return {
        "circle_radius_m": _number(circle, "radius_m", "mission.circle"),
        "circle_speed_m_s": _number(circle, "speed_m_s", "mission.circle"),
        "circle_laps": _number(circle, "laps", "mission.circle"),
        "circle_clockwise": clockwise,
    }


def _load_circle(mission: Dict[str, Any]) -> CircleMission:
    circle = _mapping(mission, "circle")
    return CircleMission(
        mission_type=MissionType.CIRCLE,
        circle_center_x_m=_number(
            circle, "center_x_m", "mission.circle"
        ),
        circle_center_y_m=_number(
            circle, "center_y_m", "mission.circle"
        ),
        **_common_fields(mission),
        **_circle_fields(circle),
    )


def _load_dock_circle(mission: Dict[str, Any]) -> CircleMission:
    """Load a circle whose center is always the configured landing dock."""
    circle = _mapping(mission, "circle")
    common = _common_fields(mission)
    return CircleMission(
        mission_type=MissionType.DOCK_CIRCLE,
        circle_center_x_m=common["dock_x_m"],
        circle_center_y_m=common["dock_y_m"],
        **common,
        **_circle_fields(circle),
    )


_MISSION_LOADERS: Dict[
    MissionType, Callable[[Dict[str, Any]], CircleMission]
] = {
    MissionType.CIRCLE: _load_circle,
    MissionType.DOCK_CIRCLE: _load_dock_circle,
}


def load_mission(path: str) -> CircleMission:
    """Load a typed mission from *path* using the registered parser."""
    mission_path = Path(path).expanduser()
    if not mission_path.is_file():
        raise ValueError(f"mission file does not exist: {mission_path}")

    with mission_path.open("r", encoding="utf-8") as mission_file:
        document = yaml.safe_load(mission_file)

    if not isinstance(document, dict):
        raise ValueError("mission file must contain a YAML mapping")
    mission = _mapping(document, "mission", path="")
    type_value = mission.get("type")
    if not isinstance(type_value, str):
        raise ValueError("mission.type must be a string")
    try:
        mission_type = MissionType(type_value)
    except ValueError as exception:
        accepted = ", ".join(value.value for value in MissionType)
        raise ValueError(
            f"mission.type must be one of: {accepted}"
        ) from exception

    result = _MISSION_LOADERS[mission_type](mission)
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
    if result.repeat_count > 1 and not result.precision_landing_enabled:
        raise ValueError(
            "mission.repeat.count greater than one requires precision landing"
        )
    return result


def load_circle_mission(path: str) -> CircleMission:
    """Backward-compatible alias for callers of the original loader."""
    return load_mission(path)

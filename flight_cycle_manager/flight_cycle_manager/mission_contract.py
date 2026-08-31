"""Pure validation of the onboard docking mission contract.

This module deliberately contains only numeric MAVLink values and standard
library functionality.  The transport adapter is responsible for converting
wire messages into :class:`MissionItem` instances.
"""

from dataclasses import dataclass
import math
from numbers import Integral, Real
from typing import Sequence


MAV_CMD_DO_LAND_START = 189
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_LAND = 21

MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_FRAME_GLOBAL_INT = 5
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

_SUPPORTED_GLOBAL_FRAMES = frozenset(
    (
        MAV_FRAME_GLOBAL,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_FRAME_GLOBAL_INT,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    )
)
_E7 = 10_000_000.0
_EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class MissionItem:
    """A transport-independent representation of one mission item."""

    seq: int
    frame: int
    command: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float


@dataclass(frozen=True)
class MissionValidationConfig:
    """Thresholds used when validating a docking mission."""

    min_handoff_loiter_s: float
    landing_location_tolerance_m: float


@dataclass(frozen=True)
class MissionContract:
    """Immutable sequence references extracted from a validated mission."""

    item_count: int
    land_start_seq: int
    handoff_seq: int
    fallback_land_seq: int


class MissionContractError(ValueError):
    """Raised when an onboard mission cannot satisfy the docking contract."""


def validate_docking_mission(
    items: Sequence[MissionItem],
    config: MissionValidationConfig,
) -> MissionContract:
    """Validate and extract the single docking contract from ``items``.

    Mission items may arrive out of order, but their sequence numbers must
    describe exactly one contiguous mission from zero through ``N - 1``.
    """

    try:
        ordered = tuple(items)
    except TypeError as exc:
        raise MissionContractError("mission items must be a sequence") from exc

    _validate_config(config)
    for value in ordered:
        _validate_item_fields(value)

    ordered = tuple(sorted(ordered, key=lambda value: value.seq))
    if [value.seq for value in ordered] != list(range(len(ordered))):
        raise MissionContractError("mission sequence numbers must be contiguous")

    markers = [
        value for value in ordered if value.command == MAV_CMD_DO_LAND_START
    ]
    if len(markers) != 1:
        raise MissionContractError("mission must contain exactly one DO_LAND_START")
    marker = markers[0]

    loiters = [
        value
        for value in ordered[marker.seq + 1 :]
        if value.command == MAV_CMD_NAV_LOITER_TIME
    ]
    if len(loiters) != 1:
        raise MissionContractError(
            "landing sequence must contain exactly one NAV_LOITER_TIME"
        )
    handoff = loiters[0]
    if handoff.param1 < config.min_handoff_loiter_s:
        raise MissionContractError(
            "NAV_LOITER_TIME is shorter than the minimum handoff"
        )

    fallback_index = handoff.seq + 1
    if (
        fallback_index >= len(ordered)
        or ordered[fallback_index].command != MAV_CMD_NAV_LAND
    ):
        raise MissionContractError("NAV_LAND must immediately follow NAV_LOITER_TIME")
    fallback = ordered[fallback_index]
    _validate_landing_location(
        handoff,
        fallback,
        config.landing_location_tolerance_m,
    )
    return MissionContract(
        item_count=len(ordered),
        land_start_seq=marker.seq,
        handoff_seq=handoff.seq,
        fallback_land_seq=fallback.seq,
    )


def _validate_config(config: MissionValidationConfig) -> None:
    if not isinstance(config, MissionValidationConfig):
        raise MissionContractError("validation config is invalid")
    for name in ("min_handoff_loiter_s", "landing_location_tolerance_m"):
        value = getattr(config, name)
        if not _is_finite_number(value):
            raise MissionContractError("validation config values must be finite")
        if value < 0:
            raise MissionContractError("validation config values must be non-negative")


def _validate_item_fields(value: MissionItem) -> None:
    if not isinstance(value, MissionItem):
        raise MissionContractError("mission contains an invalid item")
    if not isinstance(value.seq, Integral) or isinstance(value.seq, bool) or value.seq < 0:
        raise MissionContractError("mission sequence numbers must be non-negative integers")
    if not isinstance(value.frame, Integral) or isinstance(value.frame, bool):
        raise MissionContractError("mission coordinate frame is invalid")
    if not isinstance(value.command, Integral) or isinstance(value.command, bool):
        raise MissionContractError("mission command is invalid")
    if not isinstance(value.x, Integral) or isinstance(value.x, bool):
        raise MissionContractError("mission x coordinate must be an integer E7 value")
    if not isinstance(value.y, Integral) or isinstance(value.y, bool):
        raise MissionContractError("mission y coordinate must be an integer E7 value")
    for field in ("param1", "param2", "param3", "param4", "x", "y", "z"):
        if not _is_finite_number(getattr(value, field)):
            raise MissionContractError("mission fields must be finite")


def _is_finite_number(value: object) -> bool:
    if not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def _validate_landing_location(
    handoff: MissionItem,
    fallback: MissionItem,
    tolerance_m: float,
) -> None:
    if (
        handoff.frame not in _SUPPORTED_GLOBAL_FRAMES
        or fallback.frame not in _SUPPORTED_GLOBAL_FRAMES
    ):
        raise MissionContractError(
            "handoff and fallback must use a supported global coordinate frame"
        )

    handoff_lat, handoff_lon = _coordinates(handoff, allow_current=False)
    fallback_pair = (fallback.x, fallback.y)
    if fallback_pair == (0, 0):
        return
    fallback_lat, fallback_lon = _coordinates(fallback, allow_current=False)
    distance_m = _haversine_m(
        handoff_lat,
        handoff_lon,
        fallback_lat,
        fallback_lon,
    )
    if distance_m > tolerance_m:
        raise MissionContractError(
            "NAV_LAND location is beyond the configured landing location tolerance"
        )


def _coordinates(item: MissionItem, *, allow_current: bool) -> tuple[float, float]:
    x, y = item.x, item.y
    if (x, y) == (0, 0):
        if allow_current:
            return 0.0, 0.0
        raise MissionContractError("landing coordinates cannot use current location here")
    if x == 0 or y == 0:
        raise MissionContractError("landing coordinates must be both non-zero")
    latitude = float(x) / _E7
    longitude = float(y) / _E7
    if not -90.0 <= latitude <= 90.0:
        raise MissionContractError("landing latitude is out of range")
    if not -180.0 <= longitude <= 180.0:
        raise MissionContractError("landing longitude is out of range")
    return latitude, longitude


def _haversine_m(
    latitude_a: float,
    longitude_a: float,
    latitude_b: float,
    longitude_b: float,
) -> float:
    latitude_a, longitude_a, latitude_b, longitude_b = map(
        math.radians,
        (latitude_a, longitude_a, latitude_b, longitude_b),
    )
    delta_latitude = latitude_b - latitude_a
    delta_longitude = longitude_b - longitude_a
    haversine = (
        math.sin(delta_latitude / 2.0) ** 2
        + math.cos(latitude_a)
        * math.cos(latitude_b)
        * math.sin(delta_longitude / 2.0) ** 2
    )
    return 2.0 * _EARTH_RADIUS_M * math.asin(math.sqrt(min(1.0, haversine)))

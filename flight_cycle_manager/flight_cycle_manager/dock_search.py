"""Transport-independent bounded search for a dock target."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from numbers import Real
from typing import Sequence


@dataclass(frozen=True)
class SearchConfig:
    """Limits and gains for the shared dock-search controller."""

    radius_m: float
    tangential_speed_m_s: float
    radial_gain: float
    altitude_gain: float
    max_horizontal_speed_m_s: float
    max_vertical_speed_m_s: float
    target_stable_time_s: float
    timeout_s: float
    max_displacement_m: float

    def __post_init__(self) -> None:
        names = (
            "radius_m",
            "tangential_speed_m_s",
            "radial_gain",
            "altitude_gain",
            "max_horizontal_speed_m_s",
            "max_vertical_speed_m_s",
            "target_stable_time_s",
            "timeout_s",
            "max_displacement_m",
        )
        for name in names:
            value = getattr(self, name)
            if not _is_finite_number(value):
                raise ValueError("search configuration values must be finite")
            if value < 0.0:
                raise ValueError("search configuration values must be non-negative")
        if self.radius_m == 0.0:
            raise ValueError("search radius must be positive")
        if self.max_horizontal_speed_m_s == 0.0:
            raise ValueError("maximum horizontal speed must be positive")
        if self.max_vertical_speed_m_s == 0.0:
            raise ValueError("maximum vertical speed must be positive")
        if self.timeout_s == 0.0:
            raise ValueError("search timeout must be positive")
        if self.max_displacement_m == 0.0:
            raise ValueError("maximum displacement must be positive")


@dataclass(frozen=True)
class SearchObservation:
    """One manager-supplied, caller-clocked search observation."""

    now: float
    position_enu_m: tuple[float, float, float]
    target_visible: bool = False


class SearchOutcome(Enum):
    """Current state of a dock search update."""

    SEARCHING = auto()
    ACQUIRED = auto()
    TIMED_OUT = auto()
    OUT_OF_BOUNDS = auto()


@dataclass(frozen=True)
class SearchDecision:
    """Velocity decision returned to the manager."""

    outcome: SearchOutcome
    velocity_enu_m_s: tuple[float, float, float]
    reason: str


class DockSearchController:
    """Pure bounded-circle controller using only supplied observation times."""

    def __init__(self, config: SearchConfig):
        if not isinstance(config, SearchConfig):
            raise ValueError("search configuration is invalid")
        self._config = config
        self._started = False
        self._terminal_outcome: SearchOutcome | None = None
        self._center: tuple[float, float, float] | None = None
        self._started_at: float | None = None
        self._last_now: float | None = None
        self._target_visible_since: float | None = None

    def start(
        self,
        *,
        now: float,
        center_enu_m: tuple[float, float, float],
    ) -> None:
        """Capture the search center and reset all search state."""

        _validate_time(now)
        center = _validate_position(center_enu_m)
        self._started = True
        self._terminal_outcome = None
        self._center = center
        self._started_at = float(now)
        self._last_now = float(now)
        self._target_visible_since = None

    def update(self, observation: SearchObservation) -> SearchDecision:
        """Produce one bounded search velocity from a fresh observation."""

        if not self._started:
            raise RuntimeError("search controller must be started before update")
        _validate_observation(observation)
        now = float(observation.now)
        assert self._last_now is not None
        if now < self._last_now:
            raise ValueError("observation time must be monotonic")
        self._last_now = now

        if self._terminal_outcome is not None:
            return _decision(self._terminal_outcome, "search is terminal")

        assert self._center is not None
        assert self._started_at is not None
        if now - self._started_at >= self._config.timeout_s:
            self._terminal_outcome = SearchOutcome.TIMED_OUT
            return _decision(SearchOutcome.TIMED_OUT, "search timeout")

        east_from_center = observation.position_enu_m[0] - self._center[0]
        north_from_center = observation.position_enu_m[1] - self._center[1]
        displacement = math.hypot(east_from_center, north_from_center)
        if displacement > self._config.max_displacement_m:
            self._terminal_outcome = SearchOutcome.OUT_OF_BOUNDS
            return _decision(
                SearchOutcome.OUT_OF_BOUNDS,
                "search displacement bound exceeded",
            )

        if observation.target_visible:
            if self._target_visible_since is None:
                self._target_visible_since = now
            if now - self._target_visible_since >= self._config.target_stable_time_s:
                self._terminal_outcome = SearchOutcome.ACQUIRED
                return _decision(SearchOutcome.ACQUIRED, "target acquired")
        else:
            self._target_visible_since = None

        velocity = self._search_velocity(observation.position_enu_m, displacement)
        return SearchDecision(SearchOutcome.SEARCHING, velocity, "searching")

    def _search_velocity(
        self,
        position: tuple[float, float, float],
        radius: float,
    ) -> tuple[float, float, float]:
        assert self._center is not None
        radial_x = position[0] - self._center[0]
        radial_y = position[1] - self._center[1]
        if radius == 0.0:
            radial_x, radial_y = 1.0, 0.0
        else:
            radial_x /= radius
            radial_y /= radius

        radial_error = self._config.radius_m - radius
        radial_speed = self._config.radial_gain * radial_error
        if radius < self._config.radius_m:
            east = radial_speed * radial_x
            north = radial_speed * radial_y
        else:
            east = radial_speed * radial_x - self._config.tangential_speed_m_s * radial_y
            north = radial_speed * radial_y + self._config.tangential_speed_m_s * radial_x

        up = _clamp(
            self._config.altitude_gain * (self._center[2] - position[2]),
            -self._config.max_vertical_speed_m_s,
            self._config.max_vertical_speed_m_s,
        )
        east, north = _clamp_vector(
            east,
            north,
            self._config.max_horizontal_speed_m_s,
        )
        return east, north, up


def _decision(outcome: SearchOutcome, reason: str) -> SearchDecision:
    return SearchDecision(outcome, (0.0, 0.0, 0.0), reason)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(maximum, max(minimum, value))


def _clamp_vector(east: float, north: float, maximum: float) -> tuple[float, float]:
    magnitude = math.hypot(east, north)
    if magnitude <= maximum or magnitude == 0.0:
        return east, north
    scale = maximum / magnitude
    return east * scale, north * scale


def _validate_observation(observation: SearchObservation) -> None:
    if not isinstance(observation, SearchObservation):
        raise ValueError("search observation is invalid")
    _validate_time(observation.now)
    _validate_position(observation.position_enu_m)
    if not isinstance(observation.target_visible, bool):
        raise ValueError("target visibility must be boolean")


def _validate_position(position: Sequence[float]) -> tuple[float, float, float]:
    if isinstance(position, (str, bytes)):
        raise ValueError("ENU position must contain three finite values")
    try:
        values = tuple(position)
    except TypeError as exc:
        raise ValueError("ENU position must contain three finite values") from exc
    if len(values) != 3 or not all(_is_finite_number(value) for value in values):
        raise ValueError("ENU position must contain three finite values")
    return float(values[0]), float(values[1]), float(values[2])


def _validate_time(value: object) -> None:
    if not _is_finite_number(value):
        raise ValueError("caller time must be finite")


def _is_finite_number(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False

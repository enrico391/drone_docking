"""Pure quality gates used by the native precision-landing state machine."""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple


_EPSILON = 1.0e-9


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to the interval [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class FreshMeasurementWindow:
    """Track a continuous stream of newly transmitted measurements."""

    def __init__(
        self,
        *,
        required_duration_s: float,
        minimum_rate_hz: float,
        maximum_gap_s: float,
        maximum_age_s: float,
    ) -> None:
        if required_duration_s <= 0.0:
            raise ValueError("required_duration_s must be positive")
        if minimum_rate_hz <= 0.0:
            raise ValueError("minimum_rate_hz must be positive")
        if maximum_gap_s <= 0.0:
            raise ValueError("maximum_gap_s must be positive")
        if maximum_age_s <= 0.0:
            raise ValueError("maximum_age_s must be positive")
        self.required_duration_s = required_duration_s
        self.minimum_rate_hz = minimum_rate_hz
        self.maximum_gap_s = maximum_gap_s
        self.maximum_age_s = maximum_age_s
        self._samples: Deque[float] = deque()

    def reset(self) -> None:
        """Forget all evidence collected for the current stream."""
        self._samples.clear()

    def observe(self, sample_time_s: float) -> None:
        """Record one unique measurement that was successfully transmitted."""
        if not math.isfinite(sample_time_s):
            raise ValueError("sample_time_s must be finite")
        if self._samples:
            gap_s = sample_time_s - self._samples[-1]
            if gap_s <= 0.0 or gap_s > self.maximum_gap_s:
                self.reset()
        self._samples.append(sample_time_s)
        cutoff_s = sample_time_s - self.required_duration_s
        while len(self._samples) >= 2 and self._samples[1] <= cutoff_s:
            self._samples.popleft()

    def ready(self, *, now_s: float) -> bool:
        """Return whether duration, rate, gap, and age gates all pass."""
        if len(self._samples) < 2:
            return False
        if now_s - self._samples[-1] > self.maximum_age_s:
            return False
        duration_s = self._samples[-1] - self._samples[0]
        if duration_s + _EPSILON < self.required_duration_s:
            return False
        rate_hz = (len(self._samples) - 1) / duration_s
        return rate_hz + _EPSILON >= self.minimum_rate_hz


@dataclass(frozen=True)
class YawEstimate:
    """Circular mean and minimal peak-to-peak spread of a yaw window."""

    mean_rad: float
    jitter_rad: float


class CircularYawWindow:
    """Collect fresh yaw samples and calculate wrap-safe stability metrics."""

    def __init__(self, *, maximum_gap_s: float, maximum_age_s: float) -> None:
        if maximum_gap_s <= 0.0:
            raise ValueError("maximum_gap_s must be positive")
        if maximum_age_s <= 0.0:
            raise ValueError("maximum_age_s must be positive")
        self.maximum_gap_s = maximum_gap_s
        self.maximum_age_s = maximum_age_s
        self._samples: Deque[Tuple[float, float]] = deque()

    def reset(self) -> None:
        """Forget yaw samples collected before a source or filter change."""
        self._samples.clear()

    def observe(self, sample_time_s: float, yaw_rad: float) -> None:
        """Record one yaw measurement, resetting across discontinuous time."""
        if not math.isfinite(sample_time_s) or not math.isfinite(yaw_rad):
            raise ValueError("yaw sample values must be finite")
        if self._samples:
            gap_s = sample_time_s - self._samples[-1][0]
            if gap_s <= 0.0 or gap_s > self.maximum_gap_s:
                self.reset()
        self._samples.append((sample_time_s, wrap_pi(yaw_rad)))

    def estimate(
        self,
        *,
        now_s: float,
        required_duration_s: float,
        minimum_rate_hz: float,
    ) -> Optional[YawEstimate]:
        """Return an estimate only after a fresh, sufficiently dense window."""
        if len(self._samples) < 2:
            return None
        if now_s - self._samples[-1][0] > self.maximum_age_s:
            return None
        cutoff_s = now_s - required_duration_s
        while len(self._samples) >= 2 and self._samples[1][0] <= cutoff_s:
            self._samples.popleft()
        duration_s = self._samples[-1][0] - self._samples[0][0]
        if duration_s + _EPSILON < required_duration_s:
            return None
        rate_hz = (len(self._samples) - 1) / duration_s
        if rate_hz + _EPSILON < minimum_rate_hz:
            return None

        angles = [sample[1] for sample in self._samples]
        mean_rad = math.atan2(
            sum(math.sin(angle) for angle in angles),
            sum(math.cos(angle) for angle in angles),
        )
        normalized = sorted(angle % (2.0 * math.pi) for angle in angles)
        gaps = [
            normalized[index + 1] - normalized[index]
            for index in range(len(normalized) - 1)
        ]
        gaps.append(normalized[0] + 2.0 * math.pi - normalized[-1])
        jitter_rad = 2.0 * math.pi - max(gaps)
        return YawEstimate(mean_rad=mean_rad, jitter_rad=jitter_rad)


class StableCondition:
    """Require a predicate to remain continuously true for a duration."""

    def __init__(self, *, required_duration_s: float) -> None:
        if required_duration_s <= 0.0:
            raise ValueError("required_duration_s must be positive")
        self.required_duration_s = required_duration_s
        self._accepted_since_s: Optional[float] = None

    def reset(self) -> None:
        """Clear the current continuous-acceptance interval."""
        self._accepted_since_s = None

    def update(self, *, now_s: float, accepted: bool) -> bool:
        """Update the predicate and report whether its duration has elapsed."""
        if not accepted:
            self.reset()
            return False
        if self._accepted_since_s is None:
            self._accepted_since_s = now_s
        return (
            now_s - self._accepted_since_s + _EPSILON
            >= self.required_duration_s
        )


class DescendingCheckpoint:
    """Fire once after height remains at or below a descending threshold."""

    def __init__(
        self, *, threshold_m: float, confirmation_duration_s: float
    ) -> None:
        if not math.isfinite(threshold_m):
            raise ValueError("threshold_m must be finite")
        if confirmation_duration_s <= 0.0:
            raise ValueError("confirmation_duration_s must be positive")
        self.threshold_m = threshold_m
        self.confirmation_duration_s = confirmation_duration_s
        self._below_since_s: Optional[float] = None
        self._fired = False

    def reset(self) -> None:
        """Arm the checkpoint for a new landing attempt."""
        self._below_since_s = None
        self._fired = False

    def update(self, *, now_s: float, height_m: float, valid: bool) -> bool:
        """Return true once after a confirmed downward threshold crossing."""
        if self._fired:
            return False
        if (
            not valid
            or not math.isfinite(now_s)
            or not math.isfinite(height_m)
            or height_m > self.threshold_m
        ):
            self._below_since_s = None
            return False
        if self._below_since_s is None:
            self._below_since_s = now_s
            return False
        if (
            now_s - self._below_since_s + _EPSILON
            < self.confirmation_duration_s
        ):
            return False
        self._fired = True
        return True


def surface_relative_height(
    local_z_m: float, dock_surface_z_m: float
) -> float:
    """Convert FCU local height above home into height above the dock."""
    return local_z_m - dock_surface_z_m


def target_loss_requires_retry(
    *,
    height_m: Optional[float],
    clearance_height_m: float,
    policy: str,
) -> bool:
    """Select retry versus quiet continuation after visual target loss."""
    if policy not in ("continue", "retry"):
        raise ValueError("policy must be 'continue' or 'retry'")
    if policy == "retry":
        return True
    if height_m is None or not math.isfinite(height_m):
        return True
    return height_m >= clearance_height_m


def yaw_turn_can_finish_above(
    *,
    height_m: float,
    floor_height_m: float,
    correction_deg: float,
    yaw_rate_deg_s: float,
    descent_speed_m_s: float,
    clearance_margin_m: float,
) -> bool:
    """Check that a yaw slew can finish before the final yaw-lock region."""
    if yaw_rate_deg_s <= 0.0:
        return False
    turn_time_s = abs(correction_deg) / yaw_rate_deg_s
    predicted_height_m = height_m - max(descent_speed_m_s, 0.0) * turn_time_s
    return predicted_height_m >= floor_height_m + clearance_margin_m

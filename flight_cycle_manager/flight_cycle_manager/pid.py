"""Small, ROS-independent bounded PID controller."""

from dataclasses import dataclass, replace
import math


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    derivative_filter_time_constant_s: float
    min_dt_s: float = 1e-4
    max_dt_s: float = 0.5

    def __post_init__(self) -> None:
        fields = (
            "kp", "ki", "kd", "integral_limit",
            "output_limit", "derivative_filter_time_constant_s",
            "min_dt_s", "max_dt_s",
        )
        for field in fields:
            value = getattr(self, field)
            if not math.isfinite(value):
                raise ValueError(f"{field} must be finite")
        for field in ("kp", "ki", "kd", "integral_limit",
                      "derivative_filter_time_constant_s"):
            if getattr(self, field) < 0.0:
                raise ValueError(f"{field} must be nonnegative")
        for field in ("output_limit", "min_dt_s", "max_dt_s"):
            if getattr(self, field) <= 0.0:
                raise ValueError(f"{field} must be positive")
        if self.min_dt_s >= self.max_dt_s:
            raise ValueError("min_dt_s must be less than max_dt_s")


class PID:
    def __init__(self, config: PIDConfig):
        self.config = config
        self._target_config = config
        self._tuning_remaining_s = 0.0
        self.reset()

    def reset(self) -> None:
        self._previous_time = None
        self._previous_error = None
        self._filtered_derivative = 0.0
        self._integral = 0.0
        self._integral_before_update = 0.0
        self._last_error = 0.0
        self._unclamped_output = 0.0
        self._terms = (0.0, 0.0, 0.0)
        self._saturated = False

    def reconfigure(self, config: PIDConfig) -> None:
        """Ramp gains over 0.5 s of valid control updates; retain PID history."""
        if config == self._target_config:
            return
        self._target_config = config
        self._tuning_remaining_s = 0.5
        if self._previous_time is None:
            self.config = config
            self._tuning_remaining_s = 0.0

    def _advance_tuning(self, dt: float) -> None:
        if self._tuning_remaining_s <= 0.0:
            return
        fraction = min(1.0, dt / self._tuning_remaining_s)
        old = self.config
        gains = {
            name: getattr(old, name) + fraction * (
                getattr(self._target_config, name) - getattr(old, name)
            )
            for name in ("kp", "ki", "kd")
        }
        self.config = replace(self._target_config, **gains)
        # Changing Ki must not multiply an already accumulated correction.
        if old.ki > 0.0 and self.config.ki > 0.0:
            self._integral *= old.ki / self.config.ki
        else:
            self._integral = 0.0
        self._integral = max(-self.config.integral_limit,
                             min(self.config.integral_limit, self._integral))
        self._tuning_remaining_s = max(0.0, self._tuning_remaining_s - dt)

    def track_applied_output(self, output: float) -> None:
        """Undo integration into saturation, including downstream vector limits."""
        self._saturated = not math.isclose(output, self._unclamped_output,
                                           rel_tol=1e-9, abs_tol=1e-12)
        if self._saturated and (self._unclamped_output - output) * self._last_error > 0.0:
            self._integral = self._integral_before_update

    def diagnostics(self) -> dict:
        return dict(zip(("p", "i", "d"), self._terms),
                    kp=self.config.kp, ki=self.config.ki, kd=self.config.kd,
                    saturated=self._saturated)

    def update(self, error: float, now: float) -> float:
        if not math.isfinite(error):
            raise ValueError("error must be finite")
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        derivative = 0.0
        dt = None if self._previous_time is None else now - self._previous_time
        valid_dt = dt is not None and self.config.min_dt_s <= dt <= self.config.max_dt_s
        if valid_dt:
            self._advance_tuning(dt)
        self._integral_before_update = self._integral
        self._last_error = error
        if valid_dt:
            raw_derivative = (error - self._previous_error) / dt
            if self.config.derivative_filter_time_constant_s == 0.0:
                derivative = raw_derivative
                self._filtered_derivative = raw_derivative
            else:
                tau = self.config.derivative_filter_time_constant_s
                alpha = dt / (tau + dt)
                self._filtered_derivative += alpha * (raw_derivative - self._filtered_derivative)
                derivative = self._filtered_derivative

            candidate_integral = self._integral + error * dt
            candidate_integral = max(
                -self.config.integral_limit,
                min(self.config.integral_limit, candidate_integral),
            )
            unclamped_output = (
                self.config.kp * error
                + self.config.ki * self._integral
                + self.config.kd * derivative
            )
            saturated_same_direction = (
                unclamped_output > self.config.output_limit and error > 0.0
            ) or (
                unclamped_output < -self.config.output_limit and error < 0.0
            )
            if not saturated_same_direction and self.config.ki > 0.0:
                self._integral = candidate_integral

        else:
            # A discontinuous clock or stale sample must not create a D spike.
            self._filtered_derivative = 0.0
            if dt is None:
                # Retain the initial sample as the baseline for the next update.
                self._previous_time = now
                self._previous_error = error
            else:
                self._previous_time = None
                self._previous_error = None

        if valid_dt:
            self._previous_time = now
            self._previous_error = error
        self._terms = (
            self.config.kp * error,
            self.config.ki * self._integral,
            self.config.kd * derivative,
        )
        self._unclamped_output = sum(self._terms)
        output = max(-self.config.output_limit,
                     min(self.config.output_limit, self._unclamped_output))
        self.track_applied_output(output)
        return output

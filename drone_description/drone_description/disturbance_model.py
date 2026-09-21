"""Deterministic wrench generation independent of ROS and Gazebo."""

from dataclasses import dataclass
import math
import random
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Wrench:
    """Force and torque expressed in the Gazebo world frame."""

    force: Vector3 = (0.0, 0.0, 0.0)
    torque: Vector3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class DisturbanceSettings:
    """Parameters used by :class:`DisturbanceModel`."""

    frame: str = "world"
    constant_force: Vector3 = (0.0, 0.0, 0.0)
    constant_torque: Vector3 = (0.0, 0.0, 0.0)
    random_seed: int = 0
    turbulence_enabled: bool = False
    turbulence_force_stddev: Vector3 = (0.0, 0.0, 0.0)
    turbulence_torque_stddev: Vector3 = (0.0, 0.0, 0.0)
    turbulence_correlation_time: float = 0.3
    gust_enabled: bool = False
    gust_force: Vector3 = (0.0, 0.0, 0.0)
    gust_torque: Vector3 = (0.0, 0.0, 0.0)
    gust_duration: float = 0.0
    gust_interval: float = 1.0
    force_limits: Vector3 = (math.inf, math.inf, math.inf)
    torque_limits: Vector3 = (math.inf, math.inf, math.inf)
    ground_effect_enabled: bool = False
    ground_effect_min_height: float = 0.2
    ground_effect_max_height: float = 0.8
    ground_effect_max_force: float = 0.0
    ground_effect_turbulence_gain: float = 1.0


class DisturbanceModel:
    """Combine configured disturbance sources into one external wrench."""

    def __init__(self, settings: DisturbanceSettings) -> None:
        if settings.frame not in ("world", "body"):
            raise ValueError("disturbance frame must be 'world' or 'body'")
        if settings.gust_enabled and not (
            0.0 <= settings.gust_duration <= settings.gust_interval
            and settings.gust_interval > 0.0
        ):
            raise ValueError("gust duration must be within a positive interval")
        if any(limit < 0.0 for limit in settings.force_limits + settings.torque_limits):
            raise ValueError("wrench limits must be non-negative")
        if settings.ground_effect_turbulence_gain < 0.0:
            raise ValueError("ground-effect turbulence gain must be non-negative")
        self.settings = settings
        self._random = random.Random(settings.random_seed)
        self._turbulence_force: Vector3 = (0.0, 0.0, 0.0)
        self._turbulence_torque: Vector3 = (0.0, 0.0, 0.0)

    def step(
        self,
        *,
        dt: float,
        sim_time: float,
        rotor_height: Optional[float],
        orientation_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    ) -> Wrench:
        force = self.settings.constant_force
        torque = self.settings.constant_torque
        ground_gain = self._ground_gain(rotor_height)
        if self.settings.turbulence_enabled:
            if dt <= 0.0:
                raise ValueError("disturbance time step must be positive")
            tau = self.settings.turbulence_correlation_time
            if tau <= 0.0:
                raise ValueError("turbulence correlation time must be positive")
            alpha = math.exp(-dt / tau)
            innovation_scale = math.sqrt(1.0 - alpha * alpha)
            self._turbulence_force = self._advance_turbulence(
                self._turbulence_force,
                self.settings.turbulence_force_stddev,
                alpha,
                innovation_scale,
            )
            self._turbulence_torque = self._advance_turbulence(
                self._turbulence_torque,
                self.settings.turbulence_torque_stddev,
                alpha,
                innovation_scale,
            )
            multiplier = 1.0 + ground_gain * (
                self.settings.ground_effect_turbulence_gain - 1.0
            )
            force = _add(force, _scale(self._turbulence_force, multiplier))
            torque = _add(torque, _scale(self._turbulence_torque, multiplier))
        if (
            self.settings.gust_enabled
            and sim_time % self.settings.gust_interval < self.settings.gust_duration
        ):
            force = _add(force, self.settings.gust_force)
            torque = _add(torque, self.settings.gust_torque)
        if self.settings.frame == "body":
            force = rotate_vector(force, orientation_xyzw)
            torque = rotate_vector(torque, orientation_xyzw)
        force_z = self.settings.ground_effect_max_force * ground_gain
        force = (force[0], force[1], max(0.0, force[2] + force_z))
        return Wrench(
            force=_clamp_vector(force, self.settings.force_limits),
            torque=_clamp_vector(torque, self.settings.torque_limits),
        )

    def _ground_gain(self, rotor_height: Optional[float]) -> float:
        if not self.settings.ground_effect_enabled or rotor_height is None:
            return 0.0
        span = (
            self.settings.ground_effect_max_height
            - self.settings.ground_effect_min_height
        )
        if span <= 0.0:
            raise ValueError("ground-effect maximum height must exceed minimum height")
        linear_gain = max(
            0.0,
            min(
                1.0,
                (self.settings.ground_effect_max_height - rotor_height) / span,
            ),
        )
        return linear_gain * linear_gain

    def _advance_turbulence(
        self,
        state: Vector3,
        stddev: Vector3,
        alpha: float,
        innovation_scale: float,
    ) -> Vector3:
        return tuple(
            alpha * value
            + sigma * innovation_scale * self._random.gauss(0.0, 1.0)
            for value, sigma in zip(state, stddev)
        )  # type: ignore[return-value]


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right))  # type: ignore[return-value]


def _scale(vector: Vector3, multiplier: float) -> Vector3:
    return tuple(value * multiplier for value in vector)  # type: ignore[return-value]


def _clamp_vector(vector: Vector3, limits: Vector3) -> Vector3:
    return tuple(
        max(-limit, min(limit, value)) for value, limit in zip(vector, limits)
    )  # type: ignore[return-value]


def rotate_vector(
    vector: Vector3,
    quaternion_xyzw: Tuple[float, float, float, float],
) -> Vector3:
    """Rotate a vector by a normalized-or-normalizable quaternion."""
    x, y, z, w = quaternion_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("orientation quaternion must be non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    vx, vy, vz = vector
    # q * v * q^-1, expanded to avoid a quaternion dependency.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )

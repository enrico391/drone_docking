"""Temporal filtering and outlier rejection for dock poses."""

import math
from typing import Optional, Tuple

import numpy as np


class TemporalPoseFilter:
    """Smooth dock poses while rejecting implausible position changes."""

    def __init__(
        self,
        *,
        enabled: bool = True,
        time_constant_s: float = 0.12,
        max_translation_speed_m_s: float = 2.0,
        max_position_jump_m: float = 0.20,
        max_gate_dt_s: float = 0.20,
        reset_timeout_s: float = 1.0,
        reacquire_frames: int = 3,
    ) -> None:
        if time_constant_s < 0.0:
            raise ValueError("time_constant_s must not be negative")
        if max_translation_speed_m_s < 0.0:
            raise ValueError(
                "max_translation_speed_m_s must not be negative"
            )
        if max_position_jump_m < 0.0:
            raise ValueError("max_position_jump_m must not be negative")
        if max_gate_dt_s <= 0.0:
            raise ValueError("max_gate_dt_s must be greater than zero")
        if reset_timeout_s <= 0.0:
            raise ValueError("reset_timeout_s must be greater than zero")
        if reacquire_frames < 1:
            raise ValueError("reacquire_frames must be at least one")

        self.enabled = enabled
        self.time_constant_s = time_constant_s
        self.max_translation_speed_m_s = max_translation_speed_m_s
        self.max_position_jump_m = max_position_jump_m
        self.max_gate_dt_s = max_gate_dt_s
        self.reset_timeout_s = reset_timeout_s
        self.reacquire_frames = reacquire_frames
        self.reset()

    def reset(self) -> None:
        """Forget the current filtered pose and pending reacquisition."""
        self._filtered_pose: Optional[np.ndarray] = None
        self._measurement_pose: Optional[np.ndarray] = None
        self._measurement_stamp_s: Optional[float] = None
        self._last_input_stamp_s: Optional[float] = None
        self._pending_pose: Optional[np.ndarray] = None
        self._pending_stamp_s: Optional[float] = None
        self._pending_count = 0

    @staticmethod
    def _copy_pose(pose: np.ndarray) -> np.ndarray:
        value = np.asarray(pose, dtype=float)
        if value.shape != (4, 4) or not np.all(np.isfinite(value)):
            raise ValueError("pose must be a finite 4x4 transform")
        return value.copy()

    @staticmethod
    def _blend_pose(
        previous: np.ndarray, measurement: np.ndarray, alpha: float
    ) -> np.ndarray:
        output = np.eye(4, dtype=float)
        output[:3, 3] = (
            (1.0 - alpha) * previous[:3, 3]
            + alpha * measurement[:3, 3]
        )

        rotation_sum = (
            (1.0 - alpha) * previous[:3, :3]
            + alpha * measurement[:3, :3]
        )
        u, _, vt = np.linalg.svd(rotation_sum)
        rotation = u @ vt
        if np.linalg.det(rotation) < 0.0:
            u[:, -1] *= -1.0
            rotation = u @ vt
        output[:3, :3] = rotation
        return output

    def _accept_initial(
        self, pose: np.ndarray, stamp_s: float, reason: str
    ) -> Tuple[np.ndarray, str]:
        self._filtered_pose = pose.copy()
        self._measurement_pose = pose.copy()
        self._measurement_stamp_s = stamp_s
        self._last_input_stamp_s = stamp_s
        self._pending_pose = None
        self._pending_stamp_s = None
        self._pending_count = 0
        return pose.copy(), reason

    def update(
        self, pose: np.ndarray, stamp_s: float
    ) -> Tuple[Optional[np.ndarray], str]:
        """Filter one pose, returning no pose when an outlier is rejected."""
        measurement = self._copy_pose(pose)
        if not math.isfinite(stamp_s):
            raise ValueError("stamp_s must be finite")
        if not self.enabled:
            return measurement, "disabled"
        if self._filtered_pose is None:
            return self._accept_initial(measurement, stamp_s, "initialized")

        if (
            self._last_input_stamp_s is not None
            and stamp_s <= self._last_input_stamp_s
        ):
            return None, "non_monotonic_timestamp"

        input_gap_s = stamp_s - self._last_input_stamp_s
        self._last_input_stamp_s = stamp_s
        if input_gap_s > self.reset_timeout_s:
            return self._accept_initial(
                measurement, stamp_s, "reset_after_gap"
            )

        measurement_dt_s = stamp_s - self._measurement_stamp_s
        gate_dt_s = min(max(measurement_dt_s, 0.0), self.max_gate_dt_s)
        allowed_jump_m = (
            self.max_position_jump_m
            + self.max_translation_speed_m_s * gate_dt_s
        )
        innovation_m = float(
            np.linalg.norm(
                measurement[:3, 3] - self._measurement_pose[:3, 3]
            )
        )

        if innovation_m > allowed_jump_m:
            return self._handle_reacquisition(measurement, stamp_s)

        dt_s = stamp_s - self._measurement_stamp_s
        if self.time_constant_s <= 0.0:
            alpha = 1.0
        else:
            alpha = 1.0 - math.exp(-dt_s / self.time_constant_s)
        self._filtered_pose = self._blend_pose(
            self._filtered_pose, measurement, alpha
        )
        self._measurement_pose = measurement
        self._measurement_stamp_s = stamp_s
        self._pending_pose = None
        self._pending_stamp_s = None
        self._pending_count = 0
        return self._filtered_pose.copy(), "accepted"

    def _handle_reacquisition(
        self, measurement: np.ndarray, stamp_s: float
    ) -> Tuple[Optional[np.ndarray], str]:
        if self._pending_pose is None:
            self._pending_pose = measurement
            self._pending_stamp_s = stamp_s
            self._pending_count = 1
        else:
            pending_dt_s = min(
                max(stamp_s - self._pending_stamp_s, 0.0),
                self.max_gate_dt_s,
            )
            allowed_jump_m = (
                self.max_position_jump_m
                + self.max_translation_speed_m_s * pending_dt_s
            )
            pending_innovation_m = float(
                np.linalg.norm(
                    measurement[:3, 3] - self._pending_pose[:3, 3]
                )
            )
            if pending_innovation_m > allowed_jump_m:
                self._pending_pose = measurement
                self._pending_count = 1
            else:
                self._pending_pose = self._blend_pose(
                    self._pending_pose, measurement, 0.5
                )
                self._pending_count += 1
            self._pending_stamp_s = stamp_s

        if self._pending_count < self.reacquire_frames:
            return None, "outlier_rejected"

        reacquired_pose = self._pending_pose.copy()
        return self._accept_initial(
            reacquired_pose, stamp_s, "reacquired"
        )

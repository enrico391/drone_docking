"""Frame conversion helpers for visual precision landing."""

import math
from typing import Tuple


def camera_offset_to_body_velocity(
    camera_x_m: float,
    camera_y_m: float,
    gain: float,
    max_speed_m_s: float,
) -> Tuple[float, float]:
    """Convert a downward optical-frame dock offset to body FLU velocity.

    The simulated camera looks along body -Z. Its optical +X points toward
    body right (-Y), and optical +Y points toward body rear (-X).
    """
    velocity_x = -gain * camera_y_m
    velocity_y = -gain * camera_x_m
    speed = math.hypot(velocity_x, velocity_y)
    if speed > max_speed_m_s:
        scale = max_speed_m_s / speed
        velocity_x *= scale
        velocity_y *= scale
    return velocity_x, velocity_y

"""Frame and timestamp helpers for pymavlink precision landing."""

import math
from typing import Tuple


def measurement_time_usec(
    *, stamp_sec: int, stamp_nanosec: int, fallback_usec: int
) -> int:
    """Return a ROS acquisition timestamp or a fallback for an empty stamp."""
    timestamp_usec = int(stamp_sec) * 1_000_000 + int(stamp_nanosec) // 1000
    return timestamp_usec if timestamp_usec > 0 else int(fallback_usec)


def camera_target_to_body_frd(
    camera_x_m: float,
    camera_y_m: float,
    camera_z_m: float,
    camera_forward_m: float = 0.0,
    camera_right_m: float = 0.0,
    camera_down_m: float = 0.0,
) -> Tuple[float, float, float]:
    """Convert a downward optical-frame target into vehicle Body-FRD.

    Optical +X is vehicle right, optical +Y is vehicle rear, and optical +Z
    is vehicle down. Camera position offsets are expressed from the vehicle
    body origin to the camera in Body-FRD.
    """
    return (
        camera_forward_m - camera_y_m,
        camera_right_m + camera_x_m,
        camera_down_m + camera_z_m,
    )


def camera_dock_orientation_to_body_yaw_error(
    quaternion_x: float,
    quaternion_y: float,
    quaternion_z: float,
    quaternion_w: float,
) -> float:
    """Return dock-forward yaw relative to vehicle forward in radians.

    The quaternion describes the dock frame in the downward camera optical
    frame. Optical +X is vehicle right and optical +Y is vehicle rear, so the
    dock X axis can be projected directly into the vehicle's horizontal plane.
    Positive errors command a clockwise/right yaw in MAVLink's Body-FRD frame.
    """
    norm = math.sqrt(
        quaternion_x**2
        + quaternion_y**2
        + quaternion_z**2
        + quaternion_w**2
    )
    if norm <= 1.0e-9:
        raise ValueError("dock orientation quaternion has zero length")

    x = quaternion_x / norm
    y = quaternion_y / norm
    z = quaternion_z / norm
    w = quaternion_w / norm
    dock_forward_camera_x = 1.0 - 2.0 * (y * y + z * z)
    dock_forward_camera_y = 2.0 * (x * y + w * z)
    dock_forward_body_forward = -dock_forward_camera_y
    dock_forward_body_right = dock_forward_camera_x
    return math.atan2(
        dock_forward_body_right,
        dock_forward_body_forward,
    )

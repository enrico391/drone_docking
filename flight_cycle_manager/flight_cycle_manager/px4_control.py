"""PX4 DDS naming, frame conversions and shared-controller configuration."""

from dataclasses import fields, replace
import math

from .landing_backend import VelocityFrame
from .pid import PIDConfig
from .pid_landing import PidLandingConfig


PID_AXES = ("x", "y", "z", "yaw")
LIVE_PID_PARAMETERS = frozenset(
    [f"pid_{axis}_{field}" for axis in PID_AXES for field in (
        "kp", "ki", "kd", "integral_limit", "derivative_filter_time_constant_s",
    )]
    + [f"pid_gain_{side}_{axis}_{gain}" for side in ("near", "far")
       for axis in PID_AXES for gain in ("kp", "ki", "kd")]
    + ["pid_gain_near_distance_m", "pid_gain_far_distance_m"]
    + [f"pid_{axis}_k_v" for axis in ("x", "y", "z")]
)


def pose_tracking_telemetry(*, now, state, dock, diagnostics, parameters):
    """Keep measured camera pose distinct from active (possibly frozen) PID errors."""
    errors = diagnostics.get("control_error") or {}
    return dict(diagnostics, time_s=now, state=state,
                pose_valid=dock is not None,
                measured_pose=({"x": dock.forward_m, "y": dock.left_m,
                                "z": dock.distance_m, "yaw": dock.yaw_error_rad}
                               if dock is not None else None),
                dock_age_s=max(0.0, now - dock.received_at) if dock is not None else None,
                error={axis: errors.get(axis) for axis in PID_AXES},
                parameters={name: parameters[name] for name in sorted(LIVE_PID_PARAMETERS)})


def dds_topic(namespace, direction, name, message_type):
    """PX4 appends _vN only for nonzero message versions."""
    version = getattr(message_type, "MESSAGE_VERSION", 0)
    suffix = f"_v{version}" if version else ""
    prefix = namespace.strip("/")
    return f"/{prefix + '/' if prefix else ''}fmu/{direction}/{name}{suffix}"


def velocity_to_ned(command, heading):
    """Convert ENU or body FLU velocity and yaw rate into PX4 local NED."""
    x, y, z = command.velocity_x, command.velocity_y, command.velocity_z
    if command.frame is VelocityFrame.MAP_ENU:
        north, east = y, x
    elif command.frame is VelocityFrame.BODY_FLU:
        north = x * math.cos(heading) + y * math.sin(heading)
        east = x * math.sin(heading) - y * math.cos(heading)
    else:
        raise ValueError("unsupported velocity frame")
    result = (north, east, -z, -command.yaw_rate_rad_s)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("velocity and heading must be finite")
    return result


def pid_config(parameters):
    """Build the pure PID controller without importing the ArduPilot manager."""
    horizontal = max(parameters[f"pid_{name}"] for name in (
        "max_horizontal_speed_m_s", "descend_outer_max_horizontal_speed_m_s",
        "descend_central_max_horizontal_speed_m_s", "touchdown_max_horizontal_speed_m_s",
    ))
    limits = (horizontal, horizontal, parameters["search_max_vertical_speed_m_s"],
              parameters["pid_max_yaw_rate_rad_s"])
    axes = []
    for axis, limit in zip(("x", "y", "z", "yaw"), limits):
        axes.append(PIDConfig(
            **{name: parameters[f"pid_{axis}_{name}"] for name in (
                "kp", "ki", "kd", "integral_limit", "derivative_filter_time_constant_s",
            )}, output_limit=limit,
        ))
    values = {field.name: parameters[f"pid_{field.name}"]
              for field in fields(PidLandingConfig)
              if f"pid_{field.name}" in parameters}
    values.update(zip(("x_pid", "y_pid", "z_pid", "yaw_pid"), axes))
    for side in ("near", "far"):
        tuned = []
        for axis, base in zip(("x", "y", "z", "yaw"), axes):
            gains = {}
            for gain in ("kp", "ki", "kd"):
                value = parameters[f"pid_gain_{side}_{axis}_{gain}"]
                gains[gain] = getattr(base, gain) if value == -1.0 else value
            tuned.append(replace(base, **gains))
        values[f"{side}_pids"] = tuple(tuned)
    return PidLandingConfig(**values)

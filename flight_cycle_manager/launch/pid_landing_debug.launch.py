"""Compatibility entry point for LAND-triggered PID debug; use flight_cycle.launch.py."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("flight_cycle_manager"))
    localizer = Path(get_package_share_directory("dock_apriltag_localizer"))
    venv_python = share.parents[3] / "apriltag_venv" / "bin" / "python"
    forwarded = {
        "publish_camera": "false",
        "mavlink_connection": "tcp:127.0.0.1:5760",
        "mavlink_baud": "921600",
        "landing_attempt_log_enabled": "true",
        "touchdown_trace_enabled": "false",
        "landing_attempt_log_directory": "~/.ros/flight_cycle_manager/landing_attempts",
        "start_pid_tuner": "false",
        "pid_hold_at_state": "none",
        "pid_fixed_central_reference_mode": "off",
        "config_file": str(localizer / "config" / "dock_tags.yaml"),
        "image_topic": "/camera/image_raw",
        "camera_info_topic": "/camera/camera_info",
        "dock_frame": "dock_target",
        "localizer_python": str(venv_python) if venv_python.is_file() else "/usr/bin/python3",
        "pose_filter_enabled": "true",
        "pose_filter_time_constant_s": "0.12",
        "pose_filter_max_translation_speed_m_s": "2.0",
        "pose_filter_max_position_jump_m": "0.20",
        "pose_filter_reacquire_frames": "3",
        "camera": "0",
        "camera_width": "800",
        "camera_height": "600",
        "camera_format": "RGB888",
        "camera_info_url": "file://" + str(localizer / "config" / "ov5647_800x600.yaml"),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default)
          for name, default in forwarded.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "flight_cycle.launch.py")),
            launch_arguments={
                "operation_mode": "pid_landing_debug",
                **{name: LaunchConfiguration(name) for name in forwarded},
            }.items(),
        ),
    ])
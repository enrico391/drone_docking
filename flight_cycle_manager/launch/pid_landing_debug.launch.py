"""Launch repeatable LAND-triggered PID docking without an onboard mission."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_localizer_python(workspace_root: Path) -> str:
    workspace_venv_python = workspace_root / "apriltag_venv" / "bin" / "python"
    if workspace_venv_python.is_file():
        return str(workspace_venv_python)
    return "/usr/bin/python3"


def _launch_setup(context: LaunchContext, package_share: Path):
    return [
        Node(
            package="flight_cycle_manager",
            executable="flight_cycle_manager",
            name="flight_cycle_manager",
            output="screen",
            parameters=[
                str(package_share / "config" / "flight_cycle_manager.yaml"),
                str(package_share / "config" / "pid_landing.yaml"),
                {
                    "operation_mode": "pid_landing_debug",
                    "landing_backend": "pid",
                    "landing_attempt_log_enabled": ParameterValue(
                        LaunchConfiguration("landing_attempt_log_enabled"),
                        value_type=bool,
                    ),
                    "landing_attempt_log_directory": LaunchConfiguration(
                        "landing_attempt_log_directory"
                    ),
                    "mavlink_connection": LaunchConfiguration("mavlink_connection"),
                    "mavlink_baud": ParameterValue(
                        LaunchConfiguration("mavlink_baud"), value_type=int
                    ),
                },
            ],
        )
    ]


def _localizer_arguments() -> dict[str, LaunchConfiguration]:
    return {
        "config_file": LaunchConfiguration("config_file"),
        "image_topic": LaunchConfiguration("image_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        "dock_frame": LaunchConfiguration("dock_frame"),
        "python_executable": LaunchConfiguration("localizer_python"),
        "pose_filter_enabled": LaunchConfiguration("pose_filter_enabled"),
        "pose_filter_time_constant_s": LaunchConfiguration(
            "pose_filter_time_constant_s"
        ),
        "pose_filter_max_translation_speed_m_s": LaunchConfiguration(
            "pose_filter_max_translation_speed_m_s"
        ),
        "pose_filter_max_position_jump_m": LaunchConfiguration(
            "pose_filter_max_position_jump_m"
        ),
        "pose_filter_reacquire_frames": LaunchConfiguration(
            "pose_filter_reacquire_frames"
        ),
    }


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("flight_cycle_manager"))
    localizer_share = Path(get_package_share_directory("dock_apriltag_localizer"))
    workspace_root = package_share.parents[3]
    localizer_arguments = _localizer_arguments()

    external_camera_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(localizer_share / "launch" / "dock_localizer.launch.py")
        ),
        condition=UnlessCondition(LaunchConfiguration("publish_camera")),
        launch_arguments=localizer_arguments.items(),
    )
    onboard_camera_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                localizer_share
                / "launch"
                / "dock_localizer_with_camera.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("publish_camera")),
        launch_arguments={
            **localizer_arguments,
            "camera": LaunchConfiguration("camera"),
            "camera_width": LaunchConfiguration("camera_width"),
            "camera_height": LaunchConfiguration("camera_height"),
            "camera_format": LaunchConfiguration("camera_format"),
            "camera_info_url": LaunchConfiguration("camera_info_url"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_camera",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument("mavlink_connection", default_value="tcp:127.0.0.1:5760"),
            DeclareLaunchArgument("mavlink_baud", default_value="921600"),
            DeclareLaunchArgument(
                "landing_attempt_log_enabled",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "landing_attempt_log_directory",
                default_value="~/.ros/flight_cycle_manager/landing_attempts",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(localizer_share / "config" / "dock_tags.yaml"),
            ),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("dock_frame", default_value="dock_target"),
            DeclareLaunchArgument(
                "localizer_python",
                default_value=_default_localizer_python(workspace_root),
            ),
            DeclareLaunchArgument("pose_filter_enabled", default_value="true"),
            DeclareLaunchArgument("pose_filter_time_constant_s", default_value="0.12"),
            DeclareLaunchArgument("pose_filter_max_translation_speed_m_s", default_value="2.0"),
            DeclareLaunchArgument("pose_filter_max_position_jump_m", default_value="0.20"),
            DeclareLaunchArgument("pose_filter_reacquire_frames", default_value="3"),
            DeclareLaunchArgument("camera", default_value="0"),
            DeclareLaunchArgument("camera_width", default_value="800"),
            DeclareLaunchArgument("camera_height", default_value="600"),
            DeclareLaunchArgument("camera_format", default_value="RGB888"),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="file://"
                + str(localizer_share / "config" / "ov5647_800x600.yaml"),
            ),
            external_camera_localizer,
            onboard_camera_localizer,
            OpaqueFunction(function=_launch_setup, args=[package_share]),
        ]
    )

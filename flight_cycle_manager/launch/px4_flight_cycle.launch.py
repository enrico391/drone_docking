"""Launch the PX4 DDS manager with topic-only or Pi camera localization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("flight_cycle_manager"))
    localizer = Path(get_package_share_directory("dock_apriltag_localizer"))
    python = share.parents[3] / "apriltag_venv" / "bin" / "python"
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("px4_namespace", default_value=""),
        DeclareLaunchArgument("auto_start", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("pid_tuner", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("pid_tuner_port", default_value="8765"),
        DeclareLaunchArgument("pid_tuner_host", default_value="127.0.0.1",
                              description="Local-only by default; use SSH forwarding for remote tuning."),
        DeclareLaunchArgument("start_localizer", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "use_real_camera", default_value="false", choices=["true", "false"],
            description="Start the Pi camera_ros driver together with the dock localizer.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value=PythonExpression([
            "'false' if '", LaunchConfiguration("use_real_camera"), "' == 'true' else 'true'",
        ]), choices=["true", "false"]),
        DeclareLaunchArgument("config_file", default_value=str(share / "config/px4_flight_cycle.yaml")),
        DeclareLaunchArgument("pid_config", default_value=str(share / "config/pid_landing.yaml")),
        DeclareLaunchArgument("localizer_config", default_value=str(localizer / "config/dock_tags.yaml")),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("localizer_python", default_value=str(python) if python.is_file() else "/usr/bin/python3"),
        DeclareLaunchArgument("camera", default_value="0"),
        DeclareLaunchArgument("camera_width", default_value="800"),
        DeclareLaunchArgument("camera_height", default_value="600"),
        DeclareLaunchArgument("camera_format", default_value="RGB888"),
        DeclareLaunchArgument("camera_info_url", default_value=(
            localizer / "config/ov5647_800x600.yaml").as_uri()),
        Node(
            package="flight_cycle_manager", executable="px4_flight_cycle_manager",
            name="flight_cycle_manager", namespace=LaunchConfiguration("namespace"), output="screen",
            parameters=[LaunchConfiguration("config_file"), LaunchConfiguration("pid_config"), {
                "px4_namespace": LaunchConfiguration("px4_namespace"),
                "auto_start": ParameterValue(LaunchConfiguration("auto_start"), value_type=bool),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
        ),
        Node(
            package="flight_cycle_manager", executable="pid_tuner", name="pid_tuner",
            namespace=LaunchConfiguration("namespace"), output="screen",
            condition=IfCondition(LaunchConfiguration("pid_tuner")),
            parameters=[{
                "port": ParameterValue(LaunchConfiguration("pid_tuner_port"), value_type=int),
                "host": LaunchConfiguration("pid_tuner_host"),
            }],
        ),
        # Scope the include: its config_file is tag geometry, not manager settings.
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource([str(localizer / "launch") + "/", PythonExpression([
                "'dock_localizer_with_camera.launch.py' if '",
                LaunchConfiguration("use_real_camera"),
                "' == 'true' else 'dock_localizer.launch.py'",
            ])]),
            condition=IfCondition(LaunchConfiguration("start_localizer")),
            launch_arguments={
                "config_file": LaunchConfiguration("localizer_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "python_executable": LaunchConfiguration("localizer_python"),
                "camera": LaunchConfiguration("camera"),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
                "camera_format": LaunchConfiguration("camera_format"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
            }.items(),
        )]),
    ])

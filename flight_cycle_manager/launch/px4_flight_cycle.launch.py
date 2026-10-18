"""Launch the native PX4 DDS manager and optional simulated-camera localizer."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("flight_cycle_manager"))
    localizer = Path(get_package_share_directory("dock_apriltag_localizer"))
    python = share.parents[3] / "apriltag_venv" / "bin" / "python"
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("px4_namespace", default_value=""),
        DeclareLaunchArgument("auto_start", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("start_localizer", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("config_file", default_value=str(share / "config/px4_flight_cycle.yaml")),
        DeclareLaunchArgument("pid_config", default_value=str(share / "config/pid_landing.yaml")),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("localizer_python", default_value=str(python) if python.is_file() else "/usr/bin/python3"),
        Node(
            package="flight_cycle_manager", executable="px4_flight_cycle_manager",
            name="flight_cycle_manager", namespace=LaunchConfiguration("namespace"), output="screen",
            parameters=[LaunchConfiguration("config_file"), LaunchConfiguration("pid_config"), {
                "px4_namespace": LaunchConfiguration("px4_namespace"),
                "auto_start": ParameterValue(LaunchConfiguration("auto_start"), value_type=bool),
            }],
        ),
        GroupAction([SetParameter(name="use_sim_time", value=True), IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(localizer / "launch/dock_localizer.launch.py")),
            condition=IfCondition(LaunchConfiguration("start_localizer")),
            launch_arguments={
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "python_executable": LaunchConfiguration("localizer_python"),
            }.items(),
        )]),
    ])

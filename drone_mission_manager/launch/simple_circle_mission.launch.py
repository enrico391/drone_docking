"""Launch the takeoff-circle-return and precision-landing mission."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("drone_mission_manager"))
    default_mission = package_share / "config" / "missions" / "simple_circle.yaml"
    default_parameters = package_share / "config" / "mission_manager.yaml"
    localizer_share = Path(
        get_package_share_directory("dock_apriltag_localizer")
    )
    workspace_root = package_share.parents[3]
    workspace_venv_python = workspace_root / "apriltag_venv" / "bin" / "python"
    default_localizer_python = (
        str(workspace_venv_python)
        if workspace_venv_python.is_file()
        else "/usr/bin/python3"
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(localizer_share / "launch" / "dock_localizer.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_localizer")),
        launch_arguments={
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "python_executable": LaunchConfiguration("localizer_python"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_file", default_value=str(default_mission)
            ),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument(
                "landing_backend",
                default_value="ros",
                choices=["ros", "ardupilot"],
                description=(
                    "Use ROS velocity alignment/descent or ArduPilot native "
                    "LANDING_TARGET precision landing."
                ),
            ),
            DeclareLaunchArgument("start_localizer", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),
            DeclareLaunchArgument(
                "localizer_python", default_value=default_localizer_python
            ),
            localizer,
            Node(
                package="drone_mission_manager",
                executable="mission_manager",
                name="mission_manager",
                output="screen",
                parameters=[
                    str(default_parameters),
                    {
                        "mission_file": LaunchConfiguration("mission_file"),
                        "auto_start": LaunchConfiguration("auto_start"),
                        "landing_backend": LaunchConfiguration(
                            "landing_backend"
                        ),
                    },
                ],
            ),
        ]
    )

"""Select and launch a typed drone mission with precision landing."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def create_mission_manager(context: LaunchContext, package_share: Path):
    """Resolve a catalog mission name or an explicit mission file."""
    mission_file = LaunchConfiguration("mission_file").perform(context).strip()
    if not mission_file:
        mission_name = LaunchConfiguration("mission").perform(context)
        if not mission_name or Path(mission_name).name != mission_name:
            raise ValueError(
                "mission must be a profile filename without a path"
            )
        mission_file = str(
            package_share / "config" / "missions" / f"{mission_name}.yaml"
        )

    return [
        Node(
            package="drone_mission_manager",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
            parameters=[
                str(package_share / "config" / "mission_manager.yaml"),
                {
                    "mission_file": mission_file,
                    "auto_start": LaunchConfiguration("auto_start"),
                    "landing_backend": LaunchConfiguration("landing_backend"),
                    "landing_yaw_correction_enabled": ParameterValue(
                        LaunchConfiguration(
                            "landing_yaw_correction_enabled"
                        ),
                        value_type=bool,
                    ),
                },
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("drone_mission_manager"))
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
                "mission",
                default_value="simple_circle",
                description="Checked-in mission profile to execute.",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value="",
                description=(
                    "Optional YAML path overriding the mission profile."
                ),
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
            DeclareLaunchArgument(
                "landing_yaw_correction_enabled",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Send one dock-relative yaw correction after entering "
                    "ArduPilot LAND mode."
                ),
            ),
            DeclareLaunchArgument("start_localizer", default_value="true"),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),
            DeclareLaunchArgument(
                "localizer_python", default_value=default_localizer_python
            ),
            localizer,
            OpaqueFunction(
                function=create_mission_manager, args=[package_share]
            ),
        ]
    )

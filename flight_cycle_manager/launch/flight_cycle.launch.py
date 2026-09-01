"""Launch the flight-cycle manager with optional AprilTag localizer."""

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


def _default_localizer_python(workspace_root: Path) -> str:
    workspace_venv_python = workspace_root / "apriltag_venv" / "bin" / "python"
    if workspace_venv_python.is_file():
        return str(workspace_venv_python)
    return "/usr/bin/python3"


def _launch_bool(context: LaunchContext, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() == "true"


def create_manager_actions(
    context: LaunchContext, package_share: Path
):
    """Create the backend-aware manager node after validating launch inputs."""
    landing_backend = LaunchConfiguration("landing_backend").perform(context)
    landing_backend = landing_backend.strip().lower()
    if landing_backend not in {"pid", "precise_landing"}:
        raise ValueError("landing_backend must be 'pid' or 'precise_landing'")

    simulation_mode = _launch_bool(context, "simulation_mode")
    auto_start = _launch_bool(context, "auto_start")
    if auto_start and not simulation_mode:
        raise ValueError("auto_start=true requires simulation_mode=true")

    parameters = [str(package_share / "config" / "flight_cycle_manager.yaml")]
    if landing_backend == "pid":
        parameters.append(str(package_share / "config" / "pid_landing.yaml"))
    parameters.append(
        {
            "landing_backend": LaunchConfiguration("landing_backend"),
            "simulation_mode": ParameterValue(
                LaunchConfiguration("simulation_mode"),
                value_type=bool,
            ),
            "auto_start": ParameterValue(
                LaunchConfiguration("auto_start"),
                value_type=bool,
            ),
            "mavlink_connection": LaunchConfiguration("mavlink_connection"),
            "mavlink_baud": ParameterValue(
                LaunchConfiguration("mavlink_baud"),
                value_type=int,
            ),
        }
    )

    return [
        Node(
            package="flight_cycle_manager",
            executable="flight_cycle_manager",
            name="flight_cycle_manager",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("flight_cycle_manager"))
    localizer_share = Path(get_package_share_directory("dock_apriltag_localizer"))
    workspace_root = package_share.parents[3]

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(localizer_share / "launch" / "dock_localizer.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_localizer")),
        launch_arguments={
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
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
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "landing_backend",
                default_value="pid",
                choices=["pid", "precise_landing"],
                description="Select the docking backend.",
            ),
            DeclareLaunchArgument(
                "mavlink_connection",
                default_value="tcp:127.0.0.1:5760",
                description="Direct ArduPilot TCP or serial pymavlink endpoint.",
            ),
            DeclareLaunchArgument(
                "mavlink_baud",
                default_value="921600",
                description="Baud rate used when mavlink_connection is serial.",
            ),
            DeclareLaunchArgument(
                "simulation_mode",
                default_value="false",
                choices=["true", "false"],
                description="Enable simulation-only startup shortcuts.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                choices=["true", "false"],
                description="Queue one cycle automatically after startup validation.",
            ),
            DeclareLaunchArgument(
                "start_localizer",
                default_value="true",
                choices=["true", "false"],
                description="Start dock_apriltag_localizer with the manager.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera_info",
            ),
            DeclareLaunchArgument(
                "localizer_python",
                default_value=_default_localizer_python(workspace_root),
            ),
            DeclareLaunchArgument(
                "pose_filter_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "pose_filter_time_constant_s",
                default_value="0.12",
            ),
            DeclareLaunchArgument(
                "pose_filter_max_translation_speed_m_s",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "pose_filter_max_position_jump_m",
                default_value="0.20",
            ),
            DeclareLaunchArgument(
                "pose_filter_reacquire_frames",
                default_value="3",
            ),
            localizer,
            OpaqueFunction(function=create_manager_actions, args=[package_share]),
        ]
    )

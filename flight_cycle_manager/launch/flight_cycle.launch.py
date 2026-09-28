"""Launch the flight-cycle manager with optional AprilTag localizer."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_localizer_python(workspace_root: Path) -> str:
    workspace_venv_python = workspace_root / "apriltag_venv" / "bin" / "python"
    if workspace_venv_python.is_file():
        return str(workspace_venv_python)
    return "/usr/bin/python3"


def _launch_bool(context: LaunchContext, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() == "true"


def _localizer_arguments():
    arguments = {
        "image_topic": LaunchConfiguration("image_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        "python_executable": LaunchConfiguration("localizer_python"),
        "pose_filter_enabled": LaunchConfiguration("pose_filter_enabled"),
        "pose_filter_time_constant_s": LaunchConfiguration("pose_filter_time_constant_s"),
        "pose_filter_max_translation_speed_m_s": LaunchConfiguration("pose_filter_max_translation_speed_m_s"),
        "pose_filter_max_position_jump_m": LaunchConfiguration("pose_filter_max_position_jump_m"),
        "pose_filter_reacquire_frames": LaunchConfiguration("pose_filter_reacquire_frames"),
        "config_file": LaunchConfiguration("config_file"),
        "dock_frame": LaunchConfiguration("dock_frame"),
    }
    return arguments


def _launch_setup(
    context: LaunchContext, package_share: Path
):
    """Create the backend-aware manager node after validating launch inputs."""
    landing_backend = LaunchConfiguration("landing_backend").perform(context)
    landing_backend = landing_backend.strip().lower()
    if landing_backend not in {"pid", "precise_landing"}:
        raise ValueError("landing_backend must be 'pid' or 'precise_landing'")

    operation_mode = LaunchConfiguration("operation_mode").perform(context).strip().lower()
    if operation_mode not in {"flight_cycle", "pid_landing_debug", "pid_align_debug"}:
        raise ValueError("invalid operation_mode")
    hold = LaunchConfiguration("pid_hold_at_state").perform(context).strip().upper()
    if hold not in {"NONE", "PID_ALIGN", "DESCEND_OUTER", "ACQUIRE_CENTRAL", "DESCEND_CENTRAL", "HOLD_CENTRAL"}:
        raise ValueError("invalid pid_hold_at_state")
    if operation_mode != "flight_cycle" and landing_backend != "pid":
        raise ValueError("PID debug modes require landing_backend=pid")
    if hold != "NONE" and landing_backend != "pid":
        raise ValueError("pid_hold_at_state requires landing_backend=pid")
    if operation_mode == "pid_align_debug" and hold not in {"NONE", "PID_ALIGN"}:
        raise ValueError("pid_align_debug can only hold at PID_ALIGN")

    simulation_mode = _launch_bool(context, "simulation_mode")
    auto_start = _launch_bool(context, "auto_start")
    if auto_start and not simulation_mode:
        raise ValueError("auto_start=true requires simulation_mode=true")
    if auto_start and operation_mode != "flight_cycle":
        raise ValueError("auto_start is only available in flight_cycle mode")

    parameters = [str(package_share / "config" / "flight_cycle_manager.yaml")]
    if landing_backend == "pid":
        parameters.append(str(package_share / "config" / "pid_landing.yaml"))
    parameters.append(
        {
            "landing_backend": LaunchConfiguration("landing_backend"),
            "operation_mode": LaunchConfiguration("operation_mode"),
            "pid_hold_at_state": LaunchConfiguration("pid_hold_at_state"),
            "pid_align_auto_start": ParameterValue(
                LaunchConfiguration("pid_align_auto_start"), value_type=bool,
            ),
            "landing_attempt_log_enabled": ParameterValue(
                LaunchConfiguration("landing_attempt_log_enabled"), value_type=bool,
            ),
            "landing_attempt_log_directory": LaunchConfiguration("landing_attempt_log_directory"),
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

    nodes = [
        Node(
            package="flight_cycle_manager",
            executable="flight_cycle_manager",
            name="flight_cycle_manager",
            output="screen",
            parameters=parameters,
        ),
    ]
    if _launch_bool(context, "start_pid_tuner"):
        nodes.append(Node(
            package="flight_cycle_manager", executable="pid_tuner",
            name="pid_tuner", output="screen",
        ))
    return nodes


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("flight_cycle_manager"))
    localizer_share = Path(get_package_share_directory("dock_apriltag_localizer"))
    workspace_root = package_share.parents[3]
    arguments = _localizer_arguments()
    external = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(localizer_share / "launch" / "dock_localizer.launch.py")),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("start_localizer"), "' == 'true' and '",
            LaunchConfiguration("publish_camera"), "' == 'false'",
        ])),
        launch_arguments=arguments.items(),
    )
    onboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(localizer_share / "launch" / "dock_localizer_with_camera.launch.py")),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("start_localizer"), "' == 'true' and '",
            LaunchConfiguration("publish_camera"), "' == 'true'",
        ])),
        launch_arguments={**arguments, **{name: LaunchConfiguration(name) for name in (
            "camera", "camera_width", "camera_height", "camera_format", "camera_info_url",
        )}}.items(),
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
                "operation_mode", default_value="flight_cycle",
                choices=["flight_cycle", "pid_landing_debug", "pid_align_debug"],
            ),
            DeclareLaunchArgument(
                "pid_hold_at_state", default_value="none",
                choices=["none", "PID_ALIGN", "DESCEND_OUTER", "ACQUIRE_CENTRAL", "DESCEND_CENTRAL", "HOLD_CENTRAL"],
                description="Freeze PID forward progress when entering this state; none runs the full landing.",
            ),
            DeclareLaunchArgument("pid_align_auto_start", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("landing_attempt_log_enabled", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("landing_attempt_log_directory", default_value="~/.ros/flight_cycle_manager/landing_attempts"),
            DeclareLaunchArgument(
                "start_pid_tuner",
                default_value="false",
                choices=["true", "false"],
                description="Start the localhost PID gain dashboard on port 8765.",
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
            DeclareLaunchArgument("publish_camera", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("config_file", default_value=str(localizer_share / "config" / "dock_tags.yaml")),
            DeclareLaunchArgument("dock_frame", default_value="dock_target"),
            DeclareLaunchArgument("camera", default_value="0"),
            DeclareLaunchArgument("camera_width", default_value="800"),
            DeclareLaunchArgument("camera_height", default_value="600"),
            DeclareLaunchArgument("camera_format", default_value="RGB888"),
            DeclareLaunchArgument("camera_info_url", default_value="file://" + str(localizer_share / "config" / "ov5647_800x600.yaml")),
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
            external,
            onboard,
            OpaqueFunction(function=_launch_setup, args=[package_share]),
        ]
    )

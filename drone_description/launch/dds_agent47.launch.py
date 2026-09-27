"""Launch the DDS agent with the transport proven for ArduPilot 4.7."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    executable = (
        Path(get_package_prefix("micro_ros_agent"))
        / "lib/micro_ros_agent/micro_ros_agent"
    )
    profile = (
        Path(get_package_share_directory("drone_description"))
        / "config/dds_udp.xml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("micro_ros_agent_ns", default_value=""),
        DeclareLaunchArgument("transport", default_value="udp4"),
        DeclareLaunchArgument("port", default_value="2019"),
        DeclareLaunchArgument("verbosity", default_value="4"),
        ExecuteProcess(
            cmd=[str(executable), LaunchConfiguration("transport"), "-p",
                 LaunchConfiguration("port"), "-v",
                 LaunchConfiguration("verbosity")],
            additional_env={"FASTRTPS_DEFAULT_PROFILES_FILE": str(profile)},
            output="both",
        ),
    ])

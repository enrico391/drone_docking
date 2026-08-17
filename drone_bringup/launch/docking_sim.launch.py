"""Launch the currently implemented autonomous docking simulation phases."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("drone_description")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(description_share)
                / "launch"
                / "gazebo_docking.launch.py"
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "robot_name": LaunchConfiguration("robot_name"),
            "sim_address": LaunchConfiguration("sim_address"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "use_dds_agent": LaunchConfiguration("use_dds_agent"),
            "use_gz_tf": LaunchConfiguration("use_gz_tf"),
            "use_gz_sim_server": LaunchConfiguration("use_gz_sim_server"),
            "use_gz_sim_gui": LaunchConfiguration("use_gz_sim_gui"),
            "spawn_robot": LaunchConfiguration("spawn_robot"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("robot_name", default_value="x500"),
            DeclareLaunchArgument("sim_address", default_value="127.0.0.1"),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="-0.013",
                description="X500 model offset that places its landing feet on the ground.",
            ),
            DeclareLaunchArgument("use_dds_agent", default_value="true"),
            DeclareLaunchArgument("use_gz_tf", default_value="true"),
            DeclareLaunchArgument("use_gz_sim_server", default_value="true"),
            DeclareLaunchArgument("use_gz_sim_gui", default_value="true"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            simulation,
        ]
    )

"""Launch the docking world with the ArduPilot X500 stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("drone_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = Path(description_share) / "worlds" / "docking_world.sdf"

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ros_gz_sim_share) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                "-v4 -s -r --physics-engine ",
                LaunchConfiguration("physics_engine"),
                f" {world_path}",
            ]
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_server")),
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ros_gz_sim_share) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_gui")),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        description_share,
                        "launch",
                        "x500.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "world_name": "docking_world",
            "robot_name": LaunchConfiguration("robot_name"),
            "sim_address": LaunchConfiguration("sim_address"),
            "use_dds_agent": LaunchConfiguration("use_dds_agent"),
            "use_gz_tf": LaunchConfiguration("use_gz_tf"),
            "x": "0.0",
            "y": "0.0",
            "z": LaunchConfiguration("spawn_z"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("robot_name", default_value="x500"),
            DeclareLaunchArgument("sim_address", default_value="127.0.0.1"),
            DeclareLaunchArgument(
                "physics_engine",
                default_value="gz-physics-bullet-featherstone-plugin",
                description="Gazebo physics backend; Bullet is required for mesh collisions.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="-0.032",
                description="X500 model offset that places its triangular base on the ground.",
            ),
            DeclareLaunchArgument("use_dds_agent", default_value="true"),
            DeclareLaunchArgument("use_gz_tf", default_value="true"),
            DeclareLaunchArgument("use_gz_sim_server", default_value="true"),
            DeclareLaunchArgument("use_gz_sim_gui", default_value="true"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            gz_server,
            gz_gui,
            robot,
        ]
    )

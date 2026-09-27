"""Launch the docking world with the ArduPilot X500 stack."""

from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def create_robot_actions(
    context: LaunchContext,
) -> List[IncludeLaunchDescription]:
    """Spawn the vehicle at a named or explicitly configured initial point."""
    initial_point = LaunchConfiguration("initial_point").perform(context)
    named_points = {
        # Model z=-0.032 places the triangular base on the ground.
        "ground": ("0.0", "0.0", "-0.032"),
        # The dock recess floor is z=0.880, so add it to the model offset.
        "dock": ("0.0", "0.0", "0.848"),
    }
    if initial_point == "custom":
        x = LaunchConfiguration("spawn_x").perform(context)
        y = LaunchConfiguration("spawn_y").perform(context)
        z = LaunchConfiguration("spawn_z").perform(context)
    else:
        x, y, z = named_points[initial_point]

    description_share = get_package_share_directory("drone_description")
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [description_share, "launch", "x500.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "world_name": "docking_world",
            "robot_name": LaunchConfiguration("robot_name"),
            "sim_address": LaunchConfiguration("sim_address"),
            "use_dds_agent": LaunchConfiguration("use_dds_agent"),
            "use_mavproxy": LaunchConfiguration("use_mavproxy"),
            "use_gz_tf": LaunchConfiguration("use_gz_tf"),
            "x": x,
            "y": y,
            "z": z,
            "yaw": LaunchConfiguration("spawn_yaw"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )
    return [robot]


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
                "-v4 -s -r --headless-rendering --physics-engine ",
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("robot_name", default_value="x500"),
            DeclareLaunchArgument("sim_address", default_value="127.0.0.1"),
            DeclareLaunchArgument(
                "initial_point",
                default_value="ground",
                choices=["ground", "dock", "custom"],
                description="Named vehicle spawn point or custom coordinates.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument(
                "physics_engine",
                default_value="gz-physics-bullet-featherstone-plugin",
                description=(
                    "Gazebo physics backend; Bullet is required for mesh "
                    "collisions."
                ),
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="-0.032",
                description="Custom X500 model spawn height.",
            ),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_dds_agent", default_value="true"),
            DeclareLaunchArgument("use_mavproxy", default_value="true"),
            DeclareLaunchArgument("use_gz_tf", default_value="true"),
            DeclareLaunchArgument("use_gz_sim_server", default_value="true"),
            DeclareLaunchArgument("use_gz_sim_gui", default_value="true"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            gz_server,
            gz_gui,
            OpaqueFunction(function=create_robot_actions),
        ]
    )

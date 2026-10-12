"""Launch the currently implemented autonomous docking simulation phases."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("drone_description")

    disturbance_injector = Node(
        package="drone_description",
        executable="disturbance_injector",
        name="disturbance_injector",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("disturbance_config"),
            {"target_model": LaunchConfiguration("robot_name")},
        ],
        condition=IfCondition(LaunchConfiguration("use_disturbances")),
    )

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
            "physics_engine": LaunchConfiguration("physics_engine"),
            "initial_point": LaunchConfiguration("initial_point"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "use_dds_agent": LaunchConfiguration("use_dds_agent"),
            "use_mavproxy": LaunchConfiguration("use_mavproxy"),
            "use_gz_tf": LaunchConfiguration("use_gz_tf"),
            "camera_latency_s": LaunchConfiguration("camera_latency_s"),
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
            DeclareLaunchArgument("use_mavproxy", default_value="false"),
            DeclareLaunchArgument("use_gz_tf", default_value="true"),
            DeclareLaunchArgument(
                "camera_latency_s", default_value="0.0",
                description="Additional simulated image delivery delay in seconds.",
            ),
            DeclareLaunchArgument("use_gz_sim_server", default_value="true"),
            DeclareLaunchArgument("use_gz_sim_gui", default_value="true"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            DeclareLaunchArgument(
                "use_disturbances",
                default_value="false",
                description="Apply configured external forces and torques.",
            ),
            DeclareLaunchArgument(
                "disturbance_config",
                default_value=str(
                    Path(description_share) / "config" / "disturbances.yaml"
                ),
                description="ROS parameter file for disturbance generation.",
            ),
            simulation,
            disturbance_injector,
        ]
    )

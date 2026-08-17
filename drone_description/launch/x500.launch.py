"""Spawn the X500 and start the existing ArduPilot SITL/AP_DDS stack."""

import os
import tempfile
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def create_robot_actions(context: LaunchContext) -> List[IncludeLaunchDescription]:
    """Create the generic ArduPilot robot launch with X500 resources."""
    description_share = get_package_share_directory("drone_description")
    ardupilot_bringup_share = get_package_share_directory("ardupilot_gz_bringup")
    ardupilot_sitl_share = get_package_share_directory("ardupilot_sitl")

    source_sdf = os.path.join(
        description_share, "models", "x500_with_ardupilot", "model.sdf"
    )
    with open(source_sdf, "r", encoding="utf-8") as source_file:
        robot_description = source_file.read()

    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_description = robot_description.replace(
        '<model name="x500_with_ardupilot">',
        f'<model name="{robot_name}">',
    )

    sim_address = LaunchConfiguration("sim_address").perform(context)
    robot_description = robot_description.replace(
        "<fdm_addr>127.0.0.1</fdm_addr>",
        f"<fdm_addr>{sim_address}</fdm_addr>",
    )

    temporary_sdf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".sdf", encoding="utf-8"
    )
    with temporary_sdf:
        temporary_sdf.write(robot_description)

    defaults = ",".join(
        [
            os.path.join(
                ardupilot_sitl_share,
                "config",
                "default_params",
                "copter.parm",
            ),
            os.path.join(description_share, "config", "gazebo-x500.parm"),
            os.path.join(
                ardupilot_sitl_share,
                "config",
                "default_params",
                "dds_udp.parm",
            ),
            os.path.join(
                ardupilot_sitl_share,
                "config",
                "default_params",
                "dds_use_ns.parm",
            ),
        ]
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        ardupilot_bringup_share,
                        "launch",
                        "robots",
                        "robot.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "sdf_file": temporary_sdf.name,
            "bridge_config_file": os.path.join(
                description_share, "config", "x500_bridge.yaml"
            ),
            "command": "arducopter",
            "robot_name": LaunchConfiguration("robot_name"),
            "world_name": LaunchConfiguration("world_name"),
            "model": "json",
            "defaults": defaults,
            "synthetic_clock": "True",
            "sim_address": LaunchConfiguration("sim_address"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "R": "0.0",
            "P": "0.0",
            "Y": LaunchConfiguration("yaw"),
            "instance": LaunchConfiguration("instance"),
            "use_instance_dir": LaunchConfiguration("use_instance_dir"),
            "use_dds_agent": LaunchConfiguration("use_dds_agent"),
            "use_gz_tf": LaunchConfiguration("use_gz_tf"),
        }.items(),
    )
    return [robot]


def generate_launch_description() -> LaunchDescription:
    """Generate the X500 robot launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("world_name", default_value="docking_world"),
            DeclareLaunchArgument("robot_name", default_value="x500"),
            DeclareLaunchArgument("sim_address", default_value="127.0.0.1"),
            DeclareLaunchArgument("instance", default_value="0"),
            DeclareLaunchArgument(
                "use_instance_dir",
                default_value="True",
                description="Store SITL state in an instance-specific directory.",
            ),
            DeclareLaunchArgument("use_dds_agent", default_value="true"),
            DeclareLaunchArgument("use_gz_tf", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument(
                "z",
                default_value="-0.013",
                description="Model spawn offset that places the X500 feet on z=0.",
            ),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            OpaqueFunction(function=create_robot_actions),
        ]
    )

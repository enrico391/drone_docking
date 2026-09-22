"""Spawn the X500 and start the configurable ArduPilot SITL stack."""

import os
import tempfile
import xml.etree.ElementTree as ET
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def create_robot_actions(context: LaunchContext) -> List[IncludeLaunchDescription]:
    """Create the generic ArduPilot robot launch with X500 resources."""
    description_share = get_package_share_directory("drone_description")

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

    # Keep the Gazebo PWM conversion consistent with the real controller profile.
    profile = os.path.join(description_share, "config", "real-drone-4.7.parm")
    params = {}
    with open(profile, encoding="utf-8") as stream:
        for line in stream:
            fields = line.split("#", 1)[0].split()
            if fields:
                params[fields[0]] = float(fields[1])
    model = ET.fromstring(robot_description)
    for control in model.findall("./model/plugin[@name='ArduPilotPlugin']/control"):
        control.find("servo_min").text = str(params["MOT_PWM_MIN"])
        control.find("servo_max").text = str(params["MOT_PWM_MAX"])
    robot_description = ET.tostring(model, encoding="unicode")

    temporary_sdf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".sdf", encoding="utf-8"
    )
    with temporary_sdf:
        temporary_sdf.write(robot_description)

    instance = int(LaunchConfiguration("instance").perform(context))
    domain = int(context.environment.get("ROS_DOMAIN_ID", "0"))
    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".parm") as dds:
        dds.write(f"DDS_ENABLE 1\nDDS_UDP_PORT {2019 + 10 * instance}\n"
                  f"DDS_DOMAIN_ID {domain}\n")
    # DDS objects are allocated late in 4.7; keep these defaults last.
    defaults = ",".join([
        os.path.join(description_share, "config", "copter-4.7.parm"),
        os.path.join(description_share, "config", "gazebo-x500.parm"),
        profile,
        dds.name,
    ])

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, "launch", "robot47.launch.py")
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "sdf_file": temporary_sdf.name,
            "bridge_config_file": os.path.join(
                description_share, "config", "x500_bridge.yaml"
            ),
            "command": "arducopter",
            "sitl_launch_file": os.path.join(description_share, "launch", "sitl47.launch.py"),
            "dds_agent_launch_file": os.path.join(
                description_share, "launch", "dds_agent47.launch.py"
            ),
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
            "use_mavproxy": LaunchConfiguration("use_mavproxy"),
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
            DeclareLaunchArgument("use_mavproxy", default_value="true"),
            DeclareLaunchArgument("use_gz_tf", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument(
                "z",
                default_value="-0.032",
                description="Model spawn offset that places the triangular base on z=0.",
            ),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            OpaqueFunction(function=create_robot_actions),
        ]
    )

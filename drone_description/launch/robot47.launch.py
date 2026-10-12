"""Spawn one Gazebo vehicle with the pinned ArduCopter 4.7 stack."""

import math
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bridge_config(
    source: str, world_name: str, robot_name: str, camera_latency_s: float = 0.0
) -> str:
    config = Path(source).read_text(encoding="utf-8")
    config = config.replace("{{ world_name }}", world_name)
    config = config.replace("{{ robot_name }}", robot_name)
    if camera_latency_s > 0:
        mappings = yaml.safe_load(config)
        image = next(mapping for mapping in mappings
                     if mapping['ros_topic_name'] == 'camera/image_raw')
        image['ros_topic_name'] = 'camera/image_raw_undelayed'
        config = yaml.safe_dump(mappings)
    output = tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".yaml", encoding="utf-8"
    )
    with output:
        output.write(config)
    return output.name


def create_robot_actions(context: LaunchContext):
    camera_latency_s = float(
        LaunchConfiguration("camera_latency_s", default="0.0").perform(context)
    )
    if not math.isfinite(camera_latency_s) or camera_latency_s < 0:
        raise ValueError("camera_latency_s must be finite and nonnegative")
    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    instance = int(LaunchConfiguration("instance").perform(context))
    sysid = LaunchConfiguration("sysid").perform(context) or str(instance + 1)
    sim_address = LaunchConfiguration("sim_address").perform(context)

    sdf_path = LaunchConfiguration("sdf_file").perform(context)
    robot_description = Path(sdf_path).read_text(encoding="utf-8")
    robot_description = robot_description.replace(
        "<fdm_port_in>9002</fdm_port_in>",
        f"<fdm_port_in>{9002 + 10 * instance}</fdm_port_in>",
    )
    bridge_path = _bridge_config(
        LaunchConfiguration("bridge_config_file").perform(context),
        world_name,
        robot_name,
        camera_latency_s,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[{"robot_description": robot_description, "frame_prefix": ""}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        output="screen",
        parameters=[{
            "config_file": bridge_path,
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
    )
    tf_relay = Node(
        package="topic_tools",
        executable="relay",
        namespace=namespace,
        arguments=["gz/tf", "tf"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", robot_name,
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-R", LaunchConfiguration("R"),
            "-P", LaunchConfiguration("P"),
            "-Y", LaunchConfiguration("Y"),
        ],
    )

    port_offset = 10 * instance
    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("sitl_launch_file").perform(context)
        ),
        launch_arguments={
            "instance": str(instance),
            "sysid": sysid,
            "sim_address": sim_address,
            "defaults": LaunchConfiguration("defaults"),
            "use_instance_dir": LaunchConfiguration("use_instance_dir"),
        }.items(),
    )
    dds_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("dds_agent_launch_file").perform(context)
        ),
        launch_arguments={
            "micro_ros_agent_ns": robot_name,
            "transport": "udp4",
            "port": str(2019 + port_offset),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_dds_agent")),
    )
    mavproxy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("ardupilot_sitl"))
                / "launch"
                / "mavproxy.launch.py"
            )
        ),
        launch_arguments={
            "master": f"tcp:{sim_address}:{5760 + port_offset}",
            "sitl": f"{sim_address}:{5501 + port_offset}",
            "out": f"{sim_address}:{14550 + port_offset}",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_mavproxy")),
    )
    camera_nodes = []
    if camera_latency_s > 0:
        camera_nodes.append(Node(
            package="drone_description",
            executable="camera_latency",
            namespace=namespace,
            output="screen",
            parameters=[{"use_sim_time": True, "delay_s": camera_latency_s}],
        ))
    return [state_publisher, bridge, *camera_nodes, tf_relay,
            sitl, dds_agent, mavproxy, spawn]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("robot_name", default_value="x500"),
        DeclareLaunchArgument("world_name", default_value="docking_world"),
        DeclareLaunchArgument("sim_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("instance", default_value="0"),
        DeclareLaunchArgument("sysid", default_value=""),
        DeclareLaunchArgument("defaults"),
        DeclareLaunchArgument("sdf_file"),
        DeclareLaunchArgument("bridge_config_file"),
        DeclareLaunchArgument("sitl_launch_file"),
        DeclareLaunchArgument("dds_agent_launch_file"),
        DeclareLaunchArgument("use_instance_dir", default_value="True"),
        DeclareLaunchArgument("use_dds_agent", default_value="true"),
        DeclareLaunchArgument("use_mavproxy", default_value="false"),
        DeclareLaunchArgument("use_gz_tf", default_value="true"),
        DeclareLaunchArgument(
            "camera_latency_s", default_value="0.0",
            description="Additional simulated image delivery delay in seconds.",
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="-0.032"),
        DeclareLaunchArgument("R", default_value="0.0"),
        DeclareLaunchArgument("P", default_value="0.0"),
        DeclareLaunchArgument("Y", default_value="0.0"),
    ]
    robot_actions = OpaqueFunction(function=create_robot_actions)
    return LaunchDescription(arguments + [robot_actions])

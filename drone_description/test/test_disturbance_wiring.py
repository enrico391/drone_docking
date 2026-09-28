import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


REPOSITORY = Path(__file__).parents[2]


def _load_launch_module():
    path = REPOSITORY / "drone_bringup/launch/docking_sim.launch.py"
    spec = importlib.util.spec_from_file_location("docking_sim_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_top_level_launch_can_enable_a_configured_disturbance_injector():
    description = _load_launch_module().generate_launch_description()
    declarations = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    nodes = [action for action in description.entities if isinstance(action, Node)]

    assert "use_disturbances" in declarations
    assert "disturbance_config" in declarations
    injector = next(
        node
        for node in nodes
        if node._Node__package == "drone_description"
        and node._Node__node_executable == "disturbance_injector"
    )
    context = LaunchContext()
    context.launch_configurations["namespace"] = "test_vehicle"
    assert injector._Node__node_namespace.perform(context) == "test_vehicle"
    assert any(
        node._Node__package == "drone_description"
        and node._Node__node_executable == "disturbance_injector"
        for node in nodes
    )


def test_bridge_routes_wrench_and_clear_commands_to_gazebo():
    bridge_path = REPOSITORY / "drone_description/config/x500_bridge.yaml"
    mappings = yaml.safe_load(bridge_path.read_text(encoding="utf-8"))
    by_ros_topic = {mapping["ros_topic_name"]: mapping for mapping in mappings}

    wrench = by_ros_topic["disturbance/wrench"]
    assert wrench["gz_topic_name"] == "/world/{{ world_name }}/wrench"
    assert wrench["ros_type_name"] == "ros_gz_interfaces/msg/EntityWrench"
    assert wrench["direction"] == "ROS_TO_GZ"

    clear = by_ros_topic["disturbance/clear"]
    assert clear["gz_topic_name"] == "/world/{{ world_name }}/wrench/clear"
    assert clear["ros_type_name"] == "ros_gz_interfaces/msg/Entity"
    assert clear["direction"] == "ROS_TO_GZ"

    world_pose = by_ros_topic["gz/world_pose"]
    assert world_pose["gz_topic_name"] == "/world/{{ world_name }}/pose/info"
    assert world_pose["ros_type_name"] == "tf2_msgs/msg/TFMessage"
    assert world_pose["gz_type_name"] == "gz.msgs.Pose_V"
    assert world_pose["direction"] == "GZ_TO_ROS"


def test_disturbance_rate_matches_every_simulation_physics_step():
    """Break caught: instantaneous wrench is absent between publish ticks."""
    config_path = REPOSITORY / "drone_description/config/disturbances.yaml"
    world_path = REPOSITORY / "drone_description/worlds/docking_world.sdf"
    parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "/**"
    ]["ros__parameters"]
    max_step_size = float(
        ET.parse(world_path).getroot().findtext("world/physics/max_step_size")
    )

    assert parameters["update_rate"] == 1.0 / max_step_size

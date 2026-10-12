"""Catch bypassed image delays and multiple publishers on the public stream."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(
        name.replace('.', '_'), ROOT / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def robot_nodes(delay):
    module = load_launch('robot47.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'namespace': 'test_vehicle', 'robot_name': 'x500',
        'world_name': 'docking_world', 'instance': '0', 'sysid': '',
        'sim_address': '127.0.0.1',
        'sdf_file': str(ROOT / 'models/x500_with_ardupilot/model.sdf'),
        'bridge_config_file': str(ROOT / 'config/x500_bridge.yaml'),
        'sitl_launch_file': str(ROOT / 'launch/sitl47.launch.py'),
        'dds_agent_launch_file': str(ROOT / 'launch/dds_agent47.launch.py'),
        'camera_latency_s': str(delay),
    })
    nodes = [action for action in module.create_robot_actions(context)
             if isinstance(action, Node)]
    return context, nodes


@pytest.mark.parametrize('delay', [0.0, 0.11])
def test_bridge_and_relay_provide_one_public_image_route(delay):
    context, nodes = robot_nodes(delay)
    bridge = next(n for n in nodes if n._Node__package == 'ros_gz_bridge')
    bridge_parameters = evaluate_parameters(context, bridge._Node__parameters)[0]
    config_path = Path(bridge_parameters['config_file'])
    try:
        mappings = yaml.safe_load(config_path.read_text())
    finally:
        config_path.unlink()
    images = [m for m in mappings
              if m['ros_type_name'] == 'sensor_msgs/msg/Image']
    assert len(images) == 1
    assert images[0]['ros_topic_name'] == (
        'camera/image_raw_undelayed' if delay else 'camera/image_raw')
    camera_info = next(m for m in mappings
                       if m['ros_type_name'] == 'sensor_msgs/msg/CameraInfo')
    assert camera_info['ros_topic_name'] == 'camera/camera_info'
    relays = [n for n in nodes if n._Node__package == 'drone_description'
              and n._Node__node_executable == 'camera_latency']
    assert len(relays) == (1 if delay else 0)
    if relays:
        assert evaluate_parameters(context, relays[0]._Node__parameters) == (
            {'use_sim_time': True, 'delay_s': 0.11},)
        assert relays[0]._Node__node_namespace.perform(context) == 'test_vehicle'


@pytest.mark.parametrize('delay', [-0.1, 'nan', 'inf'])
def test_launch_rejects_invalid_latency(delay):
    with pytest.raises(ValueError, match='camera_latency_s'):
        robot_nodes(delay)

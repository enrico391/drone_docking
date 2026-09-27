"""Guard firmware/profile selection and Gazebo motor command scaling."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription


def test_default_robot_uses_real_profile_and_matching_motor_endpoints():
    path = Path(__file__).parents[1] / 'launch/x500.launch.py'
    spec = importlib.util.spec_from_file_location('x500_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(dict(namespace='', robot_name='x500',
        world_name='docking_world', sim_address='127.0.0.1', instance='0',
        use_instance_dir='True', use_dds_agent='true', use_mavproxy='false',
        use_gz_tf='true', x='0', y='0', z='-0.032', yaw='0'))
    actions = module.create_robot_actions(context)
    robot = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    robot.launch_description_source.get_launch_description(context)
    source = robot.launch_description_source.location
    assert source.endswith(
        'drone_description/launch/robot47.launch.py'
    ), 'The upstream robot launcher ignores the pinned 4.7 launch files'
    arguments = dict(robot.launch_arguments)
    assert 'sitl_launch_file' in arguments, 'Must bypass the incompatible 4.8 launcher'
    assert str(arguments['sitl_launch_file']).endswith('sitl47.launch.py')
    assert str(arguments['dds_agent_launch_file']).endswith('dds_agent47.launch.py')
    defaults = arguments['defaults'].split(',')
    assert defaults[-2].endswith('real-drone-4.7.parm')
    assert 'DDS_UDP_PORT 2019' in Path(defaults[-1]).read_text()
    assert all('dds_use_ns' not in p for p in defaults)
    controls = ET.parse(arguments['sdf_file']).findall("./model/plugin[@name='ArduPilotPlugin']/control")
    assert len(controls) == 4
    assert all(float(c.findtext('servo_min')) == 1000 and
               float(c.findtext('servo_max')) == 2000 for c in controls)

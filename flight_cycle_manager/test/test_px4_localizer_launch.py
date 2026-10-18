"""Keep manager configuration separate from the included AprilTag localizer."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.utilities import evaluate_parameters, normalize_parameters


ROOT = Path(__file__).parents[2]


def load_launch(package, filename):
    path = ROOT / package / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('use_sim_time', ['true', 'false'])
@pytest.mark.parametrize('start_localizer', ['true', 'false'])
@pytest.mark.parametrize('use_real_camera', ['true', 'false'])
def test_cycle_passes_separate_localizer_config_and_clock(
        use_sim_time, start_localizer, use_real_camera):
    module = load_launch('flight_cycle_manager', 'px4_flight_cycle.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'config_file': '/tmp/manager-profile.yaml',
        'localizer_config': '/tmp/tag-geometry.yaml',
        'use_sim_time': use_sim_time,
        'start_localizer': start_localizer,
        'use_real_camera': use_real_camera,
        'image_topic': '/test/image',
        'camera_info_topic': '/test/camera_info',
        'localizer_python': '/tmp/apriltag-python',
        'camera': '1',
        'camera_width': '640',
        'camera_height': '480',
        'camera_format': 'BGR888',
        'camera_info_url': 'file:///tmp/pi-camera.yaml',
    })
    entities = module.generate_launch_description().entities
    for action in entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    group = next(action for action in entities if isinstance(action, GroupAction))
    include = next(action for action in group.get_sub_entities()
                   if isinstance(action, IncludeLaunchDescription))
    assert include.condition.evaluate(context) is (start_localizer == 'true')
    filename = ('dock_localizer_with_camera.launch.py' if use_real_camera == 'true'
                else 'dock_localizer.launch.py')
    include.launch_description_source.get_launch_description(context)
    assert include.launch_description_source.location.endswith('/' + filename)
    arguments = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                 for name, value in include.launch_arguments}
    assert arguments == {
        'config_file': '/tmp/tag-geometry.yaml',
        'use_sim_time': use_sim_time,
        'image_topic': '/test/image',
        'camera_info_topic': '/test/camera_info',
        'python_executable': '/tmp/apriltag-python',
        'camera': '1',
        'camera_width': '640',
        'camera_height': '480',
        'camera_format': 'BGR888',
        'camera_info_url': 'file:///tmp/pi-camera.yaml',
    }
    assert context.launch_configurations['config_file'] == '/tmp/manager-profile.yaml'


@pytest.mark.parametrize('use_real_camera,expected_clock', [('false', 'true'), ('true', 'false')])
def test_cycle_defaults_enable_localizer_with_tag_config(use_real_camera, expected_clock):
    module = load_launch('flight_cycle_manager', 'px4_flight_cycle.launch.py')
    context = LaunchContext()
    context.launch_configurations['use_real_camera'] = use_real_camera
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['start_localizer'] == 'true'
    assert context.launch_configurations['use_sim_time'] == expected_clock
    assert context.launch_configurations['camera_width'] == '800'
    assert context.launch_configurations['camera_height'] == '600'
    assert context.launch_configurations['camera_info_url'].endswith('ov5647_800x600.yaml')
    assert Path(context.launch_configurations['localizer_config']).name == 'dock_tags.yaml'
    assert context.launch_configurations['localizer_config'] != (
        context.launch_configurations['config_file'])


@pytest.mark.parametrize('use_sim_time', ['true', 'false'])
def test_localizer_isolates_user_packages_and_honors_clock(monkeypatch, use_sim_time):
    module = load_launch('dock_apriltag_localizer', 'dock_localizer.launch.py')
    captured = {}
    monkeypatch.setattr(module, 'Node', lambda **kwargs: captured.update(kwargs))
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = use_sim_time
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert captured['additional_env']['PYTHONNOUSERSITE'] == '1'
    parameters = evaluate_parameters(context, normalize_parameters(captured['parameters']))[0]
    assert parameters['use_sim_time'] is (use_sim_time == 'true')


@pytest.mark.parametrize('use_sim_time', ['true', 'false'])
def test_pi_camera_launch_uses_clock_and_preserves_driver_environment(monkeypatch, use_sim_time):
    module = load_launch('dock_apriltag_localizer', 'dock_localizer_with_camera.launch.py')
    captured = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: captured.append(kwargs))
    context = LaunchContext()
    context.launch_configurations.update({
        'use_sim_time': use_sim_time,
        'camera': '1', 'camera_width': '640', 'camera_height': '480',
        'camera_format': 'BGR888', 'camera_info_url': 'file:///tmp/pi-camera.yaml',
        'config_file': '/tmp/dock-tags.yaml',
    })
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    camera, localizer = captured
    assert camera['package'] == 'camera_ros'
    assert camera['executable'] == 'camera_node'
    assert camera['additional_env']['LIBCAMERA_IPA_CONFIG_PATH'] == '/usr/share/libcamera/ipa'
    assert localizer['additional_env']['PYTHONNOUSERSITE'] == '1'
    for node in captured:
        parameters = evaluate_parameters(context, normalize_parameters(node['parameters']))[0]
        assert parameters['use_sim_time'] is (use_sim_time == 'true')
    parameters = evaluate_parameters(context, normalize_parameters(camera['parameters']))[0]
    assert parameters['width'] == 640 and parameters['height'] == 480
    assert parameters['format'] == 'BGR888'
    assert parameters['camera_info_url'] == 'file:///tmp/pi-camera.yaml'
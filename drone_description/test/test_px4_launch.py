"""Guard PX4's physics backend and airframe parameter application."""

import importlib.util
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


ROOT = Path(__file__).parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(
        name.replace('.', '_'), ROOT / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_px4_defaults_to_physics_with_working_imu_acceleration():
    module = load_launch('px4_docking.launch.py')
    arguments = {
        action.name: action for action in
        module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    context = LaunchContext()
    arguments['physics_engine'].execute(context)
    assert context.launch_configurations['physics_engine'] == (
        'gz-physics-dartsim-plugin')
    # The dock uses primitive collision proxies, not a concave mesh collider.
    dock = ET.parse(ROOT / 'models/drone_docking_base_structure/model.sdf')
    assert not dock.findall('.//collision/geometry/mesh')


def test_px4_applies_profile_before_starting_controllers(tmp_path, monkeypatch):
    module = load_launch('sitl_px4.launch.py')
    build = tmp_path / 'build/px4_sitl_default'
    (build / 'bin').mkdir(parents=True)
    (build / 'bin/px4').touch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    context = LaunchContext()
    context.launch_configurations.update({
        'px4_dir': str(tmp_path), 'robot_name': 'x500',
        'world_name': 'docking_world', 'headless': 'true',
        'home_lat': '45.3821944', 'home_lon': '12.0005833', 'home_alt': '2.0',
    })
    captured = {}

    def capture_process(**kwargs):
        captured.update(kwargs)
        return kwargs

    monkeypatch.setattr(module, 'ExecuteProcess', capture_process)
    module.start_px4(context)
    environment = captured['additional_env']
    assert environment['PX4_SYS_AUTOSTART'] == '4001'
    assert environment['PX4_GZ_MODEL_NAME'] == 'x500'
    assert environment['PX4_PARAM_MPC_THR_HOVER'] == '0.38'
    assert environment['PX4_PARAM_THR_MDL_FAC'] == '0.74'
    assert environment['PX4_PARAM_NAV_DLL_ACT'] == '0'
    # The controller echoes boot-time telemetry timestamps. Translating them
    # through WSL's wall clock corrupts the experiment when UTC steps.
    assert environment['PX4_PARAM_UXRCE_DDS_SYNCT'] == '0'

    model = ET.parse(ROOT / 'models/x500_px4_base/model.sdf')
    for rotor in range(4):
        pose = model.findtext(f"./model/link[@name='rotor_{rotor}']/pose")
        x, y = map(float, pose.split()[:2])
        prefix = f'PX4_PARAM_CA_ROTOR{rotor}'
        assert float(environment[f'{prefix}_PX']) == pytest.approx(x)
        assert float(environment[f'{prefix}_PY']) == pytest.approx(-y)
        plugin = model.find(
            f"./model/plugin[actuator_number='{rotor}']")
        spin = 1 if plugin.findtext('turningDirection') == 'ccw' else -1
        assert float(environment[f'{prefix}_KM']) == pytest.approx(
            spin * float(plugin.findtext('momentConstant')))


def test_profile_parser_accepts_whitespace_and_comments(tmp_path):
    module = load_launch('sitl_px4.launch.py')
    profile = tmp_path / 'test.parm'
    profile.write_text('# header\nMPC_THR_HOVER\t0.38 # hover\n\n')
    assert module._parameter_environment(profile) == {
        'PX4_PARAM_MPC_THR_HOVER': '0.38'}


def test_pid_touchdown_speed_satisfies_px4_land_detector_descent_gate():
    module = load_launch('sitl_px4.launch.py')
    profile = module._parameter_environment(ROOT / 'config/px4_sitl_docking.parm')
    pid = yaml.safe_load((
        ROOT.parent / 'flight_cycle_manager/config/pid_landing.yaml'
    ).read_text())['flight_cycle_manager']['ros__parameters']
    # RC1 MulticopterLandDetector requires a downward setpoint >= 1.1 times
    # LNDMC_Z_VEL_MAX before ground contact can latch. Its default is 0.25 m/s.
    vertical_motion_threshold = float(profile.get('PX4_PARAM_LNDMC_Z_VEL_MAX', 0.25))
    assert 0 < vertical_motion_threshold
    assert pid['pid_touchdown_speed_m_s'] >= 1.1 * vertical_motion_threshold


@pytest.mark.parametrize('text', [
    'MPC_THR_HOVER', 'MPC_THR_HOVER 0.38 extra',
    'MPC_THR_HOVER nan', 'MPC_THR_HOVER inf',
])
def test_profile_parser_rejects_invalid_values(tmp_path, text):
    module = load_launch('sitl_px4.launch.py')
    profile = tmp_path / 'test.parm'
    profile.write_text(text)
    with pytest.raises(ValueError):
        module._parameter_environment(profile)


def test_px4_launch_wires_agent_disturbance_and_native_cycle():
    from launch.actions import IncludeLaunchDescription
    from launch_ros.actions import Node
    module = load_launch('px4_docking.launch.py')
    entities = module.generate_launch_description().entities
    context = LaunchContext()
    for action in entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['use_dds_agent'] == 'true'
    assert context.launch_configurations['dds_port'] == '8888'
    assert context.launch_configurations['disturbance_enabled'] == 'false'
    assert context.launch_configurations['use_flight_cycle_manager'] == 'false'
    assert context.launch_configurations['pid_tuner'] == 'true'
    assert context.launch_configurations['pid_tuner_port'] == '8765'
    nodes = [action for action in entities if isinstance(action, Node)]
    agent = next(node for node in nodes if node._Node__package == 'micro_ros_agent')
    assert agent.condition.evaluate(context)
    injector = next(node for node in nodes if node._Node__node_executable == 'disturbance_injector')
    assert not injector.condition.evaluate(context)
    context.launch_configurations['use_disturbances'] = 'true'
    assert injector.condition.evaluate(context)
    includes = [action for action in entities if isinstance(action, IncludeLaunchDescription)]
    assert any('px4_flight_cycle.launch.py' in ''.join(
                   sub.perform(context) for sub in action.launch_description_source._LaunchDescriptionSource__location)
               for action in includes)
    cycle = next(action for action in includes if any(
        'px4_flight_cycle.launch.py' in sub.perform(context)
        for sub in action.launch_description_source._LaunchDescriptionSource__location))
    assert context.launch_configurations['start_localizer'] == 'true'
    assert not cycle.condition.evaluate(context)
    context.launch_configurations['use_flight_cycle_manager'] = 'true'
    assert cycle.condition.evaluate(context)
    arguments = dict(cycle.launch_arguments)
    for name in ('start_localizer', 'localizer_config', 'localizer_python', 'use_sim_time',
                 'use_real_camera', 'camera', 'camera_width', 'camera_height',
                 'camera_format', 'camera_info_url', 'pid_tuner', 'pid_tuner_port', 'pid_tuner_host'):
        assert arguments[name].perform(context) == context.launch_configurations[name]


def test_sitl_passes_dds_port_and_namespace(tmp_path, monkeypatch):
    module = load_launch('sitl_px4.launch.py')
    build = tmp_path / 'build/px4_sitl_default'
    (build / 'bin').mkdir(parents=True)
    (build / 'bin/px4').touch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    context = LaunchContext()
    context.launch_configurations.update({
        'px4_dir': str(tmp_path), 'robot_name': 'x500',
        'world_name': 'docking_world', 'headless': 'true',
        'home_lat': '45.3821944', 'home_lon': '12.0005833', 'home_alt': '2.0',
        'dds_port': '9999', 'px4_namespace': '/px4_1/',
    })
    captured = {}
    monkeypatch.setattr(module, 'ExecuteProcess', lambda **kwargs: captured.update(kwargs))
    module.start_px4(context)
    assert captured['additional_env']['PX4_UXRCE_DDS_PORT'] == '9999'
    assert captured['additional_env']['PX4_UXRCE_DDS_NS'] == 'px4_1'
    assert captured['additional_env']['PX4_PARAM_COM_OBL_RC_ACT'] == '4'


def test_default_sitl_checkout_matches_message_snapshot():
    firmware = ROOT.parents[2] / '.worktrees/PX4-Autopilot'
    if not firmware.is_dir():
        pytest.skip('local PX4 checkout unavailable')
    snapshot = json.loads((ROOT.parent / 'px4_msgs/firmware_snapshot.json').read_text())
    result = subprocess.run(
        ['git', '-C', str(firmware), 'rev-parse', 'HEAD'],
        capture_output=True, text=True, check=True, timeout=10,
    )
    assert result.stdout.strip() == snapshot['commit']
    for name, origin in snapshot['messages'].items():
        assert (firmware / origin['path']).read_bytes() == (
            ROOT.parent / 'px4_msgs/msg' / name).read_bytes()


def test_docking_profile_parameters_exist_in_sitl_build():
    metadata = ROOT.parents[2] / (
        '.worktrees/PX4-Autopilot/build/px4_sitl_default/parameters.json')
    if not metadata.is_file():
        pytest.skip('local PX4 build unavailable')
    parameters = {entry['name'] for entry in json.loads(metadata.read_text())['parameters']}
    module = load_launch('sitl_px4.launch.py')
    profile = module._parameter_environment(ROOT / 'config/px4_sitl_docking.parm')
    assert {name.removeprefix('PX4_PARAM_') for name in profile} <= parameters

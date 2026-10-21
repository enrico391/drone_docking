"""Guard GUI state publication against process-local backwards UTC steps."""

import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).parents[1]
ALIAS = 'drone_description::MonotonicSceneBroadcaster'


def test_world_loads_only_the_local_monotonic_broadcaster():
    world = ET.parse(ROOT / 'worlds/docking_world.sdf')
    plugins = world.findall('./world/plugin')
    matches = [plugin for plugin in plugins
               if 'SceneBroadcaster' in plugin.get('name', '')]
    assert len(matches) == 1
    assert matches[0].get('filename') == 'drone_scene_broadcaster'
    assert matches[0].get('name') == ALIAS
    assert 'GZ_SIM_SYSTEM_PLUGIN_PATH;lib/@PROJECT_NAME@' in (
        ROOT / 'hooks/drone_description.dsv.in').read_text()
    assert 'GZ_SIM_SYSTEM_PLUGIN_PATH' in (
        ROOT / 'hooks/drone_description.sh.in').read_text()


def test_vendored_broadcaster_uses_only_monotonic_publication_timing():
    source = (ROOT / 'vendor/gz_sim8_8_15_0/SceneBroadcaster.cc').read_text()
    assert 'std::chrono::system_clock' not in source
    assert 'lastStatePubTime{std::chrono::steady_clock::now()}' in source
    assert 'auto now = std::chrono::steady_clock::now();' in source
    assert 'using namespace docking_systems;' in source
    assert ALIAS in source


@pytest.fixture(scope='module')
def backwards_clock_library(tmp_path_factory):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('C++ compiler unavailable for clock injection')
    output = (tmp_path_factory.mktemp('backwards_clock') /
              'libbackwards_realtime.so')
    subprocess.run(
        [compiler, '-std=c++17', '-shared', '-fPIC',
         str(ROOT / 'test/backwards_realtime.cc'), '-ldl', '-o', str(output)],
        check=True, capture_output=True, text=True, timeout=30,
    )
    return output


def _plugin_library():
    workspace = ROOT.parents[2]
    candidates = [
        workspace / 'build/drone_description/libdrone_scene_broadcaster.so',
        workspace / 'install/drone_description/lib/drone_description/'
        'libdrone_scene_broadcaster.so',
    ]
    candidates.extend(
        Path(directory) / 'libdrone_scene_broadcaster.so'
        for directory in os.environ.get(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', '').split(':')
        if directory
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    pytest.skip('Build drone_description before running the Gazebo regression')


@pytest.mark.parametrize(
    'monotonic', [False, True], ids=['upstream_control', 'local_fix'])
def test_rotor_gui_states_continue_under_backwards_utc(
        tmp_path, monkeypatch, backwards_clock_library, monotonic):
    transport = pytest.importorskip('gz.transport13')
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.serialized_map_pb2 import SerializedStepMap
    from gz.msgs10.world_stats_pb2 import WorldStatistics

    if not shutil.which('gz'):
        pytest.skip('Gazebo executable is unavailable')
    plugin = (str(_plugin_library()) if monotonic else
              'gz-sim-scene-broadcaster-system')
    alias = ALIAS if monotonic else 'gz::sim::systems::SceneBroadcaster'
    world_name = f'rotor_clock_{os.getpid()}_{int(monotonic)}'
    world = tmp_path / 'rotor.sdf'
    world.write_text(f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="{world_name}">
  <gravity>0 0 0</gravity>
  <physics name="1ms" type="ignore">
    <max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
  </physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="{plugin}" name="{alias}"/>
  <model name="vehicle">
    <link name="body">
      <inertial><mass>1</mass><inertia>
        <ixx>1</ixx><iyy>1</iyy><izz>1</izz>
      </inertia></inertial>
    </link>
    <link name="rotor">
      <inertial><mass>0.1</mass><inertia>
        <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
      </inertia></inertial>
      <visual name="blade"><geometry>
        <box><size>0.3 0.02 0.01</size></box>
      </geometry></visual>
    </link>
    <joint name="rotor_joint" type="revolute">
      <parent>body</parent><child>rotor</child><axis><xyz>0 0 1</xyz></axis>
    </joint>
    <plugin filename="gz-sim-joint-controller-system"
            name="gz::sim::systems::JointController">
      <joint_name>rotor_joint</joint_name><initial_velocity>15</initial_velocity>
    </plugin>
  </model>
</world></sdf>
''')

    monkeypatch.setenv('GZ_PARTITION', f'drone_clock_regression_{os.getpid()}')
    monkeypatch.setenv('GZ_IP', '127.0.0.1')
    node = transport.Node()
    states = []
    poses = []
    stats = []
    state_topic = f'/world/{world_name}/state'
    pose_topic = f'/world/{world_name}/dynamic_pose/info'
    stats_topic = f'/world/{world_name}/stats'
    node.subscribe(SerializedStepMap, state_topic,
                   lambda message: states.append((time.monotonic(), message)))
    node.subscribe(Pose_V, pose_topic, lambda message: poses.append(message))
    node.subscribe(WorldStatistics, stats_topic,
                   lambda message: stats.append(message))

    environment = dict(os.environ, LD_PRELOAD=str(backwards_clock_library))
    home = tmp_path / 'home'
    home.mkdir()
    environment['HOME'] = str(home)
    log_path = tmp_path / 'server.log'
    with log_path.open('w') as log:
        server = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '-v', '2', str(world)],
            env=environment, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 15
            while len(poses) < 10 and time.monotonic() < deadline:
                assert server.poll() is None, log_path.read_text(
                    errors='replace')
                time.sleep(0.05)
            assert len(poses) >= 10, log_path.read_text(errors='replace')
            states.clear()
            poses.clear()
            stats.clear()
            time.sleep(4)
            assert server.poll() is None, log_path.read_text(errors='replace')
            assert len(poses) > 100, 'Physics poses must stay healthy'
            assert len(stats) > 10
            rotor_ids = {pose.id for message in poses for pose in message.pose
                         if pose.name == 'rotor'}
            assert len(rotor_ids) == 1
            rotor_id = rotor_ids.pop()
            rotations = {(pose.orientation.z, pose.orientation.w)
                         for message in poses for pose in message.pose
                         if pose.id == rotor_id}
            assert len(rotations) > 20, 'The rotor must actually rotate'
            if monotonic:
                assert len(states) > 100, 'GUI states stalled under UTC steps'
                gaps = [b[0] - a[0] for a, b in zip(states, states[1:])]
                assert gaps and max(gaps) < 0.5
                rotor_states = set()
                for _, message in states:
                    if rotor_id in message.state.entities:
                        entity = message.state.entities[rotor_id]
                        components = entity.components
                        rotor_states.add(tuple(sorted(
                            (key, component.component)
                            for key, component in components.items())))
                assert len(rotor_states) > 20, 'GUI rotor poses must change'
            else:
                assert len(states) < 60, 'Clock fault did not stall upstream'
        finally:
            for topic in (state_topic, pose_topic, stats_topic):
                node.unsubscribe(topic)
            try:
                os.killpg(server.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGKILL)
                server.wait()

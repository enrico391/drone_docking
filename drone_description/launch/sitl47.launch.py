"""Launch the pinned ArduCopter 4.7.0 executable with its supported CLI."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def start_sitl(context):
    instance = LaunchConfiguration('instance').perform(context)
    binary = Path(get_package_prefix('drone_description')) / 'lib/drone_description/arducopter-4.7.0'
    if not binary.is_file():
        raise RuntimeError('Pinned ArduCopter 4.7.0 is missing; rebuild drone_description.')
    # Never reuse 4.8 EEPROM state. Preserve changes between 4.7 sessions.
    state = Path.cwd() / '.sitl' / 'arducopter-4.7.0'
    if LaunchConfiguration('use_instance_dir').perform(context).lower() == 'true':
        state /= instance
    state.mkdir(parents=True, exist_ok=True)
    return [ExecuteProcess(
        cmd=[str(binary), '--model', 'json', '--speedup', '1',
             '--sim-address', LaunchConfiguration('sim_address'),
             '--instance', instance, '--sysid', LaunchConfiguration('sysid'),
             '--defaults', LaunchConfiguration('defaults')],
        cwd=str(state), output='both', shell=False,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('instance', default_value='0'),
        DeclareLaunchArgument('sysid', default_value='1'),
        DeclareLaunchArgument('sim_address', default_value='127.0.0.1'),
        DeclareLaunchArgument('defaults'),
        DeclareLaunchArgument('use_instance_dir', default_value='True'),
        OpaqueFunction(function=start_sitl),
    ])

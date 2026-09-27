from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('dock_apriltag_localizer'),
        'config', 'dock_tags.yaml'
    )
    virtual_env = os.environ.get('VIRTUAL_ENV')
    default_python_executable = (
        os.path.join(virtual_env, 'bin', 'python')
        if virtual_env else '/usr/bin/python3'
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('dock_frame', default_value='dock_target'),
        DeclareLaunchArgument(
            'python_executable', default_value=default_python_executable
        ),
        DeclareLaunchArgument('pose_filter_enabled', default_value='true'),
        DeclareLaunchArgument(
            'pose_filter_time_constant_s', default_value='0.12'
        ),
        DeclareLaunchArgument(
            'pose_filter_max_translation_speed_m_s', default_value='2.0'
        ),
        DeclareLaunchArgument(
            'pose_filter_max_position_jump_m', default_value='0.20'
        ),
        DeclareLaunchArgument(
            'pose_filter_reacquire_frames', default_value='3'
        ),

        Node(
            package='dock_apriltag_localizer',
            executable='dock_localizer',
            name='dock_apriltag_localizer',
            output='screen',
            prefix=[LaunchConfiguration('python_executable')],
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'dock_frame': LaunchConfiguration('dock_frame'),
                'publish_debug_image': True,
                'pose_filter_enabled': ParameterValue(
                    LaunchConfiguration('pose_filter_enabled'),
                    value_type=bool,
                ),
                'pose_filter_time_constant_s': ParameterValue(
                    LaunchConfiguration('pose_filter_time_constant_s'),
                    value_type=float,
                ),
                'pose_filter_max_translation_speed_m_s': ParameterValue(
                    LaunchConfiguration(
                        'pose_filter_max_translation_speed_m_s'
                    ),
                    value_type=float,
                ),
                'pose_filter_max_position_jump_m': ParameterValue(
                    LaunchConfiguration('pose_filter_max_position_jump_m'),
                    value_type=float,
                ),
                'pose_filter_reacquire_frames': ParameterValue(
                    LaunchConfiguration('pose_filter_reacquire_frames'),
                    value_type=int,
                ),
            }]
        ),
    ])

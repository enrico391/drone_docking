from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('dock_apriltag_localizer'),
        'config', 'dock_tags.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('dock_frame', default_value='dock_target'),
        #DeclareLaunchArgument('python_executable', default_value='/usr/bin/python3'),

        Node(
            package='dock_apriltag_localizer',
            executable='dock_localizer',
            name='dock_apriltag_localizer',
            output='screen',
            #prefix=[LaunchConfiguration('python_executable')],
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'dock_frame': LaunchConfiguration('dock_frame'),
                'publish_debug_image': True,
            }]
        )
    ])

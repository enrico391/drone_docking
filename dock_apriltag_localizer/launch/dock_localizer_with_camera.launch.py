"""Launch the libcamera driver and the dock AprilTag localizer together."""

import os
import sysconfig

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("dock_apriltag_localizer")
    default_cfg = os.path.join(
        package_share,
        "config",
        "dock_tags.yaml",
    )
    default_camera_info_url = "file://" + os.path.join(
        package_share,
        "config",
        "ov5647_800x600.yaml",
    )

    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    # ROS Lyrical ships its own libcamera in /opt/ros. On this Raspberry Pi,
    # the Ubuntu/Raspberry Pi camera stack in /usr is the stack matched to the
    # kernel and firmware, so load it first for the camera process only.
    multiarch = sysconfig.get_config_var("MULTIARCH")
    system_library_dir = os.path.join("/usr/lib", multiarch)
    camera_library_path = os.pathsep.join(
        filter(
            None,
            [system_library_dir, os.environ.get("LD_LIBRARY_PATH", "")],
        )
    )
    system_camera_environment = {
        "LD_LIBRARY_PATH": camera_library_path,
        "LIBCAMERA_IPA_MODULE_PATH": os.path.join(
            system_library_dir, "libcamera", "ipa"
        ),
        "LIBCAMERA_IPA_PROXY_PATH": os.path.join(
            "/usr/libexec", multiarch, "libcamera"
        ),
        "LIBCAMERA_IPA_CONFIG_PATH": "/usr/share/libcamera/ipa",
    }

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        additional_env=system_camera_environment,
        parameters=[
            {
                "camera": LaunchConfiguration("camera"),
                "width": ParameterValue(
                    LaunchConfiguration("camera_width"), value_type=int
                ),
                "height": ParameterValue(
                    LaunchConfiguration("camera_height"), value_type=int
                ),
                "format": LaunchConfiguration("camera_format"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
            }
        ],
        remappings=[
            ("/camera/image_raw", image_topic),
            ("/camera/camera_info", camera_info_topic),
        ],
    )

    localizer_node = Node(
        package="dock_apriltag_localizer",
        executable="dock_localizer",
        name="dock_apriltag_localizer",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "dock_frame": LaunchConfiguration("dock_frame"),
                "publish_debug_image": LaunchConfiguration(
                    "publish_debug_image"
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera", default_value="0"),
            DeclareLaunchArgument("camera_width", default_value="800"),
            DeclareLaunchArgument("camera_height", default_value="600"),
            DeclareLaunchArgument("camera_format", default_value="RGB888"),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value=default_camera_info_url,
            ),
            DeclareLaunchArgument("config_file", default_value=default_cfg),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),
            DeclareLaunchArgument("dock_frame", default_value="dock_target"),
            DeclareLaunchArgument(
                "publish_debug_image", default_value="true"
            ),
            camera_node,
            localizer_node,
        ]
    )

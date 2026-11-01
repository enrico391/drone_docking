"""Launch the libcamera driver and the dock AprilTag localizer together."""

import os
import sysconfig

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
    # kernel and firmware, so load it first for the shared component container.
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

    camera_node = ComposableNode(
        package="camera_ros",
        plugin="camera::CameraNode",
        name="camera",
        extra_arguments=[{"use_intra_process_comms": ParameterValue(
            LaunchConfiguration("use_intra_process_comms"), value_type=bool)}],
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
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

    localizer_node = ComposableNode(
        package="dock_apriltag_localizer",
        plugin="dock_apriltag_localizer::DockLocalizer",
        name="dock_apriltag_localizer",
        extra_arguments=[{"use_intra_process_comms": ParameterValue(
            LaunchConfiguration("use_intra_process_comms"), value_type=bool)}],
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "config_file": LaunchConfiguration("config_file"),
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "dock_frame": LaunchConfiguration("dock_frame"),
                "publish_debug_image": ParameterValue(
                    LaunchConfiguration("publish_debug_image"), value_type=bool
                ),
                "pose_filter_enabled": ParameterValue(
                    LaunchConfiguration("pose_filter_enabled"),
                    value_type=bool,
                ),
                "pose_filter_time_constant_s": ParameterValue(
                    LaunchConfiguration("pose_filter_time_constant_s"),
                    value_type=float,
                ),
                "pose_filter_max_translation_speed_m_s": ParameterValue(
                    LaunchConfiguration(
                        "pose_filter_max_translation_speed_m_s"
                    ),
                    value_type=float,
                ),
                "pose_filter_max_position_jump_m": ParameterValue(
                    LaunchConfiguration(
                        "pose_filter_max_position_jump_m"
                    ),
                    value_type=float,
                ),
                "pose_filter_reacquire_frames": ParameterValue(
                    LaunchConfiguration("pose_filter_reacquire_frames"),
                    value_type=int,
                ),
            }
        ],
    )

    container = ComposableNodeContainer(
        name="dock_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        additional_env=system_camera_environment,
        composable_node_descriptions=[camera_node, localizer_node],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_intra_process_comms", default_value="true"),
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
                "python_executable",
                default_value='/usr/bin/python3',
                description="Deprecated compatibility argument; localizer is C++.",
            ),
            DeclareLaunchArgument(
                "publish_debug_image", default_value="false"
            ),
            DeclareLaunchArgument(
                "pose_filter_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "pose_filter_time_constant_s", default_value="0.12"
            ),
            DeclareLaunchArgument(
                "pose_filter_max_translation_speed_m_s",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "pose_filter_max_position_jump_m", default_value="0.20"
            ),
            DeclareLaunchArgument(
                "pose_filter_reacquire_frames", default_value="3"
            ),
            container,
        ]
    )

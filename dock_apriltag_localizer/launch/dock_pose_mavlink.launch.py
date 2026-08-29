"""Launch the ROS dock-pose to MAVLink LANDING_TARGET bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dock_pose_topic", default_value="/dock/pose"
            ),
            DeclareLaunchArgument(
                "mavlink_connection",
                default_value="/dev/ttyAMA0",
                description=(
                    "Direct ArduPilot serial or TCP pymavlink endpoint."
                ),
            ),
            DeclareLaunchArgument("mavlink_baud", default_value="921600"),
            DeclareLaunchArgument("source_system", default_value="254"),
            DeclareLaunchArgument("source_component", default_value="191"),
            DeclareLaunchArgument("camera_forward_m", default_value="0.0"),
            DeclareLaunchArgument("camera_right_m", default_value="0.0"),
            DeclareLaunchArgument("camera_down_m", default_value="0.206"),
            Node(
                package="dock_apriltag_localizer",
                executable="dock_pose_mavlink",
                name="dock_pose_mavlink",
                output="screen",
                parameters=[
                    {
                        "dock_pose_topic": LaunchConfiguration(
                            "dock_pose_topic"
                        ),
                        "mavlink_connection": LaunchConfiguration(
                            "mavlink_connection"
                        ),
                        "mavlink_baud": ParameterValue(
                            LaunchConfiguration("mavlink_baud"), value_type=int
                        ),
                        "source_system": ParameterValue(
                            LaunchConfiguration("source_system"),
                            value_type=int,
                        ),
                        "source_component": ParameterValue(
                            LaunchConfiguration("source_component"),
                            value_type=int,
                        ),
                        "camera_forward_m": ParameterValue(
                            LaunchConfiguration("camera_forward_m"),
                            value_type=float,
                        ),
                        "camera_right_m": ParameterValue(
                            LaunchConfiguration("camera_right_m"),
                            value_type=float,
                        ),
                        "camera_down_m": ParameterValue(
                            LaunchConfiguration("camera_down_m"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )

"""Bridge ROS dock poses to MAVLink LANDING_TARGET messages."""

import math
import os
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


LANDING_TARGET_TYPE_FIDUCIAL = 2


def measurement_time_usec(
    *, stamp_sec: int, stamp_nanosec: int, fallback_usec: int
) -> int:
    """Return a ROS acquisition timestamp or a fallback for an empty stamp."""
    timestamp_usec = int(stamp_sec) * 1_000_000 + int(stamp_nanosec) // 1000
    return timestamp_usec if timestamp_usec > 0 else int(fallback_usec)


def camera_target_to_body_frd(
    camera_x_m: float,
    camera_y_m: float,
    camera_z_m: float,
    camera_forward_m: float = 0.0,
    camera_right_m: float = 0.0,
    camera_down_m: float = 0.0,
) -> Tuple[float, float, float]:
    """Convert a downward optical-frame target into vehicle Body-FRD."""
    return (
        camera_forward_m - camera_y_m,
        camera_right_m + camera_x_m,
        camera_down_m + camera_z_m,
    )


def load_mavutil():
    """Load pymavlink using MAVLink 2 and the ArduPilot dialect."""
    os.environ["MAVLINK20"] = "1"
    try:
        from pymavlink import mavutil
    except ImportError as error:
        raise RuntimeError(
            "dock_pose_mavlink requires pymavlink>=2.4.49,<3"
        ) from error
    mavutil.set_dialect("ardupilotmega")
    return mavutil


class MavlinkLandingTargetSender:
    """Send LANDING_TARGET through one direct pymavlink connection."""

    def __init__(
        self,
        connection: str,
        baud: int,
        source_system: int,
        source_component: int,
        mavutil_module=None,
    ) -> None:
        self._mavutil = mavutil_module or load_mavutil()
        self._connection = self._mavutil.mavlink_connection(
            connection,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            dialect="ardupilotmega",
            autoreconnect=True,
            robust_parsing=True,
        )

    def send(
        self,
        *,
        time_usec: int,
        forward_m: float,
        right_m: float,
        down_m: float,
    ) -> bool:
        """Send one position-valid Body-FRD landing target."""
        distance_m = math.sqrt(forward_m**2 + right_m**2 + down_m**2)
        self._connection.mav.landing_target_send(
            int(time_usec),
            0,
            self._mavutil.mavlink.MAV_FRAME_BODY_FRD,
            0.0,
            0.0,
            distance_m,
            0.0,
            0.0,
            float(forward_m),
            float(right_m),
            float(down_m),
            (1.0, 0.0, 0.0, 0.0),
            LANDING_TARGET_TYPE_FIDUCIAL,
            1,
        )
        return True

    def close(self) -> None:
        """Close the direct pymavlink connection."""
        self._connection.close()


class DockPoseMavlinkBridge(Node):
    """Convert dock PoseStamped samples into MAVLink LANDING_TARGET packets."""

    def __init__(self) -> None:
        self._sender: Optional[MavlinkLandingTargetSender] = None
        super().__init__("dock_pose_mavlink")
        self.declare_parameter("dock_pose_topic", "/dock/pose")
        self.declare_parameter("mavlink_connection", "/dev/ttyAMA0")
        self.declare_parameter("mavlink_baud", 921600)
        self.declare_parameter("source_system", 254)
        self.declare_parameter("source_component", 191)
        self.declare_parameter("camera_forward_m", 0.0)
        self.declare_parameter("camera_right_m", 0.0)
        self.declare_parameter("camera_down_m", 0.206)

        connection = str(self.get_parameter("mavlink_connection").value)
        baud = int(self.get_parameter("mavlink_baud").value)
        if not connection:
            raise ValueError("mavlink_connection must not be empty")
        if baud <= 0:
            raise ValueError("mavlink_baud must be greater than zero")
        for name in ("source_system", "source_component"):
            value = int(self.get_parameter(name).value)
            if not 1 <= value <= 255:
                raise ValueError(f"{name} must be in [1, 255]")

        self._sender = MavlinkLandingTargetSender(
            connection=connection,
            baud=baud,
            source_system=int(self.get_parameter("source_system").value),
            source_component=int(
                self.get_parameter("source_component").value
            ),
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        topic = str(self.get_parameter("dock_pose_topic").value)
        self._subscription = self.create_subscription(
            PoseStamped, topic, self._pose_callback, sensor_qos
        )
        self.get_logger().info(
            f"Bridging {topic} to MAVLink LANDING_TARGET on {connection}"
        )

    def _pose_callback(self, message: PoseStamped) -> None:
        position = message.pose.position
        if not all(
            math.isfinite(value)
            for value in (position.x, position.y, position.z)
        ):
            self.get_logger().warning(
                "Ignoring dock pose with a non-finite position",
                throttle_duration_sec=2.0,
            )
            return

        forward_m, right_m, down_m = camera_target_to_body_frd(
            camera_x_m=position.x,
            camera_y_m=position.y,
            camera_z_m=position.z,
            camera_forward_m=float(
                self.get_parameter("camera_forward_m").value
            ),
            camera_right_m=float(
                self.get_parameter("camera_right_m").value
            ),
            camera_down_m=float(self.get_parameter("camera_down_m").value),
        )
        sender = self._sender
        if sender is None:
            return
        stamp = message.header.stamp
        sender.send(
            time_usec=measurement_time_usec(
                stamp_sec=stamp.sec,
                stamp_nanosec=stamp.nanosec,
                fallback_usec=self.get_clock().now().nanoseconds // 1000,
            ),
            forward_m=forward_m,
            right_m=right_m,
            down_m=down_m,
        )

    def destroy_node(self):
        """Release the pymavlink connection before destroying the ROS node."""
        if self._sender is not None:
            self._sender.close()
            self._sender = None
        return super().destroy_node()


def main(args=None) -> None:
    """Run the dock-pose MAVLink bridge."""
    rclpy.init(args=args)
    node = None
    try:
        node = DockPoseMavlinkBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, ValueError, OSError) as exception:
        if node is not None and not isinstance(exception, KeyboardInterrupt):
            node.get_logger().fatal(str(exception))
        elif not isinstance(exception, KeyboardInterrupt):
            print(f"dock_pose_mavlink: {exception}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

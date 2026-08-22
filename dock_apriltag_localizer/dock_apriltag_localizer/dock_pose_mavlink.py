"""Bridge ROS dock poses to MAVLink LANDING_TARGET messages."""

import math
import socket
import struct
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


MAVLINK2_MAGIC = 0xFD
MAVLINK1_MAGIC = 0xFE
MAVLINK_MSG_ID_LANDING_TARGET = 149
MAVLINK_MSG_ID_LANDING_TARGET_CRC = 200
MAV_FRAME_BODY_FRD = 12
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


def _x25_crc(data: bytes, extra: int) -> int:
    """Return the MAVLink X.25 checksum for header, payload, and CRC extra."""
    crc = 0xFFFF
    for byte in data + bytes((extra,)):
        temporary = byte ^ (crc & 0xFF)
        temporary ^= (temporary << 4) & 0xFF
        crc = (
            (crc >> 8)
            ^ (temporary << 8)
            ^ (temporary << 3)
            ^ (temporary >> 4)
        ) & 0xFFFF
    return crc


def encode_landing_target(
    *,
    sequence: int,
    source_system: int,
    source_component: int,
    time_usec: int,
    forward_m: float,
    right_m: float,
    down_m: float,
) -> bytes:
    """Encode one MAVLink 2 LANDING_TARGET with valid Body-FRD position."""
    distance_m = math.sqrt(forward_m**2 + right_m**2 + down_m**2)
    payload = struct.pack(
        "<QfffffBBfff4fBB",
        time_usec,
        0.0,
        0.0,
        distance_m,
        0.0,
        0.0,
        0,
        MAV_FRAME_BODY_FRD,
        forward_m,
        right_m,
        down_m,
        1.0,
        0.0,
        0.0,
        0.0,
        LANDING_TARGET_TYPE_FIDUCIAL,
        1,
    )
    message_id = MAVLINK_MSG_ID_LANDING_TARGET
    header = bytes(
        (
            len(payload),
            0,
            0,
            sequence & 0xFF,
            source_system & 0xFF,
            source_component & 0xFF,
            message_id & 0xFF,
            (message_id >> 8) & 0xFF,
            (message_id >> 16) & 0xFF,
        )
    )
    checksum = _x25_crc(
        header + payload, MAVLINK_MSG_ID_LANDING_TARGET_CRC
    )
    return bytes((MAVLINK2_MAGIC,)) + header + payload + struct.pack(
        "<H", checksum
    )


class MavlinkLandingTargetSender:
    """Send LANDING_TARGET to a peer discovered on a MAVProxy UDP output."""

    def __init__(
        self,
        bind_address: str,
        port: int,
        source_system: int,
        source_component: int,
    ) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((bind_address, port))
        self._socket.setblocking(False)
        self._source_system = source_system
        self._source_component = source_component
        self._sequence = 0
        self._peer: Optional[Tuple[str, int]] = None

    @property
    def connected(self) -> bool:
        """Whether a valid MAVLink datagram has established a UDP peer."""
        return self._peer is not None

    def poll(self) -> None:
        """Drain input and remember the MAVLink peer's return address."""
        while True:
            try:
                data, peer = self._socket.recvfrom(65535)
            except BlockingIOError:
                return
            if data and data[0] in (MAVLINK1_MAGIC, MAVLINK2_MAGIC):
                self._peer = peer

    def send(
        self,
        *,
        time_usec: int,
        forward_m: float,
        right_m: float,
        down_m: float,
    ) -> bool:
        """Send one target to the discovered peer, returning whether it sent."""
        self.poll()
        if self._peer is None:
            return False
        packet = encode_landing_target(
            sequence=self._sequence,
            source_system=self._source_system,
            source_component=self._source_component,
            time_usec=time_usec,
            forward_m=forward_m,
            right_m=right_m,
            down_m=down_m,
        )
        self._socket.sendto(packet, self._peer)
        self._sequence = (self._sequence + 1) & 0xFF
        return True

    def close(self) -> None:
        """Close the UDP endpoint."""
        self._socket.close()


class DockPoseMavlinkBridge(Node):
    """Convert dock PoseStamped samples into MAVLink LANDING_TARGET packets."""

    def __init__(self) -> None:
        self._sender: Optional[MavlinkLandingTargetSender] = None
        super().__init__("dock_pose_mavlink")
        self.declare_parameter("dock_pose_topic", "/dock/pose")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("udp_port", 14551)
        self.declare_parameter("source_system", 254)
        self.declare_parameter("source_component", 191)
        self.declare_parameter("camera_forward_m", 0.0)
        self.declare_parameter("camera_right_m", 0.0)
        self.declare_parameter("camera_down_m", 0.206)

        port = int(self.get_parameter("udp_port").value)
        if not 1 <= port <= 65535:
            raise ValueError("udp_port must be in [1, 65535]")
        for name in ("source_system", "source_component"):
            value = int(self.get_parameter(name).value)
            if not 1 <= value <= 255:
                raise ValueError(f"{name} must be in [1, 255]")

        self._sender = MavlinkLandingTargetSender(
            bind_address=str(self.get_parameter("bind_address").value),
            port=port,
            source_system=int(self.get_parameter("source_system").value),
            source_component=int(
                self.get_parameter("source_component").value
            ),
        )
        self._peer_logged = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        topic = str(self.get_parameter("dock_pose_topic").value)
        self._subscription = self.create_subscription(
            PoseStamped, topic, self._pose_callback, sensor_qos
        )
        self._poll_timer = self.create_timer(0.05, self._poll_mavlink)
        self.get_logger().info(
            f"Bridging {topic} to MAVLink LANDING_TARGET on UDP port {port}"
        )

    def _poll_mavlink(self) -> None:
        sender = self._sender
        if sender is None:
            return
        sender.poll()
        if sender.connected and not self._peer_logged:
            self._peer_logged = True
            self.get_logger().info("MAVLink UDP peer discovered")

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
        sent = sender.send(
            time_usec=measurement_time_usec(
                stamp_sec=stamp.sec,
                stamp_nanosec=stamp.nanosec,
                fallback_usec=self.get_clock().now().nanoseconds // 1000,
            ),
            forward_m=forward_m,
            right_m=right_m,
            down_m=down_m,
        )
        if not sent:
            self.get_logger().warning(
                "Waiting for MAVLink traffic to discover the UDP peer",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self):
        """Release the MAVLink socket before destroying the ROS node."""
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
    except (KeyboardInterrupt, ValueError, OSError) as exception:
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

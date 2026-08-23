"""MAVLink LANDING_TARGET encoding and UDP transport helpers."""

import math
import socket
import struct
from collections import deque
from typing import Deque, List, Optional, Tuple


MAVLINK2_MAGIC = 0xFD
MAVLINK1_MAGIC = 0xFE
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_LONG_CRC = 152
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_LANDING_TARGET = 149
MAVLINK_MSG_ID_LANDING_TARGET_CRC = 200
MAV_FRAME_BODY_FRD = 12
MAV_CMD_CONDITION_YAW = 115
MAV_CMD_DO_AUX_FUNCTION = 218
LANDING_TARGET_TYPE_FIDUCIAL = 2
ARDUPILOT_SYSTEM_ID = 1
ARDUPILOT_COMPONENT_ID = 1


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
    """Convert a downward optical-frame target into vehicle Body-FRD.

    Optical +X is vehicle right, optical +Y is vehicle rear, and optical +Z
    is vehicle down. Camera position offsets are expressed from the vehicle
    body origin to the camera in Body-FRD.
    """
    return (
        camera_forward_m - camera_y_m,
        camera_right_m + camera_x_m,
        camera_down_m + camera_z_m,
    )


def camera_dock_orientation_to_body_yaw_error(
    quaternion_x: float,
    quaternion_y: float,
    quaternion_z: float,
    quaternion_w: float,
) -> float:
    """Return dock-forward yaw relative to vehicle forward in radians.

    The quaternion describes the dock frame in the downward camera optical
    frame. Optical +X is vehicle right and optical +Y is vehicle rear, so the
    dock X axis can be projected directly into the vehicle's horizontal plane.
    Positive errors command a clockwise/right yaw in MAVLink's Body-FRD frame.
    """
    norm = math.sqrt(
        quaternion_x**2
        + quaternion_y**2
        + quaternion_z**2
        + quaternion_w**2
    )
    if norm <= 1.0e-9:
        raise ValueError("dock orientation quaternion has zero length")

    x = quaternion_x / norm
    y = quaternion_y / norm
    z = quaternion_z / norm
    w = quaternion_w / norm
    dock_forward_camera_x = 1.0 - 2.0 * (y * y + z * z)
    dock_forward_camera_y = 2.0 * (x * y + w * z)
    dock_forward_body_forward = -dock_forward_camera_y
    dock_forward_body_right = dock_forward_camera_x
    return math.atan2(
        dock_forward_body_right,
        dock_forward_body_forward,
    )


def ros_enu_quaternion_to_mavlink_heading_deg(
    quaternion_x: float,
    quaternion_y: float,
    quaternion_z: float,
    quaternion_w: float,
) -> float:
    """Convert a ROS ENU attitude quaternion to clockwise NED heading."""
    norm = math.sqrt(
        quaternion_x**2
        + quaternion_y**2
        + quaternion_z**2
        + quaternion_w**2
    )
    if norm <= 1.0e-9:
        raise ValueError("vehicle orientation quaternion has zero length")
    x = quaternion_x / norm
    y = quaternion_y / norm
    z = quaternion_z / norm
    w = quaternion_w / norm
    yaw_enu_rad = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    heading_deg = math.degrees(math.pi / 2.0 - yaw_enu_rad) % 360.0
    return 0.0 if heading_deg >= 360.0 - 1.0e-9 else heading_deg


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
        0.0,  # angle_x is unused when position_valid is true
        0.0,  # angle_y is unused when position_valid is true
        distance_m,
        0.0,  # size_x is unused by ArduPilot
        0.0,  # size_y is unused by ArduPilot
        0,  # target_num is unused by ArduPilot
        MAV_FRAME_BODY_FRD,
        forward_m,
        right_m,
        down_m,
        1.0,
        0.0,
        0.0,
        0.0,
        LANDING_TARGET_TYPE_FIDUCIAL,
        1,  # position_valid
    )
    message_id = MAVLINK_MSG_ID_LANDING_TARGET
    header = bytes(
        (
            len(payload),
            0,  # incompatibility flags
            0,  # compatibility flags
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


def _encode_command_long(
    *,
    sequence: int,
    source_system: int,
    source_component: int,
    params: Tuple[float, float, float, float, float, float, float],
    command: int,
) -> bytes:
    payload = struct.pack(
        "<7fHBB",
        *params,
        command,
        ARDUPILOT_SYSTEM_ID,
        ARDUPILOT_COMPONENT_ID,
    )
    message_id = MAVLINK_MSG_ID_COMMAND_LONG
    header = bytes(
        (
            len(payload),
            0,
            0,
            sequence & 0xFF,
            source_system & 0xFF,
            source_component & 0xFF,
            message_id,
            0,
            0,
        )
    )
    checksum = _x25_crc(header + payload, MAVLINK_MSG_ID_COMMAND_LONG_CRC)
    return bytes((MAVLINK2_MAGIC,)) + header + payload + struct.pack(
        "<H", checksum
    )


def encode_condition_yaw(
    *,
    sequence: int,
    source_system: int,
    source_component: int,
    yaw_error_rad: float,
) -> bytes:
    """Encode a relative MAV_CMD_CONDITION_YAW in a MAVLink 2 COMMAND_LONG."""
    direction = 1.0 if yaw_error_rad >= 0.0 else -1.0
    return _encode_command_long(
        sequence=sequence,
        source_system=source_system,
        source_component=source_component,
        params=(
            abs(math.degrees(yaw_error_rad)),
            0.0,
            direction,
            1.0,
            0.0,
            0.0,
            0.0,
        ),
        command=MAV_CMD_CONDITION_YAW,
    )


def encode_condition_yaw_absolute(
    *,
    sequence: int,
    source_system: int,
    source_component: int,
    heading_deg: float,
    yaw_rate_deg_s: float,
) -> bytes:
    """Encode an idempotent absolute-heading MAV_CMD_CONDITION_YAW."""
    if not 0.0 <= heading_deg <= 360.0:
        raise ValueError("heading_deg must be in [0, 360]")
    if yaw_rate_deg_s <= 0.0:
        raise ValueError("yaw_rate_deg_s must be positive")
    return _encode_command_long(
        sequence=sequence,
        source_system=source_system,
        source_component=source_component,
        params=(heading_deg, yaw_rate_deg_s, 0.0, 0.0, 0.0, 0.0, 0.0),
        command=MAV_CMD_CONDITION_YAW,
    )


def encode_aux_function(
    *,
    sequence: int,
    source_system: int,
    source_component: int,
    function: int,
    switch_position: int,
) -> bytes:
    """Encode MAV_CMD_DO_AUX_FUNCTION for a three-position auxiliary input."""
    if switch_position not in (0, 1, 2):
        raise ValueError("switch_position must be LOW=0, MIDDLE=1, or HIGH=2")
    return _encode_command_long(
        sequence=sequence,
        source_system=source_system,
        source_component=source_component,
        params=(
            float(function),
            float(switch_position),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
        command=MAV_CMD_DO_AUX_FUNCTION,
    )


def decode_command_acks(datagram: bytes) -> List[Tuple[int, int]]:
    """Extract command/result pairs from MAVLink 1 or 2 datagrams."""
    acknowledgements: List[Tuple[int, int]] = []
    offset = 0
    while offset < len(datagram):
        magic = datagram[offset]
        if magic == MAVLINK2_MAGIC and offset + 10 <= len(datagram):
            payload_size = datagram[offset + 1]
            incompatibility_flags = datagram[offset + 2]
            message_id = (
                datagram[offset + 7]
                | datagram[offset + 8] << 8
                | datagram[offset + 9] << 16
            )
            payload_offset = offset + 10
            frame_size = 10 + payload_size + 2
            if incompatibility_flags & 0x01:
                frame_size += 13
        elif magic == MAVLINK1_MAGIC and offset + 6 <= len(datagram):
            payload_size = datagram[offset + 1]
            message_id = datagram[offset + 5]
            payload_offset = offset + 6
            frame_size = 6 + payload_size + 2
        else:
            offset += 1
            continue
        if offset + frame_size > len(datagram):
            break
        if message_id == MAVLINK_MSG_ID_COMMAND_ACK and payload_size >= 3:
            command, result = struct.unpack_from(
                "<HB", datagram, payload_offset
            )
            acknowledgements.append((command, result))
        offset += frame_size
    return acknowledgements


class MavlinkLandingTargetSender:
    """Send LANDING_TARGET back through a MAVProxy UDP output connection."""

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
        self._command_acks: Deque[Tuple[int, int]] = deque()

    @property
    def connected(self) -> bool:
        """Whether at least one MAVLink datagram has established a UDP peer."""
        return self._peer is not None

    def poll(self) -> None:
        """Drain input and remember MAVProxy's return address."""
        while True:
            try:
                data, peer = self._socket.recvfrom(65535)
            except BlockingIOError:
                return
            if data and data[0] in (MAVLINK1_MAGIC, MAVLINK2_MAGIC):
                self._peer = peer
                self._command_acks.extend(decode_command_acks(data))

    def take_command_ack(self, command: int) -> Optional[int]:
        """Consume and return the oldest acknowledgement for *command*."""
        for acknowledgement in tuple(self._command_acks):
            if acknowledgement[0] == command:
                self._command_acks.remove(acknowledgement)
                return acknowledgement[1]
        return None

    def send(
        self,
        *,
        time_usec: int,
        forward_m: float,
        right_m: float,
        down_m: float,
    ) -> bool:
        """Send one target and report whether a MAVLink peer was available."""
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

    def send_condition_yaw(self, *, yaw_error_rad: float) -> bool:
        """Send one relative yaw-alignment command to ArduPilot."""
        self.poll()
        if self._peer is None:
            return False
        packet = encode_condition_yaw(
            sequence=self._sequence,
            source_system=self._source_system,
            source_component=self._source_component,
            yaw_error_rad=yaw_error_rad,
        )
        self._socket.sendto(packet, self._peer)
        self._sequence = (self._sequence + 1) & 0xFF
        return True

    def send_condition_yaw_absolute(
        self, *, heading_deg: float, yaw_rate_deg_s: float
    ) -> bool:
        """Send an absolute heading target that is safe to retry."""
        self.poll()
        if self._peer is None:
            return False
        packet = encode_condition_yaw_absolute(
            sequence=self._sequence,
            source_system=self._source_system,
            source_component=self._source_component,
            heading_deg=heading_deg,
            yaw_rate_deg_s=yaw_rate_deg_s,
        )
        self._socket.sendto(packet, self._peer)
        self._sequence = (self._sequence + 1) & 0xFF
        return True

    def send_aux_function(
        self, *, function: int, switch_position: int
    ) -> bool:
        """Set an ArduPilot auxiliary function switch position."""
        self.poll()
        if self._peer is None:
            return False
        packet = encode_aux_function(
            sequence=self._sequence,
            source_system=self._source_system,
            source_component=self._source_component,
            function=function,
            switch_position=switch_position,
        )
        self._socket.sendto(packet, self._peer)
        self._sequence = (self._sequence + 1) & 0xFF
        return True

    def close(self) -> None:
        """Close the UDP endpoint."""
        self._socket.close()

"""MAVLink LANDING_TARGET encoding and UDP transport helpers."""

import math
import socket
import struct
from typing import Optional, Tuple


MAVLINK2_MAGIC = 0xFD
MAVLINK1_MAGIC = 0xFE
MAVLINK_MSG_ID_LANDING_TARGET = 149
MAVLINK_MSG_ID_LANDING_TARGET_CRC = 200
MAV_FRAME_BODY_FRD = 12
LANDING_TARGET_TYPE_FIDUCIAL = 2


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
    return bytes((MAVLINK2_MAGIC,)) + header + payload + struct.pack("<H", checksum)


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

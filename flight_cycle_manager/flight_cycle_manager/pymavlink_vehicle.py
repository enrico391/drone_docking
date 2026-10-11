"""Lightweight, non-blocking pymavlink transport for the mission manager."""

import math
import os
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Dict, Generic, Optional, Sequence, Tuple, TypeVar

from flight_cycle_manager.landing_backend import VelocityFrame
from flight_cycle_manager.mission_contract import MissionItem


_POSITION_TARGET_TYPEMASK_VELOCITY_ONLY = 3527
_POSITION_TARGET_TYPEMASK_VELOCITY_YAW_RATE = 1479
_LANDING_TARGET_TYPE_FIDUCIAL = 2
_STATUSTEXT_MAX_BYTES = 50
_COMMAND_RETRY_S = 0.5
_COMMAND_TIMEOUT_S = 3.0
_MISSION_TYPE_MISSION = 0
_SET_MISSION_CURRENT_MAX_SEQUENCE = 32767
_MAVLINK_UINT16_MAX = 65535
_AUTO_RTL_MODE = 27
_LAND_MODE = 9
_MESSAGE_INTERVALS_US = (
    (32, 50_000),  # LOCAL_POSITION_NED at 20 Hz
    (33, 50_000),  # GLOBAL_POSITION_INT (altitude above home) at 20 Hz
    (30, 50_000),  # ATTITUDE at 20 Hz
    (36, 100_000),  # SERVO_OUTPUT_RAW at 10 Hz (PWM outputs, not thrust)
    (245, 200_000),  # EXTENDED_SYS_STATE at 5 Hz
    (42, 200_000),  # MISSION_CURRENT at 5 Hz
    (1, 500_000),  # SYS_STATUS at 2 Hz
)


def _optional_int(value: object) -> Optional[int]:
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    return None


def _optional_nonzero_int(value: object) -> Optional[int]:
    parsed = _optional_int(value)
    return parsed if parsed else None


def _message_mission_type(message: Any) -> Optional[int]:
    value = getattr(message, "mission_type", _MISSION_TYPE_MISSION)
    return _optional_int(value)


def _valid_mission_download_sequence(value: object) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 0 <= value <= _MAVLINK_UINT16_MAX
    )


def _valid_mission_download_count(value: object) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 0 <= value <= _MAVLINK_UINT16_MAX
    )


@dataclass(frozen=True)
class VehicleState:
    """Latest flight-controller state decoded from MAVLink telemetry."""

    connected: bool = False
    armed: Optional[bool] = None
    flying: Optional[bool] = None
    mode: Optional[int] = None
    system_status: Optional[int] = None
    position_enu_m: Optional[Tuple[float, float, float]] = None
    velocity_enu_m_s: Optional[Tuple[float, float, float]] = None
    heading_rad: Optional[float] = None
    prearm_ok: Optional[bool] = None
    diagnostic: str = ""
    last_message_at: float = 0.0
    heartbeat_at: float = 0.0
    position_at: float = 0.0
    relative_altitude_m: Optional[float] = None
    relative_altitude_at: float = 0.0
    landed_at: float = 0.0
    mission_seq: Optional[int] = None
    mission_total: Optional[int] = None
    mission_state: Optional[int] = None
    mission_mode: Optional[int] = None
    mission_id: Optional[int] = None
    mission_current_at: float = 0.0
    position_time_boot_ms: Optional[int] = None
    attitude_ned_rad: Optional[Tuple[float, float, float]] = None
    angular_velocity_body_frd_rad_s: Optional[Tuple[float, float, float]] = None
    attitude_time_boot_ms: Optional[int] = None
    attitude_at: float = 0.0
    actuator_outputs_pwm_us: Optional[Tuple[Optional[int], ...]] = None
    actuator_time_usec: Optional[int] = None
    actuator_at: float = 0.0


@dataclass(frozen=True)
class VehicleCommandResult:
    """Result returned by the future-like command compatibility API."""

    success: bool
    message: str = ""
    current_mode: Optional[int] = None
    parameters: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class MissionDownloadResult:
    """Mission items read from the flight controller without modifying it."""

    success: bool
    items: Tuple[MissionItem, ...] = ()
    opaque_id: Optional[int] = None
    message: str = ""


_ResultT = TypeVar("_ResultT")


class MavlinkFuture(Generic[_ResultT]):
    """Small future used by the existing asynchronous mission state machine."""

    def __init__(self) -> None:
        self._done = False
        self._result: Optional[_ResultT] = None

    def done(self) -> bool:
        return self._done

    def result(self) -> _ResultT:
        if not self._done or self._result is None:
            raise RuntimeError("MAVLink operation has not completed")
        return self._result

    def _finish(self, result: _ResultT) -> None:
        self._result = result
        self._done = True


@dataclass
class _PendingOperation:
    name: str
    future: MavlinkFuture[Any]
    started_at: float
    command: Optional[int] = None
    payload: Tuple[float, ...] = ()
    expected_value: Optional[object] = None
    parameter_names: Tuple[str, ...] = ()
    parameters: Dict[str, float] = field(default_factory=dict)
    mission_count: Optional[int] = None
    mission_items: Dict[int, MissionItem] = field(default_factory=dict)
    next_mission_request_at: float = 0.0
    mission_type: int = _MISSION_TYPE_MISSION
    opaque_id: Optional[int] = None
    acknowledged: bool = False
    last_sent_at: float = 0.0


def load_mavutil():
    """Load pymavlink using MAVLink 2 and the ArduPilot dialect."""
    os.environ["MAVLINK20"] = "1"
    try:
        from pymavlink import mavutil
    except ImportError as error:
        raise RuntimeError(
            "flight_cycle_manager requires pymavlink>=2.4.49,<3"
        ) from error
    mavutil.set_dialect("ardupilotmega")
    return mavutil


class MavlinkVehicle:
    """Own one pymavlink connection and expose mission-oriented operations."""

    def __init__(
        self,
        *,
        connection: str,
        baud: int = 921600,
        target_system: int = 1,
        target_component: int = 1,
        source_system: int = 254,
        source_component: int = 191,
        connection_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        if not isinstance(connection, str) or not connection:
            raise ValueError("mavlink_connection must not be empty")
        for name, value in (
            ("target_system", target_system),
            ("target_component", target_component),
            ("source_system", source_system),
            ("source_component", source_component),
        ):
            if not isinstance(value, int) or not 0 <= value <= 255:
                raise ValueError(f"{name} must be in [0, 255]")
        if target_system == 0 or source_system == 0:
            raise ValueError("target_system and source_system must be nonzero")

        self._mavutil = load_mavutil()
        factory = connection_factory or self._mavutil.mavlink_connection
        self._connection = factory(
            connection,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            dialect="ardupilotmega",
            autoreconnect=True,
            robust_parsing=True,
        )
        self._target_system = target_system
        self._target_component = target_component
        self._source_system = source_system
        self._source_component = source_component
        self._state = VehicleState()
        self._pending: Optional[_PendingOperation] = None
        self._streams_configured = False
        self._last_heartbeat_sent_at = 0.0
        self._last_velocity_frame = VelocityFrame.MAP_ENU
        self._closed = False

    @property
    def state(self) -> VehicleState:
        return self._state

    def ready(self, now: float, heartbeat_timeout_s: float = 2.5) -> bool:
        """Report whether a recent heartbeat identifies an operational FC."""
        mavlink = self._mavutil.mavlink
        return (
            self._state.connected
            and 0.0 <= now - self._state.heartbeat_at <= heartbeat_timeout_s
            and self._state.system_status
            in (mavlink.MAV_STATE_STANDBY, mavlink.MAV_STATE_ACTIVE)
        )

    def poll(self, now: float) -> None:
        """Drain available messages and advance the active transaction."""
        self.receive_telemetry(now)
        self.service_pending(now)

    def receive_telemetry(self, now: float) -> None:
        """Drain available telemetry without retrying or timing out commands."""
        if self._closed:
            return
        self._send_companion_heartbeat(now)
        while True:
            message = self._connection.recv_match(blocking=False)
            if message is None:
                break
            if not self._matches_source(message):
                continue
            self._handle_message(message, now)

    def service_pending(self, now: float) -> None:
        """Advance the active transaction after higher-level authority checks."""
        if self._closed:
            return
        self._advance_pending(now)

    def prearm_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        return self._begin_command(
            "prearm",
            self._mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
            (0.0,) * 7,
            now,
        )

    def set_mode_async(
        self, mode: int, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        return self._begin_command(
            "mode",
            self._mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            (
                float(self._mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                float(mode),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            now,
            expected_value=int(mode),
        )

    def arm_async(
        self, arm: bool, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        return self._begin_command(
            "arm",
            self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1.0 if arm else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            now,
            expected_value=bool(arm),
        )

    def takeoff_async(
        self, altitude_m: float, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        return self._begin_command(
            "takeoff",
            self._mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            (0.0, 0.0, 0.0, math.nan, 0.0, 0.0, float(altitude_m)),
            now,
        )

    def set_mission_current_async(
        self, sequence: int, *, reset: bool, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        """Set the active onboard mission sequence through COMMAND_LONG."""
        if (
            not isinstance(sequence, int)
            or isinstance(sequence, bool)
            or not 0 <= sequence <= _SET_MISSION_CURRENT_MAX_SEQUENCE
        ):
            raise ValueError("mission sequence must be an integer in [0, 32767]")
        return self._begin_command(
            "set_mission_current",
            self._mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            (
                float(sequence),
                1.0 if reset else 0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            now,
        )

    def start_mission_async(
        self, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        """Start the current onboard mission from its current sequence."""
        return self._begin_command(
            "start_mission",
            self._mavutil.mavlink.MAV_CMD_MISSION_START,
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            now,
        )

    def request_dock_async(
        self, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        """Command the onboard mission to begin its landing sequence."""
        return self._begin_command(
            "request_dock",
            self._mavutil.mavlink.MAV_CMD_DO_LAND_START,
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            now,
        )

    def rtl_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        """Command a return-to-launch fallback."""
        return self._begin_command(
            "rtl",
            self._mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            now,
            expected_value=_AUTO_RTL_MODE,
        )

    def land_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        """Command a direct landing fallback."""
        return self._begin_command(
            "land",
            self._mavutil.mavlink.MAV_CMD_NAV_LAND,
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            now,
            expected_value=_LAND_MODE,
        )

    def download_mission_async(
        self, now: float
    ) -> MavlinkFuture[MissionDownloadResult]:
        """Begin a non-blocking, read-only MISSION_ITEM_INT download."""
        self._require_idle()
        future: MavlinkFuture[MissionDownloadResult] = MavlinkFuture()
        self._pending = _PendingOperation(
            name="mission_download",
            future=future,
            started_at=now,
            mission_type=_MISSION_TYPE_MISSION,
        )
        self._send_mission_request_list(now)
        return future

    def get_parameters_async(
        self, names: Sequence[str], now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        normalized = tuple(names)
        if (
            not normalized
            or len(set(normalized)) != len(normalized)
            or any(
                not isinstance(name, str)
                or not name
                or not name.isascii()
                or len(name.encode("ascii")) > 16
                for name in normalized
            )
        ):
            raise ValueError("parameter names must be unique 1-16 byte ASCII")
        self._require_idle()
        future: MavlinkFuture[VehicleCommandResult] = MavlinkFuture()
        self._pending = _PendingOperation(
            name="parameters",
            future=future,
            started_at=now,
            parameter_names=normalized,
        )
        self._send_parameter_requests(normalized)
        self._pending.last_sent_at = now
        return future

    def send_velocity(
        self,
        frame: VelocityFrame,
        velocity_x: float,
        velocity_y: float,
        velocity_z: float,
        *,
        now: float,
        yaw_rate_rad_s: Optional[float] = None,
    ) -> bool:
        """Send one SET_POSITION_TARGET_LOCAL_NED velocity setpoint."""
        if self._closed:
            return False
        if not all(
            math.isfinite(value) for value in (velocity_x, velocity_y, velocity_z)
        ):
            raise ValueError("velocity values must be finite")
        if yaw_rate_rad_s is not None and not math.isfinite(yaw_rate_rad_s):
            raise ValueError("yaw_rate_rad_s must be finite")
        if frame is VelocityFrame.MAP_ENU:
            mavlink_frame = self._mavutil.mavlink.MAV_FRAME_LOCAL_NED
            north, east, down = velocity_y, velocity_x, -velocity_z
        elif frame is VelocityFrame.BODY_FLU:
            mavlink_frame = self._mavutil.mavlink.MAV_FRAME_BODY_NED
            north, east, down = velocity_x, -velocity_y, -velocity_z
        else:
            raise ValueError(f"unsupported velocity frame: {frame}")
        self._connection.mav.set_position_target_local_ned_send(
            int(now * 1000.0) & 0xFFFFFFFF,
            self._target_system,
            self._target_component,
            mavlink_frame,
            (
                _POSITION_TARGET_TYPEMASK_VELOCITY_ONLY
                if yaw_rate_rad_s is None
                else _POSITION_TARGET_TYPEMASK_VELOCITY_YAW_RATE
            ),
            0.0,
            0.0,
            0.0,
            float(north),
            float(east),
            float(down),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0 if yaw_rate_rad_s is None else -float(yaw_rate_rad_s),
        )
        self._last_velocity_frame = frame
        return True

    def send_landing_target(
        self,
        *,
        time_usec: int,
        forward_m: float,
        right_m: float,
        down_m: float,
    ) -> bool:
        """Send a position-valid Body-FRD LANDING_TARGET."""
        if self._closed:
            return False
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
            _LANDING_TARGET_TYPE_FIDUCIAL,
            1,
        )
        return True

    def send_condition_yaw(self, *, yaw_error_rad: float) -> bool:
        """Send the existing relative dock-yaw correction via pymavlink."""
        if self._closed:
            return False
        direction = 1.0 if yaw_error_rad >= 0.0 else -1.0
        self._connection.mav.command_long_send(
            self._target_system,
            self._target_component,
            self._mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            abs(math.degrees(yaw_error_rad)),
            0.0,
            direction,
            1.0,
            0.0,
            0.0,
            0.0,
        )
        return True

    def send_status_text(self, text: str) -> bool:
        """Send one informational MAVLink status message."""
        if self._closed:
            return False
        payload = text.encode("ascii", errors="replace")[
            :_STATUSTEXT_MAX_BYTES
        ]
        self._connection.mav.statustext_send(
            self._mavutil.mavlink.MAV_SEVERITY_INFO,
            payload,
        )
        return True

    def close(self) -> None:
        """Send a final zero setpoint and release the connection."""
        self._close(send_final_zero=True)

    def close_without_output(self) -> None:
        """Release the connection without sending any final flight command."""
        self._close(send_final_zero=False)

    def cancel_pending_operation(self) -> None:
        """Drop transport ownership of the active operation and ignore late ACKs."""
        self._pending = None

    def _close(self, *, send_final_zero: bool) -> None:
        if self._closed:
            return
        if send_final_zero:
            self.send_velocity(
                self._last_velocity_frame, 0.0, 0.0, 0.0, now=0.0
            )
        self._connection.close()
        self._closed = True
        self._state = replace(self._state, connected=False)

    def _begin_command(
        self,
        name: str,
        command: int,
        payload: Tuple[float, ...],
        now: float,
        expected_value: Optional[object] = None,
    ) -> MavlinkFuture[VehicleCommandResult]:
        self._require_idle()
        future: MavlinkFuture[VehicleCommandResult] = MavlinkFuture()
        self._pending = _PendingOperation(
            name=name,
            future=future,
            started_at=now,
            command=command,
            payload=payload,
            expected_value=expected_value,
        )
        self._send_pending_command(now)
        return future

    def _require_idle(self) -> None:
        if self._pending is not None and not self._pending.future.done():
            raise RuntimeError("another MAVLink operation is already active")
        self._pending = None

    def _send_pending_command(self, now: float) -> None:
        pending = self._pending
        if pending is None or pending.command is None:
            return
        self._connection.mav.command_long_send(
            self._target_system,
            self._target_component,
            pending.command,
            0,
            *pending.payload,
        )
        pending.last_sent_at = now

    def _send_parameter_requests(self, names: Sequence[str]) -> None:
        for name in names:
            self._connection.mav.param_request_read_send(
                self._target_system,
                self._target_component,
                name.encode("ascii"),
                -1,
            )

    def _send_mission_request_list(self, now: float) -> None:
        pending = self._pending
        if pending is None:
            return
        self._connection.mav.mission_request_list_send(
            self._target_system,
            self._target_component,
            pending.mission_type,
        )
        pending.last_sent_at = now

    def _send_next_missing_mission_request(self, now: float) -> None:
        pending = self._pending
        if pending is None or pending.mission_count is None:
            return
        sequence = next(
            (
                sequence
                for sequence in range(pending.mission_count)
                if sequence not in pending.mission_items
            ),
            None,
        )
        if sequence is None:
            return
        self._connection.mav.mission_request_int_send(
            self._target_system,
            self._target_component,
            sequence,
            pending.mission_type,
        )
        pending.last_sent_at = now
        pending.next_mission_request_at = now + _COMMAND_RETRY_S

    def _advance_pending(self, now: float) -> None:
        pending = self._pending
        if pending is None or pending.future.done():
            return
        self._try_complete_pending()
        if pending.future.done():
            return
        if now - pending.started_at > _COMMAND_TIMEOUT_S:
            if pending.name == "mission_download":
                pending.future._finish(
                    MissionDownloadResult(False, message="MAVLink operation timed out")
                )
            elif pending.name == "mode":
                observed_mode = (
                    "unknown" if self._state.mode is None else str(self._state.mode)
                )
                if pending.acknowledged:
                    message = (
                        "MAVLink mode change acknowledged but timed out "
                        f"waiting for mode {pending.expected_value}; "
                        f"observed mode {observed_mode}"
                    )
                else:
                    message = (
                        "MAVLink mode change timed out waiting for COMMAND_ACK; "
                        f"expected mode {pending.expected_value}, "
                        f"observed mode {observed_mode}"
                    )
                pending.future._finish(
                    VehicleCommandResult(
                        False,
                        message,
                        current_mode=self._state.mode,
                    )
                )
            else:
                pending.future._finish(
                    VehicleCommandResult(False, "MAVLink operation timed out")
                )
            return
        if now - pending.last_sent_at < _COMMAND_RETRY_S:
            return
        if pending.name == "parameters":
            missing = tuple(
                name
                for name in pending.parameter_names
                if name not in pending.parameters
            )
            self._send_parameter_requests(missing)
            pending.last_sent_at = now
        elif pending.name == "mission_download":
            if pending.mission_count is None:
                self._send_mission_request_list(now)
            elif now >= pending.next_mission_request_at:
                self._send_next_missing_mission_request(now)
        elif not pending.acknowledged:
            self._send_pending_command(now)

    def _try_complete_pending(self) -> None:
        pending = self._pending
        if pending is None or pending.future.done():
            return
        if pending.name == "parameters":
            if len(pending.parameters) == len(pending.parameter_names):
                ordered = {
                    name: pending.parameters[name]
                    for name in pending.parameter_names
                }
                pending.future._finish(
                    VehicleCommandResult(True, parameters=ordered)
                )
            return
        if pending.name == "mission_download":
            if (
                pending.mission_count is not None
                and len(pending.mission_items) == pending.mission_count
                and not pending.acknowledged
            ):
                self._connection.mav.mission_ack_send(
                    self._target_system,
                    self._target_component,
                    self._mavutil.mavlink.MAV_MISSION_ACCEPTED,
                    pending.mission_type,
                )
                pending.acknowledged = True
                pending.future._finish(
                    MissionDownloadResult(
                        True,
                        tuple(
                            pending.mission_items[sequence]
                            for sequence in range(pending.mission_count)
                        ),
                        pending.opaque_id,
                    )
                )
            return
        if not pending.acknowledged:
            return
        if pending.name in {"mode", "rtl", "land"}:
            if self._state.mode == pending.expected_value:
                pending.future._finish(
                    VehicleCommandResult(
                        True, current_mode=self._state.mode
                    )
                )
        elif pending.name == "arm":
            if self._state.armed is pending.expected_value:
                pending.future._finish(VehicleCommandResult(True))
        elif pending.name == "prearm":
            if self._state.prearm_ok is not None:
                pending.future._finish(
                    VehicleCommandResult(
                        self._state.prearm_ok,
                        "pre-arm checks failed"
                        if not self._state.prearm_ok
                        else "",
                    )
                )
        else:
            pending.future._finish(VehicleCommandResult(True))

    def _handle_message(self, message: Any, now: float) -> None:
        message_type = message.get_type()
        state = replace(
            self._state,
            connected=True,
            last_message_at=max(self._state.last_message_at, now),
        )
        mavlink = self._mavutil.mavlink
        if message_type == "HEARTBEAT":
            state = replace(
                state,
                armed=bool(
                    message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ),
                mode=int(message.custom_mode),
                system_status=int(message.system_status),
                heartbeat_at=max(state.heartbeat_at, now),
            )
            if not self._streams_configured:
                self._configure_message_intervals()
                self._streams_configured = True
        elif message_type == "LOCAL_POSITION_NED":
            state = replace(
                state,
                position_enu_m=(float(message.y), float(message.x), -float(message.z)),
                velocity_enu_m_s=(
                    float(message.vy),
                    float(message.vx),
                    -float(message.vz),
                ),
                position_at=max(state.position_at, now),
                position_time_boot_ms=_optional_int(getattr(message, "time_boot_ms", None)),
            )
        elif message_type == "GLOBAL_POSITION_INT":
            state = replace(
                state,
                relative_altitude_m=float(message.relative_alt) * 0.001,
                relative_altitude_at=max(state.relative_altitude_at, now),
            )
        elif message_type == "ATTITUDE":
            attitude = tuple(getattr(message, axis, None) for axis in ("roll", "pitch", "yaw"))
            rates = tuple(getattr(message, axis, None)
                          for axis in ("rollspeed", "pitchspeed", "yawspeed"))
            state = replace(
                state, heading_rad=float(message.yaw),
                attitude_ned_rad=attitude if all(
                    value is not None and math.isfinite(value) for value in attitude
                ) else None,
                angular_velocity_body_frd_rad_s=rates if all(
                    value is not None and math.isfinite(value) for value in rates
                ) else None,
                attitude_time_boot_ms=_optional_int(getattr(message, "time_boot_ms", None)),
                attitude_at=max(state.attitude_at, now),
            )
        elif message_type == "SERVO_OUTPUT_RAW" and int(message.port) == 0:
            state = replace(
                state,
                actuator_outputs_pwm_us=tuple(
                    _optional_int(getattr(message, f"servo{channel}_raw", None))
                    for channel in range(1, 17)
                ),
                actuator_time_usec=_optional_int(getattr(message, "time_usec", None)),
                actuator_at=max(state.actuator_at, now),
            )
        elif message_type == "EXTENDED_SYS_STATE":
            landed_state = int(message.landed_state)
            flying = {
                mavlink.MAV_LANDED_STATE_ON_GROUND: False,
                mavlink.MAV_LANDED_STATE_TAKEOFF: True,
                mavlink.MAV_LANDED_STATE_IN_AIR: True,
                mavlink.MAV_LANDED_STATE_LANDING: True,
            }.get(landed_state)
            state = replace(
                state,
                flying=flying,
                landed_at=max(state.landed_at, now),
            )
        elif message_type == "SYS_STATUS":
            bit = mavlink.MAV_SYS_STATUS_PREARM_CHECK
            state = replace(
                state,
                prearm_ok=bool(
                    message.onboard_control_sensors_present & bit
                    and message.onboard_control_sensors_enabled & bit
                    and message.onboard_control_sensors_health & bit
                ),
            )
        elif message_type == "MISSION_CURRENT":
            state = replace(
                state,
                mission_seq=int(message.seq),
                mission_total=int(message.total),
                mission_state=_optional_int(
                    getattr(message, "mission_state", None)
                ),
                mission_mode=_optional_int(
                    getattr(message, "mission_mode", None)
                ),
                mission_id=_optional_nonzero_int(
                    getattr(message, "mission_id", None)
                ),
                mission_current_at=max(state.mission_current_at, now),
            )
        elif message_type == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.split(b"\0", 1)[0].decode(errors="replace")
            state = replace(state, diagnostic=str(text).split("\0", 1)[0])
        self._state = state

        if message_type == "COMMAND_ACK":
            self._handle_command_ack(message)
        elif message_type == "PARAM_VALUE":
            self._handle_parameter_value(message)
        elif message_type == "MISSION_COUNT":
            self._handle_mission_count(message, now)
        elif message_type == "MISSION_ITEM_INT":
            self._handle_mission_item_int(message, now)
        self._try_complete_pending()

    def _handle_command_ack(self, message: Any) -> None:
        pending = self._pending
        if (
            pending is None
            or pending.future.done()
            or pending.command != int(message.command)
        ):
            return
        result = int(message.result)
        if result in (
            self._mavutil.mavlink.MAV_RESULT_ACCEPTED,
            self._mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            pending.acknowledged = True
            return
        pending.future._finish(
            VehicleCommandResult(
                False,
                f"ArduPilot rejected MAVLink command {pending.command} "
                f"with result {result}",
                current_mode=self._state.mode,
            )
        )

    def _handle_parameter_value(self, message: Any) -> None:
        pending = self._pending
        if pending is None or pending.name != "parameters":
            return
        name = message.param_id
        if isinstance(name, bytes):
            name = name.split(b"\0", 1)[0].decode("ascii", errors="ignore")
        else:
            name = str(name).split("\0", 1)[0]
        if name in pending.parameter_names:
            pending.parameters[name] = float(message.param_value)

    def _handle_mission_count(self, message: Any, now: float) -> None:
        pending = self._pending
        if (
            pending is None
            or pending.name != "mission_download"
            or pending.mission_count is not None
            or _message_mission_type(message) != pending.mission_type
        ):
            return
        count = getattr(message, "count", None)
        if not _valid_mission_download_count(count):
            return
        pending.mission_count = int(count)
        opaque_id = _optional_nonzero_int(getattr(message, "opaque_id", None))
        pending.opaque_id = opaque_id
        if pending.mission_count:
            self._send_next_missing_mission_request(now)

    def _handle_mission_item_int(self, message: Any, now: float) -> None:
        pending = self._pending
        if (
            pending is None
            or pending.name != "mission_download"
            or pending.mission_count is None
            or _message_mission_type(message) != pending.mission_type
        ):
            return
        sequence = getattr(message, "seq", None)
        if (
            not _valid_mission_download_sequence(sequence)
            or sequence >= pending.mission_count
            or sequence in pending.mission_items
        ):
            self._send_next_missing_mission_request(now)
            return
        try:
            item = MissionItem(
                int(sequence),
                int(message.frame),
                int(message.command),
                float(message.param1),
                float(message.param2),
                float(message.param3),
                float(message.param4),
                int(message.x),
                int(message.y),
                float(message.z),
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._send_next_missing_mission_request(now)
            return
        pending.mission_items[item.seq] = item
        self._send_next_missing_mission_request(now)

    def _matches_source(self, message: Any) -> bool:
        return (
            message.get_srcSystem() == self._target_system
            and message.get_srcComponent() == self._target_component
        )

    def _send_companion_heartbeat(self, now: float) -> None:
        if self._last_heartbeat_sent_at and now < self._last_heartbeat_sent_at + 1.0:
            return
        mavlink = self._mavutil.mavlink
        self._connection.mav.heartbeat_send(
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
            3,
        )
        self._last_heartbeat_sent_at = now

    def _configure_message_intervals(self) -> None:
        mavlink = self._mavutil.mavlink
        for message_id, interval_us in _MESSAGE_INTERVALS_US:
            self._connection.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                float(interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

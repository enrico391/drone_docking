"""Reusable SITL mission fixture, uploader, and ROS acceptance harness."""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, Optional, Sequence

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))

from flight_cycle_manager.mission_contract import MissionItem
from flight_cycle_manager.pymavlink_vehicle import load_mavutil


_PRIMARY_CONNECTION = "tcp:127.0.0.1:5760"
_OBSERVER_CONNECTION = "tcp:127.0.0.1:5762"
_AUTO_MODE = 3
_GUIDED_MODE = 4
_LOITER_MODE = 5
_LAND_MODE = 9
_AUTO_RTL_MODE = 27
_STABILIZE_MODE = 0
_ARMED_FLAG = 128
_QGC_WPL_HEADER = "QGC WPL 110"
_PROCESS_TAIL_LINES = 120
_GPS_READY_STATUSTEXT = "EKF3 IMU0 is using GPS"
_MAX_PREFLIGHT_BOOT_ATTEMPTS = 3


@dataclass(frozen=True)
class FixtureMissionItem:
    """One uploadable QGC WPL mission row."""

    seq: int
    current: int
    frame: int
    command: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float
    autocontinue: int

    def as_contract_item(self) -> MissionItem:
        return MissionItem(
            self.seq,
            self.frame,
            self.command,
            self.param1,
            self.param2,
            self.param3,
            self.param4,
            self.x,
            self.y,
            self.z,
        )


class MissionUploadError(RuntimeError):
    """Raised when the preflight mission upload protocol fails."""


class HarnessError(RuntimeError):
    """Raised when the ROS or process harness cannot satisfy a test action."""


def _wait_ready_heartbeat(
    connection: object,
    *,
    mavutil: object,
    timeout_s: float,
) -> object:
    deadline = time.monotonic() + timeout_s
    ready_states = {
        mavutil.mavlink.MAV_STATE_STANDBY,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    }
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise MissionUploadError(
                "timed out waiting for a ready flight-controller heartbeat"
            )
        connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        heartbeat = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=min(1.0, remaining),
        )
        if heartbeat is None:
            continue
        system_status = getattr(heartbeat, "system_status", None)
        if system_status in ready_states:
            return heartbeat


def _decode_statustext(message: object) -> str:
    text = getattr(message, "text", "")
    if isinstance(text, bytes):
        return text.split(b"\0", 1)[0].decode(errors="replace")
    return str(text)


def _wait_for_status_text(
    connection: object,
    *,
    timeout_s: float,
    expected_substring: str,
) -> None:
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise MissionUploadError(
                "timed out waiting for a GPS-ready flight-controller status text"
            )
        message = connection.recv_match(
            blocking=True,
            timeout=min(1.0, remaining),
        )
        if message is None or message.get_type() != "STATUSTEXT":
            continue
        if expected_substring in _decode_statustext(message):
            return


def _fixture_path() -> Path:
    return Path(__file__).resolve().parent / "data" / "docking_contract.waypoints"


def load_valid_docking_mission() -> tuple[FixtureMissionItem, ...]:
    """Parse the exact checked-in QGC WPL 110 docking mission fixture."""
    return load_qgc_wpl_fixture(_fixture_path())


def load_qgc_wpl_fixture(path: Path) -> tuple[FixtureMissionItem, ...]:
    """Load a narrow QGC WPL 110 fixture into uploadable mission rows."""
    with path.open("r", encoding="utf-8") as fixture:
        header = fixture.readline().strip()
        if header != _QGC_WPL_HEADER:
            raise MissionUploadError(
                f"expected {_QGC_WPL_HEADER!r} header, got {header!r}"
            )

        items = []
        for line_number, raw_line in enumerate(fixture, start=2):
            line = raw_line.strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 12:
                raise MissionUploadError(
                    f"line {line_number} has {len(fields)} fields; expected 12"
                )
            seq, current, frame, command = (int(value) for value in fields[:4])
            param1, param2, param3, param4 = (
                float(value) for value in fields[4:8]
            )
            latitude = int(round(float(fields[8]) * 10_000_000.0))
            longitude = int(round(float(fields[9]) * 10_000_000.0))
            altitude = float(fields[10])
            autocontinue = int(fields[11])
            items.append(
                FixtureMissionItem(
                    seq=seq,
                    current=current,
                    frame=frame,
                    command=command,
                    param1=param1,
                    param2=param2,
                    param3=param3,
                    param4=param4,
                    x=latitude,
                    y=longitude,
                    z=altitude,
                    autocontinue=autocontinue,
                )
            )
    return tuple(items)


def items_without_nav_land() -> tuple[FixtureMissionItem, ...]:
    """Break caught: the fixture can lose its fallback NAV_LAND and still pass."""
    items = load_valid_docking_mission()
    if not items or items[-1].command != 21:
        raise MissionUploadError("valid fixture no longer ends with NAV_LAND")
    return items[:-1]


class MissionUploader:
    """Upload a mission over one short-lived preflight pymavlink connection."""

    def __init__(
        self,
        *,
        connection_string: str = _PRIMARY_CONNECTION,
        timeout_s: float = 30.0,
        source_system: int = 252,
        source_component: int = 1,
        connection_factory: Optional[Callable[..., object]] = None,
    ) -> None:
        self._connection_string = connection_string
        self._timeout_s = timeout_s
        self._source_system = source_system
        self._source_component = source_component
        self._mavutil = load_mavutil()
        self._connection_factory = (
            connection_factory or self._mavutil.mavlink_connection
        )

    def upload(self, items: Sequence[FixtureMissionItem]) -> None:
        mission = tuple(items)
        if not mission:
            raise MissionUploadError("cannot upload an empty mission")

        connection = self._connection_factory(
            self._connection_string,
            source_system=self._source_system,
            source_component=self._source_component,
            dialect="ardupilotmega",
            autoreconnect=False,
            robust_parsing=True,
        )
        try:
            heartbeat = _wait_ready_heartbeat(
                connection,
                mavutil=self._mavutil,
                timeout_s=self._timeout_s,
            )
            target_system = int(heartbeat.get_srcSystem())
            target_component = int(heartbeat.get_srcComponent())
            connection.mav.mission_count_send(
                target_system,
                target_component,
                len(mission),
            )
            deadline = time.monotonic() + self._timeout_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise MissionUploadError(
                        "timed out waiting for mission upload completion"
                    )
                message = connection.recv_match(blocking=True, timeout=remaining)
                if message is None:
                    raise MissionUploadError(
                        "mission upload timed out waiting for FC requests"
                    )
                message_type = message.get_type()
                if message_type in {"MISSION_REQUEST", "MISSION_REQUEST_INT"}:
                    self._send_requested_item(
                        connection,
                        mission,
                        target_system,
                        target_component,
                        int(message.seq),
                    )
                    continue
                if message_type != "MISSION_ACK":
                    continue
                result = int(getattr(message, "type", -1))
                if result != self._mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise MissionUploadError(
                        f"mission upload was rejected with MAV_MISSION result {result}"
                    )
                return
        finally:
            connection.close()

    @staticmethod
    def _send_requested_item(
        connection: object,
        mission: tuple[FixtureMissionItem, ...],
        target_system: int,
        target_component: int,
        requested_seq: int,
    ) -> None:
        if not 0 <= requested_seq < len(mission):
            raise MissionUploadError(
                f"flight controller requested mission item {requested_seq}, "
                f"but mission has only {len(mission)} items"
            )
        item = mission[requested_seq]
        connection.mav.mission_item_int_send(
            target_system,
            target_component,
            item.seq,
            item.frame,
            item.command,
            item.current,
            item.autocontinue,
            item.param1,
            item.param2,
            item.param3,
            item.param4,
            item.x,
            item.y,
            item.z,
        )


class PreflightVehicleGate:
    """Wait for the simulator FC to become upload- and arm-ready."""

    def __init__(
        self,
        *,
        connection_string: str = _PRIMARY_CONNECTION,
        timeout_s: float = 60.0,
        source_system: int = 251,
        source_component: int = 1,
        connection_factory: Optional[Callable[..., object]] = None,
        gps_ready_text: str = _GPS_READY_STATUSTEXT,
    ) -> None:
        self._connection_string = connection_string
        self._timeout_s = timeout_s
        self._source_system = source_system
        self._source_component = source_component
        self._gps_ready_text = gps_ready_text
        self._mavutil = load_mavutil()
        self._connection_factory = (
            connection_factory or self._mavutil.mavlink_connection
        )

    def wait_until_ready(self) -> None:
        connection = self._connection_factory(
            self._connection_string,
            source_system=self._source_system,
            source_component=self._source_component,
            dialect="ardupilotmega",
            autoreconnect=False,
            robust_parsing=True,
        )
        try:
            _wait_ready_heartbeat(
                connection,
                mavutil=self._mavutil,
                timeout_s=self._timeout_s,
            )
            _wait_for_status_text(
                connection,
                timeout_s=self._timeout_s,
                expected_substring=self._gps_ready_text,
            )
        finally:
            connection.close()


class ManagedLaunchProcess:
    """Track one launched process group and its recent combined output."""

    def __init__(self, process: subprocess.Popen[str], log_path: Path) -> None:
        self.process = process
        self.process_group_id = os.getpgid(process.pid)
        self.log_path = log_path
        self._tail: deque[str] = deque(maxlen=_PROCESS_TAIL_LINES)
        self._capture_thread = threading.Thread(
            target=self._capture_output,
            name=f"log-tail-{process.pid}",
            daemon=True,
        )
        self._capture_thread.start()

    @classmethod
    def start(
        cls,
        command: Sequence[str],
        *,
        env: Mapping[str, str],
        cwd: Path,
        log_name: str,
    ) -> "ManagedLaunchProcess":
        log_path = cwd / f"{log_name}.log"
        process = subprocess.Popen(
            list(command),
            cwd=str(cwd),
            env=dict(env),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        return cls(process, log_path)

    def _capture_output(self) -> None:
        stream = self.process.stdout
        if stream is None:
            return
        with self.log_path.open("w", encoding="utf-8") as log_file:
            for line in stream:
                self._tail.append(line.rstrip("\n"))
                log_file.write(line)
                log_file.flush()

    def poll(self) -> Optional[int]:
        return self.process.poll()

    def _process_group_alive(self) -> bool:
        try:
            os.killpg(self.process_group_id, 0)
        except ProcessLookupError:
            return False
        return True

    def terminate(self, timeout_s: float = 20.0) -> None:
        if not self._process_group_alive():
            if self.process.stdout is not None:
                self.process.stdout.close()
            self._capture_thread.join(timeout=1.0)
            return
        for signal_name, grace_s in (
            (signal.SIGINT, timeout_s / 2.0),
            (signal.SIGTERM, timeout_s / 2.0),
            (signal.SIGKILL, 1.0),
        ):
            if not self._process_group_alive():
                break
            try:
                os.killpg(self.process_group_id, signal_name)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + grace_s
            while time.monotonic() < deadline:
                if not self._process_group_alive():
                    break
                if self.poll() is None:
                    try:
                        self.process.wait(timeout=0.1)
                    except subprocess.TimeoutExpired:
                        pass
                time.sleep(0.05)
        if self.process.stdout is not None:
            self.process.stdout.close()
        self._capture_thread.join(timeout=1.0)

    def wait_for_output(self, pattern: str, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if any(pattern in line for line in self._tail):
                return
            code = self.poll()
            if code is not None:
                raise HarnessError(
                    f"process exited with code {code} before output contained "
                    f"{pattern!r}\n{self.tail()}"
                )
            time.sleep(0.1)
        raise HarnessError(
            f"timed out waiting for output {pattern!r}\n{self.tail()}"
        )

    def tail(self, lines: int = 40) -> str:
        excerpt = list(self._tail)[-lines:]
        return "\n".join(excerpt)


@dataclass
class _ProcessConflict:
    pid: int
    command: str


def _find_workspace_root(start: Path) -> Path:
    current = start.resolve()
    for candidate in (current, *current.parents):
        if (candidate / "install" / "setup.bash").is_file():
            return candidate
    raise HarnessError("could not locate workspace root containing install/setup.bash")


def _quoted(path: Path) -> str:
    return str(path).replace("'", "'\"'\"'")


def _find_conflicting_sim_processes() -> list[_ProcessConflict]:
    output = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        check=True,
        text=True,
        capture_output=True,
    ).stdout.splitlines()
    signatures = (
        "ros2 launch drone_bringup docking_sim.launch.py",
        "arducopter --model json",
        "gz sim -v4 -s -r --physics-engine",
    )
    conflicts = []
    for line in output:
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, command = stripped.partition(" ")
        if not pid_text.isdigit():
            continue
        if any(signature in command for signature in signatures):
            conflicts.append(_ProcessConflict(pid=int(pid_text), command=command))
    return conflicts


def _supports_sitl(workspace_root: Path) -> tuple[bool, str]:
    required = [
        workspace_root / "install" / "setup.bash",
        workspace_root / "apriltag_venv" / "bin" / "python",
        _fixture_path(),
    ]
    for path in required:
        if not path.exists():
            return False, f"missing prerequisite: {path}"

    check = subprocess.run(
        [
            "bash",
            "-lc",
            (
                f"source /opt/ros/humble/setup.bash && "
                f"source '{_quoted(workspace_root / 'install' / 'setup.bash')}' && "
                "command -v ros2 arducopter gz >/dev/null"
            ),
        ],
        check=False,
        text=True,
        capture_output=True,
    )
    if check.returncode != 0:
        return False, "ROS or SITL executables are unavailable after sourcing the workspace"
    return True, ""


class _RosProbeNode(Node):
    """Service clients and topic history for the public manager surface."""

    def __init__(self) -> None:
        super().__init__(f"sitl_probe_{os.getpid()}_{time.monotonic_ns()}")
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state_history: list[tuple[float, str]] = []
        self.reason_history: list[tuple[float, str]] = []
        self.dock_mode_history: list[tuple[float, str]] = []
        self.dock_pose_count = 0
        self.create_subscription(
            String,
            "/flight_cycle_manager/state",
            self._on_state,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/flight_cycle_manager/reason",
            self._on_reason,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/dock/mode",
            self._on_dock_mode,
            sensor_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/dock/pose",
            self._on_dock_pose,
            sensor_qos,
        )
        self.start_cycle_client = self.create_client(
            Trigger,
            "/flight_cycle_manager/start_cycle",
        )
        self.request_dock_client = self.create_client(
            Trigger,
            "/flight_cycle_manager/request_dock",
        )

    def _on_state(self, message: String) -> None:
        self.state_history.append((time.monotonic(), message.data))

    def _on_reason(self, message: String) -> None:
        self.reason_history.append((time.monotonic(), message.data))

    def _on_dock_mode(self, message: String) -> None:
        self.dock_mode_history.append((time.monotonic(), message.data))

    def _on_dock_pose(self, message: PoseStamped) -> None:
        del message
        self.dock_pose_count += 1


@dataclass
class _ObserverState:
    mode: Optional[int] = None
    armed: Optional[bool] = None
    flying: Optional[bool] = None
    mission_seq: Optional[int] = None
    mission_total: Optional[int] = None


class _SitlObserver:
    """Read and command the secondary SITL MAVLink port."""

    def __init__(
        self,
        connection_string: str = _OBSERVER_CONNECTION,
        *,
        timeout_s: float = 20.0,
        source_system: int = 253,
        source_component: int = 1,
    ) -> None:
        self._mavutil = load_mavutil()
        self._connection = self._mavutil.mavlink_connection(
            connection_string,
            source_system=source_system,
            source_component=source_component,
            dialect="ardupilotmega",
            autoreconnect=False,
            robust_parsing=True,
        )
        heartbeat = self._connection.wait_heartbeat(timeout=timeout_s)
        self._target_system = int(heartbeat.get_srcSystem())
        self._target_component = int(heartbeat.get_srcComponent())
        self.state = _ObserverState()
        self.mode_history: list[tuple[float, int]] = []
        self._apply_message(heartbeat)
        self.poll()

    def close(self) -> None:
        self._connection.close()

    def poll(self) -> None:
        while True:
            message = self._connection.recv_match(blocking=False)
            if message is None:
                return
            self._apply_message(message)

    def wait_for_mode(self, expected_mode: int, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll()
            if self.state.mode == expected_mode:
                return
            time.sleep(0.1)
        raise HarnessError(
            f"timed out waiting for flight mode {mode_name(expected_mode)}; "
            f"last mode was {mode_name(self.state.mode)}"
        )

    def set_mode(self, custom_mode: int, timeout_s: float = 10.0) -> None:
        self._connection.mav.command_long_send(
            self._target_system,
            self._target_component,
            self._mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            self._mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
            0,
            0,
            0,
            0,
            0,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = self._connection.recv_match(blocking=True, timeout=0.5)
            if message is None:
                self.poll()
                continue
            self._apply_message(message)
            if (
                message.get_type() == "COMMAND_ACK"
                and int(getattr(message, "command", -1))
                == self._mavutil.mavlink.MAV_CMD_DO_SET_MODE
            ):
                result = int(getattr(message, "result", -1))
                if result != self._mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise HarnessError(
                        f"flight controller rejected mode change to "
                        f"{mode_name(custom_mode)} with result {result}"
                    )
            if self.state.mode == custom_mode:
                return
        raise HarnessError(
            f"mode change to {mode_name(custom_mode)} was not observed before timeout"
        )

    def _apply_message(self, message: object) -> None:
        now = time.monotonic()
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            mode = int(getattr(message, "custom_mode", 0))
            if not self.mode_history or self.mode_history[-1][1] != mode:
                self.mode_history.append((now, mode))
            self.state.mode = mode
            base_mode = int(getattr(message, "base_mode", 0))
            self.state.armed = bool(base_mode & _ARMED_FLAG)
        elif message_type == "EXTENDED_SYS_STATE":
            landed_state = int(getattr(message, "landed_state", -1))
            if landed_state == self._mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                self.state.flying = False
            elif landed_state == self._mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
                self.state.flying = True
        elif message_type == "MISSION_CURRENT":
            self.state.mission_seq = int(getattr(message, "seq", 0))
            mission_total = getattr(message, "total", None)
            if mission_total is not None:
                self.state.mission_total = int(mission_total)


def mode_name(mode: Optional[int]) -> str:
    return {
        None: "UNKNOWN",
        _STABILIZE_MODE: "STABILIZE",
        _AUTO_MODE: "AUTO",
        _GUIDED_MODE: "GUIDED",
        _LOITER_MODE: "LOITER",
        _LAND_MODE: "LAND",
        _AUTO_RTL_MODE: "AUTO_RTL",
    }.get(mode, str(mode))


class SitlStack:
    """One isolated simulator + manager acceptance environment."""

    def __init__(self) -> None:
        self.package_root = Path(__file__).resolve().parents[1]
        self.workspace_root = _find_workspace_root(self.package_root)
        supported, reason = _supports_sitl(self.workspace_root)
        if not supported:
            pytest.skip(reason)
        conflicts = _find_conflicting_sim_processes()
        if conflicts:
            detail = ", ".join(
                f"{conflict.pid}:{conflict.command}" for conflict in conflicts
            )
            pytest.skip(f"existing simulator processes detected: {detail}")

        self._temp_root = Path(
            tempfile.mkdtemp(prefix="flight_cycle_manager_sitl_")
        )
        self._ros_log_dir = self._temp_root / "ros_logs"
        self._ros_log_dir.mkdir()
        unique_id = f"{os.getpid()}_{time.monotonic_ns()}"
        self.environment = dict(os.environ)
        self.environment["ROS_DOMAIN_ID"] = str(100 + (os.getpid() % 100))
        self.environment["GZ_PARTITION"] = f"flight_cycle_manager_{unique_id}"
        self.environment["ROS_LOG_DIR"] = str(self._ros_log_dir)
        self.environment["PYTHONUNBUFFERED"] = "1"
        self.environment["RCUTILS_COLORIZED_OUTPUT"] = "0"

        os.environ["ROS_DOMAIN_ID"] = self.environment["ROS_DOMAIN_ID"]
        os.environ["GZ_PARTITION"] = self.environment["GZ_PARTITION"]
        os.environ["ROS_LOG_DIR"] = self.environment["ROS_LOG_DIR"]
        if not rclpy.ok():
            rclpy.init()
        self.node = _RosProbeNode()

        self._sim_process: Optional[ManagedLaunchProcess] = None
        self._manager_process: Optional[ManagedLaunchProcess] = None
        self._observer: Optional[_SitlObserver] = None
        self._start_simulator()

    def close(self) -> None:
        if self._observer is not None:
            self._observer.close()
            self._observer = None
        if self._manager_process is not None:
            self._manager_process.terminate()
            self._manager_process = None
        if self._sim_process is not None:
            self._sim_process.terminate()
            self._sim_process = None
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        shutil.rmtree(self._temp_root, ignore_errors=True)

    def upload_mission(self, items: Sequence[FixtureMissionItem]) -> None:
        if self._manager_process is not None:
            raise HarnessError("mission upload is preflight-only; stop the manager first")
        retryable_failures: list[str] = []
        for attempt in range(1, _MAX_PREFLIGHT_BOOT_ATTEMPTS + 1):
            try:
                PreflightVehicleGate().wait_until_ready()
                MissionUploader().upload(items)
                return
            except MissionUploadError as error:
                if not self._retryable_preflight_error(error):
                    raise
                retryable_failures.append(f"attempt {attempt}: {error}")
                if attempt == _MAX_PREFLIGHT_BOOT_ATTEMPTS:
                    raise HarnessError(
                        self._debug_snapshot(
                            "preflight readiness never stabilized across simulator boots: "
                            + "; ".join(retryable_failures)
                        )
                    ) from error
                self._restart_simulator()

    def upload_valid_docking_mission(self) -> None:
        self.upload_mission(load_valid_docking_mission())

    def start_manager(self, *, backend: str, start_localizer: bool = True) -> None:
        if self._manager_process is not None:
            raise HarnessError("manager is already running")
        if backend not in {"pid", "precise_landing"}:
            raise ValueError("backend must be 'pid' or 'precise_landing'")

        self._manager_process = ManagedLaunchProcess.start(
            [
                "bash",
                "-lc",
                (
                    f"source /opt/ros/humble/setup.bash && "
                    f"source '{_quoted(self.workspace_root / 'install' / 'setup.bash')}' && "
                    "ros2 launch flight_cycle_manager flight_cycle.launch.py "
                    f"landing_backend:={backend} simulation_mode:=true "
                    f"start_localizer:={'true' if start_localizer else 'false'}"
                ),
            ],
            env=self.environment,
            cwd=self._temp_root,
            log_name=f"manager_{backend}",
        )

        wait_started = time.monotonic()
        while time.monotonic() - wait_started < 45.0:
            self._pump(0.1)
            if self.node.start_cycle_client.wait_for_service(timeout_sec=0.1):
                break
            if self._manager_process.poll() is not None:
                raise HarnessError(self._debug_snapshot("manager exited during startup"))
        else:
            raise HarnessError(self._debug_snapshot("manager service never appeared"))

        self.wait_any_state({"READY", "INVALID_ONBOARD_MISSION"}, timeout=45.0)
        self._observer = _SitlObserver()

    def call_start_cycle(self) -> Trigger.Response:
        return self._call_trigger(self.node.start_cycle_client)

    def call_request_dock(self) -> Trigger.Response:
        return self._call_trigger(self.node.request_dock_client)

    def vehicle_armed(self) -> bool:
        observer = self._require_observer()
        observer.poll()
        return bool(observer.state.armed)

    def current_mode(self) -> Optional[int]:
        observer = self._require_observer()
        observer.poll()
        return observer.state.mode

    def set_vehicle_mode(self, custom_mode: int) -> None:
        self._require_observer().set_mode(custom_mode)

    def wait_for_mode(self, custom_mode: int, timeout: float) -> None:
        self._require_observer().wait_for_mode(custom_mode, timeout)

    def wait_state(self, expected_state: str, timeout: float) -> None:
        self.wait_any_state({expected_state}, timeout=timeout)

    def wait_any_state(self, states: set[str], timeout: float) -> None:
        start_index = len(self.node.state_history)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump(0.1)
            for _, state in self.node.state_history[start_index:]:
                if state in states:
                    return
                start_index += 1
        joined = ", ".join(sorted(states))
        raise HarnessError(self._debug_snapshot(f"timed out waiting for state in {joined}"))

    def wait_states(self, expected_states: Sequence[str], timeout: float) -> None:
        start_index = len(self.node.state_history)
        progress = 0
        deadline = time.monotonic() + timeout
        expected = list(expected_states)
        while progress < len(expected) and time.monotonic() < deadline:
            self._pump(0.1)
            while start_index < len(self.node.state_history):
                _, state = self.node.state_history[start_index]
                start_index += 1
                if state == expected[progress]:
                    progress += 1
                    if progress == len(expected):
                        return
        raise HarnessError(
            self._debug_snapshot(
                "timed out waiting for state sequence " + " -> ".join(expected)
            )
        )

    def wait_reason_contains(self, text: str, timeout: float) -> None:
        start_index = len(self.node.reason_history)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump(0.1)
            for _, reason in self.node.reason_history[start_index:]:
                if text in reason:
                    return
                start_index += 1
        raise HarnessError(
            self._debug_snapshot(f"timed out waiting for reason containing {text!r}")
        )

    def assert_state_holds(self, expected_state: str, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._pump(0.1)
            state = self.node.state_history[-1][1] if self.node.state_history else None
            if state != expected_state:
                raise HarnessError(
                    self._debug_snapshot(
                        f"expected state {expected_state} to hold for {duration}s"
                    )
                )

    def mode_history_names(self) -> list[str]:
        observer = self._require_observer()
        observer.poll()
        return [mode_name(mode) for _, mode in observer.mode_history]

    def _call_trigger(self, client: object) -> Trigger.Response:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.1):
                break
            self._pump(0.1)
        else:
            raise HarnessError(self._debug_snapshot("required Trigger service is unavailable"))

        future = client.call_async(Trigger.Request())
        while rclpy.ok() and not future.done():
            self._pump(0.1)
        result = future.result()
        if result is None:
            raise HarnessError(self._debug_snapshot("Trigger service returned no result"))
        return result

    def _start_simulator(self) -> None:
        self._sim_process = ManagedLaunchProcess.start(
            [
                "bash",
                "-lc",
                (
                    f"source /opt/ros/humble/setup.bash && "
                    f"source '{_quoted(self.workspace_root / 'install' / 'setup.bash')}' && "
                    "ros2 launch drone_bringup docking_sim.launch.py "
                    "initial_point:=dock use_dds_agent:=false "
                    "use_mavproxy:=false use_gz_sim_gui:=false"
                ),
            ],
            env=self.environment,
            cwd=self._temp_root,
            log_name="simulator",
        )
        self._sim_process.wait_for_output("SERIAL0 on TCP port 5760", timeout_s=60.0)

    @staticmethod
    def _retryable_preflight_error(error: MissionUploadError) -> bool:
        text = str(error)
        return any(
            marker in text
            for marker in (
                "ready flight-controller heartbeat",
                "mission upload timed out",
                "GPS-ready flight-controller status text",
            )
        )

    def _restart_simulator(self) -> None:
        if self._manager_process is not None or self._observer is not None:
            raise HarnessError(
                "simulator restart is only allowed before the manager starts"
            )
        if self._sim_process is not None:
            self._sim_process.terminate()
            self._sim_process = None
        self._start_simulator()

    def _pump(self, spin_timeout_s: float) -> None:
        rclpy.spin_once(self.node, timeout_sec=spin_timeout_s)
        if self._observer is not None:
            self._observer.poll()
        for process in (self._sim_process, self._manager_process):
            if process is None:
                continue
            code = process.poll()
            if code is not None:
                raise HarnessError(
                    self._debug_snapshot(
                        f"managed process exited unexpectedly with code {code}"
                    )
                )

    def _debug_snapshot(self, prefix: str) -> str:
        latest_state = self.node.state_history[-1][1] if self.node.state_history else "NONE"
        latest_reason = self.node.reason_history[-1][1] if self.node.reason_history else "NONE"
        latest_dock_mode = (
            self.node.dock_mode_history[-1][1] if self.node.dock_mode_history else "NONE"
        )
        mode_history = self.mode_history_names()[-10:] if self._observer is not None else []
        current_mode = mode_name(self.current_mode()) if self._observer is not None else "NONE"
        armed = self.vehicle_armed() if self._observer is not None else False
        pieces = [
            prefix,
            f"manager_state={latest_state}",
            f"manager_reason={latest_reason}",
            f"dock_mode={latest_dock_mode}",
            f"dock_pose_count={self.node.dock_pose_count}",
            f"vehicle_mode={current_mode}",
            f"vehicle_armed={armed}",
            f"mode_history={mode_history}",
        ]
        if self._manager_process is not None:
            pieces.append("manager_log_tail:\n" + self._manager_process.tail())
        if self._sim_process is not None:
            pieces.append("sim_log_tail:\n" + self._sim_process.tail())
        return "\n".join(pieces)

    def _require_observer(self) -> _SitlObserver:
        if self._observer is None:
            raise HarnessError("observer connection is unavailable until the manager starts")
        return self._observer


@pytest.fixture
def sitl_stack():
    stack = SitlStack()
    try:
        yield stack
    finally:
        stack.close()


__all__ = [
    "FixtureMissionItem",
    "ManagedLaunchProcess",
    "MissionUploadError",
    "MissionUploader",
    "PreflightVehicleGate",
    "SitlStack",
    "items_without_nav_land",
    "load_qgc_wpl_fixture",
    "load_valid_docking_mission",
    "mode_name",
    "sitl_stack",
]

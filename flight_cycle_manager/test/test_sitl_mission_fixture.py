"""Unit coverage for the SITL mission fixture and harness support code."""

from __future__ import annotations

import subprocess
import time

import pytest

from sitl_mission_fixture import (
    ManagedLaunchProcess,
    MissionUploadError,
    MissionUploader,
    PreflightVehicleGate,
    load_valid_docking_mission,
)


class FakeMessage:
    def __init__(self, message_type: str, **fields):
        self._message_type = message_type
        self.__dict__.update(fields)

    def get_type(self) -> str:
        return self._message_type

    def get_srcSystem(self) -> int:
        return getattr(self, "src_system", 1)

    def get_srcComponent(self) -> int:
        return getattr(self, "src_component", 1)

    def to_dict(self) -> dict[str, object]:
        payload = dict(self.__dict__)
        payload["mavpackettype"] = self._message_type
        return payload


class FakeMav:
    def __init__(self) -> None:
        self.heartbeat_calls: list[tuple[object, ...]] = []
        self.mission_count_calls: list[tuple[int, int, int]] = []
        self.mission_item_int_calls: list[tuple[object, ...]] = []

    def heartbeat_send(self, *args: object) -> None:
        self.heartbeat_calls.append(args)

    def mission_count_send(
        self,
        target_system: int,
        target_component: int,
        count: int,
    ) -> None:
        self.mission_count_calls.append((target_system, target_component, count))

    def mission_item_int_send(self, *args: object) -> None:
        self.mission_item_int_calls.append(args)


class FakeConnection:
    _DEFAULT_HEARTBEAT = object()

    def __init__(
        self,
        messages: list[FakeMessage],
        *,
        heartbeat: FakeMessage | None | object = _DEFAULT_HEARTBEAT,
        heartbeat_messages: list[FakeMessage | None] | None = None,
    ) -> None:
        self.mav = FakeMav()
        self._messages = list(messages)
        if heartbeat_messages is not None:
            self._heartbeat_messages = list(heartbeat_messages)
        else:
            if heartbeat is self._DEFAULT_HEARTBEAT:
                heartbeat = FakeMessage(
                    "HEARTBEAT",
                    src_system=1,
                    src_component=1,
                    system_status=3,
                )
            self._heartbeat_messages = [heartbeat]
        self.closed = False

    def wait_heartbeat(self, timeout: float) -> FakeMessage:
        del timeout
        return self._heartbeat_messages.pop(0) if self._heartbeat_messages else None

    def recv_match(self, blocking: bool, timeout: float, type=None):
        assert blocking is True
        assert timeout >= 0.0
        if type == "HEARTBEAT":
            return self._heartbeat_messages.pop(0) if self._heartbeat_messages else None
        while self._messages:
            message = self._messages.pop(0)
            if type is None or message.get_type() == type:
                return message
        return None

    def close(self) -> None:
        self.closed = True


def test_valid_fixture_parses_exact_qgc_wpl_contract():
    """Break caught: the WPL parser swaps columns or drifts the dock coordinates."""
    items = load_valid_docking_mission()

    assert [
        (
            item.seq,
            item.frame,
            item.command,
            item.current,
            item.autocontinue,
            item.x,
            item.y,
            item.z,
        )
        for item in items
    ] == [
        (0, 3, 16, 1, 1, 453821944, 120006033, 4.0),
        (1, 3, 189, 0, 1, 453821944, 120005833, 4.0),
        (2, 3, 16, 0, 1, 453822044, 120005833, 4.0),
        (3, 3, 16, 0, 1, 453821944, 120005833, 4.0),
        (4, 3, 19, 0, 1, 453821944, 120005833, 4.0),
        (5, 3, 21, 0, 1, 453821944, 120005833, 0.0),
    ]


@pytest.mark.parametrize("request_type", ["MISSION_REQUEST", "MISSION_REQUEST_INT"])
def test_uploader_sends_requested_items_until_mission_accepted(request_type: str):
    """Break caught: upload stops before ArduPilot requests every mission item."""
    items = load_valid_docking_mission()
    connection = FakeConnection(
        [FakeMessage(request_type, seq=index) for index in range(len(items))]
        + [FakeMessage("MISSION_ACK", type=0)]
    )
    uploader = MissionUploader(connection_factory=lambda *args, **kwargs: connection)

    uploader.upload(items)

    assert connection.mav.mission_count_calls == [(1, 1, len(items))]
    assert [call[2] for call in connection.mav.mission_item_int_calls] == list(
        range(len(items))
    )
    assert connection.closed is True


def test_uploader_waits_for_ready_heartbeat_before_mission_upload():
    """Break caught: upload starts before the flight controller reaches a mission-ready heartbeat."""
    items = load_valid_docking_mission()
    connection = FakeConnection(
        [
            *[FakeMessage("MISSION_REQUEST", seq=index) for index in range(len(items))],
            FakeMessage("MISSION_ACK", type=0),
        ],
        heartbeat_messages=[
            FakeMessage(
                "HEARTBEAT",
                src_system=1,
                src_component=1,
                system_status=1,
            ),
            FakeMessage(
                "HEARTBEAT",
                src_system=1,
                src_component=1,
                system_status=1,
            ),
            FakeMessage(
                "HEARTBEAT",
                src_system=1,
                src_component=1,
                system_status=3,
            ),
        ],
    )
    uploader = MissionUploader(connection_factory=lambda *args, **kwargs: connection)

    uploader.upload(items)

    assert len(connection.mav.heartbeat_calls) == 3
    assert connection.mav.mission_count_calls == [(1, 1, len(items))]
    assert connection.closed is True


def test_preflight_vehicle_gate_waits_for_gps_ready_status_text():
    """Break caught: the harness starts uploads before ArduPilot finishes its GPS/EKF startup."""
    connection = FakeConnection(
        [
            FakeMessage("STATUSTEXT", text=b"EKF3 IMU0 initialised\0"),
            FakeMessage("STATUSTEXT", text=b"EKF3 IMU0 is using GPS\0"),
        ],
        heartbeat_messages=[
            FakeMessage(
                "HEARTBEAT",
                src_system=1,
                src_component=1,
                system_status=1,
            ),
            FakeMessage(
                "HEARTBEAT",
                src_system=1,
                src_component=1,
                system_status=3,
            ),
        ],
    )
    gate = PreflightVehicleGate(connection_factory=lambda *args, **kwargs: connection)

    gate.wait_until_ready()

    assert len(connection.mav.heartbeat_calls) == 2
    assert connection.closed is True


def test_uploader_rejects_out_of_range_request_sequence():
    """Break caught: upload accepts an impossible requested mission sequence."""
    items = load_valid_docking_mission()
    connection = FakeConnection(
        [
            FakeMessage("MISSION_REQUEST", seq=0),
            FakeMessage("MISSION_REQUEST", seq=len(items)),
        ]
    )
    uploader = MissionUploader(connection_factory=lambda *args, **kwargs: connection)

    with pytest.raises(MissionUploadError, match="requested mission item"):
        uploader.upload(items)

    assert connection.closed is True


def test_uploader_raises_when_heartbeat_never_arrives():
    """Break caught: uploader dereferences a timed-out heartbeat instead of failing clearly."""
    items = load_valid_docking_mission()
    connection = FakeConnection([], heartbeat=None)
    uploader = MissionUploader(
        timeout_s=0.01,
        connection_factory=lambda *args, **kwargs: connection,
    )

    with pytest.raises(MissionUploadError, match="ready flight-controller heartbeat"):
        uploader.upload(items)

    assert connection.closed is True


def test_preflight_vehicle_gate_raises_when_gps_ready_text_never_arrives():
    """Break caught: the harness treats a heartbeat-only boot as arm-ready."""
    connection = FakeConnection(
        [FakeMessage("STATUSTEXT", text="EKF3 IMU0 initialised")],
    )
    gate = PreflightVehicleGate(
        timeout_s=0.01,
        connection_factory=lambda *args, **kwargs: connection,
    )

    with pytest.raises(MissionUploadError, match="GPS-ready flight-controller status text"):
        gate.wait_until_ready()

    assert connection.closed is True


def test_managed_process_terminates_only_its_own_process_group(tmp_path):
    """Break caught: harness cleanup leaves its launched subprocess tree running."""
    process = ManagedLaunchProcess.start(
        ["bash", "-lc", "sleep 60"],
        env={"PATH": "/usr/bin:/bin"},
        cwd=tmp_path,
        log_name="sleep-probe",
    )
    try:
        assert process.poll() is None
    finally:
        process.terminate()

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break
        time.sleep(0.05)

    assert process.poll() is not None
    remaining = subprocess.run(
        ["pgrep", "-g", str(process.process_group_id)],
        check=False,
        text=True,
        capture_output=True,
    )
    assert remaining.returncode == 1
    assert remaining.stdout.strip() == ""


def test_managed_process_terminate_cleans_orphaned_group_after_leader_exits(tmp_path):
    """Break caught: cleanup skips a background child if the launch leader exits first."""
    process = ManagedLaunchProcess.start(
        [
            "/usr/bin/python3",
            "-c",
            "import subprocess; subprocess.Popen(['sleep', '60'])",
        ],
        env={"PATH": "/usr/bin:/bin"},
        cwd=tmp_path,
        log_name="orphan-probe",
    )

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break
        time.sleep(0.05)

    assert process.poll() == 0
    process.terminate()

    remaining = subprocess.run(
        ["pgrep", "-g", str(process.process_group_id)],
        check=False,
        text=True,
        capture_output=True,
    )
    assert remaining.returncode == 1
    assert remaining.stdout.strip() == ""

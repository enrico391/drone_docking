"""HTTP validation and pose-only telemetry without ROS or a flying vehicle."""

import json
import math
from pathlib import Path
import shutil
import subprocess
import threading
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from flight_cycle_manager.landing_backend import DockObservation, DockTargetMode
from flight_cycle_manager.pid_tuner import (
    DASHBOARD_HTML, TelemetryStore, _handler_for, parse_gain_request,
)
from flight_cycle_manager.px4_control import LIVE_PID_PARAMETERS, pose_tracking_telemetry


@pytest.mark.parametrize("bank,prefix", [
    ("base", "pid_x"), ("near", "pid_gain_near_x"), ("far", "pid_gain_far_x"),
])
def test_gain_bank_updates_are_complete_and_axis_specific(bank, prefix):
    assert parse_gain_request(dict(axis="x", bank=bank, kp=0.5, ki=0, kd=0.1)) == {
        f"{prefix}_kp": 0.5, f"{prefix}_ki": 0.0, f"{prefix}_kd": 0.1,
    }


def test_scheduled_gains_can_inherit_base():
    assert parse_gain_request(dict(axis="yaw", bank="far", kp=-1, ki=-1, kd=-1)) == {
        f"pid_gain_far_yaw_{gain}": -1.0 for gain in ("kp", "ki", "kd")}


@pytest.mark.parametrize("override", [
    {"axis": "roll"}, {"bank": "invalid"}, {"kp": True}, {"ki": "0"},
    {"kd": math.inf}, {"kp": math.nan}, {"kp": -1}, {"bank": "near", "kp": -0.5},
])
def test_invalid_gains_are_rejected(override):
    with pytest.raises(ValueError):
        parse_gain_request(dict(dict(axis="x", kp=1, ki=0, kd=0), **override))


@pytest.mark.parametrize("payload", [None, [], {}, {"axis": "x", "kp": 1, "ki": 0}])
def test_incomplete_gain_requests_are_rejected(payload):
    with pytest.raises(ValueError):
        parse_gain_request(payload)


def test_telemetry_is_thread_safe_and_expires_without_showing_stale_pose():
    clock = [10.0]
    store = TelemetryStore(now=lambda: clock[0])
    assert not store.snapshot()["connected"]
    assert store.update('{"error":{"x":0.2}}')
    snapshot = store.snapshot()
    snapshot["telemetry"]["error"]["x"] = 100
    for raw in ("invalid", "[]", '{"error":NaN}'):
        assert not store.update(raw)
    assert store.snapshot()["telemetry"]["error"]["x"] == 0.2
    clock[0] += 2.1
    assert store.snapshot() == {"connected": False, "telemetry": None}


def test_pose_telemetry_uses_control_errors_not_raw_pose_or_old_pid_samples():
    dock = DockObservation(0.4, -0.2, 2.0, 0.3, DockTargetMode.OUTER, 5.0, 9.9)
    parameters = {name: 1.0 for name in LIVE_PID_PARAMETERS}
    diagnostics = {"control_error": {"x": 0.1, "y": 0.05, "z": None, "yaw": -0.1},
                   "distance_setpoint_m": 1.5, "axes": {"z": {"error": 999}}}
    data = pose_tracking_telemetry(now=10.0, state="PID_ALIGN", dock=dock,
                                   diagnostics=diagnostics, parameters=parameters)
    assert data["error"] == diagnostics["control_error"]
    assert data["measured_pose"] == {"x": 0.4, "y": -0.2, "z": 2.0, "yaw": 0.3}
    assert data["dock_age_s"] == pytest.approx(0.1)
    lost = pose_tracking_telemetry(now=10.0, state="RECOVERY_CLIMB", dock=None,
                                   diagnostics={}, parameters=parameters)
    assert lost["measured_pose"] is None and not lost["pose_valid"]
    assert all(value is None for value in lost["error"].values())


@pytest.fixture
def web_server():
    received = []
    store = TelemetryStore()
    server = ThreadingHTTPServer(("127.0.0.1", 0), _handler_for(
        store, lambda parameters: received.append(parameters) or {"ok": True, "message": "applied"}))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", received, store
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def request_json(url, payload, **headers):
    request = Request(url, data=json.dumps(payload).encode(), method="POST", headers=headers)
    try:
        response = urlopen(request, timeout=3)
    except HTTPError as error:
        response = error
    with response:
        return response.status, json.load(response)


def test_http_serves_dashboard_state_and_atomic_gain_requests(web_server):
    url, received, store = web_server
    with urlopen(url) as response:
        assert "Dock pose PID tuner" in response.read().decode()
    store.update('{"state":"PID_ALIGN"}')
    with urlopen(url + "/api/state") as response:
        assert json.load(response)["telemetry"]["state"] == "PID_ALIGN"
    assert request_json(url + "/api/gains", dict(axis="y", bank="far", kp=0.4, ki=0, kd=0))[0] == 200
    assert received == [{"pid_gain_far_y_kp": 0.4, "pid_gain_far_y_ki": 0.0, "pid_gain_far_y_kd": 0.0}]
    assert request_json(url + "/api/gains", dict(axis="x", kp=-1, ki=0, kd=0))[0] == 400
    assert request_json(url + "/api/gains", {}, Origin="http://evil.example")[0] == 403
    assert len(received) == 1
    with pytest.raises(HTTPError) as error:
        urlopen(url + "/camera")
    assert error.value.code == 404


def test_dashboard_shows_only_pose_tracking_with_fixed_scales_and_gains():
    for text in ("Measured pose", "Tracking error", "distance_setpoint_m", "gain_near_distance_m",
                 "Download YAML", "-1 inherits", "error:null", "Outside displayed range"):
        assert text in DASHBOARD_HTML
    for text in ("command_body_flu", "measured_velocity", "surfaceSamples", "<video", "<img"):
        assert text not in DASHBOARD_HTML


def test_dashboard_exposes_all_axis_editors_without_axis_dropdown():
    assert '<select id="axis">' not in DASHBOARD_HTML
    assert 'id="axis-controls"' in DASHBOARD_HTML
    for control in ("bank", "apply", "gain-status", "effective"):
        assert f'id="{control}-${{a}}"' in DASHBOARD_HTML
    assert 'id="${g}-${a}"' in DASHBOARD_HTML
    assert '.plots,.axis-controls{grid-template-columns:1fr}' in DASHBOARD_HTML


def test_dashboard_javascript_keeps_axis_edits_and_updates_independent():
    node = shutil.which("node") or shutil.which("nodejs")
    if node is None:
        candidates = sorted((Path.home() / ".vscode-server/bin").glob("*/node"))
        node = str(candidates[-1]) if candidates else None
    if node is None:
        pytest.skip("Node.js unavailable for dashboard JavaScript smoke test")
    script = DASHBOARD_HTML.split("<script>", 1)[1].split("</script>", 1)[0]
    parameters = {name: 0.1 for name in LIVE_PID_PARAMETERS}
    result = subprocess.run(
        [node, str(Path(__file__).with_name("test_pid_tuner_dashboard.js"))],
        input=json.dumps({"script": script, "parameters": parameters}),
        text=True, capture_output=True, timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
import json
import math
import threading
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from flight_cycle_manager.pid_tuner import (
    DASHBOARD_HTML, TelemetryStore, _WebApp, _handler_for,
    parse_distance_request, parse_gain_request, parse_hold_request,
)
from flight_cycle_manager.pid_landing import PidLandingState
from flight_cycle_manager.pid_landing_backend import PidLandingBackend
from test_pid_landing_backend import pid_config


def test_gain_request_accepts_one_complete_axis_update():
    assert parse_gain_request({
        "axis": "yaw", "kp": 1, "ki": 0.2, "kd": 0.03,
    }) == ("yaw", 1.0, 0.2, 0.03)


@pytest.mark.parametrize("payload", [
    {"axis": "roll", "kp": 1.0, "ki": 0.0, "kd": 0.0},
    {"axis": "x", "kp": -0.1, "ki": 0.0, "kd": 0.0},
    {"axis": "x", "kp": 1.0, "ki": math.inf, "kd": 0.0},
    {"axis": "x", "kp": True, "ki": 0.0, "kd": 0.0},
    {"axis": "x", "kp": 1.0, "ki": 0.0},
])
def test_gain_request_rejects_unsafe_or_incomplete_updates(payload):
    with pytest.raises(ValueError):
        parse_gain_request(payload)


def test_telemetry_store_keeps_last_valid_message():
    store = TelemetryStore()
    assert store.update(json.dumps({"state": "PID_ALIGN", "error": {"x": 0.1}}))
    assert not store.update("not json")
    assert store.snapshot() == {
        "connected": True,
        "telemetry": {"state": "PID_ALIGN", "error": {"x": 0.1}},
    }


def test_distance_request_validates_dock_distance():
    assert parse_distance_request({"distance_m": 2}) == 2.0
    for payload in ({}, {"distance_m": True}, {"distance_m": "2"},
                    {"distance_m": 0.2}, {"distance_m": 21},
                    {"distance_m": math.nan}):
        with pytest.raises(ValueError):
            parse_distance_request(payload)


@pytest.mark.parametrize("hold", ["none", "PID_ALIGN", "DESCEND_OUTER",
                                  "ACQUIRE_CENTRAL", "DESCEND_CENTRAL", "HOLD_CENTRAL"])
def test_hold_request_accepts_supported_states(hold):
    assert parse_hold_request({"pid_hold_at_state": hold}) == hold


@pytest.mark.parametrize("payload", [{}, None, {"pid_hold_at_state": "DISARM"},
                                     {"pid_hold_at_state": "TOUCHDOWN_APPROACH"},
                                     {"pid_hold_at_state": 1},
                                     {"pid_hold_at_state": "hold_central"}])
def test_hold_request_rejects_invalid_states(payload):
    with pytest.raises(ValueError, match="pid_hold_at_state"):
        parse_hold_request(payload)


def test_hold_endpoint_forwards_only_hold_parameter_and_reports_rejections():
    received = []

    def submit(parameters):
        received.append(parameters)
        return ({"ok": False, "message": "PID hold state has already been passed"}
                if parameters["pid_hold_at_state"] == "PID_ALIGN"
                else {"ok": True, "message": "parameters applied"})

    server = ThreadingHTTPServer(("127.0.0.1", 0),
                                 _handler_for(_WebApp(TelemetryStore(), submit)))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        def post(payload):
            request = Request(
                f"http://127.0.0.1:{server.server_port}/api/hold",
                data=json.dumps(payload).encode(),
                headers={"Content-Type": "application/json"}, method="POST",
            )
            try:
                response = urlopen(request, timeout=2)
            except HTTPError as error:
                response = error
            with response:
                return response.status, json.load(response)

        assert post({"pid_hold_at_state": "HOLD_CENTRAL"}) == (
            200, {"ok": True, "message": "parameters applied"})
        assert post({"pid_hold_at_state": "PID_ALIGN"}) == (
            503, {"ok": False, "message": "PID hold state has already been passed"})
        status, result = post({"pid_hold_at_state": "DISARM"})
        assert status == 400
        assert not result["ok"]
        assert received == [{"pid_hold_at_state": "HOLD_CENTRAL"},
                            {"pid_hold_at_state": "PID_ALIGN"}]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_pid_diagnostics_expose_active_hold_for_dashboard():
    backend = PidLandingBackend(pid_config(), hold_at_state=PidLandingState.HOLD_CENTRAL)
    assert backend.diagnostics()["pid_hold_at_state"] == "HOLD_CENTRAL"
    backend.set_hold_at_state(None)
    assert backend.diagnostics()["pid_hold_at_state"] == "none"


def test_dashboard_has_live_hold_selector_and_failure_feedback():
    for hold in ("none", "PID_ALIGN", "DESCEND_OUTER", "ACQUIRE_CENTRAL",
                 "DESCEND_CENTRAL", "HOLD_CENTRAL"):
        assert f'<option value="{hold}">' in DASHBOARD_HTML
    assert "fetch('/api/hold'" in DASHBOARD_HTML
    assert 'id="hold-message" role="status"' in DASHBOARD_HTML
    assert "latest.pid_hold_at_state" in DASHBOARD_HTML
    assert "holdSelect.disabled=true" in DASHBOARD_HTML


def test_stale_pid_messages_are_not_reported_as_live(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(
        "flight_cycle_manager.pid_tuner.time.monotonic", lambda: clock[0],
    )
    store = TelemetryStore()
    assert store.update(json.dumps({"error": {"x": 0.1}}))
    clock[0] = 12.1
    assert store.snapshot() == {"connected": False, "telemetry": None}
    assert store.update(json.dumps({"error": {"x": 0.2}}))
    assert store.snapshot()["connected"] is True


def test_relative_error_view_stays_visible_on_all_axis_tabs():
    assert DASHBOARD_HTML.count('id="position"') == 1
    assert DASHBOARD_HTML.count('id="readings"') == 1
    assert "names=['x','y','z','yaw']" in DASHBOARD_HTML
    assert 'fill();drawPosition();' in DASHBOARD_HTML
    assert 'Number.isFinite(error.x)' in DASHBOARD_HTML
    assert 'Number.isFinite(error.yaw)' in DASHBOARD_HTML
    assert 'Z error unavailable (no setpoint)' in DASHBOARD_HTML
    assert 'latest.dock_age_s < 1' in DASHBOARD_HTML
    assert "(error.yaw*180/Math.PI).toFixed(1)" in DASHBOARD_HTML


def test_dashboard_places_selection_and_errors_left_of_plot():
    left = DASHBOARD_HTML.split('<div class="dashboard-left">', 1)[1].split(
        '<div class="dashboard-right">', 1
    )[0]
    right = DASHBOARD_HTML.split('<div class="dashboard-right">', 1)[1].split(
        '</main>', 1
    )[0]
    assert 'id="axes"' in left
    assert 'id="readings"' in left
    assert 'id="plot"' in left
    assert 'id="position"' not in left
    assert 'id="position"' in right
    assert 'id="readings"' not in right
    assert 'grid-template-columns:minmax(320px,1fr) minmax(0,1.25fr)' in DASHBOARD_HTML
    assert '@media(max-width:850px)' in DASHBOARD_HTML


def test_error_surface_only_interpolates_near_fresh_observations():
    assert 'const surfaceSamples=[]' in DASHBOARD_HTML
    assert 'surfaceSamples.length>160' in DASHBOARD_HTML
    assert 'latest.dock_age_s >= 1' in DASHBOARD_HTML
    assert '!Number.isFinite(e.z)' in DASHBOARD_HTML
    assert 'surfaceSetpoint!==setpoint' in DASHBOARD_HTML
    assert 'lastSurfaceTime===latest.time_s' in DASHBOARD_HTML
    assert 'function surfaceHeight(x,y,cell)' in DASHBOARD_HTML
    assert '.filter(v=>v.d<=cell*4)' in DASHBOARD_HTML
    assert 'if(u < -1e-8 || v < -1e-8 || t < -1e-8)continue;' in DASHBOARD_HTML
    assert 'if(corners.every(p=>p.z!==null))' in DASHBOARD_HTML
    assert 'recordSurfaceSample();drawPosition();' in DASHBOARD_HTML
    assert 'blank areas are unknown' in DASHBOARD_HTML

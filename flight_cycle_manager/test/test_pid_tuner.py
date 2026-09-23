import json
import math

import pytest

from flight_cycle_manager.pid_tuner import (
    TelemetryStore, parse_distance_request, parse_gain_request,
)


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

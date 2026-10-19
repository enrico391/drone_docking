"""Read-only gate snapshots and safe live transition tuning without hardware."""

from dataclasses import replace
import json
import math

import pytest

from flight_cycle_manager.pid_landing import PidLandingState
from test_pid_landing_backend import (
    central_observation, enter_central_descent, pid_config, start_backend, vehicle_observation,
)


def sample(backend, now, *, velocity=(0.0, 0.0, 0.0), missing=False, **pose):
    dock = None if missing else replace(central_observation(now, distance_m=0.20), **pose)
    decision = backend.update(now=now, dock=dock,
                              vehicle=vehicle_observation(velocity_enu_m_s=velocity))
    diagnostics = backend.diagnostics()["touchdown_transition"]
    # No nonfinite speed may break the JSON telemetry publisher.
    json.dumps(diagnostics, allow_nan=False)
    return decision, diagnostics


@pytest.mark.parametrize("pose,velocity,blocker", [
    ({"forward_m": 0.061}, (0.0, 0.0, 0.0), "x"),
    ({"left_m": -0.061}, (0.0, 0.0, 0.0), "y"),
    ({"yaw_error_rad": 0.081}, (0.0, 0.0, 0.0), "yaw"),
    ({}, (0.04, 0.04, 0.0), "horizontal_speed"),
    ({}, (math.nan, 0.0, 0.0), "horizontal_speed"),
    ({"distance_m": 0.23}, (0.0, 0.0, 0.0), "height"),
    ({}, (0.0, 0.0, -0.051), "vertical_speed"),
    ({}, (0.0, 0.0, math.nan), "vertical_speed"),
])
def test_gate_snapshot_identifies_exact_blocking_measurement(pose, velocity, blocker):
    backend = start_backend()
    enter_central_descent(backend)
    sample(backend, 1.8, velocity=velocity, **pose)
    decision, diagnostics = sample(backend, 2.81, velocity=velocity, **pose)
    assert decision.substate == "DESCEND_CENTRAL"
    assert blocker in diagnostics["blockers"]
    check = diagnostics["checks"][blocker]
    assert not check["passed"]
    assert check["parameter"].startswith("pid_")
    assert check["value"] is None or check["value"] > check["limit"]
    assert diagnostics["fine_alignment_stable"] == (blocker in ("height", "vertical_speed"))


def test_dwell_snapshot_tracks_progress_reset_and_final_success():
    backend = start_backend(pid_config(touchdown_horizontal_speed_stable_time_s=2.0))
    enter_central_descent(backend)
    sample(backend, 1.8)
    _, data = sample(backend, 2.9)
    assert data["checks"]["alignment_dwell"]["passed"]
    assert data["checks"]["speed_dwell"]["value"] == pytest.approx(1.1)
    assert not data["fine_alignment_stable"]
    _, data = sample(backend, 3.0, forward_m=0.1)
    assert data["checks"]["alignment_dwell"]["value"] == 0
    assert data["checks"]["speed_dwell"]["value"] == 0
    _, data = sample(backend, 3.1)
    assert data["last_dwell_interruption"]["time_s"] == 3.0
    assert data["last_dwell_interruption"]["checks"]["x"]["value"] == 0.1
    _, data = sample(backend, 3.2, missing=True)
    assert "central_vision" in data["blockers"]
    assert data["checks"]["x"]["value"] is None
    assert not data["evaluated"]
    sample(backend, 3.3)
    decision, data = sample(backend, 5.31)
    assert decision.substate == "TOUCHDOWN_APPROACH"
    assert data["ready"] and data["fine_alignment_stable"] and data["height_ready"]
    assert data["blockers"] == []
    sample(backend, 5.4)
    assert backend.diagnostics()["touchdown_transition"] is None


def test_yaw_gate_uses_wrapped_error():
    backend = start_backend()
    enter_central_descent(backend)
    _, data = sample(backend, 1.8, yaw_error_rad=2 * math.pi + 0.02)
    assert data["checks"]["yaw"]["value"] == pytest.approx(0.02)
    assert data["checks"]["yaw"]["passed"]


def test_debug_hold_is_explicit_and_does_not_evaluate_or_advance_dwell():
    backend = start_backend()
    enter_central_descent(backend)
    backend.set_hold_at_state(PidLandingState.DESCEND_CENTRAL)
    _, data = sample(backend, 1.8)
    assert "debug_hold" in data["blockers"]
    assert not data["evaluated"]
    assert data["checks"]["alignment_dwell"]["value"] == 0


def test_live_transition_edit_keeps_state_and_restarts_dwell():
    config = pid_config()
    backend = start_backend(config)
    enter_central_descent(backend)
    sample(backend, 1.8)
    sample(backend, 2.3)
    backend.reconfigure_touchdown_transition(replace(config, alignment_stable_time_s=0.8))
    decision, data = sample(backend, 2.4)
    assert decision.substate == "DESCEND_CENTRAL"
    assert data["checks"]["alignment_dwell"]["value"] == 0
    assert data["checks"]["alignment_dwell"]["limit"] == 0.8
    # Reapplying unchanged settings must not continually restart the timer.
    backend.reconfigure_touchdown_transition(replace(config, alignment_stable_time_s=0.8))
    assert sample(backend, 3.41)[0].substate == "TOUCHDOWN_APPROACH"
    before = backend.diagnostics()
    with pytest.raises(ValueError, match="commitment"):
        backend.reconfigure_touchdown_transition(config)
    assert backend.diagnostics() == before


def test_new_attempt_clears_gate_history():
    backend = start_backend()
    enter_central_descent(backend)
    sample(backend, 1.8, forward_m=0.1)
    backend.start(now=10.0, dock=None, vehicle=vehicle_observation())
    assert backend.diagnostics()["touchdown_transition"] is None


def test_height_edit_rebases_descent_target_without_resetting_horizontal_pid():
    config = pid_config()
    backend = start_backend(config)
    enter_central_descent(backend)
    sample(backend, 1.8, distance_m=0.5)
    assert backend.diagnostics()["distance_setpoint_m"] is not None
    before_x = backend.diagnostics()["axes"]["x"]
    backend.reconfigure_touchdown_transition(replace(config, touchdown_commit_height_m=0.3))
    assert backend.diagnostics()["distance_setpoint_m"] is None
    assert backend.diagnostics()["axes"]["x"] == before_x
    _, data = sample(backend, 1.9, distance_m=0.5)
    assert data["approach_height_m"] == 0.3
    assert backend.diagnostics()["distance_setpoint_m"] >= 0.3


def test_gate_diagnostics_use_frozen_control_reference_not_camera_xy_yaw():
    from test_fixed_central_reference import backend_for, central_entry, disturbed, shifted_vehicle

    backend = backend_for("descend_central")
    central_entry(backend)
    backend.update(now=1.8, dock=disturbed(1.8, distance=0.2), vehicle=shifted_vehicle())
    data = backend.diagnostics()["touchdown_transition"]
    assert data["checks"]["control_reference"]["detail"] == "fixed reference"
    for key in ("x", "y", "yaw"):
        assert data["checks"][key]["value"] == pytest.approx(0.05)
        assert data["checks"][key]["passed"]
    backend.update(now=1.9, dock=disturbed(1.9, distance=0.2),
                   vehicle=vehicle_observation(heading_rad=None))
    data = backend.diagnostics()["touchdown_transition"]
    assert "control_reference" in data["blockers"]
    assert data["checks"]["x"]["value"] is None
    assert "heading unavailable" in data["reason"]
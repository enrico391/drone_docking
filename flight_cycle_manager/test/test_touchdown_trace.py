"""Full-procedure recording must retain frame semantics without affecting flight."""

from dataclasses import replace
import json
import math
from types import SimpleNamespace

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy

from flight_cycle_manager.landing_backend import (
    LandingBackendDecision, LandingBackendOutcome, VelocityCommand, VelocityFrame,
)
from flight_cycle_manager.manager import CycleState, FlightCycleManager, LAND_MODE, GUIDED_MODE
from flight_cycle_manager.pid_landing import PidLandingState
from test_manager_start import airborne_state
from test_pid_landing_backend import central_observation


@pytest.mark.parametrize("operation_mode", ["pid_landing_debug", "pid_align_debug"])
def test_land_trigger_opens_trace_before_guided_ack_and_records_alignment(
    tmp_path, monkeypatch, operation_mode,
):
    """Catch late trace start and missing data during handoff/alignment holds."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=5))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": operation_mode, "pid_align_auto_start": False,
        "pid_hold_at_state": "PID_ALIGN", "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
        "pid_x_k_v": 0.2, "pid_y_k_v": 0.3,
    })
    clock = SimpleNamespace(nanoseconds=100_200_000_000)
    monkeypatch.setattr(manager, "get_clock", lambda: SimpleNamespace(now=lambda: clock))
    try:
        manager.tick(2.0)
        assert not list(tmp_path.glob("touchdown_*.jsonl"))
        vehicle.state = airborne_state(2.1, mode=LAND_MODE)
        manager.tick(2.1)
        trace = next(tmp_path.glob("touchdown_*.jsonl"))
        rows = [json.loads(line) for line in trace.read_text().splitlines()]
        assert rows[0]["event"] == "START"
        assert rows[0]["time_monotonic_s"] == 2.1
        assert rows[0]["flight_mode"] == LAND_MODE
        assert rows[0]["operation_mode"] == operation_mode
        assert rows[0]["trace_id"] == trace.stem
        assert rows[-1]["manager_state"] == "GUIDED_TAKEOVER"
        assert rows[-1]["command_body_flu_m_s"] is None

        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = replace(airborne_state(2.2), heading_rad=0.0,
                                velocity_enu_m_s=(0.1, 0.2, 0.0))
        if operation_mode == "pid_landing_debug":
            # Select alignment directly after handoff; search has its own test below.
            monkeypatch.setattr(manager, "_start_shared_search", manager._start_pid_alignment)
            monkeypatch.setattr(manager, "_advance_guided_search_start",
                                lambda now: manager._start_selected_backend(now))
        manager.tick(2.2)
        dock = replace(central_observation(2.3, distance_m=0.5),
                       forward_m=0.03, left_m=-0.04, source_stamp_s=100.0)
        monkeypatch.setattr(manager, "_dock_observation", lambda now: dock)
        vehicle.state = replace(vehicle.state, heartbeat_at=2.3, position_at=2.3,
                                landed_at=2.3)
        manager.tick(2.3)
        row = json.loads(trace.read_text().splitlines()[-1])
        assert row["state"] == "PID_ALIGN"
        assert row["measured_velocity_body_flu_m_s"] == pytest.approx([0.2, -0.1, 0.0])
        assert row["control_error"]["x"] == 0.03
        assert row["control_error"]["y"] == -0.04
        assert row["diagnostics"]["axes"]["x"]["p"] == pytest.approx(0.03)
        assert row["diagnostics"]["velocity_feedback_terms"]["x"] == pytest.approx(-0.04)
        assert row["diagnostics"]["velocity_feedback_terms"]["y"] == pytest.approx(0.03)
        assert row["diagnostics"]["pid_hold_at_state"] == "PID_ALIGN"
        assert row["source_image_stamp_s"] == 100.0
        assert row["frame_age_s"] == pytest.approx(0.2)
        assert row["frame_age_status"] == "ok"
        assert row["command_send"]["sent"] is True
        assert row["command_send"]["ros_time_s"] == pytest.approx(100.2)
        assert row["command_send"]["end_monotonic_s"] >= row["command_send"]["start_monotonic_s"]
        assert len(list(tmp_path.glob("touchdown_*.jsonl"))) == 1
        vehicle.state = airborne_state(2.4, mode=5)
        manager.tick(2.4)
        terminal_rows = [json.loads(line) for line in trace.read_text().splitlines()]
        assert terminal_rows[-1]["outcome"] == "CANCELLED"
        assert terminal_rows[-2]["manager_state"] == "WAITING_LAND_TRIGGER"
        assert terminal_rows[-2]["flight_mode"] == 5
        assert terminal_rows[-2]["time_monotonic_s"] == 2.4
    finally:
        manager.close()


@pytest.mark.parametrize("stamp,ros_time,status,age", [
    (100.0, 100.5, "ok", 0.5),
    (100.0, 99.0, "future_stamp", -1.0),
    (0.0, 100.5, "missing_stamp", None),
])
def test_trace_keeps_last_image_age_during_target_loss(tmp_path, monkeypatch, stamp, ros_time, status, age):
    """Catch receipt-time substitution and silently discarded clock errors."""
    vehicle = FakeVehicle(airborne_state(2.0))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug", "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager._landing_controller = manager._build_landing_backend()
        manager._start_touchdown_trace(2.0, "airborne LAND trigger")
        manager._dock_pose = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(stamp), nanosec=0)))
        monkeypatch.setattr(manager, "get_clock", lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(ros_time * 1e9))))
        decision = LandingBackendDecision(LandingBackendOutcome.RUNNING,
            "DESCEND_CENTRAL", velocity=VelocityCommand(VelocityFrame.BODY_FLU, 0, 0, 0))
        manager._record_touchdown_sample(2.0, None, manager._vehicle_observation(), decision)
        row = json.loads(next(tmp_path.glob("touchdown_*.jsonl")).read_text().splitlines()[-1])
        assert row["frame_age_status"] == status
        assert row["frame_age_s"] == age
        assert row["dock_usable"] is False
        assert row["measured_velocity_body_flu_m_s"] is None
    finally:
        manager.close()


def test_trace_records_search_commands_in_enu_and_does_not_reuse_them(tmp_path):
    """Catch ENU search commands mislabelled as body velocity or reused on idle ticks."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=5))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug", "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager.tick(2.0)
        vehicle.state = airborne_state(2.1, mode=LAND_MODE)
        manager.tick(2.1)
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(2.2)
        manager.tick(2.2)
        assert manager.state is CycleState.SEARCH_DOCK
        vehicle.state = airborne_state(2.3)
        manager.tick(2.3)
        trace = next(tmp_path.glob("touchdown_*.jsonl"))
        row = json.loads(trace.read_text().splitlines()[-1])
        assert row["manager_state"] == "SEARCH_DOCK"
        assert row["command_frame"] == "MAP_ENU"
        assert row["command_velocity_m_s"] == list(vehicle.velocity_calls[-1][1:4])
        assert row["command_body_flu_m_s"] is None
        assert row["measured_velocity_command_frame_m_s"] == [0.0, 0.0, 0.05]
        assert row["command_send"]["sent"] is True
        assert row["diagnostics"] is None
        # A pending mode transition has no velocity command; the search command
        # must not be attached to this later tick.
        manager._transition(CycleState.GUIDED_TAKEOVER, "waiting for telemetry", force=True)
        manager._guided_search_telemetry_deadline = 5.0
        vehicle.state = replace(airborne_state(2.4), position_enu_m=None)
        manager.tick(2.4)
        row = json.loads(trace.read_text().splitlines()[-1])
        assert row["command_velocity_m_s"] is None
        assert row["command_send"] is None
        assert row["position_enu_m"] is None
        assert len(list(tmp_path.glob("touchdown_*.jsonl"))) == 1
    finally:
        manager.close()


def test_trace_failure_at_land_trigger_does_not_block_guided_request(tmp_path):
    """Catch recording failure preventing the authorized docking handoff."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=5))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug", "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager.tick(2.0)
        manager._touchdown_trace.write = lambda row: (_ for _ in ()).throw(OSError("disk full"))
        vehicle.state = airborne_state(2.1, mode=LAND_MODE)
        manager.tick(2.1)
        assert vehicle.set_mode_calls == [GUIDED_MODE]
        assert manager.state is CycleState.GUIDED_TAKEOVER
        assert not manager._touchdown_trace.active
    finally:
        manager.close()


def test_trace_close_failure_cannot_suppress_velocity_send(tmp_path, monkeypatch):
    """Catch an I/O cleanup error escaping the diagnostic failure handler."""
    vehicle = FakeVehicle(airborne_state(2.0))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug", "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager._start_touchdown_trace(2.0, "airborne LAND trigger")
        original_file = manager._touchdown_trace._file

        def failed_close():
            original_file.close()
            raise OSError("flush failed during close")

        manager._touchdown_trace._file = SimpleNamespace(name=original_file.name, close=failed_close)
        monkeypatch.setattr(manager, "get_clock", lambda: (_ for _ in ()).throw(RuntimeError("clock unavailable")))
        assert manager._send_traced_velocity(VelocityFrame.BODY_FLU, 0.1, -0.2, 0.0, now=2.1)
        assert vehicle.velocity_calls[-1][1:4] == (0.1, -0.2, 0.0)
        assert not manager._touchdown_trace.active
    finally:
        manager.close()


def test_alignment_auto_start_and_repeated_land_attempts_get_distinct_traces(tmp_path):
    """Catch missed auto-start recordings or mixing two attempts in one file."""
    vehicle = FakeVehicle(airborne_state(2.0, mode=5))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_align_debug", "pid_align_auto_start": True,
        "touchdown_trace_enabled": True, "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager.tick(2.0)
        first = next(tmp_path.glob("touchdown_*.jsonl"))
        assert json.loads(first.read_text().splitlines()[0])["reason"] == "alignment auto-start"
        vehicle.finish_command(current_mode=GUIDED_MODE)
        vehicle.state = airborne_state(2.1)
        manager.tick(2.1)
        vehicle.state = airborne_state(2.2, mode=5)
        manager.tick(2.2)
        assert json.loads(first.read_text().splitlines()[-1])["outcome"] == "CANCELLED"
        vehicle.state = airborne_state(2.3, mode=LAND_MODE)
        manager.tick(2.3)
        traces = list(tmp_path.glob("touchdown_*.jsonl"))
        assert len(traces) == 2
        second = next(path for path in traces if path != first)
        assert json.loads(second.read_text().splitlines()[0])["reason"] == "airborne LAND trigger"
    finally:
        manager.close()


def test_completed_trace_keeps_final_command_telemetry_and_reference(tmp_path):
    """Catch END preceding the disarmed sample or dropping the frozen target."""
    vehicle = FakeVehicle(replace(airborne_state(2.0), armed=False, flying=False, heading_rad=0.0))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug", "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager._start_selected_backend(2.0)
        backend = manager._landing_controller
        backend._controller._state = PidLandingState.DISARM
        backend._controller._latch_touchdown_reference(central_observation(2.0),
                                                       manager._vehicle_observation())
        reference = backend.diagnostics()["touchdown_reference_enu_m"]
        manager._start_touchdown_trace(2.0, "airborne LAND trigger")
        manager.tick(2.0)
        rows = [json.loads(line) for line in next(tmp_path.glob("touchdown_*.jsonl")).read_text().splitlines()]
        assert rows[-1]["event"] == "END"
        assert rows[-1]["outcome"] == "COMPLETED"
        assert rows[-2]["manager_state"] == "DOCKED"
        assert rows[-2]["armed"] is False
        assert rows[-2]["command_body_flu_m_s"] == [0.0, 0.0, 0.0]
        assert rows[-2]["command_send"]["sent"] is True
        assert rows[-2]["frozen_target_enu_m"] == list(reference)
    finally:
        manager.close()


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_final_approach_records_commit_and_blind_descent(tmp_path):
    vehicle = FakeVehicle(airborne_state(2.0))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug",
        "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager._landing_controller = manager._build_landing_backend()
        backend = manager._landing_controller
        backend.start(now=2.0, dock=None, vehicle=manager._vehicle_observation())
        manager._start_touchdown_trace(2.0, "airborne LAND trigger")
        backend._controller._state = PidLandingState.DESCEND_CENTRAL
        sample_vehicle = replace(
            manager._vehicle_observation(),
            velocity_enu_m_s=(0.03, -0.04, 0.05),
        )
        dock = central_observation(2.0, distance_m=0.35)
        command = VelocityCommand(VelocityFrame.BODY_FLU, 0.04, -0.02, -0.10, 0.01)
        decision = LandingBackendDecision(LandingBackendOutcome.RUNNING,
                                          "DESCEND_CENTRAL", velocity=command)
        manager._record_touchdown_sample(2.0, dock, sample_vehicle, decision)
        backend._controller._latch_touchdown_reference(dock, sample_vehicle)
        backend._controller._state = PidLandingState.TOUCHDOWN_APPROACH
        decision = LandingBackendDecision(LandingBackendOutcome.RUNNING,
                                          "TOUCHDOWN_APPROACH", velocity=command)
        manager._record_touchdown_sample(2.05, dock, sample_vehicle, decision)
        disturbed_vehicle = replace(
            sample_vehicle,
            position_enu_m=(sample_vehicle.position_enu_m[0] + 0.03,
                            sample_vehicle.position_enu_m[1] - 0.04,
                            sample_vehicle.position_enu_m[2]),
            velocity_enu_m_s=(-0.06, 0.08, sample_vehicle.velocity_enu_m_s[2]),
        )
        manager._record_touchdown_sample(2.1, None, disturbed_vehicle, decision)
        manager._transition(CycleState.WAITING_LAND_TRIGGER, "cancelled", force=True)
        trace = next(tmp_path.glob("touchdown_*.jsonl"))
        rows = [json.loads(line) for line in trace.read_text().splitlines()]
        assert rows[0]["event"] == "START"
        rows = rows[1:]
        assert [row["state"] for row in rows[:-1]] == [
            "DESCEND_CENTRAL", "TOUCHDOWN_APPROACH", "TOUCHDOWN_APPROACH"]
        assert rows[0]["frozen_target_enu_m"] is None
        assert rows[0]["position_error_to_frozen_target_m"] is None
        assert rows[0]["frozen_error_norm_m"] is None
        assert rows[0]["horizontal_speed_m_s"] == pytest.approx(0.05)
        assert rows[1]["frozen_target_enu_m"] is not None
        assert rows[2]["frozen_target_enu_m"] == rows[1]["frozen_target_enu_m"]
        assert rows[1]["position_error_to_frozen_target_m"] == pytest.approx([
            rows[1]["frozen_target_enu_m"][0] - sample_vehicle.position_enu_m[0],
            rows[1]["frozen_target_enu_m"][1] - sample_vehicle.position_enu_m[1],
        ])
        assert rows[2]["position_error_to_frozen_target_m"] == pytest.approx([
            rows[1]["position_error_to_frozen_target_m"][0] - 0.03,
            rows[1]["position_error_to_frozen_target_m"][1] + 0.04,
        ])
        assert rows[1]["frozen_error_norm_m"] == pytest.approx(math.hypot(
            *rows[1]["position_error_to_frozen_target_m"]
        ))
        assert rows[2]["frozen_error_norm_m"] == pytest.approx(math.hypot(
            *rows[2]["position_error_to_frozen_target_m"]
        ))
        assert rows[2]["dock_offset_body_flu_m"] is None
        assert rows[2]["dock_distance_m"] is None
        assert rows[2]["position_enu_m"] == list(disturbed_vehicle.position_enu_m)
        assert rows[2]["velocity_enu_m_s"] == list(disturbed_vehicle.velocity_enu_m_s)
        assert rows[2]["horizontal_speed_m_s"] == pytest.approx(0.10)
        for row in rows[:-1]:
            assert "ekf_position_enu_m" not in row
            assert "ekf_velocity_enu_m_s" not in row
        assert rows[2]["command_frame"] == "BODY_FLU"
        assert rows[2]["command_body_flu_m_s"] == [0.04, -0.02, -0.1]
        assert rows[2]["command_yaw_rate_rad_s"] == 0.01
        assert rows[-1]["outcome"] == "CANCELLED"
    finally:
        manager.close()


def test_trace_is_opt_in_and_closes_on_shutdown(tmp_path):
    vehicle = FakeVehicle(airborne_state(2.0))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug",
        "touchdown_trace_enabled": False,
        "landing_attempt_log_directory": str(tmp_path),
    })
    manager.close()
    assert list(tmp_path.iterdir()) == []


def test_trace_shutdown_and_next_attempt_have_distinct_files(tmp_path):
    vehicle = FakeVehicle(airborne_state(2.0))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug",
        "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager._landing_controller = manager._build_landing_backend()
        manager._start_touchdown_trace(2.0, "airborne LAND trigger")
        decision = LandingBackendDecision(
            LandingBackendOutcome.RUNNING, "DESCEND_CENTRAL",
            velocity=VelocityCommand(VelocityFrame.BODY_FLU, 0, 0, -0.1))
        manager._record_touchdown_sample(2.0, central_observation(2.0),
                                         manager._vehicle_observation(), decision)
        manager._transition(CycleState.WAITING_LAND_TRIGGER, "cancelled", force=True)
        manager._start_touchdown_trace(2.1, "airborne LAND trigger")
        manager._record_touchdown_sample(2.1, central_observation(2.1),
                                         manager._vehicle_observation(), decision)
    finally:
        manager.close()
    traces = sorted(tmp_path.glob("touchdown_*.jsonl"))
    assert len(traces) == 2
    first = [json.loads(line) for line in traces[0].read_text().splitlines()]
    second = [json.loads(line) for line in traces[1].read_text().splitlines()]
    assert first[-1]["outcome"] == "CANCELLED"
    assert second[-1]["outcome"] == "SHUTDOWN"


def test_trace_write_failure_does_not_raise_into_control(tmp_path):
    vehicle = FakeVehicle(airborne_state(2.0))
    manager = FlightCycleManager(vehicle=vehicle, parameter_values={
        "operation_mode": "pid_landing_debug",
        "touchdown_trace_enabled": True,
        "landing_attempt_log_directory": str(tmp_path),
    })
    try:
        manager._landing_controller = manager._build_landing_backend()
        manager._start_touchdown_trace(2.0, "airborne LAND trigger")
        manager._touchdown_trace.write = lambda row: (_ for _ in ()).throw(OSError("disk full"))
        decision = LandingBackendDecision(
            LandingBackendOutcome.RUNNING, "DESCEND_CENTRAL",
            velocity=VelocityCommand(VelocityFrame.BODY_FLU, 0, 0, -0.1))
        manager._record_touchdown_sample(2.0, central_observation(2.0),
                                         manager._vehicle_observation(), decision)
        assert not manager._touchdown_trace.active
    finally:
        manager.close()

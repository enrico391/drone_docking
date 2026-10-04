"""Final approach recording must retain frame semantics without affecting flight."""

from dataclasses import replace
import json
import math

import pytest

from vehicle_fakes import FakeVehicle, install_ros_fakes

install_ros_fakes()

import rclpy

from flight_cycle_manager.landing_backend import (
    LandingBackendDecision, LandingBackendOutcome, VelocityCommand, VelocityFrame,
)
from flight_cycle_manager.manager import CycleState, FlightCycleManager
from flight_cycle_manager.pid_landing import PidLandingState
from test_manager_start import airborne_state
from test_pid_landing_backend import central_observation


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
        decision = LandingBackendDecision(
            LandingBackendOutcome.RUNNING, "DESCEND_CENTRAL",
            velocity=VelocityCommand(VelocityFrame.BODY_FLU, 0, 0, -0.1))
        manager._record_touchdown_sample(2.0, central_observation(2.0),
                                         manager._vehicle_observation(), decision)
        manager._transition(CycleState.WAITING_LAND_TRIGGER, "cancelled", force=True)
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
        manager._touchdown_trace.write = lambda row: (_ for _ in ()).throw(OSError("disk full"))
        decision = LandingBackendDecision(
            LandingBackendOutcome.RUNNING, "DESCEND_CENTRAL",
            velocity=VelocityCommand(VelocityFrame.BODY_FLU, 0, 0, -0.1))
        manager._record_touchdown_sample(2.0, central_observation(2.0),
                                         manager._vehicle_observation(), decision)
        assert not manager._touchdown_trace.active
    finally:
        manager.close()
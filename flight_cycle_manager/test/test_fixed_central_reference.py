"""Fixed target PID tests, including both landing-hold choices."""
from dataclasses import replace
import math

import pytest

from flight_cycle_manager.pid_landing import PidLandingState
from flight_cycle_manager.pid_landing_backend import PidLandingBackend
from test_pid_landing_backend import (
    central_observation, dock_observation,
    pid_config, vehicle_observation,
)


def backend_for(mode, hold=None):
    backend = PidLandingBackend(pid_config(fixed_central_reference_mode=mode),
                                hold_at_state=hold)
    backend.start(now=0.0, dock=dock_observation(0.0), vehicle=vehicle_observation(heading_rad=0.0))
    return backend


def central_entry(backend):
    # Helpers use a heading-less fake; supply heading explicitly to latch.
    low = dock_observation(0.1, forward_m=0.05, left_m=-0.05, distance_m=1.0)
    backend.update(now=0.1, dock=low, vehicle=vehicle_observation(heading_rad=0.0))
    backend.update(now=1.11, dock=replace(low, received_at=1.11, source_stamp_s=1.11),
                   vehicle=vehicle_observation(heading_rad=0.0))
    backend.update(now=1.2, dock=central_observation(1.2),
                   vehicle=vehicle_observation(heading_rad=0.0))
    return backend.update(now=1.71, dock=central_observation(1.71),
                          vehicle=vehicle_observation(heading_rad=0.0))


def disturbed(now, distance=0.9):
    return replace(central_observation(now, distance_m=distance),
                   forward_m=-0.4, left_m=0.3, yaw_error_rad=-0.6)


def shifted_vehicle(east=4.0, north=-2.0, heading=0.0):
    return vehicle_observation(position_enu_m=(east, north, 4.0), heading_rad=heading)


@pytest.mark.parametrize("mode,hold", [
    ("descend_central", PidLandingState.DESCEND_CENTRAL),
    ("descend_central", PidLandingState.HOLD_CENTRAL),
])
def test_descend_latch_ignores_later_camera_xy_yaw(mode, hold):
    backend = backend_for(mode, hold)
    assert central_entry(backend).substate == "DESCEND_CENTRAL"
    reference = backend.diagnostics()["fixed_reference_enu_m"]
    assert reference == pytest.approx((4.05, -1.95))
    result = backend.update(now=1.8, dock=disturbed(1.8), vehicle=shifted_vehicle())
    assert result.velocity.velocity_x > 0
    assert result.velocity.velocity_y < 0
    assert result.velocity.yaw_rate_rad_s > 0
    assert backend.diagnostics()["control_error"] == pytest.approx(
        {"x": 0.05, "y": -0.05, "yaw": 0.05})
    assert backend.diagnostics()["fixed_reference_enu_m"] == reference


def test_vehicle_motion_changes_error_even_if_camera_pose_does_not():
    backend = backend_for("descend_central", PidLandingState.DESCEND_CENTRAL)
    central_entry(backend)
    backend.update(now=1.8, dock=disturbed(1.8),
                   vehicle=shifted_vehicle(east=4.0, north=-1.90, heading=0.1))
    error = backend.diagnostics()["control_error"]
    assert error["x"] < 0
    assert error["yaw"] == pytest.approx(0.15)
    assert math.hypot(error["x"], error["y"]) == pytest.approx(math.hypot(0.05, -0.05))


def test_hold_central_latches_at_hold_not_during_descent_and_keeps_it():
    backend = backend_for("hold_central", PidLandingState.HOLD_CENTRAL)
    central_entry(backend)
    assert not backend.diagnostics()["fixed_reference_active"]
    backend.update(now=1.8, dock=central_observation(1.8, distance_m=0.20),
                   vehicle=shifted_vehicle())
    backend.update(now=2.81, dock=central_observation(2.81, distance_m=0.20),
                   vehicle=shifted_vehicle())
    assert backend.diagnostics()["fixed_reference_active"]
    reference = backend.diagnostics()["fixed_reference_enu_m"]
    decision = backend.update(now=3.0, dock=disturbed(3.0, 0.21),
                              vehicle=shifted_vehicle())
    assert decision.substate == "HOLD_CENTRAL"
    assert backend.diagnostics()["fixed_reference_enu_m"] == reference
    assert backend.diagnostics()["control_error"]["x"] == pytest.approx(0.05)


def test_hold_central_mode_with_descent_hold_stays_inactive():
    backend = backend_for("hold_central", PidLandingState.DESCEND_CENTRAL)
    central_entry(backend)
    backend.update(now=1.8, dock=disturbed(1.8), vehicle=shifted_vehicle())
    assert not backend.diagnostics()["fixed_reference_active"]
    assert backend.diagnostics()["control_error"]["x"] == pytest.approx(-0.4)


def test_missing_heading_prevents_latching_and_stops_control():
    backend = backend_for("descend_central")
    low = dock_observation(0.1, distance_m=1.0, forward_m=0.05, left_m=-0.05)
    backend.update(now=0.1, dock=low, vehicle=shifted_vehicle())
    backend.update(now=1.11, dock=replace(low, received_at=1.11), vehicle=shifted_vehicle())
    backend.update(now=1.2, dock=central_observation(1.2), vehicle=shifted_vehicle())
    result = backend.update(now=1.71, dock=central_observation(1.71),
                            vehicle=vehicle_observation(heading_rad=None))
    assert result.substate == "ACQUIRE_CENTRAL"
    assert result.velocity.velocity_x == 0.0
    assert not backend.diagnostics()["fixed_reference_active"]


@pytest.mark.parametrize("invalid", ["both", "", "OFF", 1])
def test_invalid_mode_rejected(invalid):
    with pytest.raises(ValueError, match="fixed_central_reference_mode"):
        pid_config(fixed_central_reference_mode=invalid)


def test_direct_outer_to_central_latches_reference():
    backend = backend_for("descend_central")
    outer = dock_observation(0.1, distance_m=2.0,
                             forward_m=0.05, left_m=-0.05)
    backend.update(now=0.1, dock=outer, vehicle=shifted_vehicle())
    assert backend.update(now=1.11, dock=replace(outer, received_at=1.11),
                          vehicle=shifted_vehicle()).substate == "DESCEND_OUTER"
    backend.update(now=1.2, dock=central_observation(1.2, distance_m=1.5),
                   vehicle=shifted_vehicle())
    entered = backend.update(now=1.71, dock=central_observation(1.71, distance_m=0.9),
                             vehicle=shifted_vehicle())
    assert entered.substate == "DESCEND_CENTRAL"
    assert backend.diagnostics()["fixed_reference_enu_m"] == pytest.approx((4.05, -1.95))


def test_reference_cleared_on_loss_and_new_attempt():
    backend = backend_for("descend_central")
    central_entry(backend)
    assert backend.diagnostics()["fixed_reference_active"]
    backend.update(now=1.8, dock=None, vehicle=shifted_vehicle())
    lost = backend.update(now=6.0, dock=None, vehicle=shifted_vehicle())
    assert lost.substate == "RECOVERY_CLIMB"
    assert not backend.diagnostics()["fixed_reference_active"]
    backend.start(now=7.0, dock=None, vehicle=shifted_vehicle())
    assert backend.diagnostics()["fixed_reference_enu_m"] is None


def test_missing_heading_while_fixed_stops_commands():
    backend = backend_for("descend_central", PidLandingState.DESCEND_CENTRAL)
    central_entry(backend)
    result = backend.update(now=1.8, dock=central_observation(1.8),
                            vehicle=vehicle_observation(heading_rad=None))
    assert result.velocity.velocity_x == 0.0
    assert result.velocity.velocity_z == 0.0


def test_fixed_reference_kept_for_touchdown_and_hold_release():
    backend = backend_for("descend_central", PidLandingState.HOLD_CENTRAL)
    central_entry(backend)
    backend.update(now=1.8, dock=central_observation(1.8, distance_m=0.2),
                   vehicle=shifted_vehicle())
    backend.update(now=2.81, dock=central_observation(2.81, distance_m=0.2),
                   vehicle=shifted_vehicle())
    reference = backend.diagnostics()["fixed_reference_enu_m"]
    backend.set_hold_at_state(None)
    backend.update(now=3.0, dock=central_observation(3.0, distance_m=0.2),
                   vehicle=shifted_vehicle())
    committed = backend.update(now=4.1, dock=central_observation(4.1, distance_m=0.2),
                               vehicle=shifted_vehicle())
    assert committed.substate == "TOUCHDOWN_APPROACH"
    assert backend._controller._touchdown_reference_enu_m == reference


def test_off_mode_keeps_live_central_tracking():
    backend = backend_for("off", PidLandingState.DESCEND_CENTRAL)
    central_entry(backend)
    decision = backend.update(now=1.8, dock=disturbed(1.8),
                              vehicle=shifted_vehicle())
    assert not backend.diagnostics()["fixed_reference_active"]
    assert backend.diagnostics()["control_error"]["x"] == pytest.approx(-0.4)
    assert decision.velocity.velocity_x < 0.0


def test_live_enable_in_central_hold_waits_for_fresh_tag_then_latches():
    backend = backend_for("off", PidLandingState.DESCEND_CENTRAL)
    central_entry(backend)
    backend.set_fixed_central_reference_mode("descend_central")
    assert backend.diagnostics()["fixed_reference_pending"]
    assert backend.update(now=1.8, dock=None, vehicle=shifted_vehicle()).velocity.velocity_x == 0
    assert backend.diagnostics()["fixed_reference_pending"]
    latched = backend.update(now=1.9, dock=central_observation(1.9),
                             vehicle=shifted_vehicle(east=5.0))
    assert latched.velocity.velocity_x == 0
    assert backend.diagnostics()["fixed_reference_enu_m"] == pytest.approx((5.05, -1.95))
    assert not backend.diagnostics()["fixed_reference_pending"]
    backend.update(now=2.0, dock=disturbed(2.0), vehicle=shifted_vehicle(east=5.0))
    assert backend.diagnostics()["control_error"]["x"] == pytest.approx(0.05)


def test_live_enable_hold_mode_after_entering_hold_latches_new_observation():
    backend = backend_for("off", PidLandingState.HOLD_CENTRAL)
    central_entry(backend)
    backend.update(now=1.8, dock=central_observation(1.8, distance_m=0.2),
                   vehicle=shifted_vehicle())
    assert backend.update(now=2.81, dock=central_observation(2.81, distance_m=0.2),
                          vehicle=shifted_vehicle()).substate == "HOLD_CENTRAL"
    backend.set_fixed_central_reference_mode("hold_central")
    assert backend.diagnostics()["fixed_reference_pending"]
    pending = backend.update(now=3.0, dock=None, vehicle=shifted_vehicle())
    assert pending.velocity.velocity_z == 0
    assert not backend.diagnostics()["fixed_reference_active"]
    latched = backend.update(now=3.1, dock=central_observation(3.1, distance_m=0.2),
                             vehicle=shifted_vehicle(east=5.0))
    assert latched.velocity.velocity_x == 0
    assert backend.diagnostics()["fixed_reference_enu_m"] == pytest.approx((5.05, -1.95))
    backend.update(now=3.2, dock=disturbed(3.2, 0.2),
                   vehicle=shifted_vehicle(east=5.0))
    assert backend.diagnostics()["control_error"]["x"] == pytest.approx(0.05)


def test_live_switch_between_fixed_modes_discards_old_target():
    backend = backend_for("descend_central", PidLandingState.DESCEND_CENTRAL)
    central_entry(backend)
    assert backend.diagnostics()["fixed_reference_active"]
    backend.set_fixed_central_reference_mode("hold_central")
    assert not backend.diagnostics()["fixed_reference_active"]
    assert not backend.diagnostics()["fixed_reference_pending"]
    backend.update(now=1.8, dock=disturbed(1.8), vehicle=shifted_vehicle())
    assert backend.diagnostics()["control_error"]["x"] == pytest.approx(-0.4)


def test_live_disable_fixed_reference_uses_visual_error_immediately():
    backend = backend_for("descend_central", PidLandingState.DESCEND_CENTRAL)
    central_entry(backend)
    backend.set_fixed_central_reference_mode("off")
    assert not backend.diagnostics()["fixed_reference_active"]
    backend.update(now=1.8, dock=disturbed(1.8), vehicle=shifted_vehicle())
    assert backend.diagnostics()["control_error"]["x"] == pytest.approx(-0.4)


def test_live_mode_change_before_transition_latches_on_entry():
    backend = backend_for("off")
    backend.set_fixed_central_reference_mode("descend_central")
    assert not backend.diagnostics()["fixed_reference_pending"]
    assert central_entry(backend).substate == "DESCEND_CENTRAL"
    assert backend.diagnostics()["fixed_reference_active"]


def test_live_mode_change_after_touchdown_rejected_without_mutation():
    backend = backend_for("descend_central")
    central_entry(backend)
    backend._controller._state = PidLandingState.TOUCHDOWN_APPROACH
    with pytest.raises(ValueError, match="touchdown commitment"):
        backend.set_fixed_central_reference_mode("off")
    assert backend.diagnostics()["fixed_central_reference_mode"] == "descend_central"

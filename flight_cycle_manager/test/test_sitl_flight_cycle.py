"""Behavior-level SITL acceptance scenarios for the flight cycle manager."""

from __future__ import annotations

import pytest

from sitl_mission_fixture import items_without_nav_land, sitl_stack


@pytest.mark.sitl
def test_invalid_onboard_mission_refuses_to_arm(sitl_stack):
    """Break caught: a mission missing NAV_LAND can still arm a docking cycle."""
    sitl_stack.upload_mission(items_without_nav_land())
    sitl_stack.start_manager(backend="pid")
    sitl_stack.call_start_cycle()
    sitl_stack.wait_state("INVALID_ONBOARD_MISSION", timeout=15.0)

    assert sitl_stack.vehicle_armed() is False

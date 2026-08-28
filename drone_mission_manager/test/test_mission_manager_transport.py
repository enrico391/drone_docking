"""Integration boundary tests for MissionManager's direct vehicle transport."""

from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

from drone_mission_manager.mission_manager import MissionManager
from drone_mission_manager.pymavlink_vehicle import VehicleState


MISSION_FILE = (
    Path(__file__).parents[1] / "config" / "missions" / "simple_circle.yaml"
)


class FakeVehicle:
    """Minimal direct-vehicle boundary needed during node construction."""

    def __init__(self):
        self.state = VehicleState()
        self.closed = False

    def poll(self, now):
        pass

    def close(self):
        self.closed = True


def test_manager_creates_no_ap_dds_vehicle_entities():
    """Break caught: an AP_DDS topic or service remains in the direct manager."""
    context = Context()
    rclpy.init(context=context)
    vehicle = FakeVehicle()
    node = None
    try:
        node = MissionManager(
            vehicle_factory=lambda _node: vehicle,
            parameter_overrides=[
                Parameter("mission_file", value=str(MISSION_FILE)),
                Parameter("landing_backend", value="ros"),
            ],
            context=context,
        )

        topics = {name for name, _types in node.get_topic_names_and_types()}
        services = {
            name for name, _types in node.get_service_names_and_types()
        }
        assert not {
            "/ap/v1/pose/filtered",
            "/ap/v1/status",
            "/ap/v1/cmd_vel",
        } & topics
        assert not {
            "/ap/v1/prearm_check",
            "/ap/v1/mode_switch",
            "/ap/v1/arm_motors",
            "/ap/v1/experimental/takeoff",
            "/ap/v1/get_parameters",
        } & services
        assert {"/dock/pose", "/dock/mode", "/mission_manager/state"} <= topics
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown(context=context)

    assert vehicle.closed is True

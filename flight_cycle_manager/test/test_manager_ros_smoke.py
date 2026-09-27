"""Smoke coverage for the real sourced ROS Python boundary."""

from __future__ import annotations

from vehicle_fakes import FakeVehicle, install_ros_fakes


def test_real_humble_ros_constructs_manager_and_main_boundary(monkeypatch):
    """Break caught: test fakes replace real rclpy before manager construction."""
    install_ros_fakes()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    import flight_cycle_manager.manager as manager_module
    from flight_cycle_manager.manager import FlightCycleManager

    assert rclpy.__file__.startswith("/opt/ros/humble/")
    assert Node.__module__ == "rclpy.node"
    assert Parameter.__module__ == "rclpy.parameter"
    assert QoSProfile.__module__ == "rclpy.qos"
    assert String.__module__.startswith("std_msgs.msg")
    assert Trigger.__module__.startswith("std_srvs.srv")

    rclpy.init()
    manager = FlightCycleManager(
        vehicle=FakeVehicle(), parameter_values={"update_rate_hz": 10.0}
    )
    try:
        assert isinstance(manager, Node)
        assert manager.get_parameter("update_rate_hz").value == 10.0
        assert manager._start_service.srv_type is Trigger
    finally:
        manager.close()
        rclpy.shutdown()

    events: list[str] = []

    class MainNode:
        def close(self) -> None:
            events.append("close")

    monkeypatch.setattr(manager_module, "FlightCycleManager", MainNode)
    monkeypatch.setattr(manager_module.rclpy, "spin", lambda node: events.append("spin"))
    manager_module.main()

    assert events == ["spin", "close"]

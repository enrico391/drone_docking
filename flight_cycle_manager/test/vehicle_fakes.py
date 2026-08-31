"""Deterministic MAVLink boundary fakes shared by manager lifecycle tests."""

from __future__ import annotations

import sys
from types import ModuleType
from typing import Iterable, Optional, Sequence

from flight_cycle_manager.mission_contract import MissionItem
from flight_cycle_manager.pymavlink_vehicle import (
    MavlinkFuture,
    MissionDownloadResult,
    VehicleCommandResult,
    VehicleState,
)


def install_ros_fakes() -> None:
    """Install the narrow ROS API boundary manager unit tests need.

    The workspace image intentionally has ROS interface metadata but no Python
    ``rclpy`` wheel.  These fakes replace only ROS transport/lifecycle plumbing;
    the manager state machine and MAVLink boundary remain real test subjects.
    """
    if "rclpy" in sys.modules:
        return

    class Parameter:
        def __init__(self, name: str = "", value: object = None) -> None:
            self.name = name
            self.value = value

    class _ParameterValue:
        def __init__(self, value: object) -> None:
            self.value = value

    class _Publisher:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class _Timer:
        def __init__(self, period: float, callback: object) -> None:
            self.period = period
            self.callback = callback
            self.cancelled = False

        def cancel(self) -> None:
            self.cancelled = True

    class _Logger:
        def info(self, message: str) -> None:
            del message

        def warning(self, message: str) -> None:
            del message

        def error(self, message: str) -> None:
            del message

    class Node:
        def __init__(
            self,
            name: str,
            *,
            parameter_overrides: Optional[Sequence[Parameter]] = None,
        ) -> None:
            self.name = name
            self._parameter_values = {
                parameter.name: parameter.value
                for parameter in (parameter_overrides or ())
            }
            self.destroyed = False

        def declare_parameter(self, name: str, value: object) -> _ParameterValue:
            self._parameter_values.setdefault(name, value)
            return _ParameterValue(self._parameter_values[name])

        def get_parameter(self, name: str) -> _ParameterValue:
            return _ParameterValue(self._parameter_values[name])

        def create_publisher(self, message_type: object, topic: str, qos: object) -> _Publisher:
            del message_type, qos
            return _Publisher(topic)

        def create_service(self, service_type: object, topic: str, callback: object) -> object:
            return type("Service", (), {"topic": topic, "callback": callback, "service_type": service_type})()

        def create_timer(self, period: float, callback: object) -> _Timer:
            return _Timer(period, callback)

        def get_logger(self) -> _Logger:
            return _Logger()

        def destroy_node(self) -> None:
            self.destroyed = True

    class QoSProfile:
        def __init__(self, *, depth: int, reliability: object, durability: object) -> None:
            self.depth = depth
            self.reliability = reliability
            self.durability = durability

    class ReliabilityPolicy:
        RELIABLE = "reliable"

    class DurabilityPolicy:
        TRANSIENT_LOCAL = "transient_local"

    class String:
        def __init__(self) -> None:
            self.data = ""

    class Trigger:
        class Request:
            pass

        class Response:
            def __init__(self) -> None:
                self.success = False
                self.message = ""

    rclpy_module = ModuleType("rclpy")
    rclpy_module.init = lambda args=None: None
    rclpy_module.shutdown = lambda: None
    rclpy_module.spin = lambda node: None
    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    parameter_module = ModuleType("rclpy.parameter")
    parameter_module.Parameter = Parameter
    qos_module = ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = DurabilityPolicy
    qos_module.QoSProfile = QoSProfile
    qos_module.ReliabilityPolicy = ReliabilityPolicy
    std_msgs_module = ModuleType("std_msgs")
    std_msgs_msg_module = ModuleType("std_msgs.msg")
    std_msgs_msg_module.String = String
    std_srvs_module = ModuleType("std_srvs")
    std_srvs_srv_module = ModuleType("std_srvs.srv")
    std_srvs_srv_module.Trigger = Trigger
    sys.modules.update(
        {
            "rclpy": rclpy_module,
            "rclpy.node": node_module,
            "rclpy.parameter": parameter_module,
            "rclpy.qos": qos_module,
            "std_msgs": std_msgs_module,
            "std_msgs.msg": std_msgs_msg_module,
            "std_srvs": std_srvs_module,
            "std_srvs.srv": std_srvs_srv_module,
        }
    )


class FakeVehicle:
    """In-memory implementation of the manager's asynchronous vehicle boundary."""

    def __init__(self, state: Optional[VehicleState] = None) -> None:
        self.state = state or VehicleState()
        self.download_calls = 0
        self.parameter_calls: list[tuple[str, ...]] = []
        self.prearm_calls = 0
        self.set_mode_calls: list[int] = []
        self.arm_calls: list[bool] = []
        self.takeoff_calls: list[float] = []
        self.set_mission_current_calls: list[tuple[int, bool]] = []
        self.start_mission_calls = 0
        self.land_calls = 0
        self.rtl_calls = 0
        self.mission_mutation_calls: list[str] = []
        self.closed = False
        self._active_name = ""
        self._active_future: Optional[MavlinkFuture[object]] = None

    def poll(self, now: float) -> None:
        del now

    def ready(self, now: float, heartbeat_timeout_s: float = 2.5) -> bool:
        return (
            self.state.connected
            and 0.0 <= now - self.state.heartbeat_at <= heartbeat_timeout_s
        )

    def download_mission_async(
        self, now: float
    ) -> MavlinkFuture[MissionDownloadResult]:
        del now
        self.download_calls += 1
        return self._begin("mission_download")  # type: ignore[return-value]

    def get_parameters_async(
        self, names: Sequence[str], now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.parameter_calls.append(tuple(names))
        return self._begin("parameters")  # type: ignore[return-value]

    def prearm_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.prearm_calls += 1
        return self._begin("prearm")  # type: ignore[return-value]

    def set_mode_async(
        self, mode: int, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.set_mode_calls.append(mode)
        return self._begin("set_mode")  # type: ignore[return-value]

    def arm_async(
        self, arm: bool, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.arm_calls.append(arm)
        return self._begin("arm")  # type: ignore[return-value]

    def takeoff_async(
        self, altitude_m: float, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.takeoff_calls.append(altitude_m)
        return self._begin("takeoff")  # type: ignore[return-value]

    def set_mission_current_async(
        self, sequence: int, *, reset: bool, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.set_mission_current_calls.append((sequence, reset))
        return self._begin("set_mission_current")  # type: ignore[return-value]

    def start_mission_async(
        self, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.start_mission_calls += 1
        return self._begin("start_mission")  # type: ignore[return-value]

    def land_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.land_calls += 1
        return self._begin("land")  # type: ignore[return-value]

    def rtl_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        del now
        self.rtl_calls += 1
        return self._begin("rtl")  # type: ignore[return-value]

    def finish_download(
        self,
        items: Iterable[MissionItem] = (),
        *,
        success: bool = True,
        message: str = "",
    ) -> None:
        future = self._take_active("mission_download")
        future._finish(
            MissionDownloadResult(success, tuple(items), message=message)
        )

    def finish_parameters(
        self,
        parameters: Optional[dict[str, float]] = None,
        *,
        success: bool = True,
        message: str = "",
    ) -> None:
        future = self._take_active("parameters")
        future._finish(
            VehicleCommandResult(
                success,
                message,
                parameters=dict(parameters or {}),
            )
        )

    def finish_command(
        self,
        *,
        success: bool = True,
        message: str = "",
        current_mode: Optional[int] = None,
    ) -> None:
        future = self._take_active()
        future._finish(VehicleCommandResult(success, message, current_mode))

    def close(self) -> None:
        self.closed = True

    def _begin(self, name: str) -> MavlinkFuture[object]:
        if self._active_future is not None:
            raise RuntimeError(f"operation already active: {self._active_name}")
        future: MavlinkFuture[object] = MavlinkFuture()
        self._active_name = name
        self._active_future = future
        return future

    def _take_active(self, expected_name: Optional[str] = None) -> MavlinkFuture[object]:
        if self._active_future is None:
            raise RuntimeError("no active fake vehicle operation")
        if expected_name is not None and self._active_name != expected_name:
            raise AssertionError(
                f"expected {expected_name}, active operation is {self._active_name}"
            )
        future = self._active_future
        self._active_name = ""
        self._active_future = None
        return future

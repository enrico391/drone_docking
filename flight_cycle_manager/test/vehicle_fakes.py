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

    Prefer the sourced ROS runtime whenever all manager imports are available.
    The fakes exist only for environments where that import boundary is absent;
    the manager state machine and MAVLink boundary remain real test subjects.
    """
    try:
        import rclpy  # noqa: F401
        from rclpy.node import Node  # noqa: F401
        from rclpy.parameter import Parameter  # noqa: F401
        from geometry_msgs.msg import PoseStamped  # noqa: F401
        from rclpy.qos import QoSProfile  # noqa: F401
        from std_msgs.msg import String  # noqa: F401
        from std_srvs.srv import Trigger  # noqa: F401
    except ImportError:
        pass
    else:
        return

    class Parameter:
        def __init__(self, name: str = "", value: object = None) -> None:
            self.name = name
            self.value = value

    class SetParametersResult:
        def __init__(self, *, successful: bool = False, reason: str = "") -> None:
            self.successful = successful
            self.reason = reason

    class _ParameterValue:
        def __init__(self, value: object) -> None:
            self.value = value

    class _Publisher:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class _Subscription:
        def __init__(self, topic: str, callback: object) -> None:
            self.topic = topic
            self.callback = callback

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
            self._parameters_callbacks: list[object] = []

        def declare_parameter(self, name: str, value: object) -> _ParameterValue:
            self._parameter_values.setdefault(name, value)
            return _ParameterValue(self._parameter_values[name])

        def get_parameter(self, name: str) -> _ParameterValue:
            return _ParameterValue(self._parameter_values[name])

        def add_on_set_parameters_callback(self, callback: object) -> object:
            self._parameters_callbacks.append(callback)
            return callback

        def set_parameters_atomically(
            self, parameters: Sequence[Parameter]
        ) -> SetParametersResult:
            for callback in reversed(self._parameters_callbacks):
                result = callback(list(parameters))
                if not result.successful:
                    return result
            self._parameter_values.update({item.name: item.value for item in parameters})
            return SetParametersResult(successful=True)

        def create_publisher(self, message_type: object, topic: str, qos: object) -> _Publisher:
            del message_type, qos
            return _Publisher(topic)

        def create_service(self, service_type: object, topic: str, callback: object) -> object:
            return type(
                "Service",
                (),
                {
                    "topic": topic,
                    "callback": callback,
                    "service_type": service_type,
                    "srv_type": service_type,
                },
            )()

        def create_subscription(
            self,
            message_type: object,
            topic: str,
            callback: object,
            qos: object,
        ) -> _Subscription:
            del message_type, qos
            return _Subscription(topic, callback)

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
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"

    class DurabilityPolicy:
        VOLATILE = "volatile"
        TRANSIENT_LOCAL = "transient_local"

    class String:
        def __init__(self) -> None:
            self.data = ""

    class _Stamp:
        def __init__(self) -> None:
            self.sec = 0
            self.nanosec = 0

    class _Header:
        def __init__(self) -> None:
            self.stamp = _Stamp()

    class _Point:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Quaternion:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _Pose:
        def __init__(self) -> None:
            self.position = _Point()
            self.orientation = _Quaternion()

    class PoseStamped:
        def __init__(self) -> None:
            self.header = _Header()
            self.pose = _Pose()

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
    geometry_msgs_module = ModuleType("geometry_msgs")
    geometry_msgs_msg_module = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg_module.PoseStamped = PoseStamped
    std_msgs_module = ModuleType("std_msgs")
    std_msgs_msg_module = ModuleType("std_msgs.msg")
    std_msgs_msg_module.String = String
    std_srvs_module = ModuleType("std_srvs")
    std_srvs_srv_module = ModuleType("std_srvs.srv")
    std_srvs_srv_module.Trigger = Trigger
    rcl_interfaces_module = ModuleType("rcl_interfaces")
    rcl_interfaces_msg_module = ModuleType("rcl_interfaces.msg")
    rcl_interfaces_msg_module.SetParametersResult = SetParametersResult
    sys.modules.update(
        {
            "rclpy": rclpy_module,
            "rclpy.node": node_module,
            "rclpy.parameter": parameter_module,
            "rclpy.qos": qos_module,
            "geometry_msgs": geometry_msgs_module,
            "geometry_msgs.msg": geometry_msgs_msg_module,
            "std_msgs": std_msgs_module,
            "std_msgs.msg": std_msgs_msg_module,
            "std_srvs": std_srvs_module,
            "std_srvs.srv": std_srvs_srv_module,
            "rcl_interfaces": rcl_interfaces_module,
            "rcl_interfaces.msg": rcl_interfaces_msg_module,
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
        self.request_dock_calls = 0
        self.land_calls = 0
        self.rtl_calls = 0
        self.velocity_calls: list[tuple[object, float, float, float, Optional[float], float]] = []
        self.landing_target_calls: list[dict[str, object]] = []
        self.condition_yaw_calls: list[dict[str, object]] = []
        self.status_text_calls: list[str] = []
        self.mission_mutation_calls: list[str] = []
        self.closed = False
        self._active_name = ""
        self._active_future: Optional[MavlinkFuture[object]] = None

    def poll(self, now: float) -> None:
        del now

    def receive_telemetry(self, now: float) -> None:
        del now

    def service_pending(self, now: float) -> None:
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
        future = self._begin("mission_download")
        self.download_calls += 1
        return future  # type: ignore[return-value]

    def get_parameters_async(
        self, names: Sequence[str], now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("parameters")
        self.parameter_calls.append(tuple(names))
        return future  # type: ignore[return-value]

    def prearm_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("prearm")
        self.prearm_calls += 1
        return future  # type: ignore[return-value]

    def set_mode_async(
        self, mode: int, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("set_mode")
        self.set_mode_calls.append(mode)
        return future  # type: ignore[return-value]

    def arm_async(
        self, arm: bool, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("arm")
        self.arm_calls.append(arm)
        return future  # type: ignore[return-value]

    def takeoff_async(
        self, altitude_m: float, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("takeoff")
        self.takeoff_calls.append(altitude_m)
        return future  # type: ignore[return-value]

    def set_mission_current_async(
        self, sequence: int, *, reset: bool, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("set_mission_current")
        self.set_mission_current_calls.append((sequence, reset))
        return future  # type: ignore[return-value]

    def start_mission_async(
        self, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("start_mission")
        self.start_mission_calls += 1
        return future  # type: ignore[return-value]

    def request_dock_async(
        self, now: float
    ) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("request_dock")
        self.request_dock_calls += 1
        return future  # type: ignore[return-value]

    def land_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("land")
        self.land_calls += 1
        return future  # type: ignore[return-value]

    def rtl_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        del now
        future = self._begin("rtl")
        self.rtl_calls += 1
        return future  # type: ignore[return-value]

    def send_velocity(
        self,
        frame: object,
        velocity_x: float,
        velocity_y: float,
        velocity_z: float,
        *,
        now: float,
        yaw_rate_rad_s: Optional[float] = None,
    ) -> bool:
        self.velocity_calls.append(
            (
                frame,
                velocity_x,
                velocity_y,
                velocity_z,
                yaw_rate_rad_s,
                now,
            )
        )
        return True

    def send_landing_target(self, **values: object) -> bool:
        self.landing_target_calls.append(dict(values))
        return True

    def send_condition_yaw(self, **values: object) -> bool:
        self.condition_yaw_calls.append(dict(values))
        return True

    def send_status_text(self, text: str) -> bool:
        self.status_text_calls.append(text)
        return True

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

    def reject_pending(self, message: str) -> None:
        self.finish_command(success=False, message=message)

    def cancel_pending_operation(self) -> None:
        self._active_name = ""
        self._active_future = None

    def close(self) -> None:
        self.closed = True

    def close_without_output(self) -> None:
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

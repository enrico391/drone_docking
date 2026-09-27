"""ROS 2 node that publishes configurable external wrenches for Gazebo."""

import signal
import math
from typing import Iterable, Tuple

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import SetParametersResult
from ros_gz_interfaces.msg import Entity, EntityWrench
from tf2_msgs.msg import TFMessage

from .disturbance_model import (
    DisturbanceModel,
    DisturbanceSettings,
    Wrench,
    rotate_vector,
)


def _vector(values: Iterable[float]) -> Tuple[float, float, float]:
    vector = tuple(float(value) for value in values)
    if len(vector) != 3 or not all(math.isfinite(value) for value in vector):
        raise ValueError("wrench vectors must contain three finite values")
    return vector  # type: ignore[return-value]


class DisturbanceInjector(Node):
    """Generate one bounded external wrench per Gazebo physics step."""

    def __init__(self, **kwargs) -> None:
        super().__init__("disturbance_injector", **kwargs)
        self.declare_parameter("enabled", True)
        self.declare_parameter("target_model", "x500")
        self.declare_parameter("update_rate", 1000.0)
        self.declare_parameter("frame", "world")
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("constant.force", [0.0, 0.0, 0.0])
        self.declare_parameter("constant.torque", [0.0, 0.0, 0.0])
        self.declare_parameter("turbulence.enabled", False)
        self.declare_parameter("turbulence.force_stddev", [0.0, 0.0, 0.0])
        self.declare_parameter("turbulence.torque_stddev", [0.0, 0.0, 0.0])
        self.declare_parameter("turbulence.correlation_time", 0.3)
        self.declare_parameter("gust.enabled", False)
        self.declare_parameter("gust.force", [0.0, 0.0, 0.0])
        self.declare_parameter("gust.torque", [0.0, 0.0, 0.0])
        self.declare_parameter("gust.duration", 0.0)
        self.declare_parameter("gust.interval", 1.0)
        self.declare_parameter("ground_effect.enabled", False)
        self.declare_parameter("ground_effect.surface_z", 0.0)
        self.declare_parameter("ground_effect.rotor_plane_offset", 0.0)
        self.declare_parameter("ground_effect.minimum_height", 0.2)
        self.declare_parameter("ground_effect.maximum_height", 0.8)
        self.declare_parameter("ground_effect.maximum_upward_force", 0.0)
        self.declare_parameter("ground_effect.turbulence_gain", 1.0)
        self.declare_parameter("limits.force", [1.0e9, 1.0e9, 1.0e9])
        self.declare_parameter("limits.torque", [1.0e9, 1.0e9, 1.0e9])

        self._enabled = bool(self.get_parameter("enabled").value)
        self._target_model = self.get_parameter("target_model").value
        self._surface_z = float(self.get_parameter("ground_effect.surface_z").value)
        self._rotor_plane_offset = float(self.get_parameter("ground_effect.rotor_plane_offset").value)
        self._orientation_xyzw = (0.0, 0.0, 0.0, 1.0)
        self._rotor_height = None
        update_rate = float(self.get_parameter("update_rate").value)
        if update_rate <= 0.0:
            raise ValueError("update rate must be positive")
        self._period = 1.0 / update_rate
        self._model = self._build_model({})
        self._wrench_publisher = self.create_publisher(
            EntityWrench, "disturbance/wrench", 10
        )
        self._clear_publisher = self.create_publisher(Entity, "disturbance/clear", 10)
        self._start_time = self.get_clock().now()
        self._last_update_time = self._start_time
        self._pose_subscription = self.create_subscription(
            TFMessage, "gz/tf", self._pose_callback, 10
        )
        self._timer = self.create_timer(self._period, self._publish_wrench)
        self.add_on_set_parameters_callback(self._parameters_changed)

    def _build_model(self, changes) -> DisturbanceModel:
        def value(name):
            return changes.get(name, self.get_parameter(name).value)

        def finite(name):
            number = float(value(name))
            if not math.isfinite(number):
                raise ValueError(f"{name} must be finite")
            return number

        settings = DisturbanceSettings(
            frame=str(value("frame")),
            constant_force=_vector(value("constant.force")),
            constant_torque=_vector(value("constant.torque")),
            random_seed=int(value("random_seed")),
            turbulence_enabled=bool(value("turbulence.enabled")),
            turbulence_force_stddev=_vector(value("turbulence.force_stddev")),
            turbulence_torque_stddev=_vector(value("turbulence.torque_stddev")),
            turbulence_correlation_time=finite("turbulence.correlation_time"),
            gust_enabled=bool(value("gust.enabled")),
            gust_force=_vector(value("gust.force")),
            gust_torque=_vector(value("gust.torque")),
            gust_duration=finite("gust.duration"),
            gust_interval=finite("gust.interval"),
            ground_effect_enabled=bool(value("ground_effect.enabled")),
            ground_effect_min_height=finite("ground_effect.minimum_height"),
            ground_effect_max_height=finite("ground_effect.maximum_height"),
            ground_effect_max_force=finite("ground_effect.maximum_upward_force"),
            ground_effect_turbulence_gain=finite("ground_effect.turbulence_gain"),
            force_limits=_vector(value("limits.force")),
            torque_limits=_vector(value("limits.torque")),
        )
        if settings.turbulence_enabled and settings.turbulence_correlation_time <= 0:
            raise ValueError("turbulence correlation time must be positive")
        if any(x < 0 for x in settings.turbulence_force_stddev + settings.turbulence_torque_stddev):
            raise ValueError("turbulence standard deviations must be non-negative")
        if settings.ground_effect_enabled and settings.ground_effect_max_height <= settings.ground_effect_min_height:
            raise ValueError("ground-effect maximum height must exceed minimum height")
        if settings.ground_effect_max_force < 0:
            raise ValueError("ground-effect maximum force must be non-negative")
        return DisturbanceModel(settings)

    def _publish_wrench(self) -> None:
        if not self._enabled:
            return
        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        dt = (now - self._last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            if elapsed < 0.0:
                self._start_time = now
            self._last_update_time = now
            return
        self._last_update_time = now
        wrench = self._model.step(
            dt=dt,
            sim_time=elapsed,
            rotor_height=self._rotor_height,
            orientation_xyzw=self._orientation_xyzw,
        )
        self._wrench_publisher.publish(self._message(wrench))

    def clear_disturbance(self) -> None:
        """Stop output and clear any legacy persistent wrench for this model."""
        self._timer.cancel()
        self._publish_clear()

    def _publish_clear(self) -> None:
        self._wrench_publisher.publish(self._message(Wrench()))
        entity = Entity()
        entity.name = str(self._target_model)
        entity.type = Entity.MODEL
        self._clear_publisher.publish(entity)

    def _parameters_changed(self, parameters) -> SetParametersResult:
        changes = {parameter.name: parameter.value for parameter in parameters}
        try:
            model = self._build_model(changes)
            rate = float(changes.get("update_rate", self.get_parameter("update_rate").value))
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError("update_rate must be positive and finite")
            target = str(changes.get("target_model", self._target_model))
            if not target:
                raise ValueError("target_model must not be empty")
            surface = float(changes.get("ground_effect.surface_z", self._surface_z))
            offset = float(changes.get("ground_effect.rotor_plane_offset", self._rotor_plane_offset))
            if not math.isfinite(surface) or not math.isfinite(offset):
                raise ValueError("surface height and rotor offset must be finite")
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        requested = bool(changes.get("enabled", self._enabled))
        if (self._enabled and not requested) or target != self._target_model:
            self._publish_clear()
        self._model = model
        self._target_model = target
        self._surface_z = surface
        self._rotor_plane_offset = offset
        if rate != 1.0 / self._period:
            self.destroy_timer(self._timer)
            self._period = 1.0 / rate
            self._timer = self.create_timer(self._period, self._publish_wrench)
        if requested != self._enabled:
            now = self.get_clock().now()
            self._last_update_time = now
            if requested:
                self._start_time = now
        self._enabled = requested
        return SetParametersResult(successful=True)

    def _pose_callback(self, message: TFMessage) -> None:
        suffixes = ("base_link", "::base_link", "/base_link")
        candidates = [
            transform
            for transform in message.transforms
            if transform.child_frame_id.endswith(suffixes)
        ]
        matching = [
            transform
            for transform in candidates
            if str(self._target_model) in transform.child_frame_id
        ]
        if matching:
            transform = matching[0]
        elif len(candidates) == 1:
            transform = candidates[0]
        else:
            return
        rotation = transform.transform.rotation
        self._orientation_xyzw = (rotation.x, rotation.y, rotation.z, rotation.w)
        offset_world = rotate_vector(
            (0.0, 0.0, self._rotor_plane_offset), self._orientation_xyzw
        )
        self._rotor_height = (
            transform.transform.translation.z + offset_world[2] - self._surface_z
        )

    def _message(self, wrench: Wrench) -> EntityWrench:
        message = EntityWrench()
        message.entity.name = str(self._target_model)
        message.entity.type = Entity.MODEL
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = (
            wrench.force
        )
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = (
            wrench.torque
        )
        return message


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DisturbanceInjector()
    previous_handlers = {
        signum: signal.getsignal(signum)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }

    def request_shutdown(signum, frame) -> None:
        del signum, frame
        raise KeyboardInterrupt

    for signum in previous_handlers:
        signal.signal(signum, request_shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.clear_disturbance()
            rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)


if __name__ == "__main__":
    main()

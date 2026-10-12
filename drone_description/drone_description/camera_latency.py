"""Add bounded, nonblocking image delivery latency to the Gazebo camera."""

from collections import deque
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DelayQueue:
    """Keep original messages until the requested arrival-to-release delay."""

    def __init__(self, delay_s, capacity=64):
        if not math.isfinite(delay_s) or delay_s < 0:
            raise ValueError('delay_s must be finite and nonnegative')
        if capacity < 1:
            raise ValueError('capacity must be positive')
        self.delay_ns = round(delay_s * 1_000_000_000)
        self._pending = deque(maxlen=capacity)
        self._last_ns = None

    def clear(self):
        self._pending.clear()
        self._last_ns = None

    def _observe_time(self, now_ns):
        if self._last_ns is not None and now_ns < self._last_ns:
            self.clear()
        self._last_ns = now_ns

    def push(self, message, now_ns):
        self._observe_time(now_ns)
        self._pending.append((now_ns + self.delay_ns, message))

    def ready(self, now_ns):
        self._observe_time(now_ns)
        messages = []
        while self._pending and self._pending[0][0] <= now_ns:
            messages.append(self._pending.popleft()[1])
        return messages


class CameraLatency(Node):
    """Relay images on the ROS clock without altering capture timestamps."""

    def __init__(self, **kwargs):
        super().__init__('camera_latency', **kwargs)
        delay_s = self.declare_parameter(
            'delay_s', 0.0, ParameterDescriptor(read_only=True)).value
        capacity = self.declare_parameter(
            'max_pending_frames', 64, ParameterDescriptor(read_only=True)).value
        self._queue = DelayQueue(delay_s, capacity)
        self._publisher = self.create_publisher(
            Image, 'camera/image_raw', qos_profile_sensor_data)
        self.create_subscription(
            Image, 'camera/image_raw_undelayed', self._receive,
            qos_profile_sensor_data)
        self._jump_handler = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None,
                          min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True),
            post_callback=lambda jump: self._queue.clear(),
        )
        self.create_timer(0.002, self._release)
        self.get_logger().info(
            f'Adding {delay_s * 1000:.1f} ms camera delivery delay; '
            'capture timestamps are preserved')

    def _receive(self, message):
        self._queue.push(message, self.get_clock().now().nanoseconds)
        self._release()

    def _release(self):
        for message in self._queue.ready(self.get_clock().now().nanoseconds):
            self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = CameraLatency()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

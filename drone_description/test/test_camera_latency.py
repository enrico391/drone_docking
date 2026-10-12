"""Catch early delivery, stale frames after reset, and unbounded buffering."""

import importlib

import pytest


def queue(delay_s=0.11, capacity=64):
    module = importlib.import_module('drone_description.camera_latency')
    return module.DelayQueue(delay_s, capacity)


def test_frames_wait_full_delay_and_keep_order_and_original_objects():
    pending = queue()
    first, second = object(), object()
    pending.push(first, 1_000_000_000)
    pending.push(second, 1_030_000_000)
    assert pending.ready(1_109_999_999) == []
    assert pending.ready(1_110_000_000) == [first]
    assert pending.ready(1_139_999_999) == []
    assert pending.ready(1_140_000_000) == [second]
    assert pending.ready(1_200_000_000) == []


def test_zero_delay_can_publish_immediately():
    pending = queue(0.0)
    frame = object()
    pending.push(frame, 500)
    assert pending.ready(500) == [frame]


def test_clock_reset_drops_frames_from_previous_simulation():
    pending = queue()
    pending.push('old', 5_000_000_000)
    assert pending.ready(0) == []
    pending.push('new', 10_000_000)
    assert pending.ready(120_000_000) == ['new']


def test_overflow_drops_oldest_instead_of_accumulating_latency():
    pending = queue(capacity=2)
    pending.push('old', 1_000_000_000)
    pending.push('middle', 1_010_000_000)
    pending.push('new', 1_020_000_000)
    assert pending.ready(1_130_000_000) == ['middle', 'new']


@pytest.mark.parametrize('delay', [-0.1, float('nan'), float('inf')])
def test_invalid_latency_is_rejected(delay):
    with pytest.raises(ValueError):
        queue(delay)


def test_invalid_buffer_capacity_is_rejected():
    with pytest.raises(ValueError):
        queue(capacity=0)


def test_ros_relay_waits_on_simulation_clock_and_preserves_image():
    import time

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image

    relay_class = importlib.import_module(
        'drone_description.camera_latency').CameraLatency
    context = Context()
    rclpy.init(context=context, domain_id=127)
    relay = relay_class(context=context, parameter_overrides=[
        Parameter('use_sim_time', value=True),
        Parameter('delay_s', value=0.11),
    ])
    peer = rclpy.create_node('camera_latency_test_peer', context=context)
    images = peer.create_publisher(
        Image, 'camera/image_raw_undelayed', qos_profile_sensor_data)
    clocks = peer.create_publisher(Clock, '/clock', 10)
    received = []
    peer.create_subscription(
        Image, 'camera/image_raw', received.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(relay)
    executor.add_node(peer)

    def until(predicate):
        deadline = time.monotonic() + 2.0
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate()

    try:
        until(lambda: images.get_subscription_count() == 1
              and peer.count_publishers('camera/image_raw') == 1
              and clocks.get_subscription_count() >= 1)
        clock = Clock()
        clock.clock.sec = 10
        clocks.publish(clock)
        until(lambda: relay.get_clock().now().nanoseconds == 10_000_000_000)

        frame = Image()
        frame.header.stamp.sec = 9
        frame.header.stamp.nanosec = 900_000_000
        frame.header.frame_id = 'camera_optical_frame'
        frame.height = 1
        frame.width = 1
        frame.encoding = 'mono8'
        frame.step = 1
        frame.data = [7]
        images.publish(frame)
        # Process the input while simulation time is paused.
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert received == []

        clock.clock.nanosec = 100_000_000
        clocks.publish(clock)
        until(lambda: relay.get_clock().now().nanoseconds == 10_100_000_000)
        assert received == []
        clock.clock.nanosec = 120_000_000
        clocks.publish(clock)
        until(lambda: bool(received))
        assert len(received) == 1
        assert received[0] == frame
    finally:
        executor.remove_node(peer)
        executor.remove_node(relay)
        peer.destroy_node()
        relay.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_executable_shuts_down_cleanly_on_interrupt():
    """Break caught: Ctrl+C calls shutdown twice and produces a traceback."""
    import os
    from pathlib import Path
    import select
    import signal
    import subprocess

    script = Path(__file__).parents[1] / 'scripts/camera_latency'
    process = subprocess.Popen(
        [str(script), '--ros-args', '-p', 'delay_s:=0.11'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env=dict(os.environ, ROS_DOMAIN_ID='128'),
    )
    try:
        assert select.select([process.stdout], [], [], 3.0)[0]
        first_line = process.stdout.readline()
        assert 'Adding 110.0 ms' in first_line
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=3.0)
        assert process.returncode == 0, first_line + output
        assert 'Traceback' not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()

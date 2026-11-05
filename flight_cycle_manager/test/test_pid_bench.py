"""Run the real PID on bench poses without a flight controller."""

import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from flight_cycle_manager.px4_manager import Px4FlightCycleManager
from test_px4_manager import telemetry


@pytest.fixture
def bench(monkeypatch):
    root = Path(__file__).parents[1]
    monkeypatch.setattr('flight_cycle_manager.px4_manager.get_package_share_directory',
                        lambda _: str(root))
    context = Context()
    rclpy.init(context=context)
    clock = [100.0]
    node = Px4FlightCycleManager(
        context=context, now=lambda: clock[0],
        parameter_overrides=[Parameter('use_sim_time', value=False),
                             Parameter('auto_start', value=True)],
    )
    node._timer.cancel()
    node.test_clock = clock
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


def pose(node, *, x=0.2, z=2.0):
    message = PoseStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = x
    message.pose.position.z = z
    message.pose.orientation.w = 1.0
    return message


def outputs(node, monkeypatch):
    velocities, debug = [], []
    monkeypatch.setattr(node._pid_velocity_pub, 'publish', velocities.append)
    monkeypatch.setattr(node._debug_pub, 'publish', debug.append)
    return velocities, debug


def test_default_bench_waits_for_pose_then_calculates_on_each_tick(bench, monkeypatch):
    velocities, debug = outputs(bench, monkeypatch)
    assert bench.get_parameter('pid_bench_mode').value is True
    assert bench._pid_velocity_pub.topic_name == '/flight_cycle_manager/pid_velocity'
    bench._tick()
    assert not velocities
    bench._on_dock_pose(pose(bench))  # No mode or PX4 messages.
    assert not velocities  # Arrival must not drive the output frequency.
    for tick in (100.05, 100.10, 100.15):
        bench.test_clock[0] = tick
        bench._tick()
    assert len(velocities) == 3
    assert velocities[-1].header.frame_id == 'base_link'
    assert velocities[-1].header.stamp.sec > 0
    assert velocities[-1].twist.linear.y < 0.0
    assert velocities[-1].twist.linear.x == pytest.approx(0.0)
    data = json.loads(debug[-1].data)
    assert data['cycle_state'] == 'PID_BENCH'
    assert data['state'] == 'PID_ALIGN'
    assert data['pose_valid'] is True
    assert data['pose_timing']['receive_to_pid_s'] == pytest.approx(0.15)


def test_bench_ignores_ground_telemetry_and_never_sends_flight_commands(bench, monkeypatch):
    velocities, _ = outputs(bench, monkeypatch)
    flight_messages = []
    for publisher in bench._dds_publishers.values():
        monkeypatch.setattr(publisher, 'publish', flight_messages.append)
    telemetry(bench, armed=False, landed=True)
    bench._on_dock_pose(pose(bench))
    for tick in (100.0, 100.05, 100.1):
        bench.test_clock[0] = tick
        bench._tick()
        telemetry(bench, armed=False, landed=True)
    for service in (bench._start, bench._request_dock):
        result = service(None, Trigger.Response())
        assert result.success is False
        assert 'bench' in result.message.lower()
    assert len(velocities) == 3
    assert not flight_messages
    assert bench._pending is None


def test_bench_stops_on_stale_pose_and_restarts_on_reacquisition(bench, monkeypatch):
    velocities, debug = outputs(bench, monkeypatch)
    bench._on_dock_pose(pose(bench))
    bench._tick()
    bench.test_clock[0] += bench._p['dock_pose_timeout_s'] + 0.1
    bench._tick()
    assert len(velocities) == 1
    assert json.loads(debug[-1].data)['pose_valid'] is False
    bench._on_dock_pose(pose(bench))
    bench._tick()
    assert len(velocities) == 2
    assert json.loads(debug[-1].data)['state'] == 'PID_ALIGN'


@pytest.mark.parametrize('bad', ['nan', 'behind', 'quaternion', 'old_stamp'])
def test_bench_does_not_publish_for_invalid_pose(bench, monkeypatch, bad):
    velocities, _ = outputs(bench, monkeypatch)
    message = pose(bench)
    if bad == 'nan':
        message.pose.position.x = math.nan
    elif bad == 'behind':
        message.pose.position.z = -1.0
    elif bad == 'quaternion':
        message.pose.orientation.w = 0.0
    else:
        message.header.stamp.sec -= 5
    bench._on_dock_pose(message)
    bench._tick()
    assert not velocities


def test_bench_applies_live_pid_gains(bench, monkeypatch):
    velocities, _ = outputs(bench, monkeypatch)
    bench._on_dock_pose(pose(bench))
    bench._tick()
    before = velocities[-1].twist.linear.y
    result = bench.set_parameters_atomically([
        Parameter('pid_y_kp', value=0.0), Parameter('pid_y_ki', value=0.0),
        Parameter('pid_y_kd', value=0.0), Parameter('pid_gain_far_y_kp', value=0.0),
        Parameter('pid_gain_far_y_ki', value=0.0), Parameter('pid_gain_far_y_kd', value=0.0),
    ])
    assert result.successful
    # Existing live tuning ramps gains over 0.5 seconds of control updates.
    for _ in range(12):
        bench.test_clock[0] += 0.05
        bench._tick()
    assert before < 0.0
    assert velocities[-1].twist.linear.y == pytest.approx(0.0)

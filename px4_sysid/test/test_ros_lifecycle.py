"""Isolated DDS lifecycle smoke test; skips unless the ROS overlay is sourced."""

import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.parameter import Parameter
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    from px4_msgs.msg import (
        OffboardControlMode, TrajectorySetpoint, VehicleCommand,
        VehicleLandDetected, VehicleLocalPosition, VehicleStatus,
    )
    from std_srvs.srv import Trigger
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, "source ROS and the PX4 workspace overlay")
class TestRosLifecycle(unittest.TestCase):
    def test_stream_survives_paused_and_backwards_ros_clock(self):
        from px4_sysid.px4_sysid_node import build_node, parse_args
        from px4_sysid.policy import Phase

        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "219"}):
            rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = build_node(parse_args(["--out", "/unused", "--no-stdin"]))
        self.addCleanup(node.destroy_node)
        node.set_parameters([Parameter("use_sim_time", value=True)])
        peer = rclpy.create_node("px4_sysid_clock_test_peer")
        self.addCleanup(peer.destroy_node)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)
        self.addCleanup(executor.shutdown)
        clock_pub = peer.create_publisher(Clock, "/clock", 10)
        modes = []
        peer.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode",
                                 lambda msg: modes.append(time.monotonic()), 10)
        # Keep real policy/publishing behavior; only supply a healthy vehicle.
        # No FC commands leave isolated domain 219.
        from test_px4_sysid import healthy
        node.snap = healthy(time.monotonic())
        node.last_local_msg = VehicleLocalPosition(timestamp=100_000_000, heading=0.0)
        node.last_local_stamp = 100_000_000
        node.policy.request_offboard(node.snap, node._now(), offboard=True)
        node.offboard = True
        conflicts = []
        # The existing conflict timer captures its callback at construction;
        # count_publishers is called by that real callback.
        node.count_publishers = lambda topic: (conflicts.append(time.monotonic()) or 1)

        def spin_for(seconds, stamp):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                now = time.monotonic()
                node.snap.local_at = node.snap.status_at = node.snap.land_at = now
                clock_pub.publish(Clock(clock=Time(sec=stamp)))
                executor.spin_once(timeout_sec=0.005)

        spin_for(0.3, 100)
        modes.clear()
        spin_for(0.5, 100)  # paused /clock must not pause the heartbeat
        self.assertGreaterEqual(len(modes), 20)
        before = len(modes)
        spin_for(0.7, 90)   # backwards /clock must not pause it either
        self.assertGreaterEqual(len(modes) - before, 25)
        self.assertLess(max(b - a for a, b in zip(modes, modes[1:])), 0.2)
        self.assertTrue(conflicts, "conflict watchdog stopped with ROS time")
        self.assertEqual(node.policy.stage, Phase.READY.value)

    def test_start_run_hold_and_mode_override(self):
        from px4_sysid.px4_sysid_node import build_node, parse_args
        from px4_sysid.policy import Phase
        from px4_sysid.recorder import CsvRecorder

        # Never publish test flight commands on the user's vehicle domain.
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "219"}):
            rclpy.init()
        self.addCleanup(rclpy.shutdown)
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        out = Path(temp.name) / "run"
        node = build_node(parse_args([
            "--out", str(out), "--no-stdin", "--hover", "0.1",
            "--chirp", "0.2", "--prbs", "0.2",
        ]))
        self.addCleanup(node.destroy_node)
        node.recorder = CsvRecorder(out, "x", node.cfg)
        self.addCleanup(lambda: node.recorder.close("test cleanup") if node.recorder else None)
        peer = rclpy.create_node("px4_sysid_test_peer")
        self.addCleanup(peer.destroy_node)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)
        self.addCleanup(executor.shutdown)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        status_pub = peer.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v4", qos)
        local_pub = peer.create_publisher(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", qos)
        land_pub = peer.create_publisher(VehicleLandDetected, "/fmu/out/vehicle_land_detected", qos)
        commands, modes, setpoints = [], [], []
        peer.create_subscription(VehicleCommand, "/fmu/in/vehicle_command", commands.append, qos)
        peer.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode", modes.append, qos)
        peer.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", setpoints.append, qos)
        client = peer.create_client(Trigger, "/px4_sysid/start")
        airborne = False
        nav = VehicleStatus.NAVIGATION_STATE_POSCTL
        last = 0.0

        def spin_for(seconds, until=lambda: False):
            nonlocal last
            end = time.monotonic() + seconds
            while time.monotonic() < end and not until():
                now = time.monotonic()
                if now - last >= 0.02:
                    stamp = int(now * 1e6)
                    status = VehicleStatus(timestamp=stamp)
                    status.arming_state = VehicleStatus.ARMING_STATE_ARMED
                    status.nav_state = nav
                    status.failsafe = False
                    status_pub.publish(status)
                    local = VehicleLocalPosition(timestamp=stamp)
                    local.x, local.y, local.z = 1.0, 2.0, (-5.0 if airborne else 0.0)
                    local.vx = local.vy = local.vz = 0.0
                    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = True
                    local.heading_good_for_control = True
                    local.heading = 0.25
                    local_pub.publish(local)
                    # This airframe's misleading landed flag must not gate control.
                    land_pub.publish(VehicleLandDetected(timestamp=stamp, landed=True))
                    last = now
                executor.spin_once(timeout_sec=0.005)

        def request_start():
            future = client.call_async(Trigger.Request())
            spin_for(2.0, future.done)
            self.assertTrue(future.done(), "start service did not respond")
            return future.result()

        spin_for(2.0, lambda: client.service_is_ready() and node.snap.armed)
        self.assertTrue(client.service_is_ready())
        self.assertFalse(modes)
        self.assertFalse(commands)
        response = request_start()
        self.assertFalse(response.success)
        self.assertIn("not airborne", response.message)
        airborne = True
        spin_for(0.1)
        self.assertTrue(request_start().success)
        self.assertFalse(request_start().success)
        spin_for(5.0, lambda: bool(commands) or node.policy.stage == Phase.DONE.value)
        self.assertEqual(len(commands), 1, (node.policy, getattr(node, "_tick_error", None), getattr(node, "_publish_error", None), len(modes)))
        self.assertGreaterEqual(len(modes), 50)
        self.assertEqual(node.policy.stage, Phase.WARMUP.value)
        nav = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        spin_for(5.0, lambda: node.policy.stage in (Phase.HOLD.value, Phase.DONE.value))
        self.assertEqual(node.policy.stage, Phase.HOLD.value,
                         (node.policy, node.snap, getattr(node, "_tick_error", None)))
        self.assertIsNone(node.recorder)
        before = len(setpoints)
        spin_for(0.6)
        self.assertGreaterEqual(len(setpoints), before + 20)
        self.assertTrue(modes[-1].position)
        self.assertEqual(list(setpoints[-1].position), [1.0, 2.0, -5.0])
        self.assertTrue(all(math.isnan(v) for v in setpoints[-1].velocity))
        self.assertEqual(len(commands), 1)
        self.assertEqual(json.loads((out / "metadata.json").read_text())["outcome"], "complete")
        nav = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        spin_for(0.2)
        self.assertEqual(node.policy.stage, Phase.DONE.value)
        before = len(setpoints)
        spin_for(0.2)
        self.assertEqual(len(setpoints), before)


if __name__ == "__main__":
    unittest.main()

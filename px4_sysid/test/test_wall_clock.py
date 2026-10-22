"""Reproduce WSL UTC corrections inside a child process, never on the host."""

import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
CLOCK_SHIM = HERE.parents[1] / "drone_description/test/backwards_realtime.cc"
HAVE_ROS = importlib.util.find_spec("rclpy") is not None


@unittest.skipUnless(HAVE_ROS and shutil.which("c++") and CLOCK_SHIM.is_file(),
                     "requires ROS, C++ compiler and the clock fault fixture")
class TestWallClock(unittest.TestCase):
    def test_run_keeps_publishing_during_backwards_utc_steps(self):
        with tempfile.TemporaryDirectory() as directory:
            library = str(Path(directory) / "backwards_realtime.so")
            subprocess.run(["c++", "-shared", "-fPIC", str(CLOCK_SHIM),
                            "-o", library, "-ldl"], check=True, timeout=30)
            # Exercise the real ROS timer and RUN callback. Fake publishers
            # capture sends locally; the child's DDS domain is also isolated.
            code = '''
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, sys.argv[1])
from test_px4_sysid import FakePublisher, healthy
from px4_sysid.px4_sysid_node import build_node, parse_args
from px4_sysid.recorder import CsvRecorder
from px4_msgs.msg import VehicleLocalPosition
import rclpy

rclpy.init()
node = build_node(parse_args(["--out", sys.argv[2], "--no-stdin"]))
node.pub_mode = FakePublisher()
node.pub_sp = FakePublisher()
node.pub_cmd = FakePublisher()
node.count_publishers = lambda topic: 1
node.recorder = CsvRecorder(Path(sys.argv[2]), "x", node.cfg)
start = time.monotonic()
wall_start = time.time()
node.snap = healthy(start)
node.last_local_msg = VehicleLocalPosition(timestamp=100_000_000, heading=0.0)
node.offboard = True
node.command("start", start)
send_times = []
try:
    while time.monotonic() - start < 2.2:
        now = time.monotonic()
        node.snap.local_at = node.snap.status_at = node.snap.land_at = now
        node.snap.local_fc_s = 100.0 + now - start
        node.last_local_msg.timestamp = int(node.snap.local_fc_s * 1e6)
        before = node.pub_sp.count
        rclpy.spin_once(node, timeout_sec=0.005)
        if node.pub_sp.count > before:
            send_times.append(time.monotonic())
    print(json.dumps({
        "published": len(send_times),
        "max_gap": max((b-a for a,b in zip(send_times, send_times[1:])), default=2.2),
        "wall_elapsed": time.time() - wall_start,
        "stage": node.policy.stage,
    }))
finally:
    node.recorder.close("clock regression")
    node.destroy_node()
    rclpy.shutdown()
'''
            environment = dict(os.environ, ROS_DOMAIN_ID="219", LD_PRELOAD=library)
            result = subprocess.run(
                [sys.executable, "-c", code, str(HERE), str(Path(directory) / "run")],
                env=environment, text=True, capture_output=True, timeout=20,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            report = json.loads(result.stdout.strip().splitlines()[-1])
            self.assertLess(report["wall_elapsed"], -1.0, report)
            self.assertGreaterEqual(report["published"], 80, report)
            self.assertLess(report["max_gap"], 0.2, report)
            self.assertEqual(report["stage"], "RUN", report)


if __name__ == "__main__":
    unittest.main()

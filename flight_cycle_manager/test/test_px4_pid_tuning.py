"""Live PID reconfiguration on real ROS nodes, including the HTTP-to-ROS path."""

from concurrent.futures import ThreadPoolExecutor
import json
import math
import socket
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen

import pytest
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor

from flight_cycle_manager.landing_backend import DockObservation, DockTargetMode
from flight_cycle_manager.pid_landing_backend import PidLandingBackend
from test_pid_tuner import request_json
from test_px4_manager import manager, telemetry


def landing(node):
    telemetry(node, armed=True, landed=False, offboard=True, z=-3)
    node._landing = PidLandingBackend(node._pid_config)
    dock = DockObservation(0.2, 0.1, 3.0, 0.0, DockTargetMode.OUTER, 1.0, node._now())
    node._landing.start(now=node._now(), dock=dock, vehicle=node._vehicle())
    node._state = "LANDING_BACKEND"
    node._landing.update(now=node._now(), dock=dock, vehicle=node._vehicle())
    return dock


def test_live_gain_update_preserves_backend_and_updates_scheduled_banks(manager):
    dock = landing(manager)
    backend = manager._landing
    assert manager.set_parameters_atomically([
        Parameter("pid_gain_far_x_kp", value=0.7),
        Parameter("pid_x_kp", value=0.8),
        Parameter("pid_x_k_v", value=0.2),
    ]).successful
    assert manager._landing is backend
    assert manager._pid_config.x_pid.kp == 0.8
    assert manager._pid_config.far_pids[0].kp == 0.7
    assert manager._pid_config.near_pids[0].kp == 0.8
    assert backend.diagnostics()["velocity_feedback"]["x"] == 0.2
    for i in range(1, 16):
        backend.update(now=manager._now() + i * .05, dock=dock, vehicle=manager._vehicle())
    fraction = (dock.distance_m - manager._pid_config.gain_near_distance_m) / (
        manager._pid_config.gain_far_distance_m - manager._pid_config.gain_near_distance_m)
    effective = backend.diagnostics()["axes"]["x"]["kp"]
    assert effective == pytest.approx(0.8 + fraction * (0.7 - 0.8), abs=.005)


@pytest.mark.parametrize("name,value", [
    ("pid_x_kp", -1.0), ("pid_x_kp", math.nan), ("pid_x_ki", math.inf),
    ("pid_gain_far_y_kp", -0.2), ("pid_x_k_v", -1.0),
    ("pid_gain_near_distance_m", 8.0), ("takeoff_altitude_m", 7.0),
])
def test_invalid_batch_does_not_change_any_parameters_or_running_controller(manager, name, value):
    landing(manager)
    config = manager._pid_config
    before = manager._landing.diagnostics()
    result = manager.set_parameters_atomically([
        Parameter("pid_z_kp", value=0.9), Parameter(name, value=value),
    ])
    assert not result.successful
    assert manager._pid_config is config
    assert manager.get_parameter("pid_z_kp").value == config.z_pid.kp
    assert manager._landing.diagnostics() == before


def test_ready_tuning_is_used_by_next_landing(manager):
    assert manager.set_parameters_atomically([Parameter("pid_y_kp", value=0.9)]).successful
    landing(manager)
    assert manager._landing._controller._base_pids()[1].kp == 0.9


def test_pose_debug_clears_errors_after_docking_stops(manager):
    dock = landing(manager)
    captured = []

    class Publisher:
        def publish(self, message):
            captured.append(json.loads(message.data))

    manager._debug_pub = Publisher()
    manager._publish_pid_debug(manager._now(), dock, "PID_ALIGN")
    assert captured[-1]["measured_pose"]["x"] == 0.2
    assert captured[-1]["error"]["x"] == 0.2
    assert captured[-1]["error"]["z"] is None
    manager._transition("PILOT_OVERRIDE", "test")
    assert captured[-1]["measured_pose"] is None
    assert all(value is None for value in captured[-1]["error"].values())


@pytest.mark.parametrize("manager", ["", "/drone1"], indirect=True)
def test_real_http_tuner_applies_gains_via_ros_service(manager, tmp_path):
    dock = landing(manager)
    executor = SingleThreadedExecutor(context=manager.context)
    executor.add_node(manager)
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    url = f"http://127.0.0.1:{port}"
    with (tmp_path / "tuner.log").open("w+") as log:
        process = subprocess.Popen([
            sys.executable, "-m", "flight_cycle_manager.pid_tuner", "--ros-args",
            "-p", f"port:={port}", "-r", f"__ns:={manager.get_namespace()}",
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while True:
                try:
                    with urlopen(url + "/api/state", timeout=.2):
                        break
                except (URLError, TimeoutError):
                    if time.monotonic() > deadline or process.poll() is not None:
                        log.seek(0)
                        pytest.fail("tuner did not start: " + log.read())
                    time.sleep(.05)
            # Wait for actual DDS discovery and observe a fresh pose at the HTTP endpoint.
            while True:
                manager._publish_pid_debug(manager._now(), dock, "PID_ALIGN")
                executor.spin_once(timeout_sec=.05)
                with urlopen(url + "/api/state", timeout=1) as response:
                    state = json.load(response)
                if state["connected"]:
                    break
                assert time.monotonic() < deadline
            assert state["telemetry"]["measured_pose"]["z"] == 3.0
            with ThreadPoolExecutor(max_workers=1) as workers:
                future = workers.submit(request_json, url + "/api/gains",
                                         dict(axis="x", bank="near", kp=.72, ki=.01, kd=.02))
                while not future.done():
                    executor.spin_once(timeout_sec=.05)
                status, result = future.result()
            assert status == 200, result
            assert manager.get_parameter("pid_gain_near_x_kp").value == .72
            assert manager._pid_config.near_pids[0].kp == .72
        finally:
            executor.shutdown()
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
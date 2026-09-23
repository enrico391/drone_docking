import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Entity, EntityWrench
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from drone_description.disturbance_injector import DisturbanceInjector


CONFIG_PATH = Path(__file__).parents[1] / "config/disturbances.yaml"


def test_injector_publishes_configured_wrench_for_target_model():
    context = Context()
    rclpy.init(context=context)
    injector = DisturbanceInjector(
        context=context,
        parameter_overrides=[
            Parameter("target_model", value="test_x500"),
            Parameter("update_rate", value=100.0),
            Parameter("constant.force", value=[0.4, -0.2, 0.1]),
            Parameter("constant.torque", value=[0.01, 0.02, -0.03]),
        ],
    )
    observer = rclpy.create_node("disturbance_observer", context=context)
    received = []
    observer.create_subscription(
        EntityWrench,
        "disturbance/wrench",
        received.append,
        10,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(injector)
    executor.add_node(observer)

    try:
        deadline = time.monotonic() + 1.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        assert received
        message = received[-1]
        assert message.entity.name == "test_x500"
        assert message.entity.type == Entity.MODEL
        assert (
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
        ) == (0.4, -0.2, 0.1)
        assert (
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z,
        ) == (0.01, 0.02, -0.03)
    finally:
        executor.remove_node(observer)
        executor.remove_node(injector)
        observer.destroy_node()
        injector.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_injector_uses_vehicle_pose_for_surface_relative_ground_effect():
    context = Context()
    rclpy.init(context=context)
    injector = DisturbanceInjector(
        context=context,
        parameter_overrides=[
            Parameter("target_model", value="test_x500"),
            Parameter("update_rate", value=100.0),
            Parameter("ground_effect.enabled", value=True),
            Parameter("ground_effect.surface_z", value=0.5),
            Parameter("ground_effect.rotor_plane_offset", value=0.0),
            Parameter("ground_effect.minimum_height", value=0.2),
            Parameter("ground_effect.maximum_height", value=0.8),
            Parameter("ground_effect.maximum_upward_force", value=1.2),
        ],
    )
    observer = rclpy.create_node("ground_effect_observer", context=context)
    received = []
    observer.create_subscription(EntityWrench, "disturbance/wrench", received.append, 10)
    pose_publisher = observer.create_publisher(TFMessage, "gz/tf", 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(injector)
    executor.add_node(observer)

    pose = TransformStamped()
    pose.child_frame_id = "test_x500::base_link"
    pose.transform.translation.z = 1.0
    pose.transform.rotation.w = 1.0
    tf_message = TFMessage(transforms=[pose])

    try:
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            pose_publisher.publish(tf_message)
            executor.spin_once(timeout_sec=0.05)
            if any(abs(msg.wrench.force.z - 0.3) < 1e-9 for msg in received):
                break

        assert any(abs(msg.wrench.force.z - 0.3) < 1e-9 for msg in received)
    finally:
        executor.remove_node(observer)
        executor.remove_node(injector)
        observer.destroy_node()
        injector.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_clear_disturbance_publishes_zero_wrench_and_clear_entity():
    context = Context()
    rclpy.init(context=context)
    injector = DisturbanceInjector(
        context=context,
        parameter_overrides=[
            Parameter("target_model", value="test_x500"),
            Parameter("constant.force", value=[1.0, 0.0, 0.0]),
        ],
    )
    observer = rclpy.create_node("clear_disturbance_observer", context=context)
    wrenches = []
    cleared = []
    observer.create_subscription(EntityWrench, "disturbance/wrench", wrenches.append, 10)
    observer.create_subscription(Entity, "disturbance/clear", cleared.append, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(injector)
    executor.add_node(observer)

    try:
        injector.clear_disturbance()
        deadline = time.monotonic() + 1.0
        while (not wrenches or not cleared) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        settle_deadline = time.monotonic() + 0.1
        while time.monotonic() < settle_deadline:
            executor.spin_once(timeout_sec=0.02)

        assert wrenches
        assert all(message.wrench.force.x == 0.0 for message in wrenches)
        assert cleared[-1].name == "test_x500"
        assert cleared[-1].type == Entity.MODEL
    finally:
        executor.remove_node(observer)
        executor.remove_node(injector)
        observer.destroy_node()
        injector.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_namespaced_injector_loads_the_shared_parameter_profile():
    context = Context()
    rclpy.init(context=context)
    injector = DisturbanceInjector(
        context=context,
        namespace="test_vehicle",
        cli_args=["--ros-args", "--params-file", str(CONFIG_PATH)],
    )

    try:
        assert injector.get_namespace() == "/test_vehicle"
        assert injector.get_parameter("use_sim_time").value is True
        assert injector.get_parameter("enabled").value is False
        assert injector.get_parameter("frame").value == "body"
        assert injector.get_parameter("turbulence.enabled").value is True
    finally:
        injector.destroy_node()
        context.shutdown()


def test_main_handles_sigterm_without_leaving_an_error_exit():
    with tempfile.TemporaryDirectory(
        prefix="disturbance_sigterm_", dir="/tmp"
    ) as ros_home:
        environment = os.environ.copy()
        environment["ROS_HOME"] = ros_home
        process = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "from drone_description.disturbance_injector import main; main()",
            ],
            cwd=Path(__file__).parents[1],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            time.sleep(0.4)
            process.terminate()
            output, _ = process.communicate(timeout=3.0)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=1.0)

    assert process.returncode == 0, output


def test_enabled_parameter_clears_and_reenables_wrench_generation():
    context = Context()
    rclpy.init(context=context)
    injector = DisturbanceInjector(
        context=context,
        parameter_overrides=[
            Parameter("enabled", value=True),
            Parameter("update_rate", value=100.0),
            Parameter("constant.force", value=[0.5, 0.0, 0.0]),
        ],
    )
    observer = rclpy.create_node("toggle_disturbance_observer", context=context)
    wrenches = []
    cleared = []
    observer.create_subscription(EntityWrench, "disturbance/wrench", wrenches.append, 10)
    observer.create_subscription(Entity, "disturbance/clear", cleared.append, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(injector)
    executor.add_node(observer)

    try:
        deadline = time.monotonic() + 1.0
        while not any(message.wrench.force.x == 0.5 for message in wrenches):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.05)

        result = injector.set_parameters([Parameter("enabled", value=False)])
        assert result[0].successful
        deadline = time.monotonic() + 1.0
        while not cleared or not any(
            message.wrench.force.x == 0.0 for message in wrenches
        ):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.05)

        disabled_message_count = len(wrenches)
        settle_deadline = time.monotonic() + 0.1
        while time.monotonic() < settle_deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(wrenches) == disabled_message_count

        result = injector.set_parameters([Parameter("enabled", value=True)])
        assert result[0].successful
        deadline = time.monotonic() + 1.0
        while len(wrenches) == disabled_message_count:
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.05)
        assert wrenches[-1].wrench.force.x == 0.5
    finally:
        executor.remove_node(observer)
        executor.remove_node(injector)
        observer.destroy_node()
        injector.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_runtime_force_change_reaches_published_wrench():
    context = Context()
    rclpy.init(context=context)
    injector = DisturbanceInjector(
        context=context,
        parameter_overrides=[
            Parameter("update_rate", value=100.0),
            Parameter("constant.force", value=[0.1, 0.0, 0.0]),
        ],
    )
    observer = rclpy.create_node("runtime_disturbance_observer", context=context)
    received = []
    observer.create_subscription(EntityWrench, "disturbance/wrench", received.append, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(injector)
    executor.add_node(observer)
    try:
        result = injector.set_parameters_atomically(
            [Parameter("constant.force", value=[0.4, -0.1, 0.0])]
        )
        assert result.successful, result.reason
        deadline = time.monotonic() + 1.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received
        assert received[-1].wrench.force.x == 0.4
        assert received[-1].wrench.force.y == -0.1
    finally:
        executor.remove_node(observer)
        executor.remove_node(injector)
        observer.destroy_node()
        injector.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_invalid_runtime_settings_are_rejected_without_changing_output():
    context = Context()
    rclpy.init(context=context)
    injector = DisturbanceInjector(context=context)
    try:
        result = injector.set_parameters_atomically([
            Parameter("gust.enabled", value=True),
            Parameter("gust.duration", value=2.0),
            Parameter("gust.interval", value=1.0),
        ])
        assert not result.successful
        assert injector.get_parameter("gust.enabled").value is False
        assert injector._model.settings.gust_enabled is False
    finally:
        injector.destroy_node()
        context.shutdown()

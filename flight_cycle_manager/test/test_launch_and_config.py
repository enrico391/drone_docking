"""Tests for installed configuration ownership and launch selection."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
import pytest
import yaml


ROOT = Path(__file__).parents[1]
PACKAGE_DIR = ROOT / "flight_cycle_manager"
COMMON_CONFIG = ROOT / "config" / "flight_cycle_manager.yaml"
PID_CONFIG = ROOT / "config" / "pid_landing.yaml"
LAUNCH_FILE = ROOT / "launch" / "flight_cycle.launch.py"
LOCALIZER_ROOT = ROOT.parent / "dock_apriltag_localizer"

sys.path.insert(0, str(ROOT))

from vehicle_fakes import FakeVehicle, install_ros_fakes


EXPECTED_COMMON_PARAMETERS = {
    "simulation_mode": False,
    "auto_start": False,
    "landing_backend": "pid",
    "update_rate_hz": 20.0,
    "takeoff_altitude_m": 4.0,
    "takeoff_altitude_tolerance_m": 0.25,
    "takeoff_vertical_speed_tolerance_m_s": 0.15,
    "takeoff_stable_time_s": 1.0,
    "mission_handoff_min_loiter_s": 5.0,
    "mission_landing_location_tolerance_m": 2.0,
    "mission_download_timeout_s": 10.0,
    "mavlink_operation_timeout_s": 10.0,
    "mission_start_confirmation_timeout_s": 5.0,
    "mavlink_heartbeat_timeout_s": 2.5,
    "pose_timeout_s": 1.0,
    "landed_state_timeout_s": 1.0,
    "dock_pose_timeout_s": 0.5,
    "dock_pose_topic": "/dock/pose",
    "dock_mode_topic": "/dock/mode",
    "search_radius_m": 1.0,
    "search_tangential_speed_m_s": 0.30,
    "search_radial_gain": 0.70,
    "search_altitude_gain": 0.70,
    "search_max_horizontal_speed_m_s": 0.50,
    "search_max_vertical_speed_m_s": 0.50,
    "search_target_stable_time_s": 0.50,
    "search_timeout_s": 30.0,
    "search_max_displacement_m": 2.0,
    "landing_yaw_correction_enabled": True,
    "landing_target_warmup_s": 1.0,
    "landing_target_camera_forward_m": 0.0,
    "landing_target_camera_right_m": 0.0,
    "landing_target_camera_down_m": 0.19,
    "landing_touchdown_warn_after_s": 30.0,
    "landing_disarm_warn_after_s": 20.0,
    "mavlink_connection": "tcp:127.0.0.1:5760",
    "mavlink_baud": 921600,
    "mavlink_target_system": 1,
    "mavlink_target_component": 1,
    "mavlink_source_system": 254,
    "mavlink_source_component": 191,
}

EXPECTED_PID_PARAMETERS = {
    "pid_tag_stable_time_s": 0.5,
    "pid_central_transition_distance_m": 1.0,
    "pid_central_acquire_timeout_s": 3.0,
    "pid_central_loss_grace_s": 0.3,
    "pid_recovery_climb_m": 0.3,
    "pid_recovery_altitude_tolerance_m": 0.05,
    "pid_max_recovery_attempts": 3,
    "pid_touchdown_commit_height_m": 0.20,
    "pid_touchdown_speed_m_s": 0.05,
    "pid_touchdown_timeout_s": 15.0,
    "pid_landed_state_timeout_s": 1.0,
    "pid_landed_stable_time_s": 0.5,
    "pid_disarm_retry_period_s": 1.0,
    "pid_alignment_stable_time_s": 1.0,
    "pid_coarse_horizontal_tolerance_m": 0.2,
    "pid_fine_horizontal_tolerance_m": 0.07,
    "pid_coarse_yaw_tolerance_rad": 0.2,
    "pid_fine_yaw_tolerance_rad": 0.1,
    "pid_max_horizontal_speed_m_s": 0.5,
    "pid_outer_descent_speed_m_s": 0.3,
    "pid_central_descent_speed_m_s": 0.12,
    "pid_recovery_speed_m_s": 0.2,
    "pid_max_yaw_rate_rad_s": 0.35,
    "pid_x_kp": 1.0,
    "pid_x_ki": 0.0,
    "pid_x_kd": 0.0,
    "pid_x_integral_limit": 0.5,
    "pid_x_derivative_filter_time_constant_s": 0.1,
    "pid_y_kp": 1.0,
    "pid_y_ki": 0.0,
    "pid_y_kd": 0.0,
    "pid_y_integral_limit": 0.5,
    "pid_y_derivative_filter_time_constant_s": 0.1,
    "pid_z_kp": 1.0,
    "pid_z_ki": 0.0,
    "pid_z_kd": 0.0,
    "pid_z_integral_limit": 0.5,
    "pid_z_derivative_filter_time_constant_s": 0.1,
    "pid_yaw_kp": 1.0,
    "pid_yaw_ki": 0.0,
    "pid_yaw_kd": 0.0,
    "pid_yaw_integral_limit": 0.5,
    "pid_yaw_derivative_filter_time_constant_s": 0.1,
}


def load_parameters(path: Path) -> dict[str, object]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))[
        "flight_cycle_manager"
    ]["ros__parameters"]


def load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "flight_cycle_launch_under_test", LAUNCH_FILE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def configured_launch_context(module):
    declarations = [
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    service = LaunchService()
    service.include_launch_description(LaunchDescription(declarations))
    assert service.run() == 0
    return service.context


def manager_declared_parameters() -> set[str]:
    install_ros_fakes()
    import rclpy

    from flight_cycle_manager.manager import FlightCycleManager

    initialized_here = False
    if hasattr(rclpy, "ok") and not rclpy.ok():
        rclpy.init()
        initialized_here = True
    node = FlightCycleManager(vehicle=FakeVehicle())
    try:
        if hasattr(node, "get_parameters_by_prefix"):
            declared = set(node.get_parameters_by_prefix(""))
        else:
            declared = set(node._parameter_values)
        declared.discard("use_sim_time")
        return declared
    finally:
        node.close()
        if initialized_here:
            rclpy.shutdown()


def patch_package_shares(monkeypatch, module) -> None:
    def fake_share_directory(package_name: str) -> str:
        if package_name == "flight_cycle_manager":
            return str(ROOT)
        if package_name == "dock_apriltag_localizer":
            return str(LOCALIZER_ROOT)
        raise AssertionError(f"unexpected package lookup: {package_name}")

    monkeypatch.setattr(module, "get_package_share_directory", fake_share_directory)


def configured_context_for_values(module, **values):
    context = configured_launch_context(module)
    context.launch_configurations.update(values)
    return context


def test_common_and_pid_configuration_match_declared_manager_parameters():
    """Break caught: config drift leaves a launch knob undeclared or omitted."""
    common = load_parameters(COMMON_CONFIG)
    pid = load_parameters(PID_CONFIG)
    removed_legacy_keys = ("mission_" "file", "rep" "eat", "route")

    assert common == EXPECTED_COMMON_PARAMETERS
    assert pid == EXPECTED_PID_PARAMETERS
    assert set(common).isdisjoint(pid)
    assert set(common) | set(pid) == manager_declared_parameters()
    assert not any(
        any(fragment in key for fragment in removed_legacy_keys)
        for key in set(common) | set(pid)
    )
    assert not any(
        key.startswith("pid_search_") or "initial_search" in key
        for key in set(common) | set(pid)
    )


def test_launch_declares_operational_arguments(monkeypatch):
    """Break caught: launch stops exposing required runtime override arguments."""
    module = load_launch_module()
    patch_package_shares(monkeypatch, module)

    declarations = {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    context = configured_launch_context(module)

    assert {
        "landing_backend",
        "mavlink_connection",
        "mavlink_baud",
        "simulation_mode",
        "auto_start",
        "start_localizer",
        "image_topic",
        "camera_info_topic",
        "localizer_python",
        "pose_filter_enabled",
        "pose_filter_time_constant_s",
        "pose_filter_max_translation_speed_m_s",
        "pose_filter_max_position_jump_m",
        "pose_filter_reacquire_frames",
    } <= declarations.keys()
    assert declarations["landing_backend"].choices == ["pid", "precise_landing"]
    assert context.launch_configurations["start_localizer"] == "true"


def test_launch_setup_returns_exact_pid_manager_node(monkeypatch):
    """Break caught: PID setup can change node identity or lose parameter wiring."""
    module = load_launch_module()
    patch_package_shares(monkeypatch, module)
    captured = {}

    def record_node(**kwargs):
        captured.update(kwargs)
        return {"node": kwargs}

    monkeypatch.setattr(module, "Node", record_node)
    context = configured_context_for_values(
        module,
        landing_backend="pid",
        simulation_mode="true",
        auto_start="true",
        mavlink_connection="serial:///dev/ttyAMA0",
        mavlink_baud="57600",
    )

    actions = module._launch_setup(context, ROOT)
    overrides = captured["parameters"][2]

    assert actions == [{"node": captured}]
    assert captured["package"] == "flight_cycle_manager"
    assert captured["executable"] == "flight_cycle_manager"
    assert captured["name"] == "flight_cycle_manager"
    assert captured["parameters"][0] == str(COMMON_CONFIG)
    assert captured["parameters"][1] == str(PID_CONFIG)
    assert overrides["landing_backend"].perform(context) == "pid"
    assert overrides["simulation_mode"].evaluate(context) is True
    assert overrides["auto_start"].evaluate(context) is True
    assert overrides["mavlink_connection"].perform(context) == "serial:///dev/ttyAMA0"
    assert overrides["mavlink_baud"].evaluate(context) == 57600


def test_launch_setup_returns_exact_precise_landing_manager_node(monkeypatch):
    """Break caught: native setup can inject PID config or lose node identity."""
    module = load_launch_module()
    patch_package_shares(monkeypatch, module)
    captured = {}

    def record_node(**kwargs):
        captured.update(kwargs)
        return {"node": kwargs}

    monkeypatch.setattr(module, "Node", record_node)
    context = configured_context_for_values(
        module,
        landing_backend="precise_landing",
        simulation_mode="false",
        auto_start="false",
        mavlink_connection="tcp:10.0.0.2:5770",
        mavlink_baud="115200",
    )

    actions = module._launch_setup(context, ROOT)
    overrides = captured["parameters"][1]

    assert actions == [{"node": captured}]
    assert captured["package"] == "flight_cycle_manager"
    assert captured["executable"] == "flight_cycle_manager"
    assert captured["name"] == "flight_cycle_manager"
    assert captured["parameters"][0] == str(COMMON_CONFIG)
    assert len(captured["parameters"]) == 2
    assert overrides["landing_backend"].perform(context) == "precise_landing"
    assert overrides["simulation_mode"].evaluate(context) is False
    assert overrides["auto_start"].evaluate(context) is False
    assert overrides["mavlink_connection"].perform(context) == "tcp:10.0.0.2:5770"
    assert overrides["mavlink_baud"].evaluate(context) == 115200


def test_launch_setup_rejects_auto_start_outside_simulation_before_node_creation(
    monkeypatch,
):
    """Break caught: invalid auto-start reaches node construction."""
    module = load_launch_module()
    patch_package_shares(monkeypatch, module)
    node_calls = []

    def fail_if_called(**kwargs):
        node_calls.append(kwargs)
        raise AssertionError("Node must not be created for invalid auto_start")

    monkeypatch.setattr(module, "Node", fail_if_called)
    context = configured_context_for_values(
        module,
        landing_backend="pid",
        simulation_mode="false",
        auto_start="true",
    )

    with pytest.raises(
        ValueError, match="auto_start=true requires simulation_mode=true"
    ):
        module._launch_setup(context, ROOT)

    assert node_calls == []


def test_launch_description_includes_localizer_with_expected_wiring(monkeypatch):
    """Break caught: localizer include path or forwarded launch arguments drift."""
    module = load_launch_module()
    patch_package_shares(monkeypatch, module)
    context = configured_context_for_values(
        module,
        start_localizer="true",
        image_topic="/stereo/down/image_raw",
        camera_info_topic="/stereo/down/camera_info",
        localizer_python="/opt/localizer/bin/python3",
        pose_filter_enabled="false",
        pose_filter_time_constant_s="0.25",
        pose_filter_max_translation_speed_m_s="1.8",
        pose_filter_max_position_jump_m="0.11",
        pose_filter_reacquire_frames="6",
    )
    include = next(
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, IncludeLaunchDescription)
    )

    wired_arguments = dict(include.launch_arguments)

    assert perform_substitutions(
        context,
        include.launch_description_source._LaunchDescriptionSource__location,
    ) == str(LOCALIZER_ROOT / "launch" / "dock_localizer.launch.py")
    assert include.condition.evaluate(context) is True
    assert wired_arguments["image_topic"].perform(context) == "/stereo/down/image_raw"
    assert wired_arguments["camera_info_topic"].perform(context) == "/stereo/down/camera_info"
    assert wired_arguments["python_executable"].perform(context) == "/opt/localizer/bin/python3"
    assert wired_arguments["pose_filter_enabled"].perform(context) == "false"
    assert wired_arguments["pose_filter_time_constant_s"].perform(context) == "0.25"
    assert (
        wired_arguments["pose_filter_max_translation_speed_m_s"].perform(context)
        == "1.8"
    )
    assert wired_arguments["pose_filter_max_position_jump_m"].perform(context) == "0.11"
    assert wired_arguments["pose_filter_reacquire_frames"].perform(context) == "6"

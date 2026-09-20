"""Launch contract for repeatable mission-free PID landing debug."""

from __future__ import annotations

import importlib.util
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions


ROOT = Path(__file__).parents[1]
LAUNCH_FILE = ROOT / "launch" / "pid_landing_debug.launch.py"
LOCALIZER_ROOT = ROOT.parent / "dock_apriltag_localizer"


def load_launch_module():
    assert LAUNCH_FILE.is_file(), "pid_landing_debug.launch.py is not implemented"
    spec = importlib.util.spec_from_file_location(
        "pid_landing_debug_launch_under_test", LAUNCH_FILE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def configured_context(module, **values):
    declarations = [
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    service = LaunchService()
    service.include_launch_description(LaunchDescription(declarations))
    assert service.run() == 0
    service.context.launch_configurations.update(values)
    return service.context


def patch_package_shares(monkeypatch, module) -> None:
    def fake_share_directory(package_name: str) -> str:
        if package_name == "flight_cycle_manager":
            return str(ROOT)
        if package_name == "dock_apriltag_localizer":
            return str(LOCALIZER_ROOT)
        raise AssertionError(f"unexpected package lookup: {package_name}")

    monkeypatch.setattr(module, "get_package_share_directory", fake_share_directory)


def include_path(context, include: IncludeLaunchDescription) -> str:
    return perform_substitutions(
        context,
        include.launch_description_source._LaunchDescriptionSource__location,
    )


def test_publish_camera_selects_one_localizer_and_wires_debug_manager(monkeypatch):
    """Break caught: camera selection launches both variants or enables missions."""
    module = load_launch_module()
    patch_package_shares(monkeypatch, module)
    captured = {}

    def record_node(**kwargs):
        captured.update(kwargs)
        return {"node": kwargs}

    monkeypatch.setattr(module, "Node", record_node)
    description = module.generate_launch_description()
    declarations = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    includes = [
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert "publish_camera" in declarations
    assert len(includes) == 2

    external_camera_context = configured_context(
        module,
        publish_camera="false",
        image_topic="/sim/down/image_raw",
        camera_info_topic="/sim/down/camera_info",
    )
    enabled_external = [
        include
        for include in includes
        if include.condition.evaluate(external_camera_context)
    ]
    assert len(enabled_external) == 1
    assert include_path(external_camera_context, enabled_external[0]) == str(
        LOCALIZER_ROOT / "launch" / "dock_localizer.launch.py"
    )

    onboard_camera_context = configured_context(
        module,
        publish_camera="true",
        camera="1",
        camera_width="640",
        camera_height="480",
        camera_format="RGB888",
        camera_info_url="file:///tmp/down.yaml",
    )
    enabled_onboard = [
        include
        for include in includes
        if include.condition.evaluate(onboard_camera_context)
    ]
    assert len(enabled_onboard) == 1
    camera_include = enabled_onboard[0]
    assert include_path(onboard_camera_context, camera_include) == str(
        LOCALIZER_ROOT / "launch" / "dock_localizer_with_camera.launch.py"
    )
    camera_arguments = dict(camera_include.launch_arguments)
    assert camera_arguments["camera"].perform(onboard_camera_context) == "1"
    assert camera_arguments["camera_width"].perform(onboard_camera_context) == "640"
    assert camera_arguments["camera_height"].perform(onboard_camera_context) == "480"
    assert camera_arguments["camera_format"].perform(onboard_camera_context) == "RGB888"
    assert camera_arguments["camera_info_url"].perform(onboard_camera_context) == (
        "file:///tmp/down.yaml"
    )

    actions = module._launch_setup(onboard_camera_context, ROOT)
    overrides = captured["parameters"][2]
    assert actions == [{"node": captured}]
    assert captured["parameters"][:2] == [
        str(ROOT / "config" / "flight_cycle_manager.yaml"),
        str(ROOT / "config" / "pid_landing.yaml"),
    ]
    assert overrides["operation_mode"] == "pid_landing_debug"
    assert overrides["landing_backend"] == "pid"
    assert overrides["landing_attempt_log_enabled"].evaluate(
        onboard_camera_context
    ) is True
    assert overrides["landing_attempt_log_directory"].perform(
        onboard_camera_context
    ) == "~/.ros/flight_cycle_manager/landing_attempts"

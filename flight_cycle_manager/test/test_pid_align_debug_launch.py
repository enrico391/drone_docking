"""Launch contract for alignment-only XYZ and yaw PID tuning."""

from __future__ import annotations

import importlib.util
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument


ROOT = Path(__file__).parents[1]
LAUNCH_FILE = ROOT / "launch" / "pid_align_debug.launch.py"


def load_launch_module():
    assert LAUNCH_FILE.is_file(), (
        "pid_align_debug.launch.py is not implemented"
    )
    spec = importlib.util.spec_from_file_location(
        "pid_align_debug_launch_under_test", LAUNCH_FILE
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


def test_launch_wires_configurable_alignment_only_manager(monkeypatch):
    """Break caught: launch enables landing or ignores auto-start."""
    module = load_launch_module()

    def fake_share_directory(package: str) -> str:
        if package == "flight_cycle_manager":
            return str(ROOT)
        return str(ROOT.parent / "dock_apriltag_localizer")

    monkeypatch.setattr(
        module, "get_package_share_directory", fake_share_directory
    )
    captured = []

    def record_node(**kwargs):
        captured.append(kwargs)
        return {"node": kwargs}

    monkeypatch.setattr(module, "Node", record_node)
    context = configured_context(module, pid_align_auto_start="true")

    actions = module._launch_setup(context, ROOT)
    overrides = captured[0]["parameters"][2]

    assert actions == [{"node": captured[0]}, {"node": captured[1]}]
    assert overrides["operation_mode"] == "pid_align_debug"
    assert overrides["landing_backend"] == "pid"
    assert overrides["pid_align_auto_start"].evaluate(context) is True
    assert captured[1]["executable"] == "pid_tuner"
    assert captured[1]["condition"].evaluate(context) is False


def test_pid_tuner_launch_argument_enables_dashboard(monkeypatch):
    module = load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: str(ROOT if package == "flight_cycle_manager"
                            else ROOT.parent / "dock_apriltag_localizer"),
    )
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)

    context = configured_context(module, start_pid_tuner="true")
    manager, tuner = module._launch_setup(context, ROOT)

    assert manager["executable"] == "flight_cycle_manager"
    assert tuner["package"] == "flight_cycle_manager"
    assert tuner["executable"] == "pid_tuner"
    assert tuner["name"] == "pid_tuner"
    assert tuner["condition"].evaluate(context) is True

"""Legacy landing debug launch forwards arguments to the unified launch."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchContext
from launch.utilities import perform_substitutions


ROOT = Path(__file__).parents[1]


def test_landing_wrapper_includes_unified_launch(monkeypatch):
    path = ROOT / "launch" / "pid_landing_debug.launch.py"
    spec = importlib.util.spec_from_file_location("landing_wrapper_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda package: str(
        ROOT if package == "flight_cycle_manager" else ROOT.parent / "dock_apriltag_localizer"
    ))
    description = module.generate_launch_description()
    declarations = {entity.name: entity for entity in description.entities
                    if isinstance(entity, DeclareLaunchArgument)}
    includes = [entity for entity in description.entities
                if isinstance(entity, IncludeLaunchDescription)]
    assert len(includes) == 1
    assert perform_substitutions(
        LaunchContext(),
        includes[0].launch_description_source._LaunchDescriptionSource__location,
    ) == str(ROOT / "launch" / "flight_cycle.launch.py")
    arguments = dict(includes[0].launch_arguments)
    assert arguments["operation_mode"] == "pid_landing_debug"
    assert declarations["landing_attempt_log_enabled"].default_value[0].text == "true"
    for name in ("landing_attempt_log_enabled", "landing_attempt_log_directory",
                 "pid_hold_at_state", "pid_fixed_central_reference_mode", "start_pid_tuner", "publish_camera",
                 "camera_info_url"):
        assert name in declarations
        assert name in arguments
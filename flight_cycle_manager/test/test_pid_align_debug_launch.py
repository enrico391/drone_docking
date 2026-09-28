"""Legacy alignment launch forwards arguments to the unified launch."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchContext
from launch.utilities import perform_substitutions


ROOT = Path(__file__).parents[1]


def test_align_wrapper_includes_unified_launch(monkeypatch):
    path = ROOT / "launch" / "pid_align_debug.launch.py"
    spec = importlib.util.spec_from_file_location("align_wrapper_test", path)
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
    assert arguments["operation_mode"] == "pid_align_debug"
    for name in ("pid_align_auto_start", "pid_hold_at_state", "start_pid_tuner",
                 "publish_camera", "image_topic"):
        assert name in declarations
        assert name in arguments
"""Check that the branch's only flight-cycle profile constructs a DDS manager."""

from pathlib import Path

from rclpy.context import Context
from rclpy.parameter import Parameter
import rclpy
import yaml

from flight_cycle_manager.px4_manager import Px4FlightCycleManager


ROOT = Path(__file__).parents[1]


def test_profiles_declare_all_parameters_and_build_shared_controllers(monkeypatch):
    monkeypatch.setattr("flight_cycle_manager.px4_manager.get_package_share_directory",
                        lambda _: str(ROOT))
    profiles = [yaml.safe_load(path.read_text())["flight_cycle_manager"]["ros__parameters"]
                for path in sorted((ROOT / "config").glob("*.yaml"))]
    assert len(profiles) == 2
    assert set(profiles[0]).isdisjoint(profiles[1])
    expected = {key: value for profile in profiles for key, value in profile.items()}
    assert not any(key.startswith("mavlink_") or key.startswith("mission_") for key in expected)
    context = Context()
    rclpy.init(context=context)
    manager = None
    try:
        manager = Px4FlightCycleManager(context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in expected.items()
        ])
        for name, value in expected.items():
            assert manager.get_parameter(name).value == value
        assert manager._pid_config.x_pid.kp == expected["pid_x_kp"]
        assert manager._search_config.timeout_s == expected["search_timeout_s"]
        assert manager._start_service.srv_type.__name__ == "Trigger"
    finally:
        if manager is not None:
            manager.destroy_node()
        context.shutdown()

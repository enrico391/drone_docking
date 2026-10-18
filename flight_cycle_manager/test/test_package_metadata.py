from pathlib import Path
import runpy
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))


def test_package_metadata_uses_only_new_name():
    package = ET.parse(ROOT / "package.xml").getroot()
    assert package.findtext("name") == "flight_cycle_manager"
    setup_text = (ROOT / "setup.py").read_text(encoding="utf-8")
    assert 'package_name = "flight_cycle_manager"' in setup_text
    assert "flight_cycle_manager.px4_manager:main" in setup_text


def test_package_declares_runtime_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}
    assert {
        "ament_index_python",
        "dock_apriltag_localizer",
        "geometry_msgs",
        "launch",
        "launch_ros",
        "px4_msgs",
        "python3-yaml",
        "rclpy",
        "rcl_interfaces",
        "std_msgs",
        "std_srvs",
    } <= dependencies


def test_px4_message_package_identifies_rc1_snapshot():
    package = ET.parse(ROOT.parent / "px4_msgs/package.xml").getroot()
    assert package.findtext("version") == "1.18.0"
    assert "v1.18.0-rc1" in package.findtext("description")


def test_package_installs_px4_manager_and_tuner_without_mavlink(monkeypatch):
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}
    captured = {}
    monkeypatch.setattr("setuptools.setup", lambda **kwargs: captured.update(kwargs))
    monkeypatch.chdir(ROOT)
    runpy.run_path(str(ROOT / "setup.py"), run_name="__main__")

    assert "python3-pymavlink" not in dependencies
    assert "rcl_interfaces" in dependencies
    assert captured["install_requires"] == ["setuptools"]
    assert captured["entry_points"]["console_scripts"] == [
        "px4_flight_cycle_manager = flight_cycle_manager.px4_manager:main",
        "pid_tuner = flight_cycle_manager.pid_tuner:main",
    ]
    installed = {Path(path).name for _, paths in captured["data_files"] for path in paths}
    assert {name for name in installed if name.endswith(".launch.py")} == {
        "px4_flight_cycle.launch.py",
    }
    assert {name for name in installed if name.endswith(".yaml")} == {
        "px4_flight_cycle.yaml", "pid_landing.yaml",
    }


def test_package_has_no_legacy_runtime_modules():
    modules = {path.stem for path in (ROOT / "flight_cycle_manager").glob("*.py")}
    assert modules == {
        "__init__", "dock_search", "landing_backend", "landing_target", "pid",
        "pid_landing", "pid_landing_backend", "px4_control", "px4_manager", "pid_tuner",
    }

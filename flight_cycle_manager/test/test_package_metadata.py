from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))


def test_package_metadata_uses_only_new_name():
    legacy_name = "_".join(("drone", "mission", "manager"))
    package = ET.parse(ROOT / "package.xml").getroot()
    assert package.findtext("name") == "flight_cycle_manager"
    setup_text = (ROOT / "setup.py").read_text(encoding="utf-8")
    assert 'package_name = "flight_cycle_manager"' in setup_text
    assert "flight_cycle_manager.manager:main" in setup_text
    assert legacy_name not in setup_text


def test_package_declares_runtime_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}
    assert {
        "ament_index_python",
        "dock_apriltag_localizer",
        "geometry_msgs",
        "launch",
        "launch_ros",
        "python3-pymavlink",
        "python3-yaml",
        "rclpy",
        "std_msgs",
        "std_srvs",
    } <= dependencies


def test_package_declares_pymavlink_for_rosdep_and_python_install():
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}
    setup_text = (ROOT / "setup.py").read_text(encoding="utf-8")

    assert "python3-pymavlink" in dependencies
    assert '"pymavlink>=2.4.49,<3"' in setup_text

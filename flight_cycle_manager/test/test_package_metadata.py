from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))


def test_package_metadata_uses_only_new_name():
    package = ET.parse(ROOT / "package.xml").getroot()
    assert package.findtext("name") == "flight_cycle_manager"
    setup_text = (ROOT / "setup.py").read_text(encoding="utf-8")
    assert 'package_name = "flight_cycle_manager"' in setup_text
    assert "flight_cycle_manager.manager:main" in setup_text
    assert "drone_mission_manager" not in setup_text


def test_package_declares_runtime_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}
    assert {"rclpy", "geometry_msgs", "std_msgs", "std_srvs"} <= dependencies

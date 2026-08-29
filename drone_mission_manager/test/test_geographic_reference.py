"""Regression tests for the docking site's geographic simulation reference."""

from pathlib import Path
import xml.etree.ElementTree as ET


DESCRIPTION_ROOT = Path(__file__).parents[2] / "drone_description"
WORLD_FILE = DESCRIPTION_ROOT / "worlds" / "docking_world.sdf"
PARAM_FILE = DESCRIPTION_ROOT / "config" / "gazebo-x500.parm"


def test_dock_and_world_origin_use_future_docking_station_coordinates():
    """The dock is the geographic origin used by Gazebo and ArduPilot SITL."""
    root = ET.parse(WORLD_FILE).getroot()
    world = root.find("./world")
    spherical = world.find("spherical_coordinates")
    dock = next(
        include.find("pose")
        for include in world.findall("include")
        if include.findtext("name") == "dock_station"
    )

    assert spherical is not None
    assert float(spherical.findtext("latitude_deg")) == 45.3821944
    assert float(spherical.findtext("longitude_deg")) == 12.0005833
    assert float(spherical.findtext("elevation")) == 2.0
    assert [float(value) for value in dock.text.split()[:2]] == [0.0, 0.0]


def test_ardupilot_sitl_startup_position_matches_world_reference():
    """ArduPilot's simulated original position matches the Gazebo origin."""
    params = {}
    for line in PARAM_FILE.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) == 2 and fields[0].startswith("SIM_OPOS_"):
            params[fields[0]] = float(fields[1])

    assert params == {
        "SIM_OPOS_LAT": 45.3821944,
        "SIM_OPOS_LNG": 12.0005833,
        "SIM_OPOS_ALT": 2.0,
    }

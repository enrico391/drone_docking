"""Attach the built PX4 SITL binary to an already running Gazebo world.

PX4 runs in Gazebo "standalone" mode: Gazebo and the vehicle model are started
by the caller (see ``px4_docking.launch.py``) and PX4 attaches to the existing
model by name through its ``gz_bridge`` module. This keeps the custom docking
world, airframe and camera shared with the ArduPilot stack.

The binary is expected at
``<workspace>/.worktrees/PX4-Autopilot/build/px4_sitl_default/bin/px4`` unless
``px4_dir`` overrides the source tree location.
"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Airframe id of the stock PX4 "Gazebo x500" quadrotor.
X500_AIRFRAME_ID = "4001"


def _parameter_environment(path: Path) -> dict:
    """Apply the docking profile through PX4 rcS's parameter overrides.

    These overrides are read after the saved BSON parameters and before the
    simulator and controllers start. Unlike a one-time MAVLink upload, this also
    works with a fresh rootfs or stale stock-x500 settings.
    """
    environment = {}
    lines = path.read_text(encoding="utf-8").splitlines()
    for number, line in enumerate(lines, 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 2:
            raise ValueError(f"Invalid PX4 parameter at {path}:{number}")
        name, value = fields
        if not math.isfinite(float(value)):
            raise ValueError(f"Non-finite PX4 parameter at {path}:{number}")
        environment[f"PX4_PARAM_{name}"] = value
    return environment


def _workspace_root() -> Path:
    """Derive the colcon workspace root from the installed package location."""
    return Path(get_package_share_directory("drone_description")).parents[3]


def start_px4(context: LaunchContext):
    """Create the PX4 SITL process after validating the build artifacts."""
    px4_dir = LaunchConfiguration("px4_dir").perform(context)
    px4_root = Path(px4_dir) if px4_dir else (
        _workspace_root() / ".worktrees" / "PX4-Autopilot"
    )
    build_dir = px4_root / "build" / "px4_sitl_default"
    binary = build_dir / "bin" / "px4"
    if not binary.is_file():
        raise RuntimeError(
            f"PX4 SITL binary is missing at {binary}; build it with "
            f"'make px4_sitl_default' inside {px4_root}."
        )

    model_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    environment = {
        "PX4_SYS_AUTOSTART": X500_AIRFRAME_ID,
        "PX4_SIM_MODEL": model_name,
        # Standalone mode: Gazebo is already running, only attach to the model.
        "PX4_GZ_STANDALONE": "1",
        "PX4_GZ_MODEL_NAME": model_name,
        "PX4_GZ_WORLD": world_name,
        # Do not let PX4 start a second Gazebo GUI or server.
        "HEADLESS": "1" if headless else "0",
        "PX4_HOME_LAT": LaunchConfiguration("home_lat").perform(context),
        "PX4_HOME_LON": LaunchConfiguration("home_lon").perform(context),
        "PX4_HOME_ALT": LaunchConfiguration("home_alt").perform(context),
    }
    environment.update(_parameter_environment(
        Path(get_package_share_directory("drone_description"))
        / "config" / "px4_sitl_docking.parm"
    ))
    return [
        ExecuteProcess(
            cmd=[str(binary)],
            cwd=str(build_dir / "rootfs"),
            additional_env=environment,
            output="screen",
            shell=False,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the PX4 SITL launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_dir",
                default_value="",
                description=(
                    "PX4-Autopilot source tree; empty uses "
                    "<workspace>/.worktrees/PX4-Autopilot."
                ),
            ),
            DeclareLaunchArgument("robot_name", default_value="x500"),
            DeclareLaunchArgument("world_name", default_value="docking_world"),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Keep PX4 from starting its own Gazebo GUI.",
            ),
            # Must match the docking world's spherical_coordinates block.
            DeclareLaunchArgument("home_lat", default_value="45.3821944"),
            DeclareLaunchArgument("home_lon", default_value="12.0005833"),
            DeclareLaunchArgument("home_alt", default_value="2.0"),
            OpaqueFunction(function=start_px4),
        ]
    )

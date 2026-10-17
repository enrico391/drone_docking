"""Launch the docking world, the custom X500 and the PX4 SITL stack.

Gazebo is started here (server and optional GUI) with the project
``docking_world``, the PX4-compatible X500 model is spawned by name and PX4
attaches to it in standalone mode. The ROS-Gazebo bridge exposes the same topics
as the ArduPilot flow (clock, TF, joint states, camera and disturbance wrenches)
so the docking stack can be compared across the two firmwares.
"""

import tempfile
from pathlib import Path
from typing import List

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Model spawned for PX4. It is the PX4 counterpart of the ArduPilot
# x500_with_ardupilot chain: same airframe, camera and docking platform.
PX4_MODEL_DIR = "x500_px4_mono_cam_down"


def _render_bridge_config(
    source: str, world_name: str, robot_name: str
) -> str:
    """Substitute the world and model name placeholders in the bridge config."""
    config = Path(source).read_text(encoding="utf-8")
    config = config.replace("{{ world_name }}", world_name)
    config = config.replace("{{ robot_name }}", robot_name)
    mappings = yaml.safe_load(config)
    output = tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".yaml", encoding="utf-8"
    )
    with output:
        yaml.safe_dump(mappings, output)
    return output.name


def create_robot_actions(
    context: LaunchContext,
) -> List[Node]:
    """Spawn the vehicle at a named or explicitly configured initial point."""
    initial_point = LaunchConfiguration("initial_point").perform(context)
    named_points = {
        # Free ground start. The dock's lower body is a solid 1.6 m box centred
        # on the origin up to z=0.88, so spawning at (0, 0) would place the
        # vehicle inside the dock collision and Bullet would eject it.
        "ground": ("2.0", "2.0", "-0.032"),
        # The dock recess floor is z=0.880, so add it to the model offset.
        "dock": ("0.0", "0.0", "0.848"),
    }
    if initial_point == "custom":
        x = LaunchConfiguration("spawn_x").perform(context)
        y = LaunchConfiguration("spawn_y").perform(context)
        z = LaunchConfiguration("spawn_z").perform(context)
    else:
        x, y, z = named_points[initial_point]

    description_share = get_package_share_directory("drone_description")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    model_file = str(
        Path(description_share) / "models" / PX4_MODEL_DIR / "model.sdf"
    )

    # No -world argument: create retries /gazebo/worlds until the server answers
    # and then targets the only running world, which is docking_world.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        arguments=[
            "-file", model_file,
            "-name", robot_name,
            "-x", x,
            "-y", y,
            "-z", z,
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
    )

    bridge_file = _render_bridge_config(
        str(Path(description_share) / "config" / "x500_bridge.yaml"),
        world_name,
        robot_name,
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "config_file": bridge_file,
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
        condition=IfCondition(LaunchConfiguration("use_ros_gz_bridge")),
    )
    tf_relay = Node(
        package="topic_tools",
        executable="relay",
        namespace=LaunchConfiguration("namespace"),
        arguments=["gz/tf", "tf"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )
    return [spawn, bridge, tf_relay]


def generate_launch_description() -> LaunchDescription:
    """Generate the PX4 docking launch description."""
    description_share = get_package_share_directory("drone_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = Path(description_share) / "worlds" / "docking_world.sdf"

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ros_gz_sim_share) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                "-v4 -s -r --headless-rendering --physics-engine ",
                LaunchConfiguration("physics_engine"),
                f" {world_path}",
            ]
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_server")),
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ros_gz_sim_share) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_gui")),
    )

    px4_sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [description_share, "launch", "sitl_px4.launch.py"]
            )
        ),
        launch_arguments={
            "px4_dir": LaunchConfiguration("px4_dir"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world_name": LaunchConfiguration("world_name"),
            "headless": LaunchConfiguration("px4_headless"),
            "home_lat": LaunchConfiguration("home_lat"),
            "home_lon": LaunchConfiguration("home_lon"),
            "home_alt": LaunchConfiguration("home_alt"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("robot_name", default_value="x500"),
            DeclareLaunchArgument("world_name", default_value="docking_world"),
            DeclareLaunchArgument(
                "initial_point",
                default_value="ground",
                choices=["ground", "dock", "custom"],
                description="Named vehicle spawn point or custom coordinates.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="-0.032",
                description="Custom X500 model spawn height.",
            ),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "physics_engine",
                default_value="gz-physics-dartsim-plugin",
                description=(
                    "Gazebo physics backend. Use DART for PX4: the installed "
                    "Bullet Featherstone backend omits linear acceleration "
                    "from IMU measurements, destabilising the estimator."
                ),
            ),
            DeclareLaunchArgument(
                "px4_dir",
                default_value="",
                description=(
                    "PX4-Autopilot source tree; empty uses "
                    "<workspace>/.worktrees/PX4-Autopilot."
                ),
            ),
            DeclareLaunchArgument(
                "px4_headless",
                default_value="true",
                description="Keep PX4 from starting its own Gazebo GUI.",
            ),
            DeclareLaunchArgument("use_gz_sim_server", default_value="true"),
            DeclareLaunchArgument("use_gz_sim_gui", default_value="true"),
            DeclareLaunchArgument("use_ros_gz_bridge", default_value="true"),
            DeclareLaunchArgument("use_gz_tf", default_value="true"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            # Must match the docking world's spherical_coordinates block.
            DeclareLaunchArgument("home_lat", default_value="45.3821944"),
            DeclareLaunchArgument("home_lon", default_value="12.0005833"),
            DeclareLaunchArgument("home_alt", default_value="2.0"),
            gz_server,
            gz_gui,
            OpaqueFunction(function=create_robot_actions),
            px4_sitl,
        ]
    )

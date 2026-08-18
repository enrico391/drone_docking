# Drone Mission Manager

This package provides a non-blocking ROS 2 mission manager for the first
complete simulation mission:

1. wait for AP_DDS and a valid local pose;
2. retry the pre-arm check while ArduPilot initializes, then enter GUIDED mode,
   arm, and take off;
3. fly one configurable circle using `/ap/v1/cmd_vel`;
4. return to the simulated dock at `(10, 0)`;
5. acquire the dock pose from the outer or central AprilTags; and
6. land using either the ROS velocity controller or ArduPilot's native
   `LANDING_TARGET` precision-landing controller.

The launch file starts `dock_apriltag_localizer` by default. Pass
`start_localizer:=false` only when a localizer is already running separately.

## Mission files

Store version-controlled mission definitions in `config/missions/`. The first
mission is `config/missions/simple_circle.yaml`. Controller tuning and AP_DDS
interface names remain in `config/mission_manager.yaml`; this keeps a mission's
geometry separate from node/runtime configuration.

The coarse mission coordinates use the ROS local ENU `map` convention: X east,
Y north, and Z up. Precision corrections use `base_link`: the downward optical
frame's X/Y offsets are converted to body FLU velocity commands.

The `ros` landing backend centers and descends with `/ap/v1/cmd_vel`, switches
to the central tag for the final approach, and enters ordinary LAND near
touchdown. Descent is allowed only with a fresh dock pose inside the configured
horizontal tolerance. A lost target causes an immediate hover and
reacquisition.

The `ardupilot` backend converts `/dock/pose` from camera optical coordinates
to MAVLink Body-FRD and sends `LANDING_TARGET` at the mission update rate
(10 Hz by default) through MAVProxy's dedicated UDP endpoint on port 14551.
After a one-second measurement warmup it enters LAND; ArduPilot then owns both
horizontal correction and descent. The X500 defaults set `PLND_ENABLED=1` and
`PLND_TYPE=1`, and the mission checks those live values before arming.

After LAND mode is confirmed, the native backend also calculates the signed
drone-to-dock yaw error from `/dock/pose` and sends one relative
`MAV_CMD_CONDITION_YAW` in a MAVLink `COMMAND_LONG`. ArduPilot converts this
into a persistent heading target, so its closed-loop yaw controller rejects
disturbances such as wind and returns the vehicle to the commanded heading; it
is not a one-time yaw-rate impulse. One command is appropriate while the dock
orientation is fixed and the initial visual measurement is reliable. It does
not track a dock that rotates after the command or correct a bad initial visual
estimate. Supporting those cases would require filtered periodic updates with
a deadband, preferably using absolute heading targets to avoid accumulating
relative corrections. The mission log records the yaw error used by the
command.

Landing completion is event-driven. While ArduPilot reports `flying=true`, the
mission remains in `WAIT_LANDED` and continues the `LANDING_TARGET` stream. A
`dock_land_timeout_s` overrun emits one warning but does not fail the mission.
When ArduPilot reports `flying=false`, the mission enters `WAIT_DISARM`; it
completes only after `armed=false`. `dock_disarm_timeout_s` is likewise a
warning threshold, allowing ArduPilot's configured automatic-disarm delay to
finish without an artificial mission failure.

## Build and run

Start the docking simulation first and wait for ArduPilot and DDS readiness:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_bringup docking_sim.launch.py 
```

use the following command if your system does not support OpenGL hardware acceleration (WSL):

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch drone_bringup docking_sim.launch.py
```

In a second sourced terminal, build after changes and launch the mission with
the explicit flight opt-in:

```bash
colcon build --symlink-install --packages-select drone_mission_manager
source install/setup.bash
ros2 launch drone_mission_manager simple_circle_mission.launch.py \
  auto_start:=true landing_backend:=ros
```

To use ArduPilot's native precision landing, rebuild both changed packages and
restart the simulation so the `PLND_` defaults are loaded, then select the
second backend:

```bash
colcon build --symlink-install \
  --packages-select drone_description drone_mission_manager
source install/setup.bash
ros2 launch drone_mission_manager simple_circle_mission.launch.py \
  auto_start:=true landing_backend:=ardupilot
```

The two accepted launch values are:

- `landing_backend:=ros`: ROS commands perform visual alignment and descent;
- `landing_backend:=ardupilot`: ROS supplies visual target measurements and
  ArduPilot precision landing performs the complete descent in LAND mode.

Port 14551 must remain available to the mission manager. MAVProxy already
publishes vehicle traffic to this port in the workspace's SITL launch, so no
additional MAVLink process is required.

Monitor the state with:

```bash
ros2 topic echo /mission_manager/state
```

Useful visual-localization diagnostics are:

```bash
ros2 topic echo /dock/mode
ros2 topic echo /dock/pose
ros2 topic hz /dock/debug_image
```

To run another checked-in mission, pass its installed or absolute source path:

```bash
ros2 launch drone_mission_manager simple_circle_mission.launch.py \
  mission_file:=/absolute/path/to/mission.yaml auto_start:=true
```

This controller is currently intended for the workspace's SITL/Gazebo setup.
Stopping it before the LAND handoff stops the command stream but does not land
or disarm the vehicle.

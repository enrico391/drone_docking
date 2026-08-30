# Drone Mission Manager

This package provides a non-blocking ROS 2 mission manager with typed mission
profiles and a shared precision-landing sequence. Two profiles are included:

- `simple_circle`: take off from the ground start, fly the original circle,
  return to the fixed dock, and land;
- `dock_circle`: take off from the dock, fly a 2 m radius circle around it,
  return overhead, and precision-land on the same dock.

The common execution sequence is:

1. wait for a direct pymavlink heartbeat and valid local position;
2. retry the pre-arm check while ArduPilot initializes, then enter GUIDED mode,
   arm, and take off;
3. fly one configurable circle using MAVLink velocity setpoints;
4. return to the configured dock;
5. acquire the dock pose from the outer or central AprilTags; and
6. land using either direct GUIDED PID control or ArduPilot's native
   `LANDING_TARGET` precision-landing controller.

The launch file starts `dock_apriltag_localizer` by default. Pass
`start_localizer:=false` only when a localizer is already running separately.

## Mission files

Store version-controlled mission definitions in `config/missions/`. The
included files are `simple_circle.yaml` and `dock_circle.yaml`. Controller
tuning and direct MAVLink connection settings remain in
`config/mission_manager.yaml`;
this keeps mission geometry separate from node/runtime configuration.

Every mission declares `mission.type`. Parsers are registered in
`drone_mission_manager/mission.py`, so another mission schema can be added
without changing how YAML files are selected. The current types are `circle`
(an explicit center) and `dock_circle` (the return dock is also the center).

`mission.coordinates.reference` controls horizontal coordinates:

- `map`: X/Y values are absolute local ENU map coordinates;
- `initial_pose`: X/Y values are offsets from the pose captured before arming.

The dock mission uses `initial_pose`, which makes its `(0, 0)` dock and circle
center follow the selected simulation start point.

Missions can repeat a complete takeoff-to-landing cycle with the optional
`mission.repeat` section. `count` is the total number of executions, not the
number of extra repetitions. `delay_s` is the settling time after confirmed
disarm and before the next pre-arm sequence. For example:

```yaml
mission:
  repeat:
    count: 3
    delay_s: 3.0
```

The checked-in `dock_circle.yaml` defaults to `count: 3`. Repetition greater
than one requires precision landing because each new cycle starts only after
landing and automatic disarm have been confirmed.

The coarse mission coordinates use the ROS local ENU `map` convention: X east,
Y north, and Z up. Precision corrections use `base_link`: the downward optical
frame's X/Y offsets are converted to body FLU velocity commands.

`landing_mode:=pid` keeps ArduPilot in `GUIDED` and sends body-frame X/Y/Z
velocity and yaw-rate commands directly through pymavlink. If no configured tag
is visible after initial acquisition, it searches a bounded circle around the
expected dock position. It aligns on an outer or central tag, requires the
central tag for final descent, and climbs to reacquire it if it is lost above
the touchdown commit height. After commit, tag loss is ignored and the vehicle
continues a slow vertical touchdown approach. A normal disarm is requested only
after fresh, stable `ON_GROUND` telemetry; mission completion still requires an
`armed=false` heartbeat.

`landing_mode:=precise_landing` preserves the native ArduPilot path in
`mission_manager`. It converts `/dock/pose` from camera optical coordinates to
MAVLink Body-FRD and streams each accepted filtered pose at most once through
the direct pymavlink connection, using the source image timestamp. After a
one-second measurement warmup it enters `LAND`, where ArduPilot owns horizontal
correction and descent. The X500 defaults set `PLND_ENABLED=1` and
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

Native landing completion is event-driven. While ArduPilot reports
`flying=true`, the mission remains in `WAIT_LANDED` and continues the
`LANDING_TARGET` stream. A `dock_land_timeout_s` overrun emits one warning but
does not fail the mission. When ArduPilot reports `flying=false`, the mission
enters `WAIT_DISARM`; it completes only after `armed=false`.
`dock_disarm_timeout_s` is likewise a warning threshold, allowing ArduPilot's
configured automatic-disarm delay to finish without an artificial mission
failure.

Hardware docking contact is not used yet, and no docking-contact signal exists
in the simulation. PID touchdown confirmation currently relies on ArduPilot's
landed-state telemetry.

## Build and run

Start the docking simulation without AP_DDS or MAVProxy so the mission manager
can own ArduPilot's TCP endpoint directly:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_bringup docking_sim.launch.py \
  use_dds_agent:=false use_mavproxy:=false
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
ros2 launch drone_mission_manager mission.launch.py \
  mission:=simple_circle auto_start:=true landing_mode:=pid
```

To use ArduPilot's native precision landing, rebuild both changed packages and
restart the simulation so the `PLND_` defaults are loaded, then select the
second backend:

```bash
colcon build --symlink-install \
  --packages-select drone_description drone_mission_manager
source install/setup.bash
ros2 launch drone_mission_manager mission.launch.py \
  mission:=simple_circle auto_start:=true landing_mode:=precise_landing \
  landing_yaw_correction_enabled:=true \
  pose_filter_enabled:=true
```

The canonical launch values are:

- `landing_mode:=pid`: the manager performs GUIDED visual alignment, search,
  recovery, committed touchdown, and normal disarm;
- `landing_mode:=precise_landing`: the manager supplies visual target
  measurements and ArduPilot performs the descent in `LAND` mode.

For one compatibility period, the optional `landing_backend` argument accepts
`ros` as an alias for `pid` and `ardupilot` as an alias for
`precise_landing`. For a legacy-only invocation, leave `landing_mode` at its
blank launch default and set only `landing_backend`; blank/blank still selects
`pid`. Do not supply conflicting nonblank canonical and legacy selections.

Both values use pymavlink for all flight-controller telemetry and commands.
ROS 2 remains the application bus for dock localization and mission-state
output. The current mission state sequence is unchanged, and onboard
`DO_LAND_START` triggering is not enabled yet.

Set `landing_yaw_correction_enabled:=false` to keep the vehicle's existing
heading during ArduPilot precision landing. It defaults to `true` to preserve
the original dock-relative yaw correction.

The localizer filter defaults can be tuned from this launch file with
`pose_filter_time_constant_s`, `pose_filter_max_translation_speed_m_s`,
`pose_filter_max_position_jump_m`, and `pose_filter_reacquire_frames`. See the
`dock_apriltag_localizer` README for their behavior.

SITL uses `mavlink_connection:=tcp:127.0.0.1:5760` by default. Exactly one
process should own that endpoint. For hardware, pass a serial device and baud,
for example `mavlink_connection:=/dev/ttyAMA0 mavlink_baud:=921600`.

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

To run the new dock-origin mission, start the simulator at the named dock point:

```bash
ros2 launch drone_bringup docking_sim.launch.py initial_point:=dock \
  use_dds_agent:=false use_mavproxy:=false
```

Then start the mission in another sourced terminal:

```bash
ros2 launch drone_mission_manager mission.launch.py \
  mission:=dock_circle auto_start:=true landing_mode:=pid
```

If `dock_circle.yaml` has `repeat.count: 3`, the manager performs three full
takeoff-circle-precision-land cycles. Between cycles it publishes
`REPEAT_DELAY` on `/mission_manager/state`, waits for `repeat.delay_s`, and then
starts the normal pre-arm sequence again.

The simulation accepts `initial_point:=ground` (the backward-compatible
default), `initial_point:=dock`, or `initial_point:=custom`. For a custom point,
pass world coordinates explicitly, for example:

```bash
ros2 launch drone_bringup docking_sim.launch.py \
  initial_point:=custom spawn_x:=4.0 spawn_y:=-2.0 spawn_z:=-0.032 \
  spawn_yaw:=1.57 use_dds_agent:=false use_mavproxy:=false
```

To run an external typed mission, pass its absolute source path:

```bash
ros2 launch drone_mission_manager mission.launch.py \
  mission_file:=/absolute/path/to/mission.yaml auto_start:=true
```

This controller is currently intended for the workspace's SITL/Gazebo setup.
Stopping the mission manager stops its command stream but does not itself land
or disarm the vehicle.

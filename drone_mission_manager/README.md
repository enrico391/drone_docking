# Drone Mission Manager

This package provides a non-blocking ROS 2 mission manager with typed mission
profiles and a shared precision-landing sequence. Two profiles are included:

- `simple_circle`: take off from the ground start, fly the original circle,
  return to the fixed dock, and land;
- `dock_circle`: take off from the dock, fly a 2 m radius circle around it,
  return overhead, and precision-land on the same dock.

The common execution sequence is:

1. wait for AP_DDS and a valid local pose;
2. retry the pre-arm check while ArduPilot initializes, then enter GUIDED mode,
   arm, and take off;
3. fly one configurable circle using `/ap/v1/cmd_vel`;
4. return to the configured dock;
5. acquire the dock pose from the outer or central AprilTags; and
6. land using either the ROS velocity controller or ArduPilot's native
   `LANDING_TARGET` precision-landing controller.

The launch file starts `dock_apriltag_localizer` by default. Pass
`start_localizer:=false` only when a localizer is already running separately.

## Mission files

Store version-controlled mission definitions in `config/missions/`. The
included files are `simple_circle.yaml` and `dock_circle.yaml`. Controller
tuning and AP_DDS interface names remain in `config/mission_manager.yaml`;
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

The checked-in `dock_circle.yaml` defaults to `count: 1`. Increase it to the
required number of complete dock-circle missions. Repetition greater than one
requires precision landing because each new cycle starts only after landing
and automatic disarm have been confirmed.

The coarse mission coordinates use the ROS local ENU `map` convention: X east,
Y north, and Z up. Precision corrections use `base_link`: the downward optical
frame's X/Y offsets are converted to body FLU velocity commands.

The `ros` landing backend centers and descends with `/ap/v1/cmd_vel`, switches
to the central tag for the final approach, and enters ordinary LAND near
touchdown. Descent is allowed only with a fresh dock pose inside the configured
horizontal tolerance. A lost target causes an immediate hover and
reacquisition.

The `ardupilot` backend converts `/dock/pose` from camera optical coordinates
to MAVLink Body-FRD and checks for new measurements at the mission update rate
(10 Hz by default). Each accepted filtered pose is sent at most once through
MAVProxy's dedicated UDP endpoint on port 14551, using the source image
timestamp; timer ticks no longer turn a stale pose into a new measurement.
Advancement requires a continuous one-second stream at 5 Hz or faster, no gap
over 0.25 s, pose age below 0.35 s, and horizontal error below 0.5 m.

The native sequence then performs these autonomous stages:

1. hold the coarse hover in GUIDED and align yaw from a stable OUTER estimate;
2. enter LOITER, enable Precision Loiter through auxiliary function 39, and
   require X/Y below 0.15 m plus yaw below 5 degrees for 1.5 s;
3. switch directly from LOITER to LAND, with no ROS velocity commands;
4. optionally send bounded absolute-heading corrections at the debounced 3 m
   and 2 m height checkpoints; and
5. validate a stable CENTRAL target at 1 m, then lock yaw for the final metre.

The checkpoint height is the flight controller's filtered local Z minus the
mission's `return.dock_surface_z_m`; no rangefinder is required. The X500
defaults therefore set `PLND_ALT_MIN=0` and `PLND_ALT_MAX=0`, and the manager
checks these values along with `PLND_ENABLED=1`, `PLND_TYPE=1`, and
`PLND_EST_TYPE=0` before arming. `PLND_XY_DIST_MAX=0.25` and
`LAND_SPD_MS=0.25` are checked as recommended tuning values.

Source changes and filter initialization, gap reset, or reacquisition clear all
accumulated stream/yaw stability. Above the low-clearance final region, a dock
pose older than 0.35 s exits LAND, disables Precision Loiter, holds LOITER,
reacquires, realigns in GUIDED, and repeats pre-alignment before restarting
LAND. The default final target-loss policy is `continue`: it sends no stale
target and adds no yaw command near touchdown. Set
`landing_final_target_loss_action: retry` only when the dock has enough safe
clearance for a low-altitude retry. The same choice is exposed as the
`landing_final_target_loss_action:=continue|retry` launch argument.

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
ros2 launch drone_mission_manager mission.launch.py \
  mission:=simple_circle auto_start:=true landing_backend:=ros
```

To use ArduPilot's native precision landing, rebuild both changed packages and
restart the simulation so the `PLND_` defaults are loaded, then select the
second backend:

```bash
colcon build --symlink-install \
  --packages-select drone_description drone_mission_manager
source install/setup.bash
ros2 launch drone_mission_manager mission.launch.py \
  mission:=simple_circle auto_start:=true landing_backend:=ardupilot \
  landing_yaw_correction_enabled:=true \
  pose_filter_enabled:=true
```

The two accepted launch values are:

- `landing_backend:=ros`: ROS commands perform visual alignment and descent;
- `landing_backend:=ardupilot`: ROS supplies visual target measurements and
  ArduPilot precision landing performs the complete descent in LAND mode.

Set `landing_yaw_correction_enabled:=false` to keep the vehicle's existing
heading. It defaults to `true` for GUIDED pre-alignment and the bounded 3 m/2 m
checkpoints; yaw is never commanded in the final metre.

The localizer filter defaults can be tuned from this launch file with
`pose_filter_time_constant_s`, `pose_filter_max_translation_speed_m_s`,
`pose_filter_max_position_jump_m`, and `pose_filter_reacquire_frames`. See the
`dock_apriltag_localizer` README for their behavior.

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
ros2 topic echo /dock/filter_status
ros2 topic hz /dock/debug_image
```

To run the new dock-origin mission, start the simulator at the named dock point:

```bash
ros2 launch drone_bringup docking_sim.launch.py initial_point:=dock
```

Then start the mission in another sourced terminal:

```bash
ros2 launch drone_mission_manager mission.launch.py \
  mission:=dock_circle auto_start:=true landing_backend:=ros
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
  spawn_yaw:=1.57
```

To run an external typed mission, pass its absolute source path:

```bash
ros2 launch drone_mission_manager mission.launch.py \
  mission_file:=/absolute/path/to/mission.yaml auto_start:=true
```

This controller is currently intended for the workspace's SITL/Gazebo setup.
Stopping it before the LAND handoff stops the command stream but does not land
or disarm the vehicle.

# Flight Cycle Manager — PX4 DDS

This branch contains only the native PX4 ROS 2/uXRCE-DDS flight-cycle path.
It does not depend on pymavlink or include the old ArduPilot manager, mission
tools, precision-landing backend, PID tuner, debug launches or CSV loggers.

## Package layout

- `launch/px4_flight_cycle.launch.py`: the only manager launch file.
- `config/px4_flight_cycle.yaml`: DDS, takeoff, search and safety settings.
- `config/pid_landing.yaml`: transport-independent PID landing settings.
- `px4_manager.py`: ROS node and non-blocking flight lifecycle.
- `px4_control.py`: DDS topic naming, frame conversion and PID configuration.
- `dock_search.py`, `pid.py`, `pid_landing.py`, `pid_landing_backend.py`:
  reusable, transport-independent controllers.
- `landing_backend.py`, `landing_target.py`: observations and camera/body helpers.

The installed executable remains `px4_flight_cycle_manager`; the ROS node is
`flight_cycle_manager`. Existing PX4 simulation launch commands are unchanged.

## Lifecycle and DDS topics

`READY -> WARMUP -> SET_OFFBOARD -> ARM -> TAKEOFF -> HOVER`

An explicit `request_dock` then starts bounded visual search and PID landing.
Takeoff height is relative to the vehicle's resting position, including starts
on the dock. This manager owns one explicit cycle; it does not repeat cycles.

PX4 inputs:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

PX4 outputs:

- `/fmu/out/vehicle_status_v4`
- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_land_detected`
- `/fmu/out/vehicle_command_ack_v1`

The included `px4_msgs` subset matches PX4 **v1.18.0-rc1**, commit
`fca3df865af36124a28c9d607e850f111dbaaea9`. Topic suffixes are derived from the
generated message versions. Rebuild and source the workspace after changing
interfaces; topic remapping alone cannot convert incompatible message layouts.
Set `px4_namespace:=px4_1` on both simulation and manager to prefix DDS topics
with `/px4_1`. ROS `namespace` is separate from the PX4 DDS namespace.

## Build and run

```bash
source /opt/ros/humble/setup.bash
cd /home/enri-ubuntu/drone_ws
colcon build --symlink-install --packages-select px4_msgs flight_cycle_manager drone_description
source /home/enri-ubuntu/drone_ws/install/setup.bash

ros2 launch drone_description px4_docking.launch.py \
  use_flight_cycle_manager:=true use_disturbances:=true initial_point:=dock
```

Run an RC1 firmware build: updating these ROS interfaces does not switch or
rebuild `/home/enri-ubuntu/drone_ws/.worktrees/PX4-Autopilot`, whose existing
v1.16.2 build does not directly match them. Pass `px4_dir` to the simulation
launch to select a separately prepared RC1 checkout.

The simulation starts the installed v2.4 `micro_ros_agent` on UDP 8888 for
PX4's built-in uXRCE-DDS client (RC1's default DDS v2 build on ROS 2 Humble).
Use `use_dds_agent:=false` if an agent already owns that port. `dds_port` is
forwarded to both PX4 and the agent. Do not run
multiple identical Gazebo worlds in the same Gazebo transport partition.

For an already-running PX4 simulation, start only the manager and localizer:

```bash
ros2 launch flight_cycle_manager px4_flight_cycle.launch.py
```

Use `start_localizer:=false` if another launch owns `/dock/pose` and `/dock/mode`.
The localizer uses the workspace's AprilTag virtual environment when available.
Start near/above the dock (`initial_point:=dock`): the bounded search is not
autonomous navigation from the free-ground spawn point to the docking station.

In a second sourced terminal:

```bash
ros2 service call /flight_cycle_manager/start_cycle std_srvs/srv/Trigger '{}'
# Wait for HOVER before requesting docking.
ros2 service call /flight_cycle_manager/request_dock std_srvs/srv/Trigger '{}'

ros2 topic echo /flight_cycle_manager/state --qos-durability transient_local
ros2 topic echo /flight_cycle_manager/reason --qos-durability transient_local
ros2 topic echo /flight_cycle_manager/pid_debug
```

`auto_start:=true` queues one simulation takeoff only; it does not request
docking automatically. The manager launch also accepts `config_file`,
`pid_config`, `image_topic`, `camera_info_topic` and `localizer_python`.
Controller settings are loaded at startup; a live tuning UI is not provided.

### Disturbances

`use_disturbances:=true` starts the injector **unloaded** by default. After
takeoff, enable/disable it at runtime:

```bash
ros2 param set /disturbance_injector enabled true
ros2 param set /disturbance_injector enabled false
```

`disturbance_enabled:=true` starts with loads applied. The disturbance injector
and its configuration belong to `drone_description`, not this package.

## Safety and limitations

- Fresh PX4 status, local-position and landing telemetry are required to start.
- Arming and mode changes require command ACKs and state confirmation.
- Position-mode takeoff allows PX4's expected in-flight magnetic yaw alignment;
  body-velocity docking requires final heading alignment.
- ENU/body-FLU commands and yaw rates are converted to PX4 local NED.
- Transport timestamps follow PX4's adjusted DDS telemetry clock. Pose-age
  checks use Gazebo time; command/telemetry watchdogs use monotonic time and
  continue running while Gazebo is paused.
- Telemetry loss stops Offboard streaming, leaving PX4's configured
  Offboard-loss LAND failsafe in charge. Unexpected mode changes latch
  `PILOT_OVERRIDE` without forcing Offboard re-entry.
- Search or PID failure requests native LAND, which does not guarantee landing
  on the dock. Disarm is never forced and requires PX4 touchdown confirmation.
- The SITL profile selects no-RC input and Offboard-loss LAND. Real hardware
  requires a separately reviewed profile.

The old ArduPilot onboard-mission download/reset and AUTO handoff contract is
not implemented. PX4's default DDS topics do not expose an equivalent mission
download protocol. Debug launch modes, live tuning and touchdown CSV logging
are not exposed by this manager; the shared PID controller remains reusable.

## Validation status

A previous headless PX4 v1.16.2 run, using the previous message snapshot,
confirmed DDS discovery, command ACKs, a 5 m
dock-relative takeoff, stable hover, visual search acquisition and PID descent
through central-tag acquisition. Runtime disturbance enable/disable was also
exercised. The inherited PID touchdown timeout expired before PX4 confirmed
landing; native LAND fallback then landed and disarmed correctly.
**Precision touchdown is not yet validated:** PX4 landing response and touchdown
thresholds still need tuning. The original temporary live-run log is
`/tmp/px4_dds_sitl4.log`, not a committed artifact.

That flight result is historical, not validation of RC1. The RC1 message
snapshot requires its own SITL/hardware flight validation.

Run the retained package tests in a sourced ROS environment:

```bash
cd /home/enri-ubuntu/drone_ws/src/drone_docking
python3 -m pytest -q /home/enri-ubuntu/drone_ws/src/drone_docking/flight_cycle_manager/test
```

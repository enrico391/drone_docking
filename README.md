# Drone Docking — PX4 and ROS 2

Autonomous AprilTag docking with PX4 Offboard control over native ROS 2 DDS.
The simulation combines Gazebo Harmonic, a custom X500 with a downward camera,
a docking station, visual localization, and a PID landing controller.

The default workflow uses ROS 2 Humble and PX4 **v1.18.0-rc1**, commit
`fca3df865af36124a28c9d607e850f111dbaaea9`. The bundled `px4_msgs` definitions
match that firmware. Firmware and ROS interfaces are separate builds and must
remain compatible; see [the message snapshot](px4_msgs/README.md).

This repository lives at `src/drone_docking/` inside the ROS workspace. It is a
collection of packages, so commands use package names directly.

## Packages

| Package | Purpose |
| --- | --- |
| `drone_description` | Docking world, X500 models, PX4 SITL launch, camera bridge, and disturbance tools. |
| `dock_apriltag_localizer` | Outer/central AprilTag selection, filtered dock pose, and camera launch. |
| `flight_cycle_manager` | DDS takeoff, hover, bounded dock search, PID landing, and tuning dashboard. |
| `px4_msgs` | Seven DDS interfaces matching the pinned PX4 firmware. |
| `px4_sysid` | Optional identification of the PX4 velocity response for PID tuning. |


## Current status
### Simulation
The simulation is functional and demonstrates the full docking cycle.

Here a video of the current state in simulation: 

https://github.com/user-attachments/assets/76c2a483-0812-44c0-843a-a2e19281c5e7

Next steps include:
- Add mission validation
- Add full cycle with takeoff, mission, search docking and docking
- Close real-to-simulation gap with camera and PX4 parameter tuning

### Hardware
The hardware setup is not yet complete. I'm working on system identication and PID tuning to get the drone to land on the dock reliably. The current precision is not sufficient for a reliable and repeatable landing.

#### Hardware setup
The hardware setup uses:
- X500 drone
- Pixhawk 6c
- Raspberry Pi 4 
- Downward-facing camera (pi camera v2 clone) 
- custom docking base with magnetic landing pad with charging contacts and UART communication contact to communicate between the drone and the docking station
- BMS for Li-on battery adapted to be used with Lipo battery (4s 5000mAh)

Here are some pictures of the current hardware setup:
<p align="center">
  <img src="https://github.com/user-attachments/assets/a987ca67-990d-4e20-9960-6f9bd79e9cbf" width="320" alt="Docking setup, first view" />
  <img src="https://github.com/user-attachments/assets/950823d8-ae23-458a-aacf-92ad67502005" width="320" alt="Docking setup, second view" />
</p>



#### Software setup on the Raspberry Pi and pixhawk:
- ROS 2 Lyrical : I'm using this ROS2 version because I need ubuntu 26.04 for the camera driver.
- PX4 v1.18.0-rc1 : Compatible with ROS2 Lyrical and the `px4_msgs` definitions in this repository.


## Next steps in the hardware setup:
- Finish the system identification and PID tuning to get a reliable landing on the dock.
- Start the building of the docking station with a magnetic landing pad and electronics


## Build

The commands below assume an existing workspace with ROS 2 Humble, Gazebo Harmonic, the
ROS-Gazebo bridge, `topic_tools`, and the v2.4 `micro_ros_agent` already available.
For a fresh workspace, also check the configure-time requirements in
[drone_description/CMakeLists.txt](drone_description/CMakeLists.txt) and the
package manifests before building the simulation package.

Set `DRONE_WS` to your workspace location; the examples use `~/drone_ws`.
Create the AprilTag Python environment from the workspace root:

```bash
export DRONE_WS="$HOME/drone_ws"
cd "$DRONE_WS"
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages apriltag_venv
touch apriltag_venv/COLCON_IGNORE
source apriltag_venv/bin/activate
python -m pip install -r \
  src/drone_docking/dock_apriltag_localizer/requirements.txt

python -m colcon build --symlink-install --packages-select \
  px4_msgs dock_apriltag_localizer flight_cycle_manager drone_description
source install/setup.bash
```

Using `python -m colcon` with the environment active gives Python entry points
access to the installed localizer dependencies. `COLCON_IGNORE` prevents colcon
from treating virtual-environment contents as workspace packages.

Build the matching PX4 checkout separately. The default location is
`<workspace>/.worktrees/PX4-Autopilot`; use a separate Python environment for
firmware dependencies:

```bash
cd "$DRONE_WS"
python3 -m venv .worktrees/px4-venv
touch .worktrees/px4-venv/COLCON_IGNORE
source .worktrees/px4-venv/bin/activate
cd .worktrees/PX4-Autopilot
git submodule update --init --recursive
python -m pip install -r Tools/setup/requirements.txt
make px4_sitl_default -j4
deactivate
```

These commands build the existing checkout; verify that it matches the firmware
pin above. The launch expects `build/px4_sitl_default/bin/px4` and runs it from
`build/px4_sitl_default/rootfs`.

## Run a docking cycle

In a new terminal, start the integrated simulation:

```bash
export DRONE_WS="$HOME/drone_ws"
cd "$DRONE_WS"
source /opt/ros/humble/setup.bash
source apriltag_venv/bin/activate
source install/setup.bash

ros2 launch drone_description px4_docking.launch.py \
  use_flight_cycle_manager:=true initial_point:=dock auto_start:=false
```

This starts the docking world, X500, camera bridge, DDS agent, PX4 SITL,
flight-cycle manager, localizer, and PID dashboard. PX4 attaches to the spawned
vehicle in Gazebo standalone mode. The default physics engine is DART.

In another terminal with the ROS workspace sourced, request takeoff:

```bash
ros2 service call /flight_cycle_manager/start_cycle std_srvs/srv/Trigger '{}'
ros2 topic echo /flight_cycle_manager/state
```

The manager warms up the setpoint stream, acquires Offboard, arms, and takes off
to the configured height, currently **5 m above the resting position**. Once the
state reaches `HOVER`, request docking from another sourced terminal:

```bash
ros2 service call /flight_cycle_manager/request_dock std_srvs/srv/Trigger '{}'
```

The controller searches for the dock, aligns using the outer tags, transitions
to the central tag, and commits to touchdown after the alignment, speed, and
height checks pass. Stable PX4 landed telemetry permits disarm, then the manager
reports `DOCKED`. Takeoff and docking are separate explicit requests.

The dashboard at **http://127.0.0.1:8765** shows pose tracking and touchdown
checks, and supports live PID tuning. Set `pid_tuner:=false` to disable it or
`pid_tuner_port:=8766` to choose another port.

## Launch options

| Argument | Default | Purpose |
| --- | --- | --- |
| `use_flight_cycle_manager` | `false` | Start the manager and its optional localizer. Enable it for the docking cycle above. |
| `initial_point` | `ground` | Choose `ground`, `dock`, or `custom`. |
| `use_gz_sim_gui` | `true` | Set `false` for a server-only run. |
| `spawn_robot` | `true` | Set `false` to omit the vehicle and PX4. |
| `px4_dir` | empty | Override the default PX4 source/build directory. |
| `use_dds_agent` | `true` | Set `false` when an agent already owns the selected port. |
| `dds_port` | `8888` | UDP port forwarded to the agent and PX4 client. |
| `px4_namespace` | empty | Prefix PX4 DDS topics; independent of the ROS `namespace`. |
| `start_localizer` | `true` | Include the localizer when the manager is enabled. |
| `use_disturbances` | `false` | Start the configurable disturbance injector. |
| `disturbance_enabled` | `false` | Apply disturbance loads immediately when the injector starts. |
| `auto_start` | `false` | Automatically request takeoff in simulation. Docking still needs a request. |

The ground spawn is `(2, 2, -0.032)`; the dock spawn is `(0, 0, 0.848)`.
For a custom starting point:

```bash
ros2 launch drone_description px4_docking.launch.py \
  initial_point:=custom spawn_x:=2.0 spawn_y:=1.0 spawn_z:=-0.032
```

The dock occupies the origin, so use a clear position for a ground start.
Keep DART as the physics default: the installed Bullet Featherstone backend
omits IMU linear acceleration needed by the PX4 estimator.

To add the manager and localizer to an already running simulation:

```bash
ros2 launch flight_cycle_manager px4_flight_cycle.launch.py
```

Run one manager and one localizer for each vehicle. Use `start_localizer:=false`
when another process already publishes `/dock/pose` and `/dock/mode`.

## Configuration and touchdown

| File | Settings |
| --- | --- |
| [px4_flight_cycle.yaml](flight_cycle_manager/config/px4_flight_cycle.yaml) | Takeoff, telemetry watchdogs, search limits, and DDS namespace. |
| [pid_landing.yaml](flight_cycle_manager/config/pid_landing.yaml) | PID gains, descent speeds, alignment tolerances, and touchdown gates. |
| [dock_tags.yaml](dock_apriltag_localizer/config/dock_tags.yaml) | Physical tag sizes and dock geometry. |
| [px4_sitl_docking.parm](drone_description/config/px4_sitl_docking.parm) | SITL thrust model, rotor allocation, DDS timebase, and failsafe settings. |
| [disturbances.yaml](drone_description/config/disturbances.yaml) | Simulated external forces and torques. |

`pid_enable_vel_command_x_y_yaw` selects final `TOUCHDOWN_APPROACH` behavior:

- `false` (current YAML default): command zero X/Y velocity and yaw rate, with
  only the configured Z descent speed.
- `true`: compute X/Y/yaw commands from the latched dock position and heading
  while descending.

This switch is loaded at node startup; restart the manager after changing it.
Earlier visual alignment and descent keep their normal control. The current
final descent setting is `pid_touchdown_speed_m_s: 0.25`.

The PX4 parameter profile is applied on every SITL boot using `PX4_PARAM_*`
overrides, after saved parameters are imported. Edits in a ground station to
parameters listed in that profile are replaced at the next launch. Restart
SITL after profile changes. These are simulation settings; a hardware setup
needs its own configuration.

For the pinned firmware, keep the final descent speed consistent with the
land-detector threshold: `pid_touchdown_speed_m_s >= 1.1 * LNDMC_Z_VEL_MAX`.
The paired SITL profile sets `LNDMC_Z_VEL_MAX=0.10`. Landing confirmation also
requires the remaining PX4 ground-contact checks; see the
[touchdown diagnosis and validation](drone_description/PX4_TOUCHDOWN_VALIDATION.md).

## Camera and localization

The simulated downward camera publishes 640x480 images at a target rate of
30 Hz, with `camera_optical_frame` in the image and calibration headers:

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 topic echo /dock/mode
ros2 topic echo /dock/pose --once
ros2 topic echo /dock/timing
```

Outer tags provide coarse alignment; the central tag provides final alignment.
`/dock/pose` is the filtered dock pose in the camera optical frame, preserving
the source image timestamp. `/dock/timing` reports image age and processing time.

For a Raspberry Pi camera, the standalone manager launch can include the camera
driver and localizer with `use_real_camera:=true`. That selects the camera and
defaults to wall-clock time; it does not replace the SITL flight profile.
See [camera setup and calibration](dock_apriltag_localizer/README.md) before
configuring a hardware run.

## Disturbances and system identification

Start the simulation with `use_disturbances:=true` to include the injector.
It starts without applied loads unless `disturbance_enabled:=true` is supplied.
Enable or disable loads after takeoff:

```bash
ros2 param set /disturbance_injector enabled true
ros2 param set /disturbance_injector enabled false
```

For velocity-response identification and PID plant fitting, see
[px4_sysid](px4_sysid/README.md). Run identification separately from the
flight-cycle manager because both publish Offboard setpoints.

## Monitoring and recovery

The main diagnostics are `/flight_cycle_manager/state`,
`/flight_cycle_manager/reason`, and `/flight_cycle_manager/pid_debug`.
PX4 telemetry includes `/fmu/out/vehicle_status_v4`,
`/fmu/out/vehicle_local_position_v1`, and
`/fmu/out/vehicle_land_detected`; `px4_namespace` prefixes these topics.

A pilot mode change cancels the active cycle. Telemetry loss stops Offboard
streaming and leaves the configured PX4 failsafe in charge. Search or landing
failure requests native LAND, which can land away from the dock. Disarm requires
PX4 landed confirmation. See the [manager documentation](flight_cycle_manager/README.md)
for the complete lifecycle and tuning controls.

If the Gazebo GUI freezes while physics and camera topics continue updating,
rebuild `drone_description` and source the workspace again. Its local
`MonotonicSceneBroadcaster` avoids GUI update stalls during backward WSL/NTP
clock adjustments. The plugin and license are documented in
[the vendored broadcaster notes](drone_description/vendor/gz_sim8_8_15_0/README.md).




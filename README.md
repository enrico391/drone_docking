# Drone Docking ROS 2 Packages

This directory groups project-owned ROS 2 packages separately from the
upstream ArduPilot, Gazebo, and ROS-Gazebo repositories in the workspace.
It is intentionally not a ROS package itself: `colcon` recursively discovers
the packages below it.

## Packages

- `drone_description`: Gazebo models, worlds, resource-path hooks, and the
  simulation launch file.
- `dock_apriltag_localizer`: Existing camera-to-dock localization package.
- `drone_mission_manager`: Typed, non-blocking missions with a shared staged
  AprilTag precision-landing controller.
- `drone_bringup`: Top-level launch package for the project-owned stack.

The grouping directory is deliberate. Upstream packages remain directly under
`src/`, while every project-owned package lives under `src/drone_docking/`.
Package names do not include the grouping directory, so normal ROS 2 commands
continue to use names such as `drone_description` and `drone_bringup`.

## Model separation

- `models/x500` is the imported airframe, rotor, collision, and inertial model.
- `models/x500_with_ardupilot` is the project adapter that connects the X500
  joints and IMU to ArduPilot's Gazebo JSON interface.
- `models/mono_cam` is the downward-facing monochrome camera sensor.
- `models/x500_mono_cam_down` composes the airframe with that camera. It is not
  a second airframe copy: the active ArduPilot adapter includes this composition.
- `models/drone_docking_base_structure` is the Fusion 360 docking-base mesh
  with separate AprilTag visuals 0-4. Its world entity remains named
  `dock_station` and fixed at the docking position.

## Current milestone

Phase 1 adds the static docking base at `(10, 0, 0)`. The base has four 300 mm
outer tags at `(x, y, z) = (+/-0.52, +/-0.52, 1.001)` and a 120 mm central tag
in the recess at `z=0.881`. Phase 2 adds a separate `x500_with_ardupilot` adapter around
the imported `x500` model and reuses the known-working ArduPilot SITL + AP_DDS
launch infrastructure. Phase 3 activates the downward camera composition and
bridges its image and calibration data into ROS 2.

Current Phase 2 results:

- PASS: model and world SDF validation;
- PASS: X500 spawn with its landing feet on `z=0`;
- PASS: disarmed rotor speed and body yaw rate remain effectively zero;
- PASS: AP_DDS discovery, GUIDED mode switch, arm, and takeoff request;
- PASS: selectable ROS-command and ArduPilot `LANDING_TARGET` landing backends;
- PASS: stable 1 m takeoff, hover, LAND-mode descent, and automatic disarm.

The original powered-flight failure was an IMU coordinate-frame error. Gazebo's
sensor used forward-left-up axes while ArduPilot expects forward-right-down.
This made stationary acceleration appear as `+9.8 m/s^2`, saturated the altitude
controller, and reversed attitude feedback. The IMU now has a 180-degree roll,
and stationary `linear_acceleration.z` is the expected `-9.8 m/s^2`.

The downward camera is mounted 0.14 m below `base_link` on the vehicle centerline
and looks along the vehicle's negative Z axis. Gazebo publishes at 640x480 and 30 Hz;
the bridge exposes `/camera/image_raw` and `/camera/camera_info` with
`camera_optical_frame` in the message headers.

## Implementation passages

1. **World and dock:** validate the ground plane, fixed dock pose, and model
   resource paths without spawning a vehicle.
2. **Airframe spawn:** load `x500_with_ardupilot`, keep it disarmed, and verify
   ground contact, zero rotor velocity, zero body motion, IMU data, and TF.
3. **Base flight:** verify AP_DDS mode, arm, takeoff, hover, and landing. This
   passage now passes with the camera-free X500.
4. **Downward camera:** PASS: use the camera composition and bridge the image and
   camera-info topics. Validate image orientation and update rate after changes
   to the airframe or sensor model.
5. **Dock perception:** add dock tag visuals and validate detector output and
   camera-to-dock transforms independently of flight control.
6. **Mission control:** enable the non-blocking mission manager only after the
   preceding simulation and perception gates pass.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select drone_description dock_apriltag_localizer \
  drone_mission_manager drone_bringup
source install/setup.bash
```

## Run

The direct simulation launch and the top-level bringup launch are equivalent
at this phase:

```bash
ros2 launch drone_description gazebo_docking.launch.py
```

```bash
ros2 launch drone_bringup docking_sim.launch.py
```

Set `use_gz_sim_gui:=false` for a server-only run. Set `spawn_robot:=false`
to inspect only the docking world and station without starting SITL. Select
`initial_point:=ground` (default), `initial_point:=dock`, or use explicit
coordinates with `initial_point:=custom`:

```bash
ros2 launch drone_bringup docking_sim.launch.py \
  initial_point:=custom spawn_x:=2.0 spawn_y:=1.0 spawn_z:=-0.032
```

The launch defaults to `gz-physics-bullet-featherstone-plugin`. The exact dock
and triangular-base STLs are visual-only; stable primitive collision proxies
are used at their mating surfaces. Bullet Featherstone remains the tested
backend for the complete articulated X500 model.

SITL state is stored in the instance-specific directory (for instance `0/`)
instead of reusing the workspace-root EEPROM. This keeps X500 parameters
separate from existing Iris runs.

## Downward camera

The active vehicle publishes the simulated camera through the ROS-Gazebo
bridge. After starting the simulation, verify the stream from another sourced
terminal:

```bash
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
```

The image is 640x480, the target update rate is 30 Hz, and both messages use
`camera_optical_frame`. Use `/camera/image_raw` and `/camera/camera_info` as the inputs to
`dock_apriltag_localizer`.

## Arm, take off, and land

Wait for `AP: ArduPilot Ready` and `AP: DDS: Initialization passed`, then use a
second terminal with the workspace sourced. Check readiness before changing
the flight mode:

```bash
ros2 service call /ap/v1/prearm_check std_srvs/srv/Trigger "{}"
ros2 service call /ap/v1/mode_switch ardupilot_msgs/srv/ModeSwitch \
  "{mode: 4}"
ros2 service call /ap/v1/arm_motors ardupilot_msgs/srv/ArmMotors \
  "{arm: true}"
ros2 service call /ap/v1/experimental/takeoff ardupilot_msgs/srv/Takeoff \
  "{alt: 1.0}"
```

Mode `4` is GUIDED. To land, select mode `9` (LAND); ArduPilot disarms after
touchdown:

```bash
ros2 service call /ap/v1/mode_switch ardupilot_msgs/srv/ModeSwitch \
  "{mode: 9}"
ros2 topic echo /ap/v1/status
```

Only use the explicit disarm request after touchdown or for a stopped test:

```bash
ros2 service call /ap/v1/arm_motors ardupilot_msgs/srv/ArmMotors \
  "{arm: false}"
```

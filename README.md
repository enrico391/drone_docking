# Drone Docking ROS 2 Packages

This directory groups project-owned ROS 2 packages separately from the
upstream ArduPilot, Gazebo, and ROS-Gazebo repositories in the workspace.
It is intentionally not a ROS package itself: `colcon` recursively discovers
the packages below it.

## Packages

- `drone_description`: Gazebo models, worlds, resource-path hooks, and the
  simulation launch file.
- `dock_apriltag_localizer`: Existing camera-to-dock localization package.
- `flight_cycle_manager`: Native PX4 DDS takeoff-to-dock cycles, bounded dock
  search and PID landing. This branch no longer ships the ArduPilot manager.
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

Phase 1 adds the static docking base at `(0, 0, 0)`. The base has four 300 mm
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
6. **Mission control:** enable `flight_cycle_manager` only after the preceding
   simulation and perception gates pass. On this branch it controls PX4 through
   native ROS 2 DDS topics, not the historical ArduPilot mission workflow.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages apriltag_venv
touch apriltag_venv/COLCON_IGNORE
source apriltag_venv/bin/activate
python -m pip install -r \
  src/drone_docking/dock_apriltag_localizer/requirements.txt

# Required when rebuilding the upstream ardupilot_sitl colcon package.
export PATH="$PWD/.worktrees/Micro-XRCE-DDS-Gen/scripts:$PATH"

python -m colcon build --symlink-install \
  --packages-up-to drone_bringup flight_cycle_manager
source install/setup.bash
```

Activate `apriltag_venv` before building and running the localizer. Using
`python -m colcon` ensures its installed entry points use the virtual
environment interpreter.

## Run

The direct simulation launch and the top-level bringup launch are equivalent
at this phase:

```bash
ros2 launch drone_description gazebo_docking.launch.py
```

```bash
ros2 launch drone_bringup docking_sim.launch.py use_mavproxy:=true
```

`use_mavproxy:=true` keeps SITL's TCP serial endpoint connected for a
standalone ArduPilot simulation. Leave it false when the direct pymavlink
dock-pose bridge owns `tcp:127.0.0.1:5760`. The PX4-only flight-cycle manager
does not connect to that endpoint.

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

The flight-cycle manager on this branch is PX4-only. Start its integrated
simulation instead of attaching it to the ArduPilot simulation above:

```bash
ros2 launch drone_description px4_docking.launch.py \
  use_flight_cycle_manager:=true initial_point:=dock
ros2 service call /flight_cycle_manager/start_cycle std_srvs/srv/Trigger "{}"
```

## Drone appears frozen despite 100% real-time factor

The docking world uses the package-local `MonotonicSceneBroadcaster`. Gazebo
8.15's original broadcaster schedules GUI state updates with the adjustable
wall clock. Backwards WSL/NTP clock steps can suppress these updates for seconds
even while physics, motor commands and the camera run normally. This makes the
drone jump between positions and its propellers appear stopped.

The local broadcaster preserves upstream behavior except that its publication
timer uses a monotonic clock. It does not change PX4, flight dynamics, camera
rendering or the system clock, and does not overwrite Gazebo's installed library.
The vendored source and license are documented under
`drone_description/vendor/gz_sim8_8_15_0`.

After rebuilding `drone_description`, source the workspace again before launching
so Gazebo can find the local plugin:

```bash
source /opt/ros/humble/setup.bash
cd /home/enri-ubuntu/drone_ws
colcon build --symlink-install --packages-select drone_description
source /home/enri-ubuntu/drone_ws/install/setup.bash
ros2 launch drone_description px4_docking.launch.py \
  initial_point:=dock use_flight_cycle_manager:=true auto_start:=false
```

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

To exercise the docking controller with delayed camera feedback, start the
simulation with:

```bash
ros2 launch drone_bringup docking_sim.launch.py camera_latency_s:=0.11
```

`camera_latency_s` adds a fixed arrival-to-publication delay in seconds before
AprilTag detection. The default `0.0` keeps the direct camera bridge. With a
positive delay, Gazebo images first appear on `/camera/image_raw_undelayed`;
the delayed stream remains `/camera/image_raw`, so localizer commands stay
the same. Camera calibration on `/camera/camera_info` is unchanged. The relay
uses simulation time, pauses with Gazebo, clears pending frames on clock reset,
and preserves image capture timestamps. Its buffer holds at most 64 frames,
dropping the oldest if full. Very large delays can exceed this capacity.

The 110 ms example is an initial sensitivity test based on the real flight's
median capture-to-controller image age. It adds to existing bridge, detection,
and scheduling latency; it does not establish a matched total latency. Measure
the resulting age using a consistent clock before treating it as calibrated.
Pose-filter lag remains separate. The argument is also accepted by
`drone_description gazebo_docking.launch.py` and `x500.launch.py`.

Start the localizer in a second terminal after activating its environment and
sourcing the workspace:

```bash
source /opt/ros/humble/setup.bash
source apriltag_venv/bin/activate
source install/setup.bash
ros2 launch dock_apriltag_localizer dock_localizer.launch.py
```

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

## PX4 SITL comparison stack

The native DDS flight-cycle path is now available alongside the simulation:

```bash
ros2 launch drone_description px4_docking.launch.py \
  use_flight_cycle_manager:=true use_disturbances:=true initial_point:=dock
```

The DDS agent starts by default. Disturbances start unloaded; enable them with
`ros2 param set /disturbance_injector enabled true` after takeoff. The PX4
manager is a DDS-only executable; the ArduPilot manager and legacy debug
launches have been removed from this branch. A pose-only live PID dashboard
starts at **http://127.0.0.1:8765** when the manager is launched; set
`pid_tuner_port:=8766` to change the port or `pid_tuner:=false` to disable it.
It plots dock-relative pose tracking errors and supports live base/near/far
PID gain updates during docking. Build the included
`px4_msgs` subset first. It is pinned to **v1.18.0-rc1** (firmware commit
`fca3df865af36124a28c9d607e850f111dbaaea9`); the running firmware must match,
or use PX4's message translation node. The default SITL checkout is now pinned
to the same RC1 commit. Rebuilding the ROS packages alone does not rebuild PX4;
use the firmware build commands below after changing its checkout.
For build commands, service calls, safety behavior and current PX4 mission
limitations, see the PX4 section of
`/home/enri-ubuntu/drone_ws/src/drone_docking/flight_cycle_manager/README.md`.

The same world, airframe, camera and docking platform can be driven by PX4
instead of ArduPilot. The PX4 launch uses DART physics; the ArduPilot launch
retains its existing backend, so comparisons do not currently use identical
physics engines.

### Layout

- `/home/enri-ubuntu/drone_ws/.worktrees/PX4-Autopilot` is pinned to
  `v1.18.0-rc1`, commit `fca3df865af36124a28c9d607e850f111dbaaea9`, with
  recursively pinned submodules and the local docking airframe preserved.
- `/home/enri-ubuntu/drone_ws/.worktrees/px4-backup-v1.16.2-20261018-133931`
  preserves the previous build, rootfs (including saved parameters and flight
  logs), firmware/submodule revisions and docking-airframe changes. RC1 uses
  a clean build and rootfs instead of importing v1.16.2 saved parameters.
- `.worktrees/px4-venv` is a dedicated Python virtual environment for the PX4
  build. It is separate from `apriltag_venv` and from the system interpreter so
  PX4's newer `numpy`/`pandas` cannot shadow the ROS 2 or ArduPilot packages.
- `models/x500_px4_base` is the PX4 vehicle body: the same geometry, mass,
  inertia and meshes as `x500_triangular_base` (`model://x500_triangular_base/`
  meshes are shared so both stacks simulate one airframe), plus the sensors PX4's
  `gz_bridge` subscribes to.
- `models/x500_px4_mono_cam_down` composes the body with the existing
  `mono_cam` (same 0.19 m downward offset) and adds the joint-state and pose
  publishers the ROS-Gazebo bridge consumes, mirroring `x500_with_ardupilot`.

### Build

```bash
cd /home/enri-ubuntu/drone_ws/.worktrees/PX4-Autopilot
source /home/enri-ubuntu/drone_ws/.worktrees/px4-venv/bin/activate
git submodule update --init --recursive
python -m pip install -r /home/enri-ubuntu/drone_ws/.worktrees/PX4-Autopilot/Tools/setup/requirements.txt
make px4_sitl_default -j4
```

The Gazebo (gz/Harmonic) bridge, the `MulticopterMotorModel` support and the PX4
Gazebo plugins are part of that build. `gz-harmonic` and `libgz-*-dev` come from
the existing system setup; no additional `apt` packages are required beyond the
PX4 Python requirements already installed into `px4-venv`.

### Run

```bash
source /opt/ros/humble/setup.bash
source /home/enri-ubuntu/drone_ws/install/setup.bash
ros2 launch drone_description px4_docking.launch.py
```

The launch starts the docking world and GUI, spawns `x500_px4_mono_cam_down`
under the instance name `x500`, starts the ROS-Gazebo bridge (clock, TF, joint
states, camera, disturbance wrenches) and finally launches PX4 in Gazebo
*standalone* mode, where PX4 attaches to the already running model instead of
spawning its own world. Because Gazebo and the model are started here, the
custom `docking_world.sdf` and the custom docking platform are used unchanged.

Useful arguments: `use_gz_sim_gui:=false` (server only), `spawn_robot:=false`
(world only), `initial_point:=dock` (start in the dock recess),
`initial_point:=custom spawn_x:=2.0 spawn_y:=2.0`, `px4_dir:=<path>` to point at
a different PX4 checkout.

`initial_point:=ground` defaults to `(2, 2, -0.032)`. The dock's lower body is a
solid 1.6 m box up to `z=0.88` centred on the origin, so spawning at `(0, 0)`
would place the vehicle inside the dock collision.

### PX4-specific model requirements

Two details in `x500_px4_base` differ from the ArduPilot airframe on purpose:

1. **Sensor noise is mandatory.** PX4's sensor data validator flags a channel as
   `STALE` when it returns bit-identical values (within `1e-6`) more than a few
   times in a row. A noiseless simulated IMU/magnetometer therefore fails
   `DataValidatorGroup::get_best()`, `sensor_combined` is never published and
   EKF2 stays at zero updates (`ekf2: attitude: 0, local position: 0`), which
   surfaces as `Preflight Fail: ekf2 missing data`. The IMU and magnetometer
   noise figures mirror PX4's own `x500_base`. The ArduPilot airframe is left
   unnoised because it consumes the sensors over the ArduPilot FDM interface.
2. **IMU orientation.** PX4's `gz_bridge` expects Gazebo's native
   forward-left-up axes and applies the FLU->FRD rotation itself, so the IMU must
   *not* carry the 180 degree roll used by the ArduPilot variant.

PX4 also needs `magnetometer_sensor` and `navsat_sensor` on `base_link`, which
the ArduPilot airframe does not define; both are present in `x500_px4_base`.

### Required PX4 parameters

The PX4 launch automatically applies
`/home/enri-ubuntu/drone_ws/src/drone_docking/drone_description/config/px4_sitl_docking.parm`
on every boot, using PX4's `PX4_PARAM_*` environment overrides. PX4 reads these
after importing saved BSON parameters and before starting the simulator and
controllers. This prevents fresh or previously configured rootfs directories
from silently using stock-x500 thrust scaling and allocation geometry.

The profile also sets `UXRCE_DDS_SYNCT=0`, keeping DDS timestamps in PX4
boot/simulation time. The ROS docking manager and `px4_sysid` stamp commands
from local-position telemetry. This avoids WSL/NTP wall-clock corrections
entering the identification timebase. Restart PX4 after changing this setting;
ROS clients using this profile must use FC-domain timestamps for commands.

```text
MPC_THR_HOVER     0.38      # docking airframe, normalised thrust domain
THR_MDL_FAC       0.74      # quadratic motor thrust curve
NAV_DLL_ACT        0        # do not require a GCS datalink to arm
COM_DISARM_PRFLT   0        # no auto-disarm before a scripted takeoff
CBRK_SUPPLY_CHK    894281   # skip the avionics power check (no power module)
```

The profile also matches `CA_ROTOR{0..3}_PX/PY` to the actual rotor positions
(`+/-0.174 m`, converted from Gazebo FLU to PX4 FRD), and `_KM` to the
`momentConstant` (`+/-0.016`, signed by spin direction). QGC changes to parameters
listed in the profile are replaced on the next launch; edit the profile for
persistent changes. These are SITL settings, not a real-vehicle configuration.

Do not use PX4's `param load` command with this text file: native parameter
load/import expects BSON. To update an already running SITL, the optional
MAVLink helper remains available:

```bash
python3 /home/enri-ubuntu/drone_ws/src/drone_docking/drone_description/config/px4_param_tool.py \
  /home/enri-ubuntu/drone_ws/src/drone_docking/drone_description/config/px4_sitl_docking.parm --apply
```

### Verification status

The RC1 firmware build and a disarmed startup/DDS smoke check are validated,
including valid local position and an accepted prearm-check command ACK.
Details, the v1.16.2 backup, and known shutdown/test issues are recorded in
`/home/enri-ubuntu/drone_ws/src/drone_docking/drone_description/PX4_RC1_VALIDATION.md`.
RC1 takeoff and precision touchdown have not yet been validated.

Historical simulation verification, before the RC1 ROS message update
(not an RC1 flight validation):

- PX4 v1.16.2 SITL built with the Gazebo Harmonic bridge;
- `docking_world` loads with the custom dock and the custom `x500` model spawns
  `x500_px4_mono_cam_down`, standing level with its feet on `z=0`;
- PX4's `gz_bridge` attaches (`world: docking_world, model: x500`) and receives
  gyro, accel, magnetometer, barometer, GPS and the camera streams;
- EKF2 reports `attitude: 1, local position: 1, global position: 1` with the
  expected GPS origin (`45.3821944, 12.0005833`), matching the world's
  `spherical_coordinates`;
- the vehicle arms and the ESC outputs drive the four
  `MulticopterMotorModel` motors (`/x500/command/motor_speed`, values 150-1000).

### Takeoff instability: physics backend and IMU acceleration

The installed Bullet Featherstone backend (gz-physics 7.8.0, Gazebo Sim
8.15.0) does not supply the translational acceleration needed by Gazebo's IMU.
A free-falling body reports `linear_acceleration.z = +9.8 m/s^2` in Gazebo FLU
instead of approximately zero. The same test with DART reports zero correctly.
PX4's FLU-to-FRD conversion is correct and must not be changed to compensate.

The failure is visible even with the corrected thrust profile in flight log
`/home/enri-ubuntu/drone_ws/.worktrees/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-10-17/20_19_28.ulg`:

- at 19 s, ground-truth vertical speed is -1.95 m/s, but EKF speed is -0.09 m/s;
- at 20 s, ground-truth vertical speed is -6.15 m/s, but EKF speed is -0.53 m/s;
- the body accelerometer still reads approximately -9.79 m/s^2 in PX4 FRD.

The estimator therefore misses the climb acceleration and the altitude
controller continues increasing thrust. Later position corrections and attitude
errors produce runaway flight. Thrust matching or PID tuning alone cannot fix
missing inertial measurements.

`px4_docking.launch.py` now defaults to `gz-physics-dartsim-plugin`. The docking
world already uses primitive box collision proxies and the vehicle uses point
contacts, so it does not require Bullet for its visual CAD meshes. Keep the DART
default unless a replacement backend has been checked for correct IMU
acceleration. The ArduPilot launch and airframe are unchanged.

Validation after the fix: a headless 2.5 m takeoff, 25 s flight followed by LAND
completed with automatic disarm. During the final 13 s before LAND, simulated
ground-truth height stayed within 2.458-2.517 m and horizontal excursion was
0.084 m; MAVLink roll/pitch stayed below 0.28 degrees. This verifies basic
flight, not autonomous docking or real-hardware tuning. Details and known
shutdown issues are in
`/home/enri-ubuntu/drone_ws/src/drone_docking/docs/px4_takeoff_validation.md`.

# Flight Cycle Manager — PX4 DDS

This branch contains only the native PX4 ROS 2/uXRCE-DDS flight-cycle path.
It does not depend on pymavlink or include the old ArduPilot manager, mission
tools, precision-landing backend, debug launches or CSV loggers. A lightweight
pose-only PID tuning dashboard is available for docking.

## Package layout

- `launch/px4_flight_cycle.launch.py`: the only manager launch file.
- `config/px4_flight_cycle.yaml`: DDS, takeoff, search and safety settings.
- `config/pid_landing.yaml`: transport-independent PID landing settings.
- `px4_manager.py`: ROS node and non-blocking flight lifecycle.
- `px4_control.py`: DDS topic naming, frame conversion and PID configuration.
- `pid_tuner.py`: local HTTP dashboard, pose tracking and live ROS PID updates.
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

`start_cycle` requires a landed, disarmed vehicle. After setpoint warmup it
requests Offboard only if PX4 is not already in that mode, then arms and takes off.
`request_dock` also works when the vehicle is already airborne in another mode
and the manager is idle (including after `PILOT_OVERRIDE`). It captures the
current position/heading, streams a position hold for the configured warmup,
requests Offboard and waits for both the command ACK and mode confirmation
before starting search. It does not arm or command another takeoff. If already
in Offboard, an accepted docking request starts search directly. Fresh valid
telemetry, an armed airborne vehicle, valid control heading and no PX4 failsafe
are required for docking.

An operator mode change cancels execution, including during warmup or pending
Offboard entry. After Offboard is acquired, any departure from it latches
`PILOT_OVERRIDE`: setpoint streaming stops, pending commands are abandoned and
the manager does not request Offboard again automatically. Switching back to
Offboard alone does not resume execution; a new explicit trigger is required.
The ROS node remains running so its state and services stay available.

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

The default firmware checkout at
`/home/enri-ubuntu/drone_ws/.worktrees/PX4-Autopilot` is pinned to the same RC1
commit. Firmware and ROS interfaces are separate builds; rebuilding these ROS
packages does not rebuild PX4. Pass `px4_dir` only when selecting another
compatible firmware checkout.

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
Its tag geometry is selected with `localizer_config`; `config_file` is only for
the flight manager. Both nodes use simulation time by default; set
`use_sim_time:=false` for a hardware-time setup (with a separate hardware
manager profile). The localizer excludes user-site Python packages so unrelated
`~/.local` NumPy installations cannot shadow its ROS/OpenCV dependencies.

### Raspberry Pi camera driver

To select the existing `dock_localizer_with_camera.launch.py` instead of the
topic-only localizer launch, use:

```bash
ros2 launch flight_cycle_manager px4_flight_cycle.launch.py \
  use_real_camera:=true auto_start:=false
```

This starts `camera_ros/camera_node` and one dock localizer, using the existing
Pi libcamera environment workaround. With this standalone launch,
`use_real_camera:=true` defaults `use_sim_time` to `false` for the manager,
camera and localizer. The normal topic-only/SITL default is unchanged. An
explicit `use_sim_time` override takes precedence. `start_localizer:=false`
disables both the camera and localizer.

Camera arguments are `camera` (default `0`), `camera_width` (`800`),
`camera_height` (`600`), `camera_format` (`RGB888`) and `camera_info_url`
(the installed OV5647 800x600 calibration). Use calibration matching the real
camera and selected resolution; `localizer_config` selects the tag geometry.

**This selects the camera, not a flight-safe hardware profile.** The default
manager YAML remains SITL-only (`simulation_mode: true`, 5 m takeoff). Supply a
separately reviewed hardware configuration before a flight test. The DDS agent
and PX4 client must already be configured/running; this standalone launch does
not start them. Start with propellers removed and do not call `start_cycle`
during a camera/telemetry bench test.

When using `px4_docking.launch.py`, `start_localizer:=true` only takes effect
with `use_flight_cycle_manager:=true`; the top-level manager remains disabled
by default. To add the manager and localizer to an already running simulation,
use the standalone `px4_flight_cycle.launch.py` command above, not a second
Gazebo launch.
The top-level SITL launch forwards the real-camera arguments too, but retains
`use_sim_time:=true` by default and still launches simulation; use the
standalone flight-cycle launch on the Pi, not the Gazebo launch.
Start near/above the dock (`initial_point:=dock`): the bounded search is not
autonomous navigation from the free-ground spawn point to the docking station.

In a second sourced terminal:

```bash
ros2 service call /flight_cycle_manager/start_cycle std_srvs/srv/Trigger '{}'
# After this takeoff, wait for HOVER before requesting docking.
# For a manually flown, already airborne vehicle, request_dock can be called directly.
ros2 service call /flight_cycle_manager/request_dock std_srvs/srv/Trigger '{}'

ros2 topic echo /flight_cycle_manager/state --qos-durability transient_local
ros2 topic echo /flight_cycle_manager/reason --qos-durability transient_local
ros2 topic echo /flight_cycle_manager/pid_debug
```

`auto_start:=true` queues one simulation takeoff only; it does not request
docking automatically. The manager launch also accepts `config_file`,
`pid_config`, `localizer_config`, `use_real_camera`, `use_sim_time`, `image_topic`,
`camera_info_topic` and `localizer_python`.
Controller settings are loaded at startup; PID gains can also be tuned live below.

### Flight actions from the dashboard

The dashboard includes a separate **Flight actions** panel with:

- **Start cycle — arm & take off**, calling the manager's existing
  `~/start_cycle` ROS service.
- **Request dock — search & land**, calling the existing `~/request_dock`
  ROS service.

Both buttons require an additional confirmation in the browser. They reuse the
manager's own safety checks unchanged; eligibility and the exact rejection
reason are shown beside each button. A request may remain **pending** while one
action finishes, and no request is ever retried automatically. For a timeout or
transport failure the status is explicitly marked **OUTCOME UNKNOWN** because
the service may have executed despite the missing response.

**Accepted means queued or started, not completed.** Use the manager's state,
the vehicle and the retained service reply to assess the actual outcome. These
controls initiate motion and are not emergency-stop buttons; use the configured
pilot override to intervene. Keep the flight area clear and RC override
available.

### Real-time docking PID dashboard

The manager launch (including the simulation launch above) starts the tuner at
**http://127.0.0.1:8765** by default. Change the port with `pid_tuner_port:=8766`,
or disable it with `pid_tuner:=false`:

```bash
ros2 launch flight_cycle_manager px4_flight_cycle.launch.py pid_tuner_port:=8765
```

If the manager is already running without the dashboard, start it separately:

```bash
ros2 run flight_cycle_manager pid_tuner --ros-args -p port:=8765
# For a manager in a ROS namespace:
ros2 run flight_cycle_manager pid_tuner --ros-args -r __ns:=/drone1 -p port:=8766
```

The dashboard shows measured dock-relative X (forward),
Y (left), Z (distance to the dock), yaw and the active PID tracking errors over
the last 60 seconds, plus touchdown-transition diagnostics. There are no camera,
command or estimated-surface panels. Position errors are in metres and yaw is displayed in degrees. Z error
is distance setpoint minus measured distance, **not absolute height**. During
alignment Z control may be inactive; unavailable errors appear as gaps rather
than stale values or zero. Active control errors can use a frozen target during
touchdown; those are kept separate from measured camera pose. Stale/lost camera
pose is hidden, and loss of manager telemetry marks the dashboard disconnected.

All four axes (**X**, **Y**, **Z** and **yaw**) have visible gain panels, arranged
side by side on wide screens and stacked on narrow screens. Each panel has its
own **Base**, **Near** or **Far** gain-bank selector, Kp/Ki/Kd fields, effective
gains and **Apply** button. Edits and update status are independent per axis.
Updates are atomic per axis and immediately reconfigure the active
docking backend without restarting its state machine; the existing PID gain
ramping and anti-windup remain in effect. Near/far banks interpolate by dock
distance. `-1` in either scheduled bank inherits the base gain; changing base
alone does not override explicit near/far values. The dashboard also reports
the effective gains, which may differ during interpolation/ramping.

Gains can be prepared before docking; tuning does not start takeoff or docking.
Gain edits do not change flight limits, watchdogs or touchdown/disarm requirements.
Invalid/nonfinite gains are rejected before any controller change. Settings outside
the live gain and transition allowlists require a restart.

#### Diagnosing DESCEND_CENTRAL → TOUCHDOWN_APPROACH

The transition panel displays **`fine_alignment_stable`**, **`height_ready`** and
every blocking gate with its current measurement, configured limit, parameter name
and PASS/BLOCKED status. It includes:

- Usable central vision and availability of the camera/frozen control reference.
- Absolute X and Y errors (each checked separately), wrapped yaw error, and
  **measured** horizontal speed `hypot(vx, vy)` in the local ENU frame.
- Alignment and low-speed dwell elapsed/required seconds. **Either a pose or
  horizontal-speed excursion resets both timers**, as does central-tag loss.
  A retained **last dwell interruption** shows the offending value and limit even
  if the excursion happened between browser polls.
- Absolute camera-to-dock distance error around the approach height, and measured
  absolute vertical speed. Both must pass, independently of fine alignment;
  neither resets its dwell timers.
- Debug holds, which deliberately prevent commitment. Diagnostics do not run
  dwell checks behind a debug hold or reuse stale camera measurements.

The panel's **Live transition parameters** editor applies just the edited fields
as one atomic ROS update, without restarting the backend or changing its state.
It exposes `pid_touchdown_commit_height_m`, `pid_approach_height_tolerance_m`,
`pid_touchdown_speed_m_s`, `pid_touchdown_horizontal_tolerance_m`,
`pid_touchdown_yaw_tolerance_rad`, `pid_alignment_stable_time_s`,
`pid_touchdown_horizontal_speed_tolerance_m_s`,
`pid_touchdown_horizontal_speed_stable_time_s`,
`pid_central_horizontal_tolerance_m` and `pid_central_yaw_tolerance_rad`.
Yaw editor values are **radians**; gate diagnostics show radians and degrees.
Unsaved edits survive telemetry polling and independent gain edits; **Reload live
values** discards them. Disconnect disables updates and removes stale gate data.

**Hardware caution:** these edits change landing acceptance criteria. In particular,
`pid_touchdown_speed_m_s` is both the pre-commit vertical-speed gate and the
post-commit descent speed; alignment dwell also affects earlier alignment stages,
and central tolerances affect descent pausing. Existing positive-value and
cross-parameter constraints are enforced (fine tolerances cannot exceed central
tolerances, and the arrival band must stay below central transition distance).
Threshold edits restart both dwell timers; height edits also rebase the Z target.
Transition edits are rejected after touchdown commitment, while gain edits remain
available. No default thresholds, vision-loss recovery, watchdogs or PX4
touchdown/disarm confirmation requirements are bypassed.

**Download YAML** exports configured gains, both scheduled banks and supporting
PID and transition settings (not the transient interpolated gains). Changes are otherwise
in-memory only. Load the downloaded file via `pid_config:=/absolute/path/pid_tuned.yaml`
on the next launch; unspecified landing settings retain package defaults.

The server binds to loopback by default and has no authentication. For tuning
on a remote drone, forward the port rather than exposing it to an untrusted LAN:

```bash
ssh -L 8765:127.0.0.1:8765 user@drone
```

`pid_tuner_host` can override the launch bind address on a trusted network.
Use a distinct port per manager when launching multiple namespaces.

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
- Transport timestamps follow PX4's DDS telemetry clock (boot/simulation time
  with `UXRCE_DDS_SYNCT=0` in the docking SITL profile). Pose-age
  checks use Gazebo time; command/telemetry watchdogs use monotonic time and
  continue running while Gazebo is paused.
- Telemetry loss stops Offboard streaming, leaving PX4's configured
  Offboard-loss LAND failsafe in charge. Unexpected mode changes latch
  `PILOT_OVERRIDE` without forcing Offboard re-entry, including mode changes
  during startup. A manager-requested transition to fallback LAND is expected,
  not treated as a pilot override.
- Search or PID failure requests native LAND, which does not guarantee landing
  on the dock. Disarm is never forced and requires PX4 touchdown confirmation.
- The SITL profile selects no-RC input and Offboard-loss LAND. Real hardware
  requires a separately reviewed profile.

The final PID descent remains 0.15 m/s. The paired PX4 SITL profile sets
`LNDMC_Z_VEL_MAX=0.10`: RC1 requires commanded downward speed to be at least
`1.1 * LNDMC_Z_VEL_MAX` before it can detect ground contact. The stock 0.25
threshold prevents landing detection at this descent speed. Keep these two
profiles consistent when tuning. The touchdown deadline is 30 seconds to allow
thrust to fall after contact, PX4's landing hysteresis and the manager's stable
landed dwell. Restart SITL and the manager after changing these profiles;
disarm still requires fresh PX4 landed telemetry.

The old ArduPilot onboard-mission download/reset and AUTO handoff contract is
not implemented. PX4's default DDS topics do not expose an equivalent mission
download protocol. Debug launch modes and touchdown CSV logging
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

That flight result is historical, not validation of RC1 flight behavior. The
RC1 SITL build and disarmed startup/DDS check now pass, including valid local
position and an accepted prearm-check command ACK. See
`/home/enri-ubuntu/drone_ws/src/drone_docking/drone_description/PX4_RC1_VALIDATION.md`.
On 2026-10-24, a headless RC1 run with the matched land-detector threshold and
30-second touchdown deadline completed takeoff, visual docking, PX4 landing
detection and ordinary external disarm, reaching `DOCKED` without fallback.
Touchdown approach to `DISARM` took 16.445 seconds. This is one nominal SITL
cycle, not hardware or disturbance validation; see
[`drone_description/PX4_TOUCHDOWN_VALIDATION.md`](../drone_description/PX4_TOUCHDOWN_VALIDATION.md).

Run the retained package tests in a sourced ROS environment:

```bash
cd /home/enri-ubuntu/drone_ws/src/drone_docking
python3 -m pytest -q /home/enri-ubuntu/drone_ws/src/drone_docking/flight_cycle_manager/test
```

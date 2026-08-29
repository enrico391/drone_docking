# PID Landing Mode Design

Date: 2026-08-29

## Summary

The mission manager will expose two landing modes:

- `precise_landing`: the existing ArduPilot-native precision-landing path,
  currently named `ardupilot`, which streams `LANDING_TARGET` measurements and
  lets ArduPilot control the descent in `LAND` mode.
- `pid`: a companion-computer landing path, currently named `ros`, which keeps
  ArduPilot in `GUIDED` and sends body-frame X, Y, Z velocity and yaw-rate
  setpoints through the existing direct pymavlink connection.

ROS 2 remains the perception and observability bus. The mission manager will
continue to receive the filtered AprilTag dock pose and target mode through ROS
topics and publish mission state through ROS. No command or telemetry path to
ArduPilot will depend on ROS.

The PID mode adds bounded circular searching, outer-tag alignment and descent,
mandatory central-tag acquisition for the final approach, climb-and-retry
recovery, and a committed touchdown phase. It uses ArduPilot's landed-state
telemetry as the initial touchdown detector. Physical docking-contact support
is deliberately deferred.

## Goals

- Give the two existing landing behaviors accurate, stable names.
- Keep all flight-controller commands and telemetry on one pymavlink link.
- Control dock-relative X, Y, Z and yaw with four configurable PID controllers.
- Search around the expected dock location when no configured AprilTag is
  visible.
- Require the central tag before descending below the central-transition
  height.
- Recover upward and retry if the central tag is lost before touchdown commit.
- Stay in `GUIDED` through touchdown and normally disarm only after ArduPilot
  reports `MAV_LANDED_STATE_ON_GROUND`.
- Keep the implementation small, readable and independently testable.

## Non-goals

- Removing ROS 2 from AprilTag detection or mission-status publication.
- Adding a physical docking-contact, GPIO, weight-on-wheels or Gazebo contact
  integration in this iteration.
- Automatic PID tuning, gain scheduling or adaptive control.
- Force-disarming an airborne vehicle.
- Replacing the mission types, coarse circle flight or direct pymavlink vehicle
  adapter.
- Modifying ArduPilot itself.

## Current Behavior and Migration

Both current landing backends already send commands through
`PymavlinkVehicle`; the current `ros` name describes where the orchestration
lives rather than its transport. The configuration key will be renamed from
`landing_backend` to `landing_mode`.

The accepted canonical values will be `precise_landing` and `pid`. During one
compatibility period, `ardupilot` will map to `precise_landing` and `ros` will
map to `pid`, each with a deprecation warning. Supplying both
`landing_backend` and `landing_mode` with conflicting values will be a startup
error. Launch files and checked-in examples will use only the canonical names.

The `precise_landing` behavior is preserved: validate the ArduPilot precision
landing parameters, stream fresh `LANDING_TARGET` observations, enter `LAND`,
and monitor touchdown and disarm.

## Code Structure

The design uses four focused modules without introducing a general plugin or
strategy framework:

- `mission_manager.py` retains the main flight mission, landing-mode selection,
  shared MAVLink lifecycle, state publication and terminal mission handling.
- `pid.py` contains a small ROS-independent PID implementation.
- `pid_landing.py` contains the PID landing states and their transition logic.
- `pymavlink_vehicle.py` gains an XYZ-velocity plus yaw-rate operation.

`PidLandingController` will not own ROS subscriptions or the MAVLink connection.
On every update, the mission manager gives it plain observations and vehicle
state. It returns a plain decision containing:

- the current PID landing state;
- an optional velocity/yaw-rate setpoint and its coordinate frame;
- an optional normal-disarm request;
- an outcome of running, complete or failed, with a diagnostic reason.

This keeps state transitions and control calculations unit-testable without a
ROS executor or live flight controller. The mission manager remains responsible
for actually sending commands and publishing state.

## Observation and Command Model

A dock observation contains the filtered camera-to-dock pose, selected target
mode (`OUTER`, `CENTRAL_ACQUIRING`, `CENTRAL` or `NO_TARGET`), source timestamp
and local reception timestamp. An observation is usable only while both pose
and target mode are fresh according to `dock_pose_timeout_s`.

The PID controller consumes errors in body FLU convention:

- forward error: `-camera_y`;
- left error: `-camera_x`;
- vertical error: desired camera-to-pad distance minus measured distance;
- yaw error: the shortest signed drone-to-dock angle in `[-pi, pi]`.

The output is body-forward velocity, body-left velocity, upward velocity and a
positive-counterclockwise yaw rate. The MAVLink adapter converts body FLU to
Body NED/FRD, including the required sign inversion for lateral, vertical and
yaw-rate components.

The existing velocity-only type mask `3527` ignores yaw rate. PID mode will use
the velocity-plus-yaw-rate mask `1479`, while existing callers that do not
provide yaw rate retain velocity-only behavior. Frame conversion and the mask
will be covered by exact-value unit tests.

## PID Controller

Four independent instances of one `PID` class control X, Y, Z and yaw. The
class supports configurable proportional, integral and derivative gains,
integral bounds, output bounds, a minimum/maximum valid time step and a
first-order derivative filter.

The controller will:

- calculate from monotonic time supplied by its caller;
- clamp output before it reaches the MAVLink adapter;
- prevent integral growth while the output is saturated in the same direction
  as the error;
- reset integral, derivative history and previous time on state changes, stale
  observations and recovery entry;
- reject non-finite inputs and invalid time steps;
- wrap yaw error before each update.

Initial checked-in gains will be conservative P-only settings (`Ki=0`,
`Kd=0`) using the existing horizontal proportional behavior as the starting
point. All PID terms are implemented and configurable, but nonzero integral and
derivative gains are introduced only after SITL log-based tuning.

The Z controller does not target zero range immediately. It follows an internal
distance setpoint initialized from the first valid observation. While alignment
is stable, the setpoint ramps downward at the configured stage descent speed.
If X, Y or yaw leaves its tolerance, the distance setpoint stops moving and Z
holds the current target distance. This prevents vertical progress while the
vehicle is misaligned.

## PID Landing State Machine

### `ACQUIRE_DOCK`

The vehicle holds zero body velocity above the configured dock position for
`pid_initial_acquire_time_s`. A usable observation that remains available for
`pid_tag_stable_time_s` transitions to `PID_ALIGN`. If the initial acquisition
window expires, the controller enters `SEARCH_CIRCLE`.

### `SEARCH_CIRCLE`

The vehicle first moves from the search center to the circle perimeter and then
follows a horizontal circle centered on the configured coarse dock coordinate
at the mission hover altitude. Search commands use map ENU because the search
center is a mission coordinate. The initial defaults are a 1.0 m radius,
0.30 m/s tangential speed and a 30 s timeout; each is configurable.

Any usable configured tag starts the same stability timer used by acquisition.
Once stable, search velocity is stopped and the controller enters `PID_ALIGN`.
If the search timeout expires without stable acquisition, PID landing fails and
the mission manager continuously commands zero velocity in `GUIDED` for
operator or configured autopilot-failsafe intervention.

### `PID_ALIGN`

X, Y and yaw PIDs align the vehicle while Z holds its current distance. The
controller chooses coarse or fine tolerances from measured distance. Stable
alignment above the central-transition distance enters `DESCEND_OUTER`.
Stable alignment at or below that distance enters `DESCEND_CENTRAL` only when
the target mode is `CENTRAL`; otherwise it enters `ACQUIRE_CENTRAL`.

### `DESCEND_OUTER`

All four PIDs run, but the Z distance setpoint descends only while the coarse X,
Y and yaw tolerances are satisfied. At the central-transition distance, which
defaults to 1.0 m, vertical progress stops. A stable central target enters
`DESCEND_CENTRAL`; otherwise the controller enters `ACQUIRE_CENTRAL`.

A stale observation immediately commands zero Z velocity and returns to
`ACQUIRE_DOCK`.

### `ACQUIRE_CENTRAL`

The vehicle holds position and altitude while waiting for a stable `CENTRAL`
observation. If central acquisition times out, it enters `RECOVERY_CLIMB`.

### `DESCEND_CENTRAL`

All four PIDs use fine tolerances and reduced output limits. If the central tag
is stale or absent for longer than `pid_central_loss_grace_s`, the controller
first commands zero descent and then enters `RECOVERY_CLIMB`.

When the vehicle is stably aligned at or below
`pid_touchdown_commit_height_m`, it enters `TOUCHDOWN_APPROACH`. The initial
commit height is 0.25 m.

### `RECOVERY_CLIMB`

The PID histories are reset. The controller captures the fresh MAVLink local
altitude on state entry and climbs a configured distance from that value rather
than using camera range, because the visual target may be absent. The
horizontal command is zero during the climb. The initial recovery climb is
0.30 m, and the default maximum is three recovery attempts.

At recovery height the controller returns to `ACQUIRE_CENTRAL`. Exhausting the
retry count fails PID landing and holds zero velocity.

### `TOUCHDOWN_APPROACH`

Touchdown is committed because climbing after the tag fills or leaves the
camera view near the surface would be unsafe. Tag loss no longer triggers
recovery. X, Y and yaw corrections stop, and the vehicle sends zero horizontal
velocity, zero yaw rate and a constant slow downward velocity. The initial
touchdown speed is 0.05 m/s.

The vehicle remains in `GUIDED`. A fresh ArduPilot
`MAV_LANDED_STATE_ON_GROUND` held for `pid_landed_stable_time_s` transitions to
`DISARM`. A 15 s touchdown timeout fails the controller and commands zero
velocity; it does not request a forced disarm.

### `DISARM`

The controller requests a normal `MAV_CMD_COMPONENT_ARM_DISARM` with
`param1=0` and `param2=0`. While the request is pending, it commands zero
velocity. A rejected or timed-out request is retried every 1.0 s while the
vehicle continues to report `ON_GROUND`. Loss of landed state returns to
`TOUCHDOWN_APPROACH`; it never changes the command into a forced disarm.

The landing completes only when heartbeat telemetry reports `armed=false`.
Existing repeat-mission behavior then applies.

## Parameters

Parameters stay in `mission_manager.yaml` and use flat ROS parameter names to
match the existing package. The initial additions are:

- mode and timing: `landing_mode`, `pid_control_rate_hz`,
  `pid_tag_stable_time_s`, `pid_initial_acquire_time_s`;
- search: `pid_search_radius_m`, `pid_search_speed_m_s`,
  `pid_search_timeout_s`;
- transition and recovery: `pid_central_transition_distance_m`,
  `pid_central_acquire_timeout_s`, `pid_central_loss_grace_s`,
  `pid_recovery_climb_m`, `pid_max_recovery_attempts`;
- touchdown: `pid_touchdown_commit_height_m`,
  `pid_touchdown_speed_m_s`, `pid_touchdown_timeout_s`,
  `pid_landed_state_timeout_s`, `pid_landed_stable_time_s`,
  `pid_disarm_retry_period_s`;
- tolerances and limits: coarse/fine horizontal tolerance, coarse/fine yaw
  tolerance, maximum horizontal velocity, maximum outer/fine vertical
  velocity and maximum yaw rate;
- four gain groups: X, Y, Z and yaw `kp`, `ki`, `kd`, integral limit and
  derivative filter time constant.

Startup validation rejects non-finite values, negative timeouts or limits,
invalid transition ordering, a commit height greater than or equal to the
central-transition distance, a recovery climb that is not positive, and retry
counts below one.

Mission geometry remains in mission-profile YAML. Controller tuning and search
behavior remain in `mission_manager.yaml`.

## Failure Handling

- A stale local vehicle pose, stale heartbeat or unexpected loss of `GUIDED`
  stops PID progression and enters the existing mission failure path.
- Stale visual input never feeds a PID update.
- Before touchdown commit, target loss stops descent before any transition or
  recovery motion.
- Search and recovery are bounded by time and retry count.
- Every failed PID landing continuously sends zero velocity while the MAVLink
  connection remains healthy.
- ArduPilot's `GUID_TIMEOUT` remains a secondary stop behavior if the companion
  setpoint stream disappears.
- Automatic force-disarm (`param2=21196`) is prohibited.
- Physical docking-contact handling is added later only by changing the
  touchdown-confirmation decision; it will not require restructuring the state
  machine.

## State Publication and Diagnostics

The public mission-state topic will expose the active PID landing substate with
a stable `PID_` prefix, for example `PID_SEARCH_CIRCLE`,
`PID_DESCEND_CENTRAL`, `PID_RECOVERY_CLIMB` and
`PID_TOUCHDOWN_APPROACH`. Logs will include transition reasons, search elapsed
time, recovery attempt number, target mode, pose age, PID errors, saturated
outputs and disarm acknowledgements. High-rate control values will be
throttled to avoid flooding logs.

## Testing

### Unit tests

- PID proportional, integral, derivative, reset, anti-windup, time-step and
  saturation behavior.
- X/Y camera-to-body conversion and FLU-to-MAVLink sign conversion.
- Yaw wrapping and yaw-rate sign conversion.
- Exact MAVLink velocity-plus-yaw-rate mask and fields.
- Every state transition, timeout and retry boundary using a fake monotonic
  clock.
- Stale tag behavior before commit and ignored tag loss after commit.
- Normal disarm request only after stable, fresh `ON_GROUND`.
- Compatibility aliases and conflicting configuration rejection.

### Mission-manager integration tests

- Mode selection does not create ROS flight-command publishers or clients.
- PID decisions are delivered through the injected fake pymavlink vehicle.
- Failure holds zero velocity.
- Successful disarm feeds the existing repeat/complete mission behavior.

### SITL/Gazebo scenarios

- Dock visible immediately above the expected position.
- Dock initially outside the camera view and acquired during circular search.
- Outer-tag descent followed by central-tag transition.
- Central-tag loss above commit height, recovery climb and successful retry.
- Central-tag loss after commit without recovery climb.
- Search timeout and recovery-attempt exhaustion.
- Complete GUIDED touchdown, `ON_GROUND`, normal disarm and mission completion.

SITL tuning starts with P-only gains. Integral and derivative gains remain zero
until logs demonstrate a repeatable steady-state error or damping need. Hardware
testing begins only after the full SITL state-transition suite passes and uses
conservative speed/output limits.

## Acceptance Criteria

- Launching with `landing_mode:=precise_landing` preserves the existing native
  precision-landing behavior.
- Launching with `landing_mode:=pid` sends X/Y/Z velocity and yaw-rate commands
  directly through pymavlink and never uses ROS to command ArduPilot.
- PID mode searches a configurable circle after initial acquisition fails.
- It uses an outer or central tag for coarse alignment, requires a stable
  central tag for the final descent, and climbs to retry after pre-commit
  central-tag loss.
- It does not climb after touchdown commit.
- It stays in `GUIDED` through touchdown and requests only a normal disarm after
  a fresh, stable `ON_GROUND` report.
- Search, recovery, touchdown and disarm behavior are deterministic under fake
  time and covered by automated tests.
- The physical docking contact remains optional and absent from the first
  implementation.

# PID Landing Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a maintainable `pid` landing mode that searches for the dock, controls body X/Y/Z velocity and yaw rate through pymavlink, recovers from central-tag loss, lands in `GUIDED`, and normally disarms after ArduPilot reports touchdown while preserving the existing native precision-landing behavior as `precise_landing`.

**Architecture:** `MissionManager` continues to own ROS subscriptions, the direct MAVLink vehicle, the coarse mission, mode selection and terminal mission handling. A pure `PidLandingController` consumes immutable dock/vehicle observations and returns commands or lifecycle requests; a pure `PID` class performs each control loop. The current native ArduPilot precision-landing implementation remains in `mission_manager.py` and is renamed without behavioral refactoring.

**Tech Stack:** Python 3.10, ROS 2 Humble (`rclpy`), pymavlink 2.x, pytest, ArduPilot SITL and Gazebo.

**Spec:** `docs/superpowers/specs/2026-08-29-pid-landing-mode-design.md`

## Global Constraints

- Canonical landing modes are exactly `precise_landing` and `pid`.
- Legacy `ardupilot` maps to `precise_landing`; legacy `ros` maps to `pid` with warnings.
- ROS may carry AprilTag perception and mission status, but never commands or telemetry to/from ArduPilot.
- PID visual commands use body FLU at the controller boundary and are converted once in `PymavlinkVehicle` to MAVLink Body NED/FRD.
- Automatic force-disarm with `param2=21196` is prohibited.
- The initial touchdown source is fresh, stable `MAV_LANDED_STATE_ON_GROUND`; physical contact support is out of scope.
- The working existing `precise_landing` implementation stays in `mission_manager.py` during this change.
- Preserve unrelated modifications to `.gitignore`, `.AGENTS`, and `drone_description/config/gazebo-x500.parm`.

## File Structure

- Create `drone_mission_manager/drone_mission_manager/pid.py`: pure generic PID implementation and immutable tuning data.
- Create `drone_mission_manager/drone_mission_manager/pid_landing.py`: PID landing configuration, data model, state machine, search trajectory and command decisions.
- Modify `drone_mission_manager/drone_mission_manager/pymavlink_vehicle.py`: optional yaw-rate field and velocity-plus-yaw-rate MAVLink encoding.
- Modify `drone_mission_manager/drone_mission_manager/mission_manager.py`: canonical mode resolution, PID controller integration, observation conversion, disarm transaction and public substate reporting.
- Modify `drone_mission_manager/config/mission_manager.yaml`: conservative PID/search/recovery/touchdown defaults.
- Modify `drone_mission_manager/launch/mission.launch.py`: canonical launch argument plus compatibility argument.
- Modify `drone_mission_manager/README.md`: mode names, flow, configuration and safety behavior.
- Create `drone_mission_manager/test/test_pid.py`: focused PID unit tests.
- Create `drone_mission_manager/test/test_pid_landing.py`: deterministic state-machine and command tests with explicit timestamps.
- Modify `drone_mission_manager/test/test_pymavlink_vehicle.py`: exact type-mask, yaw-rate sign and legacy-call tests.
- Modify `drone_mission_manager/test/test_mission_manager_transport.py`: mode migration, PID wiring, ROS/MAVLink boundary and lifecycle integration tests.

---

### Task 1: Pure PID primitive

**Files:**
- Create: `drone_mission_manager/drone_mission_manager/pid.py`
- Create: `drone_mission_manager/test/test_pid.py`

**Interfaces:**
- Produces: `PIDConfig(kp, ki, kd, integral_limit, output_limit, derivative_filter_time_constant_s, min_dt_s=1e-4, max_dt_s=0.5)`.
- Produces: `PID(config: PIDConfig)`, `PID.reset() -> None`, and `PID.update(error: float, now: float) -> float`.
- Behavior: the first valid update has no derivative contribution; non-finite error/time raises `ValueError`; non-increasing or out-of-range `dt` resets derivative history and uses proportional/integral state without a derivative spike.

- [ ] **Step 1: Write failing configuration and proportional-output tests**

```python
def test_pid_rejects_invalid_limits():
    with pytest.raises(ValueError, match="output_limit"):
        PIDConfig(kp=1.0, ki=0.0, kd=0.0, integral_limit=1.0,
                  output_limit=0.0, derivative_filter_time_constant_s=0.1)


def test_pid_proportional_output_is_symmetric_and_clamped():
    controller = PID(config(output_limit=0.5, kp=2.0))
    assert controller.update(0.1, now=1.0) == pytest.approx(0.2)
    assert controller.update(-1.0, now=1.1) == pytest.approx(-0.5)
```

- [ ] **Step 2: Run the tests and confirm the missing module failure**

Run: `python3 -m pytest drone_mission_manager/test/test_pid.py -q`

Expected: collection fails with `ModuleNotFoundError: drone_mission_manager.pid`.

- [ ] **Step 3: Implement immutable configuration validation, reset and proportional output**

Implement `PIDConfig.__post_init__` using `math.isfinite`; require nonnegative gains/filter time/integral limit, positive output/min/max time limits, and `min_dt_s < max_dt_s`. Implement `PID` with private previous-time, previous-error, filtered-derivative and integral fields.

- [ ] **Step 4: Add failing integral, anti-windup, derivative-filter and reset tests**

```python
def test_integral_stops_growing_while_output_is_saturated():
    controller = PID(config(kp=0.0, ki=1.0, integral_limit=10.0,
                            output_limit=0.2))
    controller.update(1.0, now=1.0)
    assert controller.update(1.0, now=2.0) == pytest.approx(0.2)
    assert controller.update(-1.0, now=3.0) < 0.2


def test_reset_removes_integral_and_derivative_history():
    controller = PID(config(kp=0.0, ki=1.0, kd=1.0))
    controller.update(1.0, now=1.0)
    controller.update(2.0, now=1.1)
    controller.reset()
    assert controller.update(0.0, now=5.0) == 0.0
```

Also cover filtered derivative direction, integral clamping, repeated timestamp,
non-finite inputs and a time jump larger than `max_dt_s`.

- [ ] **Step 5: Implement integral clamping, conditional-integration anti-windup and filtered derivative**

Calculate the raw derivative as `(error - previous_error) / dt`. When the filter time constant is nonzero, use `alpha = dt / (tau + dt)` and update the filtered derivative. Accept a candidate integral only when the unclamped output is not saturated in the same direction as the error. Clamp the final output to `[-output_limit, output_limit]`.

- [ ] **Step 6: Run the focused PID tests**

Run: `python3 -m pytest drone_mission_manager/test/test_pid.py -q`

Expected: all PID tests pass.

- [ ] **Step 7: Commit the PID primitive**

```bash
git add drone_mission_manager/drone_mission_manager/pid.py \
  drone_mission_manager/test/test_pid.py
git commit -m "feat: add bounded PID controller"
```

---

### Task 2: MAVLink XYZ velocity plus yaw rate

**Files:**
- Modify: `drone_mission_manager/drone_mission_manager/pymavlink_vehicle.py:13-18,253-292`
- Modify: `drone_mission_manager/test/test_pymavlink_vehicle.py:230-296`

**Interfaces:**
- Changes: `MavlinkVehicle.send_velocity(frame, velocity_x, velocity_y, velocity_z, *, now, yaw_rate_rad_s: Optional[float] = None) -> bool`.
- Preserves: all existing callers and velocity-only type mask `3527` when yaw rate is omitted.
- Adds: velocity-plus-yaw-rate type mask `1479` when yaw rate is supplied.
- Convention: `yaw_rate_rad_s` is positive counterclockwise around body FLU +Z; the transmitted MAVLink NED yaw rate is its negative.

- [ ] **Step 1: Add a failing exact-message yaw-rate test**

```python
def test_body_velocity_with_yaw_rate_uses_flu_to_ned_conversion(vehicle, connection):
    vehicle.send_velocity(
        VelocityFrame.BODY_FLU, 1.0, -2.0, 0.25,
        yaw_rate_rad_s=0.4, now=30.0,
    )
    call = connection.mav.calls_for("set_position_target_local_ned_send")[-1]
    assert call[3] == MAVLINK.MAV_FRAME_BODY_NED
    assert call[4] == 1479
    assert call[8:11] == pytest.approx((1.0, 2.0, -0.25))
    assert call[15] == pytest.approx(-0.4)
```

Retain the existing test asserting mask `3527` for calls without yaw rate. Add
a non-finite yaw-rate rejection test.

- [ ] **Step 2: Run the focused test and verify the signature failure**

Run: `python3 -m pytest drone_mission_manager/test/test_pymavlink_vehicle.py -q`

Expected: the new test fails because `send_velocity` does not accept
`yaw_rate_rad_s`.

- [ ] **Step 3: Implement optional yaw-rate encoding**

Add `_POSITION_TARGET_TYPEMASK_VELOCITY_YAW_RATE = 1479`. Select it only when
`yaw_rate_rad_s` is not `None`, reject non-finite velocity/yaw values, and write
`-yaw_rate_rad_s` into the final MAVLink field. Leave the ignored yaw value at
zero. Keep `close()` on the velocity-only path.

- [ ] **Step 4: Run the complete vehicle-adapter tests**

Run: `python3 -m pytest drone_mission_manager/test/test_pymavlink_vehicle.py -q`

Expected: all adapter tests pass, including legacy velocity and native
`LANDING_TARGET` behavior.

- [ ] **Step 5: Commit the transport extension**

```bash
git add drone_mission_manager/drone_mission_manager/pymavlink_vehicle.py \
  drone_mission_manager/test/test_pymavlink_vehicle.py
git commit -m "feat: send guided velocity with yaw rate"
```

---

### Task 3: PID landing data model, acquisition and circular search

**Files:**
- Create: `drone_mission_manager/drone_mission_manager/pid_landing.py`
- Create: `drone_mission_manager/test/test_pid_landing.py`

**Interfaces:**
- Produces enums: `DockTargetMode`, `PidLandingState`, and `PidLandingOutcome`.
- Produces immutable values:
  - `DockObservation(forward_m, left_m, distance_m, yaw_error_rad, mode, source_stamp_s, received_at)`.
  - `VehicleObservation(position_enu_m, velocity_enu_m_s, armed, flying, mode, landed_at)`.
  - `VelocityCommand(frame, velocity_x, velocity_y, velocity_z, yaw_rate_rad_s)`.
  - `PidLandingDecision(state, command, outcome=RUNNING, request_disarm=False, disarm_request_id=None, reason="")`.
  - `PidLandingConfig` containing all timing, geometry, tolerances, limits and four `PIDConfig` instances.
- Produces controller methods:
  - `PidLandingController(config)`.
  - `start(now, dock_x_m, dock_y_m, hover_altitude_m) -> None`.
  - `update(now, dock: Optional[DockObservation], vehicle: VehicleObservation) -> PidLandingDecision`.
  - `notify_disarm_result(request_id, success, now) -> None`.

- [ ] **Step 1: Write failing construction and validation tests**

Create a `default_config()` fixture with explicit conservative values. Test that
commit height must be below central transition, recovery climb and search radius
must be positive, timeouts must be positive, retry count must be at least one,
fine tolerances must be smaller than coarse tolerances, recovery altitude
tolerance must be positive and smaller than recovery climb, and all values must
be finite.

- [ ] **Step 2: Write failing acquisition-state tests using explicit time**

```python
def test_stable_outer_tag_transitions_from_acquire_to_align(controller, vehicle):
    controller.start(now=0.0, dock_x_m=4.0, dock_y_m=-2.0,
                     hover_altitude_m=4.0)
    first = controller.update(0.1, outer_observation(received_at=0.1), vehicle)
    stable = controller.update(0.7, outer_observation(received_at=0.7), vehicle)
    assert first.state is PidLandingState.ACQUIRE_DOCK
    assert stable.state is PidLandingState.PID_ALIGN


def test_acquisition_timeout_enters_search_with_zero_transition_command(...):
    decision = controller.update(2.1, None, vehicle_at(4.0, -2.0, 4.0))
    assert decision.state is PidLandingState.SEARCH_CIRCLE
    assert decision.command.velocity_z == 0.0
```

- [ ] **Step 3: Run the tests and verify the missing module failure**

Run: `python3 -m pytest drone_mission_manager/test/test_pid_landing.py -q`

Expected: collection fails with `ModuleNotFoundError`.

- [ ] **Step 4: Implement data classes, validation, `start`, observation stability and acquisition**

Use only standard-library modules plus `.pid` and `.pymavlink_vehicle`. Require
`start()` before `update()`. Reset stability timers when an observation is absent
or non-finite. Return body-frame zero commands while acquiring.

- [ ] **Step 5: Add failing perimeter, orbit, acquisition-during-search and timeout tests**

Verify that search first commands toward `(dock_x + radius, dock_y,
hover_altitude)`, then advances an orbit target using
`angle = search_direction * angular_speed * elapsed`, with
`angular_speed = search_speed / search_radius`. Verify map ENU frame, bounded
horizontal velocity, fixed altitude correction, stable tag interruption and an
exact failed outcome/reason at timeout.

- [ ] **Step 6: Implement the search trajectory**

Use the existing mission controller pattern: proportional map-position
correction plus tangential feed-forward, horizontal magnitude limiting and
vertical limiting. Store an internal perimeter-reached flag and orbit start
time. A tag-stability transition always emits zero body velocity for that tick
before entering `PID_ALIGN`.

- [ ] **Step 7: Run the acquisition/search tests**

Run: `python3 -m pytest drone_mission_manager/test/test_pid_landing.py -q`

Expected: all construction, acquisition and search tests pass.

- [ ] **Step 8: Commit acquisition and search**

```bash
git add drone_mission_manager/drone_mission_manager/pid_landing.py \
  drone_mission_manager/test/test_pid_landing.py
git commit -m "feat: add dock acquisition and search controller"
```

---

### Task 4: Visual alignment, staged descent and central-tag recovery

**Files:**
- Modify: `drone_mission_manager/drone_mission_manager/pid_landing.py`
- Modify: `drone_mission_manager/test/test_pid_landing.py`

**Interfaces:**
- Extends: `PidLandingController.update` across `PID_ALIGN`, `DESCEND_OUTER`,
  `ACQUIRE_CENTRAL`, `DESCEND_CENTRAL`, and `RECOVERY_CLIMB`.
- Preserves: immutable observations and decisions from Task 3.
- Uses: four `PID` instances, body FLU command convention, local ENU altitude
  only for recovery completion.

- [ ] **Step 1: Write failing body-error, yaw-wrap and alignment tests**

Use P-only gains so output values are exact. Verify forward/left commands move
toward the dock, yaw error near `+pi/-pi` takes the shortest direction, velocity
limits apply, Z is zero during alignment, and the stable-alignment timer resets
when any controlled error leaves tolerance.

- [ ] **Step 2: Implement `_visual_command`, yaw wrapping and stable alignment**

`_visual_command` updates X/Y/yaw PIDs on every valid visual tick. It updates Z
against `distance_setpoint - observation.distance_m` only when Z is enabled.
Clamp horizontal magnitude after individual PID outputs and clamp yaw rate.

- [ ] **Step 3: Add failing outer-descent setpoint-ramp and central-transition tests**

Assert that the distance setpoint is initialized from the measured distance,
decreases by `outer_descent_speed_m_s * dt` only while X/Y/yaw are within coarse
tolerances, freezes when misaligned, and changes state at exactly the central
transition distance. Cover both an already stable `CENTRAL` observation and an
outer-only observation.

- [ ] **Step 4: Implement `PID_ALIGN` and `DESCEND_OUTER`**

Reset all PID histories on state entry. Use coarse tolerance above the central
transition and fine tolerance at/below it. Missing/stale input is represented by
`dock=None`; immediately return a zero command and enter `ACQUIRE_DOCK`.

- [ ] **Step 5: Add failing central acquisition, grace-period and recovery tests**

Test stable central acquisition, acquisition timeout, central loss shorter and
longer than the grace period, zero descent on the first missing-central tick,
recovery target equal to entry local altitude plus `recovery_climb_m`, zero X/Y
and yaw during recovery, completion based on local altitude tolerance, and
failure on the configured recovery-attempt limit.

- [ ] **Step 6: Implement central descent and recovery**

Increment the recovery-attempt counter on entry to `RECOVERY_CLIMB`. Capture the
entry ENU altitude exactly once. Command positive body-FLU Z at the configured
recovery speed until the local altitude target is reached, then enter
`ACQUIRE_CENTRAL`. Never use a missing camera range to terminate recovery.

- [ ] **Step 7: Run all PID landing tests**

Run: `python3 -m pytest drone_mission_manager/test/test_pid_landing.py -q`

Expected: all acquisition, search, alignment, descent and recovery tests pass.

- [ ] **Step 8: Commit visual descent and recovery**

```bash
git add drone_mission_manager/drone_mission_manager/pid_landing.py \
  drone_mission_manager/test/test_pid_landing.py
git commit -m "feat: add PID descent and tag-loss recovery"
```

---

### Task 5: Touchdown commit and normal-disarm lifecycle

**Files:**
- Modify: `drone_mission_manager/drone_mission_manager/pid_landing.py`
- Modify: `drone_mission_manager/test/test_pid_landing.py`

**Interfaces:**
- Extends states: `TOUCHDOWN_APPROACH`, `DISARM`, `COMPLETE`, `FAILED`.
- Produces: `request_disarm=True` with a monotonically increasing
  `disarm_request_id` only in `DISARM`, only with fresh stable
  `flying is False`, and only when its retry deadline permits.
- Consumes: `notify_disarm_result(request_id, success, now)` from the
  mission-manager asynchronous command boundary.

- [ ] **Step 1: Add failing touchdown-commit tests**

Track fine X/Y/yaw stability throughout central descent and consume it when
tag distance reaches the commit threshold. Verify that the transition command
is zero, subsequent commands are exactly
`(0, 0, -touchdown_speed, 0)` in body FLU, and missing tags after commit do not
change state or trigger recovery.

- [ ] **Step 2: Implement touchdown commit and timeout**

Store touchdown entry time. Ignore dock observations in this state. Require a
fresh landed timestamp and continuous `flying is False` for
`landed_stable_time_s`; reset the stability timer if `flying` is true/unknown or
`landed_at` is stale. At timeout, return `FAILED` with a zero command.

- [ ] **Step 3: Add failing disarm request/retry/completion tests**

```python
def test_stable_on_ground_requests_only_normal_disarm(...):
    decision = advance_to_stable_ground(controller)
    assert decision.state is PidLandingState.DISARM
    assert decision.request_disarm is True
    assert decision.disarm_request_id is not None
    assert decision.command == zero_body_command()


def test_disarm_completes_only_after_armed_false(...):
    initial = advance_to_stable_ground(controller)
    controller.notify_disarm_result(
        initial.disarm_request_id, True, now=8.0
    )
    waiting = controller.update(8.1, None, grounded_vehicle(armed=True))
    complete = controller.update(8.2, None, grounded_vehicle(armed=False))
    assert waiting.outcome is PidLandingOutcome.RUNNING
    assert complete.outcome is PidLandingOutcome.COMPLETE
```

Also test rejection schedules a normal retry, no duplicate request before the
deadline, and a return to touchdown approach if the vehicle reports flying.

- [ ] **Step 4: Implement disarm lifecycle**

Track request-in-flight and next-retry time in the controller. The decision does
not encode MAVLink parameters; `MissionManager` always translates it to
`arm_async(False)`, whose existing implementation uses `param2=0`. A successful
command acknowledgement does not complete landing until heartbeat telemetry
reports `armed=False`. Allocate a new monotonically increasing positive ID for
each request and ignore a result whose ID does not match the active request.

- [ ] **Step 5: Run all pure controller tests**

Run: `python3 -m pytest drone_mission_manager/test/test_pid.py \
  drone_mission_manager/test/test_pid_landing.py -q`

Expected: all tests pass.

- [ ] **Step 6: Commit touchdown and disarm decisions**

```bash
git add drone_mission_manager/drone_mission_manager/pid_landing.py \
  drone_mission_manager/test/test_pid_landing.py
git commit -m "feat: add guided touchdown and normal disarm"
```

---

### Task 6: Mission-manager mode migration and PID integration

**Files:**
- Modify: `drone_mission_manager/drone_mission_manager/mission_manager.py`
- Modify: `drone_mission_manager/test/test_mission_manager_transport.py`

**Interfaces:**
- Adds: `resolve_landing_mode(canonical: str, legacy: str) -> tuple[str, Optional[str]]`, where the second value is a deprecation warning.
- Adds: `MissionState.PID_LANDING` as the only main-state representation of the pure controller.
- Adds: `_pid_landing: Optional[PidLandingController]` and last-published PID substate tracking.
- Adds: retention of the active PID disarm request ID alongside the pending
  pymavlink operation.
- Uses: `camera_dock_orientation_to_body_yaw_error` plus the existing camera axis conversion to construct `DockObservation`.
- Uses: `MavlinkVehicle.send_velocity(..., yaw_rate_rad_s=...)` and `arm_async(False)`.

- [ ] **Step 1: Add failing canonical/legacy mode-resolution tests**

Test canonical values, blank canonical with both legacy aliases, invalid values,
and conflicting canonical/legacy values. Use blank defaults for both declared
parameters and resolve blank/blank to `pid`.

- [ ] **Step 2: Implement mode resolution and rename internal conditions**

Declare `landing_mode` and a compatibility `landing_backend`, resolve once at
startup into `_landing_mode`, and log the returned warning. Replace native-path
checks of `"ardupilot"` with `"precise_landing"`. Preserve the native landing
configuration check, `LANDING_TARGET` stream, yaw condition command and LAND
handoff unchanged.

- [ ] **Step 3: Add a test fixture with a command-capable `FakeVehicle`**

Record `send_velocity`, `arm_async`, `poll`, parameter and mode calls. Supply
completed fake futures so tests can advance the manager deterministically. Add
factory helpers to inject fresh `VehicleState` position, mode, landed timestamp,
armed and flying values.

- [ ] **Step 4: Add failing PID handoff and command-routing tests**

Set the main state immediately before the return-to-dock completion, inject a
fresh dock pose/mode and verify:

- `pid` starts `PidLandingController` and enters `MissionState.PID_LANDING`;
- returned body velocity/yaw rate is sent only to the fake pymavlink vehicle;
- no AP_DDS topic or service appears;
- published public state changes to `PID_<substate>`;
- `precise_landing` continues through native `ACQUIRE_DOCK` and `SET_LAND`.

- [ ] **Step 5: Implement PID observation/config construction and update dispatch**

Build `PidLandingConfig` from declared ROS parameters in one helper. Convert the
latest filtered `PoseStamped` to forward/left/distance/yaw exactly once; pass
`None` when either pose or mode is stale. Build `VehicleObservation` from the
current `VehicleState`. Send every returned command with its frame and yaw rate.
Create the ROS timer at `pid_control_rate_hz` when `_landing_mode == "pid"` and
at the existing `update_rate_hz` for `precise_landing`; test both timer-period
selections through a pure helper so executor timing is not asserted by sleeps.

- [ ] **Step 6: Add failing asynchronous disarm and terminal-outcome tests**

Verify `request_disarm` creates a pending operation named `pid_disarm`, success
and rejection call `notify_disarm_result` with the decision's request ID, no
second request is created while a future is pending, controller completion
calls `_complete_landing`, and controller failure enters the existing
zero-velocity `FAILED` behavior.

- [ ] **Step 7: Implement disarm/result integration and global safety ordering**

Handle PID landing before the general “unexpected disarm while active” failure
guard so `armed=False` can complete landing. Verify the vehicle remains in
`GUIDED_MODE`; an unexpected mode change fails PID landing. Publish PID substate
changes without changing the existing transient-local state topic. Retain the
request ID while the async disarm operation is pending and echo it unchanged to
`notify_disarm_result(request_id, success, now)` on success, rejection, timeout,
or synchronous start failure.

- [ ] **Step 8: Run mission-manager and existing landing tests**

Run: `python3 -m pytest \
  drone_mission_manager/test/test_mission_manager_transport.py \
  drone_mission_manager/test/test_landing_target.py \
  drone_mission_manager/test/test_landing_wait.py -q`

Expected: all integration and preserved native-landing tests pass.

- [ ] **Step 9: Commit mission-manager integration**

```bash
git add drone_mission_manager/drone_mission_manager/mission_manager.py \
  drone_mission_manager/test/test_mission_manager_transport.py
git commit -m "feat: integrate PID landing mode"
```

---

### Task 7: Configuration, launch and documentation migration

**Files:**
- Modify: `drone_mission_manager/config/mission_manager.yaml`
- Modify: `drone_mission_manager/launch/mission.launch.py`
- Modify: `drone_mission_manager/README.md`
- Modify: `drone_mission_manager/test/test_mission_manager_transport.py`

**Interfaces:**
- Launch exposes canonical `landing_mode` choices `pid` and
  `precise_landing`, plus blank as the compatibility-only default sentinel;
  blank/blank resolves to `pid`.
- Launch retains optional legacy `landing_backend` choices blank, `ros`, and
  `ardupilot` for one compatibility period.
- Configuration supplies every field required by `PidLandingConfig` with no
  hidden code-only tunables.

- [ ] **Step 1: Add failing launch/configuration tests**

Load `mission_manager.yaml` and assert canonical `landing_mode: pid`, 20 Hz PID
rate, positive search/recovery/touchdown values, recovery altitude tolerance
positive and below the recovery climb, commit height below central transition,
and all four gain groups. Inspect the generated launch description and assert
both canonical and legacy arguments are declared.

- [ ] **Step 2: Add the conservative checked-in parameter set**

Use these initial behavior defaults from the spec:

```yaml
landing_mode: pid
pid_control_rate_hz: 20.0
pid_initial_acquire_time_s: 2.0
pid_tag_stable_time_s: 0.5
pid_search_radius_m: 1.0
pid_search_speed_m_s: 0.30
pid_search_timeout_s: 30.0
pid_central_transition_distance_m: 1.0
pid_central_acquire_timeout_s: 3.0
pid_central_loss_grace_s: 0.30
pid_recovery_climb_m: 0.30
pid_recovery_altitude_tolerance_m: 0.05
pid_max_recovery_attempts: 3
pid_touchdown_commit_height_m: 0.20
pid_touchdown_speed_m_s: 0.05
pid_touchdown_timeout_s: 15.0
pid_landed_state_timeout_s: 1.0
pid_landed_stable_time_s: 0.50
pid_disarm_retry_period_s: 1.0
pid_alignment_stable_time_s: 1.0
pid_coarse_horizontal_tolerance_m: 0.20
pid_fine_horizontal_tolerance_m: 0.07
pid_coarse_yaw_tolerance_rad: 0.1745329252
pid_fine_yaw_tolerance_rad: 0.0872664626
pid_max_horizontal_speed_m_s: 0.50
pid_outer_descent_speed_m_s: 0.30
pid_central_descent_speed_m_s: 0.12
pid_recovery_speed_m_s: 0.20
pid_max_yaw_rate_rad_s: 0.3490658504
pid_search_position_gain: 0.70
pid_search_position_tolerance_m: 0.35
pid_search_max_vertical_speed_m_s: 0.70
```

Configure each axis with the suffixes `_kp`, `_ki`, `_kd`,
`_integral_limit`, and `_derivative_filter_time_constant_s`. Use prefixes
`pid_x`, `pid_y`, `pid_z`, and `pid_yaw`. Set X/Y `kp=0.7`, Z `kp=0.6`, yaw
`kp=0.8`, every `ki` and `kd` to zero, every integral limit to `0.5`, and every
derivative-filter time constant to `0.10 s`. Derive each PID output limit from
the matching velocity/yaw limit above rather than declaring a duplicate ROS
parameter.

- [ ] **Step 3: Update launch arguments and README examples**

Pass both parameters to `MissionManager`; document canonical invocations and a
short compatibility note. Explain that PID mode remains in GUIDED, searches a
bounded circle, requires central-tag recovery above commit, ignores tag loss
after commit, and disarms normally only after `ON_GROUND`. Remove wording that
calls `pid` a ROS command backend.

- [ ] **Step 4: Run configuration and launch tests**

Run: `python3 -m pytest drone_mission_manager/test/test_mission_manager_transport.py -q`

Expected: all mode, launch, configuration and transport-boundary tests pass.

- [ ] **Step 5: Commit configuration and documentation**

```bash
git add drone_mission_manager/config/mission_manager.yaml \
  drone_mission_manager/launch/mission.launch.py \
  drone_mission_manager/README.md \
  drone_mission_manager/test/test_mission_manager_transport.py
git commit -m "docs: configure PID landing mode"
```

---

### Task 8: Full regression, build and SITL acceptance

**Files:**
- Modify only if a verification failure exposes a requirement defect in the
  files already listed above; pair each correction with a failing regression
  test before implementation.

**Interfaces:**
- Verifies the complete public behavior; introduces no new production API.

- [ ] **Step 1: Run the complete Python test suite**

Run: `python3 -m pytest drone_mission_manager/test -q`

Expected: all tests pass with zero failures.

- [ ] **Step 2: Build and run ROS package tests**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_mission_manager
colcon test --packages-select drone_mission_manager --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero test failures and a successful package build.

- [ ] **Step 3: Run the preserved native-mode focused regression**

Run the manager tests selecting `precise_landing` with a fake vehicle and verify
that parameter checks, `LANDING_TARGET`, LAND-mode handoff and condition-yaw
tests remain green. Do not require a live simulator for this gate.

- [ ] **Step 4: Run one end-to-end PID landing in SITL/Gazebo**

Start the existing docking simulation without AP_DDS or MAVProxy:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_bringup docking_sim.launch.py \
  initial_point:=dock use_dds_agent:=false use_mavproxy:=false
```

In a second sourced terminal:

```bash
ros2 launch drone_mission_manager mission.launch.py \
  mission:=dock_circle auto_start:=true landing_mode:=pid
```

Record `/mission_manager/state`, `/dock/mode`, ArduPilot mode/armed/landed state
and the final vehicle pose. Acceptance requires a `PID_` state sequence,
continuous GUIDED control through touchdown, `ON_GROUND`, a normal disarm and
mission completion. Stop and diagnose rather than tune blindly if frame signs,
oscillation, stale-pose behavior or touchdown detection differ from the unit
model.

- [ ] **Step 5: Run a search/recovery SITL scenario or deterministic equivalent**

If the simulator launch can move the dock outside the initial camera footprint,
run a bounded search acquisition. Otherwise, retain the deterministic fake-time
search test as the automated acceptance gate and document that environmental
constraint in the verification report. Repeat the same rule for forced
central-tag dropout: use an available detector/topic control when present,
otherwise rely on the deterministic controller test.

- [ ] **Step 6: Inspect the final diff and workspace isolation**

Run:

```bash
git diff --check
git status --short
git diff --name-only 9f10d1b..HEAD
```

Expected: no whitespace errors; only the planned mission-manager files and
documents are part of the feature commits; pre-existing `.gitignore`,
`.AGENTS`, and X500 parameter changes remain outside these commits.

- [ ] **Step 7: Commit any test-driven verification corrections**

If Step 1-5 required corrections, stage only their paired production and test
files and commit with a specific `fix:` message describing the observed
failure. If no correction was required, create no empty commit.

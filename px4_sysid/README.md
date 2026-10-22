# PX4 Offboard velocity system identification

Identifies the **closed PX4 velocity loop seen by the outer docking PID**:
`TrajectorySetpoint.velocity -> VehicleLocalPosition.{vx,vy,vz}`, then adds
an integrator for the position plant `Gp(s) = Gv(s)/s`.

This is the PX4 port of `drone_sysid` (which is ArduPilot/GUIDED/MAVLink-only:
`SET_POSITION_TARGET_LOCAL_NED`, `TIMESYNC`). The excitation schedule and
`fit_plant.py` fitter are unchanged; only the transport is new.

## Plants

```
Gv(s) = K*exp(-L*s)/(1+tau*s)  or  K*exp(-L*s)/((1+tau1*s)*(1+tau2*s))
Gp(s) = Gv(s)/s
```

PX4 timebase: with **`UXRCE_DDS_SYNCT=0`**,
`VehicleLocalPosition.timestamp/1e6` is FC boot/simulation time, shared by
command echo and feedback. The docking SITL profile sets this parameter on
boot. Keep it disabled for identification and fit with `--timebase fc`.
With PX4's default `UXRCE_DDS_SYNCT=1`, DDS translates timestamps into the
Agent's wall-clock domain; those timestamps are not raw FC time.

## Install / test

```bash
source /opt/ros/humble/setup.bash
cd /home/enri-ubuntu/drone_ws
colcon build --symlink-install --packages-select px4_sysid
source /home/enri-ubuntu/drone_ws/install/setup.bash
python3 /home/enri-ubuntu/drone_ws/src/drone_docking/px4_sysid/test/test_px4_sysid.py
# ROS DDS lifecycle test (isolated domain 219; no flight controller required):
python3 /home/enri-ubuntu/drone_ws/src/drone_docking/px4_sysid/test/test_ros_lifecycle.py
# Actual backwards UTC steps, injected only into an isolated child process:
python3 /home/enri-ubuntu/drone_ws/src/drone_docking/px4_sysid/test/test_wall_clock.py
```

## Flight (SITL first, manager stopped)

**NEVER run alongside `flight_cycle_manager`: both own Offboard.**

After updating this package/profile, rebuild and **restart the SITL launch**
before taking off. `UXRCE_DDS_SYNCT` requires a PX4 restart. The node rejects
acquisition if telemetry still has epoch-sized timestamps and prints the
parameter to correct. Command timestamps echo PX4 telemetry; do not substitute
ROS/system UTC timestamps when timestamp translation is disabled.

```bash
ros2 run px4_sysid px4_sysid --axis x --out /home/enri-ubuntu/drone_ws/runs/x_px4_02
# take off, hold in Position/Hold, then in the node terminal:
start      # checks airborne/health, warms up hold, acquires Offboard, runs sysID
```

For a GUI Start button (or `--no-stdin`), call the private Trigger service:

```bash
ros2 service call /px4_sysid/start std_srvs/srv/Trigger '{}'
```

`success: true` means the request was accepted, not that Offboard acquisition or
the experiment has already completed. Progress/rejection reasons are printed
in the node terminal. The one-shot sequence is:

1. **WAIT:** passive; no setpoints or mode requests before explicit `start`.
2. **WARMUP:** reject starts on the ground, while disarmed/unhealthy, or with
   another setpoint publisher. Capture the current NED position and stream a
   position hold for 1.5 s before requesting Offboard **once**. Wait for mode
   confirmation; an ACK alone does not begin the experiment. If acquisition
   times out, abort without retrying or fighting PX4's LAND mode.
3. **RUN:** once Offboard is confirmed, run the 5 s hover / 50 s chirp / hover /
   45 s PRBS / hover schedule. FC-clock timing starts at this confirmation,
   not at the initial button press. Duplicate starts cannot restart the run.
4. **HOLD:** save the completed recording and continuously publish a fixed
   position/yaw hold at the **finishing location**, not the initial location.
   The node remains alive and streams at the configured rate (default 50 Hz).

The optional legacy `offboard` command still acquires Offboard and waits in
**READY** until `start`; a separate acquisition step is no longer required.

**Keep the node running after completion. Before `quit`, Ctrl-C, or stopping
the process, switch the drone to Position/Hold using the pilot/GCS.** Killing
the node while still in Offboard stops the heartbeat and invokes PX4's
configured Offboard-loss action (LAND in the docking profile). A stationary
Offboard hold requires the stream to remain alive; the node does not issue a
LAND command on successful completion.

### Why the hold never stops (and why the land detector is ignored)

PX4's `OffboardChecks` (`src/modules/commander/HealthAndArmingChecks/checks/
offboardCheck.cpp`) sets `offboard_control_signal_lost` unless an
`OffboardControlMode` message with a recent `timestamp` arrived within
`COM_OF_LOSS_T`. When that flag is set while `NAVIGATION_STATE_OFFBOARD` is
active, PX4 applies `COM_OBL_RC_ACT` — `4` = LAND in the docking profile.
So any gap in the setpoint stream looks exactly like "the drone switched to
LAND by itself".

Two consequences are baked into this node:

1. Once explicitly acquired, the hold is published on every healthy active
   tick, including Offboard confirmation and experiment completion, with no
   idle timeout and no gating on `VehicleLandDetected.landed`.
2. `VehicleLandDetected.landed` is **not** used as a control gate. On this
   airframe it reports `landed` while hovering (see the `MPC_THR_HOVER`
   discussion in `px4_sitl_docking.parm`), so gating on it stopped the stream
   and caused the LAND failsafe. In-air state comes from height above the EKF
   origin (`AIRBORNE_MIN_HEIGHT_M = 0.5`) instead.

`flight_cycle_manager` also streams every `STREAM_STATES` tick without a
`landed` gate. `--verbose` prints the publish rate in RUN and hold once a
second. A gap longer than `STREAM_GAP_WARN_S` (0.4 s) in hold while in Offboard
prints a `[stream gap]` warning; a RUN scheduling gap over 0.2 s aborts the run.

### WSL clock corrections and the `x_px4_05` failure

The recorded run and PX4 ULog show two effects of wall-clock corrections:

- DDS timestamps jumped forward by about 6.63 s during the run. This ended the
  5 s pre-hover after only 1.42 host seconds; backwards corrections explain
  earlier `FC position clock reset` aborts.
- Commands stopped at host monotonic time 156.042 s while telemetry continued
  until 157.083 s. PX4 set `offboard_control_signal_lost` at FC time 85.280 s
  and entered LAND/failsafe at 85.290 s, consistent with `COM_OF_LOSS_T=1.0`.

Both the stream timer and publisher-conflict watchdog now use ROS
`STEADY_TIME`, so backwards UTC steps and a paused/backwards ROS `/clock`
cannot suspend them. RUN publishes once per timer callback; a second
deadline gate previously dropped early callbacks and reduced the nominal
50 Hz stream to about 40 Hz. Disabling DDS timestamp translation separately
keeps experiment durations and recorded feedback in FC time despite UTC
corrections. Real FC clock resets still abort the experiment.

The regression tests use DDS domain 219 and a process-local clock shim.
They do not change the host clock or send commands to the running vehicle.

One axis per flight (`x`=North, `y`=East, `z`=Down). After acquisition, any mode loss, disarm,
failsafe, stale telemetry, EKF invalidity or estimator reset aborts the run
and stops streaming; PX4's configured failsafe owns recovery if Offboard was
still active. These safety checks also apply during the final HOLD. Mode loss
in READY/RUN/HOLD is latched: the node never re-enters Offboard automatically
and never arms/takes off/lands. Restart the node with a new output directory
for another experiment. Do not increase `COM_OF_LOSS_T` to conceal stream gaps.

## Fit (unchanged fitter)

```bash
python3 /home/enri-ubuntu/drone_ws/src/drone_docking/drone_sysid/fit_plant.py /home/enri-ubuntu/drone_ws/runs/x_px4_02 --timebase fc --out /home/enri-ubuntu/drone_ws/models/x_px4_02
```

Gates (same as ArduPilot): PRBS fit >= 60%, median in-band coherence >= 0.6,
residual correlation <= 0.3 at 0.5 s, cross-axis <= 30%. Exit 0 = review,
exit 2 = reject. Confirm on a new flight at another amplitude before tuning.

ENU mapping for `flight_cycle_manager`: East uses the `y` model, North uses
the `x` model, Up uses the `z` model with command and velocity signs inverted.
Add camera/AprilTag latency separately when tuning the outer PID.

## Layout

- `px4_sysid/excitation.py`: vendored chirp/PRBS schedule (same as `sysid_core.Excitation`).
- `px4_sysid/policy.py`: Offboard stages, freshness gates, latched override.
- `px4_sysid/recorder.py`: `fit_plant.py`-compatible CSVs on FC timebase.
- `px4_sysid/px4_sysid_node.py`: DDS publishers/subscribers + terminal control.

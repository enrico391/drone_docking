# Terminal-started drone system identification

This package identifies the **closed ArduPilot velocity loop seen by the outer
docking PID**. It sends North/East/Down velocity commands and fits measured
velocity. It does not identify motor-to-body dynamics or camera latency.

`log_flight.py` runs on the companion. `fit_plant.py` runs offline on a PC.
Python 3.10+ is required. The package has unit/fake-transport and synthetic-data
tests. Validate the terminal-guided workflow in SITL before using it on hardware.

## Install

Unzip this folder on the companion and PC. From this folder:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests -v
```

The companion logger needs pymavlink and pyserial for serial connections;
the numerical dependencies are used by the fitter and its tests. If a serial
connection reports `No module named 'serial'`, install `pyserial` in the same
Python environment used to run the logger:

```bash
python3 -m pip install pyserial
python3 -c "import serial; print(serial.__file__)"
```

Use a **dedicated routed MAVLink endpoint**. If MAVROS, your docking manager or
MAVProxy already owns `/dev/ttyAMA0`, do not open that UART a second time. Route
an additional UDP endpoint to this process and suspend every other process
that generates movement commands. Keep Mission Planner and RC connected.
The script does not modify FC controller parameters or disable failsafes.
**Never run it alongside `flight_cycle_manager`: both can request GUIDED or
send movement setpoints.** This script is a deliberately
started one-shot experiment, not a background docking service.

## Start and stop

| Event | Behavior |
|---|---|
| Terminal starts while already in GUIDED | Waits for `start` + Enter; no automatic excitation. |
| Terminal starts in another mode | Type `guided` + Enter to request GUIDED once; wait for FC confirmation, then type `start` + Enter. |
| Start/request on the ground or with stale state | Rejects command; fix the cause before retrying. |
| GUIDED request not confirmed within 3 s | Stops without sending setpoints; pilot must take control. |
| Any observed mode change during excitation | Stops sending setpoints; retains that mode; exits. |
| Mode changes away and back in one receive batch | Cancellation remains latched. No restart. |
| Disarm or FC critical/emergency status | Stops sending commands and yields to the FC. |
| Bound exceeded or stale state, with fresh owned GUIDED heartbeat | Requests LOITER once; waits up to 3 s for a confirming heartbeat. |
| Heartbeat lost | Stops all setpoints; does not issue a mode command using stale ownership. |
| Normal completion | After post-hover, requests LOITER once, stops sending setpoints and waits up to 3 s for mode confirmation before exiting. LOITER holds position if the FC has a healthy position estimate. |
| Ctrl+C / SIGTERM during healthy owned GUIDED | Requests LOITER once, checks confirmation, saves logs. |

**Override is detected through FC heartbeat, not directly from the RC switch.**
With a 1 Hz heartbeat, detection may take roughly a second plus link delay.
Configure an accessible RC LOITER/ALT_HOLD/RTL takeover that actually changes
the FC mode. LAND no longer triggers the default workflow. Selecting GUIDED again while it
is already GUIDED is not an observable mode change. This script never re-enters
GUIDED after cancellation.
It never arms, takes off, disarms, or commands an automatic landing.

If the process is killed or the link fails, motion does not stop instantly:
the FC may retain the last setpoint until its configured Guided timeout.
Verify `GUID_TIMEOUT` and your RC/GCS/battery failsafe behavior in SITL and on
your installed firmware; a companion process is not a replacement for those.

## Run an X-axis test

1. Manually take off and establish a **settled hover around 5 m above home**,
   in an open area with clearance exceeding the configured radius. STABILIZE
   does not hold position: stabilize the vehicle before changing modes; LOITER
   is preferable if available. Do not start near the dock.
2. Run the script in an interactive terminal. If Mission Planner already shows
   GUIDED, type `start` and press Enter. Otherwise, type `guided` and press
   Enter; once the FC confirms GUIDED, type `start` and press Enter. The logger
   checks airborne status, fresh telemetry, EKF, altitude and clock sync at
   both commands. It never switches to LAND. If currently in LAND, it may
   descend before GUIDED confirmation: use a safe hover mode instead.
3. Type `quit` or change to LOITER or another takeover mode to cancel.
4. On successful completion, the script requests LOITER and waits up to 3 s
   for confirmation before exiting; verify the FC is holding position and be
   ready to take over if LOITER is not confirmed. One invocation performs one
   axis only. Restart for the next run.

Legacy automatic mode triggers require explicit `--trigger-mode GUIDED` or
`--trigger-mode LAND`. Avoid the LAND trigger with Mission Planner: its repeated
LAND command cancelled a previous run. Neither legacy mode uses terminal start.

Dedicated UDP endpoint (MAVProxy can forward with `output add 127.0.0.1:14551`):

```bash
python3 log_flight.py --port udpin:0.0.0.0:14551 \
  --axis x --amp 0.3 --min-alt 3 --max-alt 10 --radius 15 --out runs/x_01
```

For local SITL instance 0, if this logger is the only client owning its direct
TCP endpoint (start simulation with `use_mavproxy:=false` and do not run the
flight-cycle manager), use a new run directory:

```bash
python3 log_flight.py --port tcp:127.0.0.1:5760 --axis x --out runs/x_terminal_01
```

Dedicated companion UART, only if no other program owns it:

```bash
python3 log_flight.py --port /dev/ttyAMA0 --baud 921600 \
  --axis x --amp 0.3 --out runs/x_01
```

For Y and Z, run separate tests with new output directories:

```bash
python3 log_flight.py --port udpin:0.0.0.0:14551 --axis y --amp 0.3 --out runs/y_01
python3 log_flight.py --port udpin:0.0.0.0:14551 --axis z --amp 0.2 --out runs/z_01
```

All example bounds and amplitudes are starting settings to verify in SITL,
not a flight approval. Increasing excitation does not guarantee better data.
Wait for the FC to report GUIDED and for fresh telemetry before typing `start`.

Default schedule is **5 s hover + 50 s chirp (0.1–2 Hz) + 5 s hover + 45 s
PRBS (0.2 s chips) + 5 s hover = 110 s**. Hover means zero velocity command,
not a fixed position trajectory. PRBS can drift, so bounds matter. X/Y tests
command zero on the two other velocity axes; yaw and yaw rate are ignored.
Flight bounds use horizontal displacement from the start and altitude
**above home**. NED `-z` is altitude above the EKF origin, which may differ.

Before terminal start, the logger also requires at least three low-RTT clock
exchanges with <=20 ms offset spread. Allow a few seconds for synchronization
after starting the process. This default assumes the FC and host clocks run
at the same rate, as on hardware. No usable FC clock synchronization means
no excitation.
Position, attitude and altitude must have advancing FC boot timestamps and
ages <=0.5 s on that clock; newly received backlogged data cannot renew them.
If clock synchronization becomes stale for >5 s, the run aborts. A saturated
receive queue yields control without issuing an uncertain cleanup mode change.

Freshness gates: heartbeat <=2.5 s, position/attitude/relative altitude <=0.5 s,
airborne/EKF status <=2.5 s. Absolute horizontal/vertical EKF position and both
velocity estimates must be healthy. The logger requests 50 Hz position and
attitude, 10 Hz altitude and 2 Hz status; actual rates may be lower.
Unsupported messages or stale data cause entry rejection/abort.

### Gazebo / SITL simulation time

Gazebo can run slower or faster than real time. In that case FC boot time and
host monotonic time advance at different rates, and the default clock check
can report `FC clock synchronization unavailable/stale` despite quick TIMESYNC
replies. Use the explicit SITL-only option:

```bash
python3 log_flight.py --port tcp:127.0.0.1:5760 --axis x \
  --sim-time --out runs/x_sim_01
```

The logger requests TIMESYNC at 10 Hz and estimates the local FC/host clock
rate from three low-RTT replies, rejecting more than 20 ms of fit residual
spread. All three must fall within 0.5 host seconds. Allow a few seconds before
`start`. The excitation's durations,
chirp frequencies and PRBS period then use **FC simulation seconds**. The
command scheduler remains at the requested host-time rate; the fitter checks
the actual sampling rate in FC seconds.

Simulation mode retains arming, airborne, EKF, bounds, heartbeat and telemetry
receipt checks. It requires a usable clock update within 0.5 host seconds and
checks telemetry age on the FC clock. A stalled or reset simulation clock
cancels an active run and requests LOITER if fresh GUIDED ownership remains.
RC mode takeover and normal LOITER completion behave
as in hardware mode. Never use `--sim-time` on the physical drone.

`commands.csv` also records the estimated `fc_boot_s` at each command send.
For simulation runs, `fit_plant.py` selects those command timestamps and FC
telemetry timestamps by default, so delays and time constants are in simulated
seconds even if simulation speed varies. Existing hardware runs retain their
TIMESYNC offset checks and file compatibility. Simulation runs reject an
explicit `--timebase host`, which would distort their frequency and time units.

## Terminal GUIDED handoff

The default is explicit terminal control; it never reacts automatically to LAND:

```bash
python3 log_flight.py --port udpin:0.0.0.0:14551 \
  --axis x --amp 0.3 --out runs/x_terminal_01
```

If already in GUIDED, enter `start`. Otherwise enter `guided`, wait for the
confirmation message, then enter `start`. GUIDED is requested at most once;
if confirmation does not arrive within 3 s or the mode changes unexpectedly,
the script stops without sending setpoints. Run in an interactive terminal;
closed/non-interactive stdin aborts without starting. Use LOITER to cancel at any time.

The script cannot distinguish a deliberate mode switch from one requested by
another controller or failsafe. Start it only for a supervised experiment,
with all other movement-command sources suspended; never leave it running as
a normal always-on service.

## Logged files

| File | Contents |
|---|---|
| commands.csv | Actual send time, phase and all three sent velocities. |
| telemetry.csv | Each new FC-stamped LOCAL_POSITION_NED sample plus host receipt time. |
| attitude.csv | Independent FC-stamped attitude/rates. |
| timesync.csv | Clock-offset exchanges and RTT, including rejected high-RTT samples. |
| events.jsonl | Mode, phase, start/stop, STATUSTEXT, ACKs and requested/observed mode changes. |
| metadata.json | Configuration, axis, origin, completion reason and usable TIMESYNC samples. |

Data is written incrementally and flushed; this is not a guarantee against
power loss. Do not use an existing output directory. Duplicate FC position
timestamps are ignored; a backwards timestamp terminates the run.

Also download and retain the **Pixhawk DataFlash .bin** for the same flight.
Inspect the position controller's desired, shaped target and actual velocity
fields available in your firmware. Field names vary; use the log's FMT
definitions and matching ArduPilot documentation. MAVLink send commands can
be shaped by acceleration/jerk limits. This estimator uses **sent commands**,
so that shaping is deliberately part of the identified outer-loop plant.
It does not yet ingest `.bin` files directly or write guaranteed FC markers.
Record firmware, payload, battery, wind and controller parameters with each
run; keep them fixed while comparing candidate controllers.

## Obtain the model

Copy the entire run directory to the PC, then from this package folder:

```bash
python3 fit_plant.py runs/x_01 --out models/x_01
python3 fit_plant.py runs/y_01 --out models/y_01
python3 fit_plant.py runs/z_01 --out models/z_01
```

Automatic timebase uses FC boot time only if at least five low-RTT TIMESYNC
exchanges agree with the position-message clock and have bounded offset
variation. Otherwise it uses host receipt time and warns: apparent delay then
also contains telemetry transport delay/jitter. Force `--timebase fc` to
refuse this fallback. FC alignment still includes command delivery delay;
neither method proves an exact physical motor delay.

```bash
python3 fit_plant.py runs/x_01 --timebase fc --dt 0.02 --out models/x_fc
```

If actual telemetry is slower, choose a matching `--dt`, e.g. `0.05` for
20 Hz. The loader rejects samples faster than the observed data, time resets,
nonfinite values and gaps >0.15 s. It reconstructs sent commands with previous
value hold and interpolates state onto the analysis grid. No anti-alias
filter is applied; keep the requested/actual rate well above the experiment
band and avoid large downsampling. Interrupted runs are rejected unless
`--allow-partial` is provided; adequate chirp and PRBS data are still required.

The two candidate velocity plants are:

\[
G_v(s)=\frac{K e^{-Ls}}{1+\tau s},\qquad
G_v(s)=\frac{K e^{-Ls}}{(1+\tau_1 s)(1+\tau_2 s)}.
\]

The docking position plant is \(G_p(s)=G_v(s)/s\). The exported bias is a
separate steady velocity offset, not part of the transfer function. Input
delays are searched at integer samples, with resolution `dt`. Both candidate
models fit pre-hover/chirp only; the PRBS is held out for comparison. The
second-order model is selected only for >=5% lower PRBS RMSE. Validation is
therefore also used in model selection: confirm the final model on a new
flight, at another amplitude, before tuning the docking PID.

Outputs in the new model directory:

- `model.json`: K, time constants, delay, velocity/position coefficients,
  discrete coefficients, metrics, timing quality and warnings.
- `diagnostics.png`: measured/predicted velocity and position, frequency
  response, coherence, residuals and cross-axis velocities.
- `resampled.csv`, `frequency_response.csv`, `report.txt`.

Exit code `0` means heuristics allow further review, **not flight approval**.
Exit code `2` means bad input or a model rejected for investigation; if fitting
completed, diagnostics are still written. Review thresholds: PRBS fit >=60%,
median in-band coherence >=0.6, residual correlation <=0.3 at 0.5 s,
cross-axis velocity std <=30% of tested-axis std, no fitted/search boundary.
These are practical screening rules, not statistical confidence intervals.
Real-pole models cannot represent strong oscillations. Poor fits can indicate
wind, nonlinear limiting, inadequate excitation or the need for another model.

NED axes are X=North, Y=East, Z=Down. For your ENU docking code, East uses the
Y-test model, North uses the X-test model and Up uses the Z-test model with
both command and measured velocity signs inverted. This preserves transfer
gain. Add perception latency/noise separately when testing your outer PID.

## Synthetic verification and SITL checks

No aircraft is contacted by these commands:

```bash
python3 tests/synthetic_run.py --out synthetic_run
python3 fit_plant.py synthetic_run --timebase fc --out synthetic_model
```

Synthetic ground truth is K=0.92, tau=0.42 s, L=0.12 s. The verified reference
run recovered approximately K=0.92034, tau=0.42003 s, L=0.12 s.

In your existing ArduCopter SITL, forward a dedicated output to this process.
Manually arm/take off, settle in LOITER, switch LOITER → LAND and verify the
GUIDED handoff before any setpoint and the full schedule. Verify these cases
before hardware: startup in LAND does not request GUIDED or send velocities;
entry while landed is rejected; LOITER/ALT_HOLD/LAND override terminates and
never restarts; LAND handoff waits for GUIDED; completion confirms
LOITER; too-small radius/altitude bounds abort; stale telemetry/link loss
stops commands and the FC's configured timeout/failsafe behaves as expected.
`--sim-time` adapts clock tracking and the experiment timebase for SITL; it
does not bypass the flight health checks.

## Primary references

- [ArduPilot Guided movement commands and velocity mask](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [ArduPilot individual message interval requests](https://ardupilot.org/dev/docs/mavlink-requesting-data.html)
- [MAVLink clock synchronization](https://mavlink.io/en/services/timesync.html)
- [ArduPilot log message documentation](https://ardupilot.org/copter/docs/logmessages.html)

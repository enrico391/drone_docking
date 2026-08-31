# Flight Cycle Manager Separation Design

## Purpose

Replace `drone_mission_manager` with `flight_cycle_manager` and establish a
strict responsibility boundary between the ROS 2 companion and ArduPilot.

ArduPilot owns the stored AUTO mission, the return-to-dock route, and the
last-resort `NAV_LAND`. The companion owns the explicit flight-cycle start,
vertical takeoff, transfer into the stored mission, final docking takeover,
visual target search, and the selected precision-landing procedure. A pilot or
ArduPilot failsafe always has higher authority than the companion.

This change removes companion-owned mission geometry, circle execution, and
fixed-delay repetition. It does not implement battery charging. A later
charging gate can be inserted between `DOCKED` and the next explicit cycle
start without changing the flight-control boundary.

## Selected Approach

Create a clean replacement package named `flight_cycle_manager`. Reuse focused
transport, control, and landing code where it still matches the new ownership
model, but replace the existing mission-profile parser and circle-flight state
machine.

The old package, executable, node name, typed mission YAML files, mission
selector, and repeat configuration will be removed after migration. No
compatibility executable or deprecated package alias will remain.

## Responsibility Boundary

### ArduPilot

- Stores and executes the uploaded AUTO mission.
- Executes the normal and emergency return-to-dock route.
- Provides mission progress, vehicle mode, armed state, landed state, position,
  velocity, attitude, and command acknowledgements.
- Executes the validated fallback `NAV_LAND` if companion docking cannot start.
- Retains its configured RC, GCS, battery, EKF, and other safety behavior.

### Flight cycle manager

- Downloads and validates, but never uploads or modifies, the onboard mission.
- Accepts an explicit request to start one flight cycle.
- Performs a simple vertical GUIDED takeoff to a configured altitude.
- Deterministically resets and starts the stored AUTO mission.
- Requests early docking through `MAV_CMD_DO_LAND_START`.
- Detects the validated landing handoff and takes control in GUIDED.
- Runs a shared bounded-circle dock search for both landing backends.
- Runs either companion PID landing or ArduPilot-native precision landing.
- Returns control to the validated onboard `NAV_LAND` after a pre-landing
  failure.
- Becomes permanently passive for the current flight after an unexpected mode
  change.

### Pilot and ArduPilot failsafes

The pilot or a flight-controller failsafe may change flight mode at any time.
Any unexpected mode change while airborne is an authority transfer away from
the companion. The manager must not distinguish a pilot-selected mode from a
failsafe-selected mode when deciding whether to stop commanding the aircraft.

The authority order is:

```text
ArduPilot failsafe or RC pilot
            >
Flight cycle manager
            >
Stored AUTO mission
```

## Package Architecture

The package contains one ROS 2 lifecycle orchestrator and focused internal
components:

```text
flight_cycle_manager
├── FlightCycleManager
├── MissionContract
├── MavlinkVehicle
├── DockSearchController
├── PidLandingBackend
└── PreciseLandingBackend
```

Proposed layout:

```text
flight_cycle_manager/
├── flight_cycle_manager/
│   ├── __init__.py
│   ├── manager.py
│   ├── mission_contract.py
│   ├── pymavlink_vehicle.py
│   ├── dock_search.py
│   ├── landing_backend.py
│   ├── pid_landing_backend.py
│   ├── precise_landing_backend.py
│   ├── pid.py
│   ├── control.py
│   ├── landing_target.py
│   └── landing_wait.py
├── config/
│   ├── flight_cycle_manager.yaml
│   └── pid_landing.yaml
├── launch/
│   └── flight_cycle.launch.py
├── test/
├── package.xml
├── setup.cfg
└── setup.py
```

`FlightCycleManager` is the only component allowed to change flight modes,
arm, take off, start or reposition the mission, request docking, or disarm.
Controllers return decisions or setpoints; they do not send MAVLink commands
directly.

`MissionContract` represents the immutable result of one successful onboard
mission validation. `MavlinkVehicle` owns the single pymavlink connection and
implements non-blocking command and mission-protocol transactions.

`DockSearchController` is a pure controller shared by both landing backends.
Landing backends begin only after shared target acquisition and do not own
mission validation, takeoff, search, or manual-override policy.

## Onboard Mission Contract

Before each cycle start, the manager downloads the currently stored mission
using the MAVLink mission protocol:

```text
MISSION_REQUEST_LIST
MISSION_COUNT
MISSION_REQUEST_INT <-> MISSION_ITEM_INT
MISSION_ACK
```

The manager never sends mission upload, partial-write, clear, or item-update
messages.

A valid docking mission contains exactly one unambiguous sequence:

```text
DO_LAND_START
    zero or more valid return or approach items
NAV_LOITER_TIME
NAV_LAND
```

Validation requires all of the following:

1. The mission is present and every downloaded item is received exactly once.
2. Exactly one `DO_LAND_START` exists.
3. A `NAV_LOITER_TIME` occurs after that marker.
4. `NAV_LAND` immediately follows the handoff loiter.
5. The loiter duration is at least `mission_handoff_min_loiter_s`.
6. The `NAV_LAND` destination either explicitly uses the current position or
   matches the loiter/dock location within
   `mission_landing_location_tolerance_m`.
7. The handoff and fallback sequence numbers are valid for the downloaded
   mission.

Missing, duplicate, malformed, out-of-order, or ambiguous landing structures
produce `INVALID_ONBOARD_MISSION`. The manager remains disarmed and rejects
`~/start_cycle` with a precise reason.

The mission is re-downloaded and revalidated before every flight cycle so a
mission changed while docked cannot leave stale handoff indices. Editing the
mission while armed is an operational violation. If an in-flight identity or
count change is detectable, the manager requests RTL and commands immediate
LAND if RTL is rejected; it does not trust the old docking contract.

## ROS Interfaces

Package, executable, and node names are all `flight_cycle_manager`.

### Services

`~/start_cycle` uses `std_srvs/srv/Trigger`.

- Accepted only while disarmed and in a startable state.
- Re-downloads and validates the mission before any arming action.
- Requires fresh heartbeat, position, landed state, and successful pre-arm
  checks.
- Rejects requests while an airborne pilot-override latch is active.

`~/request_dock` uses `std_srvs/srv/Trigger`.

- Accepted during stored AUTO mission execution.
- Sends `MAV_CMD_DO_LAND_START` and verifies its acknowledgement and resulting
  AUTO/AUTO_RTL mission progression.
- Returns success without duplicating commands if docking has already begun.
- Rejects requests during takeoff, after manual takeover, and after landing is
  already complete.

There is no service that clears `PILOT_OVERRIDE` while armed.

### Topics

- `~/state`: reliable, transient-local lifecycle state.
- `~/reason`: reliable, transient-local explanation for the latest material
  transition, rejection, or failure.
- `/dock/pose`: existing filtered dock-pose input.
- `/dock/mode`: existing target-selection input.

## Configuration Ownership

`config/flight_cycle_manager.yaml` contains only common lifecycle and native
landing configuration:

- `landing_backend`: `pid` or `precise_landing`.
- MAVLink connection, target/source IDs, baud, retry periods, and timeouts.
- Takeoff altitude, altitude tolerance, vertical-speed tolerance, and stable
  dwell time.
- Mission validation limits and minimum loiter handoff duration.
- Shared search radius, tangential speed, radial correction gain, velocity
  limits, target-stability time, timeout, and spatial bound.
- Dock-pose topic names and freshness limits.
- Native `LANDING_TARGET`, yaw correction, warmup, touchdown, and disarm
  settings.
- Telemetry-health and manual-override timing.
- `simulation_mode` and `auto_start`, both defaulting to false.

`auto_start=true` is valid only when `simulation_mode=true`. Production starts
require `~/start_cycle`.

`config/pid_landing.yaml` contains only PID-backend configuration:

- X, Y, Z, and yaw PID gains.
- Integral limits and derivative filter constants.
- Coarse and fine alignment tolerances.
- Outer, central, recovery, and touchdown descent speeds.
- Central-tag transition and target-loss recovery limits.
- Touchdown commitment and PID-specific disarm behavior.

The PID file contains no route geometry, mission selection, takeoff settings,
shared search settings, or MAVLink connection configuration. The launch file
loads it when `landing_backend=pid`.

## Lifecycle

The principal state flow is:

```text
VALIDATING_MISSION
        |
        v
READY -- start_cycle --> PREARM --> SET_GUIDED --> ARM --> TAKEOFF
                                                            |
                                                            v
                                                   WAIT_TAKEOFF_STABLE
                                                            |
                                                            v
                                                   RESET_ONBOARD_MISSION
                                                            |
                                                            v
                                                       START_AUTO
                                                            |
                                                            v
                                                       AUTO_MISSION
                                                  /                     \
                                     normal progression          request_dock
                                                  \                     /
                                                   v                   v
                                                    WAIT_FOR_HANDOFF
                                                            |
                                                            v
                                                    GUIDED_TAKEOVER
                                                            |
                                                            v
                                                       SEARCH_DOCK
                                                  /                     \
                                      target acquired                timeout
                                              |                         |
                                              v                         v
                                    SELECTED_BACKEND              RETURN_TO_FC
                                              |                         |
                                              v                         v
                                  WAIT_LANDED / WAIT_DISARM     FC_FALLBACK_LAND
                                                  \                     /
                                                   v                   v
                                                         DOCKED
```

Terminal or passive states also include `INVALID_ONBOARD_MISSION`,
`PILOT_OVERRIDE`, `LINK_LOSS`, and `FAILED_DISARMED`. An active airborne error
must resolve to pilot/failsafe authority, the stored `NAV_LAND`, RTL, or direct
LAND. It must not remain indefinitely in GUIDED while sending zero velocity.

## Takeoff

Takeoff remains deliberately simple:

1. Confirm the vehicle is disarmed and reports `ON_GROUND`.
2. Run pre-arm checks.
3. Enter GUIDED and verify the heartbeat mode.
4. Arm and verify the armed heartbeat flag.
5. Send one `MAV_CMD_NAV_TAKEOFF` to `takeoff_altitude_m`.
6. Require altitude error within `takeoff_altitude_tolerance_m` and absolute
   vertical speed below `takeoff_vertical_speed_tolerance_m_s` for
   `takeoff_stable_time_s`.

No companion X/Y/yaw takeoff PID is added. ArduPilot's GUIDED takeoff behavior
holds the horizontal position while ascending.

## Stored Mission Start

After stable takeoff, the manager performs this acknowledged sequence:

```text
MAV_CMD_DO_SET_MISSION_CURRENT
    param1 = 0
    param2 = 1
wait COMMAND_ACK=ACCEPTED

MAV_CMD_MISSION_START
    param1 = 0
    param2 = 0
wait COMMAND_ACK=ACCEPTED

verify HEARTBEAT mode=AUTO
verify MISSION_CURRENT mission_state=ACTIVE
verify MISSION_CURRENT mission_mode=mission mode when supported
```

This sequence makes restart behavior independent of `MIS_RESTART` and resets
mission jump counters. The manager does not use the superseded
`MISSION_SET_CURRENT` message.

`MISSION_CURRENT` is requested periodically and is the persistent source for
mission sequence and mission state. `MISSION_ITEM_REACHED` may accelerate a
transition but is never the sole trigger.

## Normal and Early Docking

During normal execution, ArduPilot progresses through `DO_LAND_START` and the
stored return/approach route. An accepted `~/request_dock` sends:

```text
MAV_CMD_DO_LAND_START
wait COMMAND_ACK=ACCEPTED
verify AUTO/AUTO_RTL and landing-sequence progression
```

Normal and early docking therefore converge on the same flight-controller-owned
route. `DO_LAND_START` is an entry marker, not the final companion handoff.

The persistent handoff condition is `MISSION_CURRENT.seq` equal to the
validated `NAV_LOITER_TIME` index.

## Shared Dock Search

Reaching the handoff transfers control to the manager even when no visual
target is initially available:

1. Confirm fresh heartbeat, local position, and landed-state telemetry.
2. Enter GUIDED and verify the mode.
3. Capture current local X/Y and altitude as the search center.
4. Run a bounded horizontal circle while holding the captured altitude.
5. Require a valid dock target for `search_target_stable_time_s`.

`DockSearchController` returns body- or map-frame velocity decisions to the
orchestrator and has no transport dependency. It applies tangential search
velocity, radial correction toward the configured radius, altitude correction,
and total speed limits. It fails if time or maximum displacement bounds are
exceeded.

Both landing backends use exactly this search controller and configuration.

## PID Landing Backend

Shared search removes the PID controller's duplicate initial acquisition and
search phase. After acquisition, PID landing begins with alignment and retains
the existing useful phases:

- coarse and fine alignment;
- outer descent;
- central-tag acquisition and transition;
- recovery climb after target loss;
- final central descent;
- touchdown commitment;
- landed-state confirmation and normal disarm.

Target loss before touchdown commitment uses bounded PID recovery. Exhausting
recovery before commitment returns control to the validated `NAV_LAND`.
Failure after touchdown commitment commands direct ArduPilot LAND; it does not
climb or rejoin the stored approach route.

## Native Precision-Landing Backend

After shared acquisition, the manager remains in GUIDED and holds position
while streaming each fresh `LANDING_TARGET` measurement. It requires the
configured warmup interval, may send the existing one-shot dock-relative yaw
correction, then commands LAND and verifies the mode transition.

The stream continues through touchdown and disarm. Once LAND is accepted,
target-loss recovery belongs to ArduPilot and the configured `PLND_STRICT`,
`PLND_RET_MAX`, and related settings. The manager does not replace that policy.

Failure before LAND acceptance returns control to the validated `NAV_LAND`.
After LAND acceptance, the manager remains in the native landing flow unless a
pilot or failsafe changes mode.

## Deterministic Fallback Handback

A shared-search timeout or pre-landing backend failure uses:

```text
MAV_CMD_DO_SET_MISSION_CURRENT
    param1 = validated NAV_LAND sequence
    param2 = 1
wait COMMAND_ACK=ACCEPTED

MAV_CMD_MISSION_START
    param1 = 0
    param2 = 0
wait COMMAND_ACK=ACCEPTED

verify HEARTBEAT mode=AUTO
verify MISSION_CURRENT at the expected NAV_LAND sequence
```

Using reset plus an explicit sequence makes the handback deterministic even if
`MIS_RESTART=1`. If selection, start, or confirmation fails, the manager sends
`MAV_CMD_NAV_LAND` at the current position. It then monitors touchdown and
disarm without attempting another docking takeover.

## Manual RC and Failsafe Takeover

Each airborne state defines its expected ArduPilot modes. A mode transition is
expected only while the manager has a matching mode-change transaction in
progress or while ArduPilot is performing the validated AUTO/AUTO_RTL mission.

Any other mode change enters latched `PILOT_OVERRIDE` behavior:

- stop velocity and position setpoints;
- stop mode, mission, arm, disarm, and landing commands;
- stop `LANDING_TARGET` transmission;
- continue receiving telemetry and publishing state/reason;
- never resume automation because the mode returns to AUTO, GUIDED, or LAND;
- clear the flight latch only after disarm and acceptance of a new explicit
  `~/start_cycle` request.

The manager applies this behavior equally to an RC mode switch and an
ArduPilot-selected failsafe mode. It never commands the aircraft back out of a
pilot or failsafe mode.

Manual LAND repositioning remains an ArduPilot configuration choice through
`LAND_REPOSITION` and `PILOT_THR_BHV`; the manager does not mutate those
parameters.

## Failure Policy

The ordered failure responses are:

| Condition | Response |
| --- | --- |
| Unexpected airborne mode change | Latch `PILOT_OVERRIDE`; stop all output |
| MAVLink heartbeat/link loss | Enter `LINK_LOSS`; rely on ArduPilot failsafe |
| Search timeout or pre-landing backend failure | Hand back to validated AUTO `NAV_LAND` |
| `NAV_LAND` handback rejected or unconfirmed | Command direct LAND |
| PID failure after touchdown commitment | Command direct LAND |
| Invalid mission while disarmed | Reject start and remain disarmed |
| Detectable mission change while airborne | Request RTL; direct LAND if rejected |
| Touchdown without disarm | Continue bounded warning/retry behavior appropriate to the active FC landing mode |

No link-loss state attempts new MAVLink recovery commands because command
delivery cannot be established. Hardware deployment therefore requires tested
ArduPilot GCS and RC failsafe settings.

## Completion and Future Charging

Touchdown requires fresh, stable `MAV_LANDED_STATE_ON_GROUND` telemetry.
Completion requires a heartbeat reporting `armed=false`. The final state is
`DOCKED`.

The existing `repeat.count` and `repeat.delay_s` behavior is removed. Every
subsequent flight requires a fresh explicit `~/start_cycle` request. A future
charging implementation will add charger contact, charge-state stability, and
battery/pre-arm gates before making that request eligible; it will not restore
a timer-only restart.

If a previous flight triggered an ArduPilot battery failsafe, the charging
design must also account for the flight controller's latched failsafe/reboot
requirements before allowing another arm.

## Migration

Implementation will:

1. Add the new package and its common/PID configuration files.
2. Port and test reusable pure control and landing modules.
3. Extend the MAVLink transport with mission download/progress and the new
   acknowledged commands.
4. Implement mission validation, shared search, both backend adapters, and the
   lifecycle orchestrator.
5. Update `drone_bringup`, launch files, package dependencies, examples, and
   documentation to use `flight_cycle_manager`.
6. Remove typed mission loaders, checked-in companion mission profiles, circle
   execution, return-to-dock execution, and repeat logic.
7. Remove `drone_mission_manager` only after the replacement package passes its
   unit and SITL verification.

Dock localization and vehicle-description behavior remain unchanged except for
necessary launch/package references.

## Verification

### Unit and transport tests

- Valid mission-contract extraction.
- Missing, duplicate, malformed, and out-of-order contract rejection.
- Insufficient loiter duration and mismatched landing-location rejection.
- Mission download retries, duplicates, timeouts, ACKs, and unsupported frames.
- `MISSION_CURRENT` and `MISSION_ITEM_REACHED` parsing.
- Deterministic reset/start and fallback handback transactions.
- Search-circle geometry, altitude hold, speed bounds, acquisition stability,
  displacement bound, and timeout.
- PID and native backend entry, completion, and failure decisions.
- Manual takeover from AUTO, AUTO_RTL, GUIDED search, PID, native warmup, and
  LAND.
- Override latching and rejection of automatic reacquisition.
- Invalid mission blocking all arming/takeoff operations.
- Every airborne failure resolving without indefinite zero-velocity GUIDED
  hover.

### Launch and configuration tests

- New package, executable, node, and private interface names.
- Common configuration loading for both backends.
- Conditional PID YAML loading only for `landing_backend=pid`.
- Rejection of `auto_start=true` outside simulation mode.
- Removal of mission-profile and repetition launch arguments.

### SITL acceptance scenarios

1. Vertical takeoff, stored AUTO mission, handoff, search, and PID landing.
2. Vertical takeoff, stored AUTO mission, handoff, search, and native precision
   landing.
3. `~/request_dock` entering the stored `DO_LAND_START` route.
4. Search timeout returning to the onboard `NAV_LAND`.
5. Failed mission handback producing direct LAND.
6. RC mode takeover during AUTO approach, GUIDED PID landing, and native LAND.
7. No manager reacquisition after the RC switch returns to an autonomous mode.
8. Invalid onboard mission refusing to arm.
9. Successful touchdown and disarm reaching `DOCKED` with no automatic repeat.

## Acceptance Criteria

The separation is complete when:

- no companion YAML or state machine defines or flies the operational route;
- ArduPilot is the sole owner of stored mission execution and return approach;
- the manager blocks takeoff without the validated docking/fallback contract;
- the manager performs only vertical takeoff before starting AUTO;
- both landing backends share the same manager-owned search and handoff;
- loss of visual acquisition returns to the FC `NAV_LAND` fallback;
- manual or failsafe mode changes permanently remove manager authority for that
  flight;
- both backends pass unit and SITL landing/takeover scenarios;
- the old `drone_mission_manager` package and fixed-delay repetition are gone.

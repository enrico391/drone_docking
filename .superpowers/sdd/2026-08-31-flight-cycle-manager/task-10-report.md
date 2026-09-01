# Task 10 Report: Latch Pilot Authority, Link Loss, and Completion

## Scope

- Date: 2026-09-01
- Branch: `flight_cycle_manager`
- Workspace: `/home/morowsl/ardu_ws/src/drone_docking`
- Preserved the pre-existing untracked `.AGENTS`; it was not edited or staged.
- Recovered from an uncommitted partial state in:
  - `flight_cycle_manager/flight_cycle_manager/manager.py`
  - `flight_cycle_manager/test/vehicle_fakes.py`
  - `flight_cycle_manager/test/test_manager_authority.py`
- Treated every partial change as untrusted. I audited the live diff against the
  Task 10 brief, reran tests before accepting any behavior, and corrected the
  implementation where the partial state diverged from the brief.

## Recovery audit

### Partial state found

- The partial manager diff already introduced passive `PILOT_OVERRIDE`,
  passive `LINK_LOSS`, mission-integrity tracking, touchdown completion, and
  deferred-command scaffolding.
- `FakeVehicle` had been tightened so only one MAVLink-like operation can be
  active at a time, exposing overlap that earlier counter-first fakes masked.
- `test_manager_authority.py` already existed with broad authority coverage,
  but one timeout expectation still assumed the fallback `LAND` could start
  immediately even if the transport layer was still busy.

### Problems found during audit

1. The real deferred finding reproduced: when the manager timed out an RTL
   request, it cleared manager-owned pending state before the fake transport
   cleared its live operation, then dropped the fallback `LAND` after receiving
   an `operation already active` error.
2. The expected-mode gate was too broad. It incorrectly treated legitimate
   in-progress FC states as pilot takeover:
   - grounded fallback ticks after a prearm validation failure,
   - `START_AUTO` while still waiting for `AUTO` after mission-start ACK,
   - `RETURN_TO_FC` while still waiting for `AUTO` after fallback-start ACK,
   - `AUTO_RTL` while the stored landing route was already active,
   - `LAND` during the native precise-landing backend's committed path.

## RED then GREEN evidence

### RED from the recovered partial state

Audit run against the recovered authority suite:

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_authority.py -q
```

Result:

```text
1 failed, 35 passed in 5.44s
```

Failing case: `test_mission_integrity_rtl_timeout_directs_land_once`
showed that a timed-out RTL never produced the required fallback `LAND`.

### New RED for the timeout fix

I tightened that timeout test to the real boundary contract: if RTL times out
while the transport is still busy, `LAND` must be deferred and then started
exactly once when the transport clears.

Focused RED:

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_authority.py -q \
  -k 'mission_integrity_rtl_timeout_defers_direct_land_until_transport_clears or \
pid_disarm_timeout_defers_direct_land_until_live_transport_clears'
```

Result:

```text
1 failed, 1 passed, 34 deselected in 0.63s
```

The new timeout regression failed because the manager still dropped the queued
fallback `LAND` after the live RTL operation finished late.

### Existing-suite RED for the mode matrix

After the timeout fix, I reran the broader manager suites to pressure-test the
partial authority implementation:

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_start.py \
  flight_cycle_manager/test/test_manager_docking.py \
  flight_cycle_manager/test/test_manager_authority.py -q
```

Result:

```text
6 failed, 72 passed in 21.54s
```

Those failures were the useful mutation check for the partial implementation:
the too-strict mode gate broke valid start, docking, and FC fallback flows.

### GREEN

Focused timeout regressions after the minimal command-deferral fix:

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_authority.py -q \
  -k 'mission_integrity_rtl_timeout_defers_direct_land_until_transport_clears or \
pid_disarm_timeout_defers_direct_land_until_live_transport_clears'
```

Result: `2 passed, 34 deselected in 0.62s`.

Focused mode-matrix regressions after narrowing authority to actual airborne
ownership and legitimate FC/native-landing modes:

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_start.py \
  flight_cycle_manager/test/test_manager_docking.py -q \
  -k 'stale_heartbeat_with_cached_ground_flags_commands_direct_land_once or \
stale_landed_state_with_cached_ground_flags_commands_direct_land_once or \
mission_confirmation_timeout_commands_direct_land or \
wait_for_handoff_accepts_auto_rtl_approach_progress or \
handback_confirmation_timeout_commands_direct_land_once or \
postcommit_backend_failure_commands_direct_land_once'
```

Result: `6 passed, 36 deselected in 1.14s`.

## Expected-mode matrix

Authority checks now run only after `poll()` and only while the vehicle is
still airborne (`state.flying is True`). Once fresh `ON_GROUND` telemetry
arrives, the manager stops airborne authority policing and uses touchdown /
disarm completion instead.

| Lifecycle state | Expected mode(s) | Notes |
| --- | --- | --- |
| `WAIT_TAKEOFF_STABLE` | `GUIDED` | Manager still owns takeoff stabilization. |
| `RESET_ONBOARD_MISSION` | `GUIDED` | Manager owns mission reset before AUTO start. |
| `START_AUTO` before mission-start ACK | `GUIDED` | `start_mission` still pending. |
| `START_AUTO` after mission-start ACK, before confirmation | `GUIDED`, `AUTO` | FC may still be changing modes; timeout can still escalate to direct `LAND`. |
| `AUTO_MISSION` | `AUTO` | Normal mission progression. |
| `AUTO_MISSION` during stored landing-route progression | `AUTO`, `AUTO_RTL` | Preserves ArduPilot approach progress before handoff. |
| `WAIT_FOR_HANDOFF` | `AUTO`, `AUTO_RTL` | FC still owns validated approach authority. |
| `GUIDED_TAKEOVER` | `GUIDED` | `handoff_guided` pending mode ACK is exempt. |
| `SEARCH_DOCK` | `GUIDED` | Manager owns search velocity outputs. |
| `LANDING_BACKEND` with PID backend | `GUIDED` | PID backend remains manager-guided until completion/fallback. |
| `LANDING_BACKEND` with precise landing backend | `GUIDED`, `LAND` | Native landing may legitimately already be in `LAND`. |
| `LANDING_BACKEND` during a native `LAND` request | prior mode plus `LAND` | Allows FC transition without latching override. |
| `RETURN_TO_FC` before fallback-start ACK | `GUIDED` | Manager has selected fallback mission but not restarted it yet. |
| `RETURN_TO_FC` after fallback-start ACK, before confirmation | `GUIDED`, `AUTO` | FC may still be changing modes; timeout can still escalate to direct `LAND`. |
| `FC_FALLBACK_LAND` after mission-integrity RTL request | current mode plus `AUTO_RTL`, `LAND` | Accepts both FC fallback modes while ceding landing authority. |
| `FC_FALLBACK_LAND` during direct `LAND` fallback | current mode plus `LAND` | Allows the final FC mode change without pilot-override latching. |
| `WAIT_LANDED` | `LAND` | FC owns the final descent until touchdown. |

Passive state rules implemented:

- Unexpected airborne mode change latches `PILOT_OVERRIDE` immediately and
  emits no manager flight outputs on that detection tick.
- Stale heartbeat while airborne latches `LINK_LOSS` immediately and emits no
  recovery commands.
- Passive states never reacquire authority just because the mode returns.
- The latch clears only after fresh `ON_GROUND`, then disarmed, then a new
  explicit `~/start_cycle` request.

## Deferred-finding disposition

The deferred Task 9 finding was real against the corrected fake transport
boundary.

### Reproduction

- `FakeVehicle` now begins the operation before incrementing command counters,
  so a second command cannot pretend to succeed after an overlap rejection.
- With that fake in place, `mission_integrity_rtl_timeout_defers_direct_land_until_transport_clears`
  failed in RED: manager timeout cleared `self._pending`, but the vehicle still
  owned a live RTL operation and rejected `land_async()` as already active.

### Root cause

- Manager-owned pending state and transport-owned pending state were not the
  same thing.
- `_begin_priority_command()` deferred only when `self._pending` still existed.
- After manager-side timeout, an `already active` transport error was treated as
  a terminal direct-`LAND` start failure instead of as "retry once the lower
  layer clears."

### Fix

- Added transport-busy detection in `_begin_priority_command()`.
- Priority commands now queue in `_deferred_command` whenever the manager or the
  transport boundary still has a live operation.
- On the next tick after the lower layer clears, the deferred command starts
  exactly once.
- The same fix preserves the PID-disarm timeout path as well as mission-integrity
  RTL timeout/rejection escalation.

## Files changed

- Modified: `flight_cycle_manager/flight_cycle_manager/manager.py`
- Modified: `flight_cycle_manager/test/vehicle_fakes.py`
- Added: `flight_cycle_manager/test/test_manager_authority.py`
- Added: `.superpowers/sdd/2026-08-31-flight-cycle-manager/task-10-report.md`

## Verification commands and results

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_start.py \
  flight_cycle_manager/test/test_manager_docking.py \
  flight_cycle_manager/test/test_manager_authority.py -q
```

Result: `78 passed in 21.58s`.

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_pymavlink_vehicle.py \
  flight_cycle_manager/test/test_pid_landing_backend.py \
  flight_cycle_manager/test/test_precise_landing_backend.py \
  flight_cycle_manager/test/test_manager_ros_smoke.py -q
```

Result: `73 passed in 0.87s`.

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test -q
```

Result: `232 passed in 21.95s`.

```text
source /opt/ros/humble/setup.bash && \
colcon build --packages-select flight_cycle_manager --symlink-install
```

Result:

```text
Summary: 1 package finished [2.42s]
```

```text
source /opt/ros/humble/setup.bash && \
colcon test --packages-select flight_cycle_manager --event-handlers console_direct+ && \
colcon test-result --verbose --test-result-base build/flight_cycle_manager
```

Result:

```text
232 passed in 11.11s
Summary: 1 package finished [13.4s]
Summary: 232 tests, 0 errors, 0 failures, 0 skipped
```

```text
git -C /home/morowsl/ardu_ws/src/drone_docking diff --check
```

Result: exit `0`, no output.

## Self-review and concerns

- Rechecked that every authority decision now happens after telemetry poll and
  before any manager-owned flight output.
- Rechecked that pilot override and link loss are passive latches, not
  auto-recovery states.
- Rechecked that touchdown completion requires fresh `ON_GROUND` followed by
  a later disarmed observation before entering `DOCKED`, and that `DOCKED`
  does not reissue commands until a fresh explicit start.
- Rechecked that the deferred transport-boundary fix covers both manager-owned
  pending overlap and lower-layer overlap exposed by the fake boundary.
- Minor concern: transport-busy deferral currently keys off the standard
  `already active` RuntimeError text shared by the real and fake vehicle
  boundaries. The behavior is covered by regression tests, but the string
  contract should stay aligned if either boundary changes its wording later.

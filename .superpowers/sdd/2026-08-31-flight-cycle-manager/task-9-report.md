# Task 9 Report: Dock Request, Handoff, Search, Backends, and FC Fallback

## Scope

- Date: 2026-08-31
- Branch: `flight_cycle_manager`
- Workspace: `/home/morowsl/ardu_ws/src/drone_docking`
- Preserved the pre-existing untracked `.AGENTS`; it was not edited or staged.
- Extended the Task 8 manager with the Task 9 docking path only. Manual
  override, link-loss, mission-integrity latching, and terminal completion
  policy remain for Task 10.

## RED then GREEN evidence

### RED

After adding the new docking manager tests first, before implementing the
docking lifecycle, I ran:

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_docking.py -q
```

Result:

```text
ImportError: cannot import name 'AUTO_RTL_MODE' from 'flight_cycle_manager.manager'
```

This was the expected Task 9 RED: the new docking states, request service, and
handoff path were not wired into the Task 8 manager yet.

### GREEN

After implementing the manager changes and tightening the new tests so they
kept telemetry and dock-target timestamps fresh through each intended action, I
reran the focused docking suite:

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_docking.py -q
```

Result: `16 passed in 2.58s`.

## Files changed

- Modified: `flight_cycle_manager/flight_cycle_manager/manager.py`
- Modified: `flight_cycle_manager/test/vehicle_fakes.py`
- Added: `flight_cycle_manager/test/test_manager_docking.py`
- Added: `.superpowers/sdd/2026-08-31-flight-cycle-manager/task-9-report.md`

## Routing and behavior implemented

- Added non-blocking `~/request_dock` admission. It accepts only from
  `AUTO_MISSION`, becomes idempotent once docking has started, and queues only
  one `request_dock_async()` transaction.
- Added AUTO approach waiting that treats both `AUTO` and ArduCopter
  `AUTO_RTL` as expected while the stored landing route progresses toward the
  validated handoff loiter.
- Added normal handoff without a dock request: persistent
  `MISSION_CURRENT.seq == contract.handoff_seq` now starts GUIDED takeover.
- Added verified GUIDED handoff with fresh ENU center capture, then one shared
  `DockSearchController` for both landing backends.
- Routed shared search `SEARCHING` decisions through
  `send_velocity(VelocityFrame.MAP_ENU, ...)`.
- Started the configured backend exactly once on `SearchOutcome.ACQUIRED`.
- Routed backend velocity, yaw correction, native LAND request, PID disarm
  request correlation, and `LANDING_TARGET` streaming from the manager only.
- Limited `LANDING_TARGET` transmission to one send per fresh dock-pose
  generation.
- Added deterministic fallback handback for search timeout and backend
  precommit failure:
  `set_mission_current_async(fallback_land_seq, reset=True)` then
  `start_mission_async()`, followed by fresh post-ACK `AUTO` plus expected
  `MISSION_CURRENT` confirmation.
- Any handback rejection or confirmation timeout now escalates to direct LAND
  once. Backend postcommit failure also escalates directly to LAND.
- Retained the Task 8 post-ACK heartbeat / `MISSION_CURRENT` baselines and the
  conservative airborne failure routing. No Task 10 manual-authority policy was
  implemented early.

## Verification commands and results

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_manager_start.py \
  flight_cycle_manager/test/test_manager_docking.py -q
```

Result: `39 passed in 5.50s`.

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test/test_pymavlink_vehicle.py \
  flight_cycle_manager/test/test_pid_landing_backend.py \
  flight_cycle_manager/test/test_precise_landing_backend.py \
  flight_cycle_manager/test/test_dock_search.py \
  flight_cycle_manager/test/test_landing_target.py \
  flight_cycle_manager/test/test_manager_ros_smoke.py -q
```

Result: `104 passed in 0.65s`.

```text
source /opt/ros/humble/setup.bash && \
PYTHONPATH=flight_cycle_manager:${PYTHONPATH} /usr/bin/python3 -m pytest \
  flight_cycle_manager/test -q
```

Result: `193 passed in 5.70s`.

```text
git diff --check
```

Result: exit `0`, no output.

## Self-review and concerns

- Rechecked that the manager is the only component issuing mode, mission,
  disarm, LAND, velocity, yaw, and `LANDING_TARGET` transport commands.
- Rechecked that every Task 9 airborne failure path now leaves GUIDED via
  validated FC handback or direct LAND, rather than hovering indefinitely on
  zero velocity.
- Rechecked that the fallback confirmation path still requires fresh post-ACK
  telemetry and does not trust pre-ACK AUTO or `MISSION_CURRENT`.
- Concern: successful touchdown/disarm completion and manual/link-loss policy
  are intentionally still incomplete here because the task brief explicitly
  reserved them for Task 10.

## Fix Round 1

### Findings addressed

1. Accepted `~/request_dock` race: `WAIT_FOR_HANDOFF` could observe
   `handoff_seq` before issuing the queued `DO_LAND_START`, so the manager
   entered `GUIDED_TAKEOVER` without ever sending the required one-time dock
   request.
2. FC-owned approach mode guard: with fresh airborne telemetry,
   `WAIT_FOR_HANDOFF` must treat only `AUTO` and `AUTO_RTL` as valid FC-owned
   approach modes. Any other airborne mode becomes passive
   `PILOT_OVERRIDE` immediately, with zero additional flight commands.

### Authority ruling applied

- In `WAIT_FOR_HANDOFF`, fresh telemetry means the flight controller still owns
  the approach only while `mode in {AUTO, AUTO_RTL}`.
- An unexpected fresh airborne mode now transitions directly to
  `PILOT_OVERRIDE` with reason text naming the mode and sends no manager-owned
  flight commands.
- This round does not generalize manual or link-loss policy beyond that narrow
  approach-state ruling; Task 10 remains responsible for the broader policy.

### RED

Added focused regressions first, then ran:

```text
source /opt/ros/humble/setup.bash && export PYTHONPATH=/home/morowsl/ardu_ws/src/drone_docking/flight_cycle_manager:${PYTHONPATH} && /usr/bin/python3 -m pytest /home/morowsl/ardu_ws/src/drone_docking/flight_cycle_manager/test/test_manager_docking.py -q -k 'request_dock_acknowledges_before_request_driven_handoff_takeover or wait_for_handoff_accepts_auto_rtl_approach_progress or wait_for_handoff_unexpected_mode_yields_passive_pilot_override'
```

Result:

```text
FAILED test_request_dock_acknowledges_before_request_driven_handoff_takeover
FAILED test_wait_for_handoff_unexpected_mode_yields_passive_pilot_override[4-GUIDED]
FAILED test_wait_for_handoff_unexpected_mode_yields_passive_pilot_override[5-LOITER]
3 failed, 1 passed, 15 deselected in 0.85s
```

This RED matched the two findings exactly: the request path skipped
`DO_LAND_START` once `handoff_seq` persisted, and unexpected fresh approach
modes were ignored instead of yielding `PILOT_OVERRIDE`.

### GREEN

After the minimal manager fix, I reran the same focused regressions:

```text
source /opt/ros/humble/setup.bash && export PYTHONPATH=/home/morowsl/ardu_ws/src/drone_docking/flight_cycle_manager:${PYTHONPATH} && /usr/bin/python3 -m pytest /home/morowsl/ardu_ws/src/drone_docking/flight_cycle_manager/test/test_manager_docking.py -q -k 'request_dock_acknowledges_before_request_driven_handoff_takeover or wait_for_handoff_accepts_auto_rtl_approach_progress or wait_for_handoff_unexpected_mode_yields_passive_pilot_override'
```

Result: `4 passed, 15 deselected in 0.80s`.

### Production changes in this fix round

- `WAIT_FOR_HANDOFF` now checks for a fresh unexpected approach mode first and
  yields passive `PILOT_OVERRIDE` instead of lingering indefinitely.
- If a dock request was accepted and not yet acknowledged,
  `WAIT_FOR_HANDOFF` now sends exactly one `request_dock_async()` before any
  request-driven takeover attempt.
- Once the `DO_LAND_START` ACK arrives, persistent `handoff_seq` is still
  honored on the next tick, so request and normal mission progression still
  converge on the same validated handoff point.

### Files changed in this fix round

- `flight_cycle_manager/flight_cycle_manager/manager.py`
- `flight_cycle_manager/test/test_manager_docking.py`

### Verification rerun for this fix round

```text
source /opt/ros/humble/setup.bash && export PYTHONPATH=/home/morowsl/ardu_ws/src/drone_docking/flight_cycle_manager:${PYTHONPATH} && /usr/bin/python3 -m pytest flight_cycle_manager/test/test_manager_start.py flight_cycle_manager/test/test_manager_docking.py -q
```

Result: `42 passed in 6.13s`.

```text
source /opt/ros/humble/setup.bash && export PYTHONPATH=/home/morowsl/ardu_ws/src/drone_docking/flight_cycle_manager:${PYTHONPATH} && /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pymavlink_vehicle.py flight_cycle_manager/test/test_pid_landing_backend.py flight_cycle_manager/test/test_precise_landing_backend.py flight_cycle_manager/test/test_dock_search.py flight_cycle_manager/test/test_landing_target.py flight_cycle_manager/test/test_manager_ros_smoke.py -q
```

Result: `104 passed in 0.90s`.

```text
source /opt/ros/humble/setup.bash && export PYTHONPATH=/home/morowsl/ardu_ws/src/drone_docking/flight_cycle_manager:${PYTHONPATH} && /usr/bin/python3 -m pytest flight_cycle_manager/test -q
```

Result: `196 passed in 6.40s`.

```text
git -C /home/morowsl/ardu_ws/src/drone_docking diff --check
```

Result: exit `0`, no output.

### Fix-round self-review / concerns

- The fix stayed narrow: it did not change Task 9 fallback ordering, shared
  search, backend routing, or handback sequencing.
- The request-race regression proves callback acceptance, `handoff_seq`
  persistence, one `request_dock_async()` call, ACK processing, and takeover
  ordering separately.
- Deferred minor remains unchanged: the PID-disarm pending-operation
  observation noted by review is not part of this round.

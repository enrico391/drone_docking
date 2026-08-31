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

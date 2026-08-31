# Task 6 Report: Post-search PID landing backend

## Recovery context

- Date: 2026-08-31
- Branch: `flight_cycle_manager`
- Workspace: `/home/morowsl/ardu_ws/src/drone_docking`
- Prior Task 6 agents were interrupted before reporting or committing.
- This pass treated the staged candidate as untrusted and audited it from the brief before committing.

## Brief audit

### Architecture checks

- `flight_cycle_manager/flight_cycle_manager/landing_backend.py` defines the shared transport-independent types:
  - `DockTargetMode`
  - `DockObservation`
  - `VehicleObservation`
  - `VelocityCommand`
  - `LandingBackendOutcome`
  - `LandingBackendDecision`
- `landing_backend.py` imports neither concrete backend.
- `flight_cycle_manager/flight_cycle_manager/pid_landing.py` imports only the shared common types and local PID primitive, not MAVLink transport code.
- `PidLandingState` no longer includes `ACQUIRE_DOCK` or `SEARCH_CIRCLE`.
- `PidLandingConfig` no longer includes PID-owned search configuration or dock-center ownership fields.
- `PidLandingController.start_after_acquisition(now)` begins directly in `PID_ALIGN`.
- `flight_cycle_manager/flight_cycle_manager/pid_landing_backend.py` maps controller decisions into the common backend contract, tracks touchdown commitment, and distinguishes `FAIL_BEFORE_COMMIT` from `FAIL_AFTER_COMMIT`.
- `PidLandingBackend` sends no MAVLink and exposes only backend decisions plus disarm-result correlation.
- Recovery climb, central reacquisition, touchdown commitment, and disarm request correlation were preserved in the staged controller/backend behavior.

### Primitive port checks

- `pid.py`, `control.py`, and `landing_wait.py` are straight ports under the new import path.
- Their staged tests exercise the same behavior under `flight_cycle_manager`.

## Mutation evidence for already-staged behavior

No new production behavior had to be added during this recovery pass, so I used focused mutation efficacy instead of inventing a false RED->GREEN chronology.

### Mutation 1: break post-commit failure classification

- Temporary mutation: removed `PidLandingState.TOUCHDOWN_APPROACH` from `_COMMITTED_STATES` in `flight_cycle_manager/flight_cycle_manager/pid_landing_backend.py`.
- Verification command:
  - `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pid_landing_backend.py -q -k postcommit_failure_requests_direct_land`
- Result: FAIL
- Failure proved the test catches the real break:
  - expected `LandingBackendOutcome.FAIL_AFTER_COMMIT`
  - received `LandingBackendOutcome.FAIL_BEFORE_COMMIT`
- Restoration: restored `PidLandingState.TOUCHDOWN_APPROACH` to `_COMMITTED_STATES`.
- Re-run on restored code:
  - `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pid.py flight_cycle_manager/test/test_control.py flight_cycle_manager/test/test_landing_wait.py flight_cycle_manager/test/test_pid_landing_backend.py -q`
  - Result: `35 passed`

## Commands and results

- `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pid.py flight_cycle_manager/test/test_control.py flight_cycle_manager/test/test_landing_wait.py flight_cycle_manager/test/test_pid_landing_backend.py -q`
  - `35 passed in 0.04s` during initial audit
- `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pid_landing_backend.py -q -k postcommit_failure_requests_direct_land`
  - `1 failed, 11 deselected` after the temporary mutation
- `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pid.py flight_cycle_manager/test/test_control.py flight_cycle_manager/test/test_landing_wait.py flight_cycle_manager/test/test_pid_landing_backend.py -q`
  - `35 passed in 0.03s` after restoration
- `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test -q`
  - `135 passed in 0.43s`
- `git -C /home/morowsl/ardu_ws/src/drone_docking diff --check`
  - exit 0, no whitespace or patch-format issues

## Files in scope

- `flight_cycle_manager/flight_cycle_manager/control.py`
- `flight_cycle_manager/flight_cycle_manager/landing_backend.py`
- `flight_cycle_manager/flight_cycle_manager/landing_wait.py`
- `flight_cycle_manager/flight_cycle_manager/pid.py`
- `flight_cycle_manager/flight_cycle_manager/pid_landing.py`
- `flight_cycle_manager/flight_cycle_manager/pid_landing_backend.py`
- `flight_cycle_manager/test/test_control.py`
- `flight_cycle_manager/test/test_landing_wait.py`
- `flight_cycle_manager/test/test_pid.py`
- `flight_cycle_manager/test/test_pid_landing_backend.py`

## Self-review

- Re-read the brief after the audit to verify the staged code still matched the intended split of responsibilities.
- Verified that the shared common-type module stays backend-agnostic.
- Verified that the PID backend starts after shared acquisition and that no PID-owned search states or config survived the port.
- Verified that tests exercise controller/backend decisions rather than mock-only behavior.
- Verified the required focused suite, the full replacement-package suite, and `git diff --check` on the restored candidate before commit.

## Concerns

- No blocking concerns from the audit.
- The task was completed from a staged recovery point, so the report explicitly documents mutation evidence instead of claiming a new RED->GREEN implementation chronology that did not happen in this pass.

## Fix Round 1

### Finding addressed

- `landing_backend.py` and `pymavlink_vehicle.py` defined separate `VelocityFrame` enums.
- `PidLandingBackend` emitted `landing_backend.VelocityFrame`.
- `MavlinkVehicle.send_velocity()` checked identity against the local `pymavlink_vehicle.VelocityFrame`.
- Result: direct routing of a PID velocity decision into the MAVLink transport raised `ValueError("unsupported velocity frame: VelocityFrame.BODY_FLU")`.

### Covering regression

- Test: `test_pid_backend_velocity_frame_routes_through_send_velocity`
- File: `flight_cycle_manager/test/test_pymavlink_vehicle.py`
- Real behavior covered:
  - constructs a real `PidLandingBackend`
  - starts it after shared acquisition
  - obtains a real PID decision carrying `decision.velocity.frame`
  - routes that frame through the real `MavlinkVehicle.send_velocity()` boundary using the in-memory fake connection
  - verifies the transport accepts the command and emits `MAV_FRAME_BODY_NED`

### RED

- Command:
  - `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pymavlink_vehicle.py -q -k pid_backend_velocity_frame_routes_through_send_velocity`
- Output:
  - `FAILED flight_cycle_manager/test/test_pymavlink_vehicle.py::test_pid_backend_velocity_frame_routes_through_send_velocity`
  - `ValueError: unsupported velocity frame: VelocityFrame.BODY_FLU`

### GREEN

- Production change:
  - removed the duplicate `VelocityFrame` enum from `flight_cycle_manager/flight_cycle_manager/pymavlink_vehicle.py`
  - imported the shared `VelocityFrame` from `flight_cycle_manager.landing_backend`
  - preserved compatibility for existing callers importing `VelocityFrame` from `flight_cycle_manager.pymavlink_vehicle` because the module now re-exports the shared type by import

- Regression re-run:
  - `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pymavlink_vehicle.py -q -k pid_backend_velocity_frame_routes_through_send_velocity`
  - `1 passed, 47 deselected in 0.14s`

### Post-fix verification

- `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pid.py flight_cycle_manager/test/test_control.py flight_cycle_manager/test/test_landing_wait.py flight_cycle_manager/test/test_pid_landing_backend.py -q`
  - `35 passed in 0.03s`
- `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test/test_pymavlink_vehicle.py -q`
  - `48 passed in 0.17s`
- `PYTHONPATH=flight_cycle_manager /usr/bin/python3 -m pytest flight_cycle_manager/test -q`
  - `136 passed in 0.27s`

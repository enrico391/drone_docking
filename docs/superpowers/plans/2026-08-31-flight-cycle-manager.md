# Flight Cycle Manager Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace companion-owned mission execution with a `flight_cycle_manager` package that performs vertical takeoff, starts and monitors the ArduPilot-stored mission, takes over at a validated landing handoff, and safely returns authority to ArduPilot or the RC pilot.

**Architecture:** Build a new ROS 2 Python package beside `drone_mission_manager`, with one non-blocking orchestrator and pure mission-contract, search, and landing-backend components. Extend the direct pymavlink adapter with mission download/progress and acknowledged mission-control commands. Migrate launch/config/docs and remove the old package only after unit, package, and SITL gates pass.

**Tech Stack:** ROS 2 Humble, Python 3, `rclpy`, `pymavlink>=2.4.49,<3`, `geometry_msgs`, `std_msgs`, `std_srvs`, pytest, launch/launch_ros, ArduCopter SITL, Gazebo.

**Spec:** `docs/superpowers/specs/2026-08-31-flight-cycle-manager-design.md`

## Global Constraints

- Package, Python module, executable, and ROS node are all named `flight_cycle_manager`.
- The manager downloads and validates the ArduPilot mission but never uploads, clears, reorders, or modifies mission items.
- Autonomous takeoff is blocked unless exactly one valid `DO_LAND_START -> return/approach items -> NAV_LOITER_TIME -> NAV_LAND` contract is present.
- Takeoff is one GUIDED `MAV_CMD_NAV_TAKEOFF` to a configured altitude; do not add companion X/Y/yaw takeoff PID.
- Both landing backends use one shared bounded-circle dock search after the validated handoff.
- Search or pre-landing backend failure returns to the validated `NAV_LAND`; failed handback commands direct LAND.
- Any unexpected airborne mode change latches passive `PILOT_OVERRIDE`; automation cannot resume until disarm and a new explicit start request.
- `~/start_cycle` and `~/request_dock` are non-blocking request-acceptance services; progress and failure are published on `~/state` and `~/reason`.
- `auto_start` and `simulation_mode` default to false, and `auto_start=true` is invalid unless `simulation_mode=true`.
- Remove fixed-delay repetition and companion mission profiles; every completed landing ends in `DOCKED`.
- Preserve the user's existing untracked `.AGENTS` file and unrelated worktree changes.

## File Map and Cross-Task Interfaces

Create these files before removing the old package:

- `flight_cycle_manager/flight_cycle_manager/mission_contract.py`: immutable mission-item model and pure contract validation.
- `flight_cycle_manager/flight_cycle_manager/dock_search.py`: pure bounded-circle acquisition controller.
- `flight_cycle_manager/flight_cycle_manager/pymavlink_vehicle.py`: the sole MAVLink connection, telemetry, mission download, and commands.
- `flight_cycle_manager/flight_cycle_manager/landing_backend.py`: shared landing-backend observations and decisions.
- `flight_cycle_manager/flight_cycle_manager/pid_landing_backend.py`: adapter around the ported PID landing controller.
- `flight_cycle_manager/flight_cycle_manager/precise_landing_backend.py`: native LANDING_TARGET/LAND state machine.
- `flight_cycle_manager/flight_cycle_manager/manager.py`: ROS callbacks and lifecycle orchestration only.
- `flight_cycle_manager/config/flight_cycle_manager.yaml`: common configuration.
- `flight_cycle_manager/config/pid_landing.yaml`: PID-only configuration.
- `flight_cycle_manager/launch/flight_cycle.launch.py`: backend-aware node/localizer launch.

Stable interfaces used across tasks:

```python
@dataclass(frozen=True)
class MissionItem:
    seq: int
    frame: int
    command: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float

@dataclass(frozen=True)
class MissionContract:
    item_count: int
    land_start_seq: int
    handoff_seq: int
    fallback_land_seq: int

def validate_docking_mission(
    items: Sequence[MissionItem],
    config: MissionValidationConfig,
) -> MissionContract:
    raise NotImplementedError

class DockSearchController:
    def start(self, *, now: float, center_enu_m: tuple[float, float, float]) -> None:
        raise NotImplementedError
    def update(self, observation: SearchObservation) -> SearchDecision:
        raise NotImplementedError

class MavlinkVehicle:
    def download_mission_async(self, now: float) -> MavlinkFuture[MissionDownloadResult]:
        raise NotImplementedError
    def set_mission_current_async(self, sequence: int, *, reset: bool, now: float) -> MavlinkFuture[VehicleCommandResult]:
        raise NotImplementedError
    def start_mission_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        raise NotImplementedError
    def request_dock_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        raise NotImplementedError
    def rtl_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        raise NotImplementedError
    def land_async(self, now: float) -> MavlinkFuture[VehicleCommandResult]:
        raise NotImplementedError

class LandingBackend:
    def start(self, *, now: float, dock: DockObservation, vehicle: VehicleObservation) -> None:
        raise NotImplementedError
    def update(self, *, now: float, dock: Optional[DockObservation], vehicle: VehicleObservation) -> LandingBackendDecision:
        raise NotImplementedError
```

---

### Task 1: Scaffold the Replacement ROS Package

**Files:**
- Create: `flight_cycle_manager/package.xml`
- Create: `flight_cycle_manager/setup.py`
- Create: `flight_cycle_manager/setup.cfg`
- Create: `flight_cycle_manager/resource/flight_cycle_manager`
- Create: `flight_cycle_manager/flight_cycle_manager/__init__.py`
- Create: `flight_cycle_manager/flight_cycle_manager/manager.py`
- Create: `flight_cycle_manager/test/test_package_metadata.py`

**Interfaces:**
- Consumes: ROS 2 ament Python packaging conventions from `drone_mission_manager`.
- Produces: importable `flight_cycle_manager` Python package and console entry point `flight_cycle_manager = flight_cycle_manager.manager:main`.

- [ ] **Step 1: Write the failing metadata test**

```python
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]


def test_package_metadata_uses_only_new_name():
    package = ET.parse(ROOT / "package.xml").getroot()
    assert package.findtext("name") == "flight_cycle_manager"
    setup_text = (ROOT / "setup.py").read_text(encoding="utf-8")
    assert 'package_name = "flight_cycle_manager"' in setup_text
    assert "flight_cycle_manager.manager:main" in setup_text
    assert "drone_mission_manager" not in setup_text


def test_package_declares_runtime_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}
    assert {"rclpy", "geometry_msgs", "std_msgs", "std_srvs"} <= dependencies
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest flight_cycle_manager/test/test_package_metadata.py -q`

Expected: FAIL because `flight_cycle_manager/package.xml` and `setup.py` do not exist.

- [ ] **Step 3: Create minimal package metadata**

Use this package declaration and setup entry point:

```xml
<package format="3">
  <name>flight_cycle_manager</name>
  <version>0.1.0</version>
  <description>Takeoff, ArduPilot mission handoff, and precision docking manager.</description>
  <maintainer email="user@example.com">Enrico</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>dock_apriltag_localizer</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

In `setup.py`, install `config/*.yaml`, `launch/*.launch.py`, the resource marker, and `package.xml`; retain `pymavlink>=2.4.49,<3`. Point the console script at `flight_cycle_manager.manager:main`. Create a temporary `manager.py` with `main()` raising a clear `RuntimeError("flight cycle manager is not implemented")` so the declared entry point resolves during package inspection without pretending flight behavior exists.

- [ ] **Step 4: Run metadata and import tests**

Run: `python3 -m pytest flight_cycle_manager/test/test_package_metadata.py -q`

Expected: PASS.

- [ ] **Step 5: Commit the scaffold**

```bash
git add flight_cycle_manager
git commit -m "build: scaffold flight cycle manager package"
```

---

### Task 2: Implement Pure Onboard-Mission Contract Validation

**Files:**
- Create: `flight_cycle_manager/flight_cycle_manager/mission_contract.py`
- Create: `flight_cycle_manager/test/test_mission_contract.py`

**Interfaces:**
- Consumes: numeric MAVLink command/frame values supplied by transport tests; no ROS or pymavlink runtime dependency in the validator.
- Produces: `MissionItem`, `MissionValidationConfig`, `MissionContract`, `MissionContractError`, and `validate_docking_mission()` with the signatures in the file map.

- [ ] **Step 1: Write failing happy-path and rejection tests**

```python
import pytest

from flight_cycle_manager.mission_contract import (
    MissionContractError,
    MissionItem,
    MissionValidationConfig,
    validate_docking_mission,
)

DO_LAND_START = 189
NAV_WAYPOINT = 16
NAV_LOITER_TIME = 19
NAV_LAND = 21
GLOBAL_RELATIVE_ALT_INT = 6


def item(seq, command, *, p1=0.0, x=450000000, y=90000000, z=4.0):
    return MissionItem(seq, GLOBAL_RELATIVE_ALT_INT, command, p1, 0, 0, 0, x, y, z)


def config():
    return MissionValidationConfig(
        min_handoff_loiter_s=5.0,
        landing_location_tolerance_m=2.0,
    )


def test_extracts_unique_docking_contract():
    mission = [
        item(0, NAV_WAYPOINT),
        item(1, DO_LAND_START),
        item(2, NAV_WAYPOINT),
        item(3, NAV_LOITER_TIME, p1=10.0),
        item(4, NAV_LAND),
    ]
    assert validate_docking_mission(mission, config()).handoff_seq == 3
    assert validate_docking_mission(mission, config()).fallback_land_seq == 4


def test_rejects_missing_or_duplicate_land_start():
    with pytest.raises(MissionContractError, match="exactly one DO_LAND_START"):
        validate_docking_mission([item(0, NAV_WAYPOINT)], config())


def test_rejects_non_adjacent_fallback_land():
    mission = [
        item(0, DO_LAND_START),
        item(1, NAV_LOITER_TIME, p1=10.0),
        item(2, NAV_WAYPOINT),
        item(3, NAV_LAND),
    ]
    with pytest.raises(MissionContractError, match="immediately follow"):
        validate_docking_mission(mission, config())
```

Also add explicit tests for non-contiguous sequence numbers, short loiter time, a landing location beyond tolerance, current-location `NAV_LAND` coordinates `(0, 0)`, NaN/non-finite parameters, and multiple candidate loiters after the marker.

- [ ] **Step 2: Run the validator tests to verify they fail**

Run: `python3 -m pytest flight_cycle_manager/test/test_mission_contract.py -q`

Expected: FAIL with `ModuleNotFoundError: flight_cycle_manager.mission_contract`.

- [ ] **Step 3: Implement exact validation**

```python
@dataclass(frozen=True)
class MissionValidationConfig:
    min_handoff_loiter_s: float
    landing_location_tolerance_m: float


def validate_docking_mission(items, config):
    ordered = tuple(sorted(items, key=lambda value: value.seq))
    if [value.seq for value in ordered] != list(range(len(ordered))):
        raise MissionContractError("mission sequence numbers must be contiguous")
    markers = [value for value in ordered if value.command == MAV_CMD_DO_LAND_START]
    if len(markers) != 1:
        raise MissionContractError("mission must contain exactly one DO_LAND_START")
    marker = markers[0]
    loiters = [
        value for value in ordered[marker.seq + 1 :]
        if value.command == MAV_CMD_NAV_LOITER_TIME
    ]
    if len(loiters) != 1:
        raise MissionContractError("landing sequence must contain exactly one NAV_LOITER_TIME")
    handoff = loiters[0]
    if handoff.param1 < config.min_handoff_loiter_s:
        raise MissionContractError("NAV_LOITER_TIME is shorter than the minimum handoff")
    fallback_index = handoff.seq + 1
    if fallback_index >= len(ordered) or ordered[fallback_index].command != MAV_CMD_NAV_LAND:
        raise MissionContractError("NAV_LAND must immediately follow NAV_LOITER_TIME")
    fallback = ordered[fallback_index]
    _validate_landing_location(handoff, fallback, config.landing_location_tolerance_m)
    return MissionContract(len(ordered), marker.seq, handoff.seq, fallback.seq)
```

Implement `_validate_landing_location()` with a haversine distance for integer-degree-E7 global coordinates. Accept `(x, y) == (0, 0)` for `NAV_LAND` as current-location fallback. Reject different coordinate frames unless both items are supported global/global-relative INT frames.

The explicit supported frame set is
`MAV_FRAME_GLOBAL`, `MAV_FRAME_GLOBAL_RELATIVE_ALT`,
`MAV_FRAME_GLOBAL_INT`, and `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`. Both landing
coordinates must be zero to mean current location; a single zero coordinate is
invalid.

- [ ] **Step 4: Run contract tests**

Run: `python3 -m pytest flight_cycle_manager/test/test_mission_contract.py -q`

Expected: PASS.

- [ ] **Step 5: Commit mission validation**

```bash
git add flight_cycle_manager/flight_cycle_manager/mission_contract.py \
  flight_cycle_manager/test/test_mission_contract.py
git commit -m "feat: validate onboard docking mission contract"
```

---

### Task 3: Implement the Shared Bounded Dock Search

**Files:**
- Create: `flight_cycle_manager/flight_cycle_manager/dock_search.py`
- Create: `flight_cycle_manager/test/test_dock_search.py`

**Interfaces:**
- Consumes: fresh local ENU position and a boolean stable-target observation from the manager.
- Produces: `SearchConfig`, `SearchObservation`, `SearchDecision`, `SearchOutcome`, and `DockSearchController`.

- [ ] **Step 1: Write failing deterministic controller tests**

```python
def test_search_moves_to_radius_then_orbits_with_altitude_hold():
    controller = DockSearchController(search_config())
    controller.start(now=0.0, center_enu_m=(10.0, 20.0, 4.0))
    first = controller.update(observation(0.1, (10.0, 20.0, 3.8)))
    assert first.outcome is SearchOutcome.SEARCHING
    assert first.velocity_enu_m_s[0] > 0.0
    assert first.velocity_enu_m_s[2] > 0.0

    orbit = controller.update(observation(1.0, (11.0, 20.0, 4.0)))
    assert orbit.velocity_enu_m_s[1] > 0.0
    assert math.hypot(*orbit.velocity_enu_m_s[:2]) <= search_config().max_horizontal_speed_m_s


def test_target_requires_stable_dwell_before_acquired():
    controller = started_controller()
    assert controller.update(observation(1.0, target_visible=True)).outcome is SearchOutcome.SEARCHING
    assert controller.update(observation(1.6, target_visible=True)).outcome is SearchOutcome.ACQUIRED


def test_timeout_and_spatial_bound_are_terminal():
    assert started_controller().update(observation(31.0)).outcome is SearchOutcome.TIMED_OUT
    assert started_controller().update(observation(1.0, (20.0, 0.0, 4.0))).outcome is SearchOutcome.OUT_OF_BOUNDS
```

Also test target-dwell reset after one invisible frame, finite-value validation, speed saturation, radial correction direction inside/outside the radius, and update-before-start rejection.

- [ ] **Step 2: Run search tests to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_dock_search.py -q`

Expected: FAIL because `dock_search.py` does not exist.

- [ ] **Step 3: Implement the pure controller**

Use an outward radial command until the configured radius is reached, then combine tangential velocity with proportional radial correction:

```python
radial_error = self._config.radius_m - radius
radial_speed = self._config.radial_gain * radial_error
east = radial_speed * radial_x - self._config.tangential_speed_m_s * radial_y
north = radial_speed * radial_y + self._config.tangential_speed_m_s * radial_x
up = clamp(
    self._config.altitude_gain * (center_z - position_z),
    -self._config.max_vertical_speed_m_s,
    self._config.max_vertical_speed_m_s,
)
east, north = clamp_vector(east, north, self._config.max_horizontal_speed_m_s)
```

`SearchDecision` carries `(east, north, up)` and a reason. Terminal outcomes return zero velocity. The controller uses caller-supplied monotonic time only.

- [ ] **Step 4: Run search tests**

Run: `python3 -m pytest flight_cycle_manager/test/test_dock_search.py -q`

Expected: PASS.

- [ ] **Step 5: Commit shared search**

```bash
git add flight_cycle_manager/flight_cycle_manager/dock_search.py \
  flight_cycle_manager/test/test_dock_search.py
git commit -m "feat: add shared bounded dock search"
```

---

### Task 4: Port and Harden the Direct MAVLink Transport

**Files:**
- Create: `flight_cycle_manager/flight_cycle_manager/pymavlink_vehicle.py`
- Create: `flight_cycle_manager/test/test_pymavlink_vehicle.py`

**Interfaces:**
- Consumes: current tested behavior from `drone_mission_manager/drone_mission_manager/pymavlink_vehicle.py` and its tests.
- Produces: `VehicleState`, `VehicleCommandResult`, `MavlinkFuture`, `MavlinkVehicle`, `VelocityFrame`, existing command/setpoint APIs, plus mission progress fields required by Task 5.

- [ ] **Step 1: Port the existing transport tests under the new import path**

Copy the behavioral tests from `drone_mission_manager/test/test_pymavlink_vehicle.py` into the new test file and change only imports to `flight_cycle_manager.pymavlink_vehicle`. Retain exact tests for source filtering, NED-to-ENU conversion, ACK-plus-observed-state completion, retries, timeouts, parameter collection, landing target encoding, yaw correction, velocity masks, heartbeat, and close behavior.

- [ ] **Step 2: Run the ported suite to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_pymavlink_vehicle.py -q`

Expected: FAIL because `flight_cycle_manager.pymavlink_vehicle` does not exist.

- [ ] **Step 3: Port the implementation with the new package name**

Port the current adapter without changing its established wire behavior. Change the import error to:

```python
raise RuntimeError(
    "flight_cycle_manager requires pymavlink>=2.4.49,<3"
) from error
```

Keep one pending transaction, caller-supplied monotonic time, MAVLink 2, the `ardupilotmega` dialect, direct source filtering, and `COMMAND_ACK` handling. Add neutral mission-progress fields now so Task 5 does not change `VehicleState` shape later:

```python
mission_seq: Optional[int] = None
mission_total: Optional[int] = None
mission_state: Optional[int] = None
mission_mode: Optional[int] = None
mission_id: Optional[int] = None
mission_current_at: float = 0.0
```

- [ ] **Step 4: Run the complete ported transport suite**

Run: `python3 -m pytest flight_cycle_manager/test/test_pymavlink_vehicle.py -q`

Expected: PASS with the same test count and behavior as the old adapter suite.

- [ ] **Step 5: Commit the transport baseline**

```bash
git add flight_cycle_manager/flight_cycle_manager/pymavlink_vehicle.py \
  flight_cycle_manager/test/test_pymavlink_vehicle.py
git commit -m "feat: port direct MAVLink vehicle transport"
```

---

### Task 5: Add Mission Download, Progress, and Control Commands

**Files:**
- Modify: `flight_cycle_manager/flight_cycle_manager/pymavlink_vehicle.py`
- Modify: `flight_cycle_manager/test/test_pymavlink_vehicle.py`

**Interfaces:**
- Consumes: `MissionItem` from Task 2 and the single pending-operation transport from Task 4.
- Produces: `MissionDownloadResult`, periodic `MISSION_CURRENT` state, and all mission-control methods declared in the file map.

- [ ] **Step 1: Write failing mission-download tests**

```python
def test_download_mission_requests_every_int_item_and_acks(vehicle, connection):
    future = vehicle.download_mission_async(now=10.0)
    assert connection.mav.calls_for("mission_request_list_send")[-1] == (1, 1, MAVLINK.MAV_MISSION_TYPE_MISSION)

    connection.incoming.append(FakeMessage("MISSION_COUNT", count=2, mission_type=0))
    vehicle.poll(10.1)
    assert connection.mav.calls_for("mission_request_int_send")[-1][:3] == (1, 1, 0)

    connection.incoming.append(mission_item_message(seq=0, command=MAVLINK.MAV_CMD_DO_LAND_START))
    vehicle.poll(10.2)
    connection.incoming.append(mission_item_message(seq=1, command=MAVLINK.MAV_CMD_NAV_LAND))
    vehicle.poll(10.3)

    assert future.done()
    assert [item.seq for item in future.result().items] == [0, 1]
    assert connection.mav.calls_for("mission_ack_send")[-1][2] == MAVLINK.MAV_MISSION_ACCEPTED
```

Add tests for out-of-order items, duplicate items, wrong mission type/source, zero count, request retry, overall timeout, and a missing item. The next missing sequence must be re-requested; duplicates must not advance completion.

- [ ] **Step 2: Write failing mission-progress and command tests**

```python
def test_mission_current_updates_persistent_progress(vehicle, connection):
    connection.incoming.append(FakeMessage(
        "MISSION_CURRENT", seq=7, total=9,
        mission_state=MAVLINK.MISSION_STATE_ACTIVE, mission_mode=1,
    ))
    vehicle.poll(20.0)
    assert vehicle.state.mission_seq == 7
    assert vehicle.state.mission_state == MAVLINK.MISSION_STATE_ACTIVE


@pytest.mark.parametrize("method,command,params", [
    (lambda v: v.set_mission_current_async(0, reset=True, now=1.0), MAVLINK.MAV_CMD_DO_SET_MISSION_CURRENT, (0.0, 1.0)),
    (lambda v: v.start_mission_async(1.0), MAVLINK.MAV_CMD_MISSION_START, (0.0, 0.0)),
    (lambda v: v.request_dock_async(1.0), MAVLINK.MAV_CMD_DO_LAND_START, (0.0, 0.0)),
    (lambda v: v.rtl_async(1.0), MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH, (0.0, 0.0)),
    (lambda v: v.land_async(1.0), MAVLINK.MAV_CMD_NAV_LAND, (0.0, 0.0)),
])
def test_acknowledged_cycle_commands(method, command, params, vehicle, connection):
    future = method(vehicle)
    sent = connection.mav.calls_for("command_long_send")[-1]
    assert sent[2] == command
    assert sent[4:6] == params
    push_ack(connection, command)
    vehicle.poll(1.1)
    assert future.result().success
```

- [ ] **Step 3: Run focused tests to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_pymavlink_vehicle.py -k 'mission or cycle_commands' -q`

Expected: FAIL because download/progress/command APIs are absent.

- [ ] **Step 4: Implement the non-blocking mission transaction**

Extend `_PendingOperation` with `mission_count`, `mission_items`, `next_mission_request_at`, and `mission_type`. `download_mission_async()` sends `MISSION_REQUEST_LIST`; `MISSION_COUNT` starts indexed requests; each accepted `MISSION_ITEM_INT` is converted to the Task 2 `MissionItem`. Finish only after all indices are present and a successful `MISSION_ACK` has been sent.

Generalize `MavlinkFuture` to `MavlinkFuture[T]` using `TypeVar` and `Generic`
so command operations return `VehicleCommandResult` and mission downloads
return `MissionDownloadResult` without changing the `done()`/`result()` API.

Add message ID 42 (`MISSION_CURRENT`) at 5 Hz to `_MESSAGE_INTERVALS_US`. Parse absent MAVLink extension fields defensively with `getattr(message, "mission_state", None)` and `getattr(message, "mission_mode", None)`.

Also parse optional `mission_id` from `MISSION_CURRENT` and optional `opaque_id`
from `MISSION_COUNT`. Include `opaque_id: Optional[int]` in
`MissionDownloadResult`; zero or an absent extension means identity is not
supported and must not itself fail validation.

Implement the command methods through `_begin_command()` with seven explicit parameters. For `set_mission_current_async()`, verify the sequence is an integer in `[0, 32767]`; param2 is exactly `1.0` when reset and `0.0` otherwise.

- [ ] **Step 5: Run the entire transport and contract suites**

Run: `python3 -m pytest flight_cycle_manager/test/test_pymavlink_vehicle.py flight_cycle_manager/test/test_mission_contract.py -q`

Expected: PASS.

- [ ] **Step 6: Commit mission transport support**

```bash
git add flight_cycle_manager/flight_cycle_manager/pymavlink_vehicle.py \
  flight_cycle_manager/test/test_pymavlink_vehicle.py
git commit -m "feat: add onboard mission MAVLink protocol"
```

---

### Task 6: Port PID Primitives and Start PID Landing After Shared Acquisition

**Files:**
- Create: `flight_cycle_manager/flight_cycle_manager/pid.py`
- Create: `flight_cycle_manager/flight_cycle_manager/control.py`
- Create: `flight_cycle_manager/flight_cycle_manager/landing_wait.py`
- Create: `flight_cycle_manager/flight_cycle_manager/pid_landing.py`
- Create: `flight_cycle_manager/flight_cycle_manager/landing_backend.py`
- Create: `flight_cycle_manager/flight_cycle_manager/pid_landing_backend.py`
- Create: `flight_cycle_manager/test/test_pid.py`
- Create: `flight_cycle_manager/test/test_control.py`
- Create: `flight_cycle_manager/test/test_landing_wait.py`
- Create: `flight_cycle_manager/test/test_pid_landing_backend.py`

**Interfaces:**
- Consumes: shared target acquisition from Task 3, vehicle/dock observation types, and the current proven PID/controller modules.
- Produces: common `LandingBackendDecision` and a PID backend whose first state is `PID_ALIGN`, not `ACQUIRE_DOCK` or `SEARCH_CIRCLE`.

- [ ] **Step 1: Port unchanged primitive tests and implementations**

Port `test_pid.py`, `test_control.py`, and `test_landing_wait.py` under the new import path. Port their corresponding implementation files without behavior changes. Run:

`python3 -m pytest flight_cycle_manager/test/test_pid.py flight_cycle_manager/test/test_control.py flight_cycle_manager/test/test_landing_wait.py -q`

Expected: PASS.

- [ ] **Step 2: Write failing post-acquisition PID backend tests**

```python
def test_pid_backend_starts_at_alignment_after_shared_search():
    backend = PidLandingBackend(pid_config())
    backend.start(now=1.0, dock=dock_observation(), vehicle=vehicle_observation())
    decision = backend.update(now=1.1, dock=dock_observation(), vehicle=vehicle_observation())
    assert decision.substate == "PID_ALIGN"
    assert decision.outcome is LandingBackendOutcome.RUNNING


def test_precommit_failure_requests_mission_handback():
    backend = backend_with_controller_decision(
        PidLandingDecision(
            state=PidLandingState.FAILED,
            command=zero_command(),
            outcome=PidLandingOutcome.FAILED,
            reason="recovery exhausted",
        )
    )
    backend.start(now=1.0, dock=dock_observation(), vehicle=vehicle_observation())
    assert backend.update(now=2.0, dock=None, vehicle=vehicle_observation()).outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT


def test_postcommit_failure_requests_direct_land():
    backend = committed_backend_with_failure()
    assert backend.update(now=5.0, dock=None, vehicle=vehicle_observation()).outcome is LandingBackendOutcome.FAIL_AFTER_COMMIT
```

- [ ] **Step 3: Run backend tests to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_pid_landing_backend.py -q`

Expected: FAIL because the backend and common decision types do not exist.

- [ ] **Step 4: Define backend decisions and refactor PID start**

```python
class LandingBackendOutcome(Enum):
    RUNNING = auto()
    COMPLETE = auto()
    FAIL_BEFORE_COMMIT = auto()
    FAIL_AFTER_COMMIT = auto()


@dataclass(frozen=True)
class LandingBackendDecision:
    outcome: LandingBackendOutcome
    substate: str
    velocity: Optional[VelocityCommand] = None
    stream_landing_target: bool = False
    request_land: bool = False
    request_disarm: bool = False
    disarm_request_id: Optional[int] = None
    yaw_correction_rad: Optional[float] = None
    reason: str = ""
```

Also define `DockTargetMode`, `DockObservation`, `VehicleObservation`, and
`VelocityCommand` in `landing_backend.py`. Both backend implementations and the
ported PID controller import these common types; `landing_backend.py` must not
import either concrete backend.

Port the PID landing controller, remove its initial `ACQUIRE_DOCK` and
`SEARCH_CIRCLE` states, configuration, dock-center fields, and search methods,
and add `start_after_acquisition(now: float)` that initializes directly to
`PID_ALIGN`. Retain central-target acquisition, recovery climb, touchdown
commitment, and disarm transaction correlation. The shared search center is no
longer a PID-controller input.

`PidLandingBackend` maps controller outcomes to the common decision. It tracks whether touchdown commitment has occurred and maps failures before/after that point accordingly.

- [ ] **Step 5: Run PID backend and primitive suites**

Run: `python3 -m pytest flight_cycle_manager/test/test_pid.py flight_cycle_manager/test/test_control.py flight_cycle_manager/test/test_landing_wait.py flight_cycle_manager/test/test_pid_landing_backend.py -q`

Expected: PASS.

- [ ] **Step 6: Commit PID backend**

```bash
git add flight_cycle_manager/flight_cycle_manager flight_cycle_manager/test
git commit -m "feat: add post-search PID landing backend"
```

---

### Task 7: Implement the Native Precision-Landing Backend

**Files:**
- Create: `flight_cycle_manager/flight_cycle_manager/landing_target.py`
- Create: `flight_cycle_manager/flight_cycle_manager/precise_landing_backend.py`
- Create: `flight_cycle_manager/test/test_landing_target.py`
- Create: `flight_cycle_manager/test/test_precise_landing_backend.py`

**Interfaces:**
- Consumes: `LandingBackendDecision`, fresh dock pose, vehicle mode/landed observations, and the existing landing-target coordinate conversion.
- Produces: a pure native backend state machine with `WARMUP`, `REQUEST_LAND`, `WAIT_LANDED`, `WAIT_DISARM`, `COMPLETE`, and `FAILED` substates.

- [ ] **Step 1: Port landing-target geometry tests and implementation**

Port `drone_mission_manager/test/test_landing_target.py` and `landing_target.py` under the new import path. Preserve camera-optical to MAVLink Body-FRD conversion, source timestamp conversion, and dock yaw error behavior.

Run: `python3 -m pytest flight_cycle_manager/test/test_landing_target.py -q`

Expected: PASS.

- [ ] **Step 2: Write failing native backend tests**

```python
def test_native_backend_streams_target_during_warmup_then_requests_land():
    backend = PreciseLandingBackend(PreciseLandingConfig(warmup_s=1.0))
    backend.start(now=0.0, dock=dock_observation(), vehicle=guided_vehicle())
    assert backend.update(now=0.5, dock=dock_observation(), vehicle=guided_vehicle()).stream_landing_target
    decision = backend.update(now=1.1, dock=dock_observation(), vehicle=guided_vehicle())
    assert decision.request_land
    assert decision.substate == "REQUEST_LAND"


def test_target_loss_before_land_is_precommit_failure():
    backend = started_native_backend()
    decision = backend.update(now=0.5, dock=None, vehicle=guided_vehicle())
    assert decision.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT


def test_mode_departure_is_not_recovered_by_backend():
    backend = native_backend_waiting_landed()
    decision = backend.update(now=2.0, dock=dock_observation(), vehicle=loiter_vehicle())
    assert decision.reason == "vehicle left LAND"
    assert decision.outcome is LandingBackendOutcome.FAIL_AFTER_COMMIT
```

Also cover one-shot yaw correction, continued target streaming in LAND, touchdown-to-disarm transition, disarm completion, stale landed-state rejection, and warning thresholds that do not terminate an accepted LAND.

- [ ] **Step 3: Run backend tests to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_precise_landing_backend.py -q`

Expected: FAIL because the backend does not exist.

- [ ] **Step 4: Implement the pure native state machine**

The backend must never send MAVLink directly. During WARMUP it returns `stream_landing_target=True`; after continuous fresh target time reaches `warmup_s`, it returns `request_land=True` once. The manager confirms LAND and calls `notify_land_result(success, now)`. After acceptance, the backend continues requesting target streaming and uses `touchdown_wait_action()` and `disarm_wait_action()` until complete.

Before LAND acceptance, stale/lost target maps to `FAIL_BEFORE_COMMIT`. After LAND acceptance, visual loss does not fail the backend because ArduPilot owns `PLND_STRICT` recovery; return `stream_landing_target=False` until a fresh measurement returns.

- [ ] **Step 5: Run native and geometry suites**

Run: `python3 -m pytest flight_cycle_manager/test/test_landing_target.py flight_cycle_manager/test/test_precise_landing_backend.py -q`

Expected: PASS.

- [ ] **Step 6: Commit native backend**

```bash
git add flight_cycle_manager/flight_cycle_manager/landing_target.py \
  flight_cycle_manager/flight_cycle_manager/precise_landing_backend.py \
  flight_cycle_manager/test/test_landing_target.py \
  flight_cycle_manager/test/test_precise_landing_backend.py
git commit -m "feat: add native precision landing backend"
```

---

### Task 8: Implement Validation, Explicit Start, and AUTO Mission Handoff

**Files:**
- Replace: `flight_cycle_manager/flight_cycle_manager/manager.py`
- Create: `flight_cycle_manager/test/test_manager_start.py`
- Create: `flight_cycle_manager/test/vehicle_fakes.py`

**Interfaces:**
- Consumes: Task 2 contract validation and Task 5 transport commands/telemetry.
- Produces: `CycleState`, `FlightCycleManager`, `~/start_cycle`, state/reason publication, vertical takeoff, deterministic mission reset/start, and a fake vehicle boundary reused by later manager tests.

- [ ] **Step 1: Write failing service and invalid-mission tests**

```python
def test_start_service_queues_validation_without_blocking(manager):
    response = manager._start_cycle_callback(Trigger.Request(), Trigger.Response())
    assert response.success
    assert manager.state is CycleState.VALIDATING_MISSION
    assert manager.vehicle.download_calls == 1
    assert manager.vehicle.arm_calls == []


def test_invalid_contract_returns_ready_error_without_arming(manager):
    manager.vehicle.finish_download(items_without_land_contract())
    manager.tick(now=1.0)
    assert manager.state is CycleState.INVALID_ONBOARD_MISSION
    assert "DO_LAND_START" in manager.reason
    assert manager.vehicle.arm_calls == []
```

- [ ] **Step 2: Write failing takeoff and mission-start tests**

```python
def test_valid_start_runs_vertical_takeoff_then_starts_auto(manager):
    request_valid_start(manager)
    drive_prearm_mode_arm_takeoff_acks(manager)
    manager.vehicle.state = airborne_state(z=4.0, vz=0.05, mode=GUIDED_MODE)
    manager.tick(now=5.0)
    manager.tick(now=6.1)
    assert manager.vehicle.set_mission_current_calls[-1] == (0, True)
    finish_last_command(manager)
    assert manager.vehicle.start_mission_calls == 1
    manager.vehicle.state = auto_active_state(seq=0)
    manager.tick(now=7.0)
    assert manager.state is CycleState.AUTO_MISSION
```

Also test service rejection while armed, `auto_start` rejection outside simulation mode, altitude without low vertical speed not completing takeoff, stale pose/heartbeat abort before arming, ACK rejection commanding LAND if already airborne, and `MISSION_CURRENT` confirmation timeout.

For `landing_backend=precise_landing`, add a failing test proving validation
also reads `PLND_ENABLED` and `PLND_TYPE` and blocks arming unless both equal
`1.0`. Prove the PID backend does not require those parameters.

- [ ] **Step 3: Run start-manager tests to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_manager_start.py -q`

Expected: FAIL because the lifecycle manager and fake boundary do not exist.

- [ ] **Step 4: Implement the non-blocking start lifecycle**

Define these states now so later tasks extend behavior without renaming:

```python
class CycleState(Enum):
    READY = auto()
    VALIDATING_MISSION = auto()
    INVALID_ONBOARD_MISSION = auto()
    PREARM = auto()
    SET_GUIDED = auto()
    ARM = auto()
    TAKEOFF = auto()
    WAIT_TAKEOFF_STABLE = auto()
    RESET_ONBOARD_MISSION = auto()
    START_AUTO = auto()
    AUTO_MISSION = auto()
    WAIT_FOR_HANDOFF = auto()
    GUIDED_TAKEOVER = auto()
    SEARCH_DOCK = auto()
    LANDING_BACKEND = auto()
    RETURN_TO_FC = auto()
    FC_FALLBACK_LAND = auto()
    WAIT_LANDED = auto()
    WAIT_DISARM = auto()
    DOCKED = auto()
    PILOT_OVERRIDE = auto()
    LINK_LOSS = auto()
    FAILED_DISARMED = auto()
```

The service callback performs only synchronous state guards, sets `VALIDATING_MISSION`, begins `download_mission_async()`, and returns `success=True, message="cycle request accepted for validation"`. Timer ticks advance one acknowledged operation at a time. Publish state/reason on every material transition.

Begin one validation transaction on node startup. A start request from `READY`
or disarmed `INVALID_ONBOARD_MISSION` always begins a fresh download rather
than trusting the startup result. When native precision landing is selected,
read and validate `PLND_ENABLED=1` and `PLND_TYPE=1` before entering PREARM.
After a mission is corrected on the flight controller, a new start request is
therefore sufficient to leave `INVALID_ONBOARD_MISSION`.

Use `MAV_CMD_NAV_TAKEOFF`, altitude error, vertical speed, and a continuous stable dwell. Then call `set_mission_current_async(0, reset=True)` followed by `start_mission_async()`. Enter `AUTO_MISSION` only after ACKs plus heartbeat AUTO and active mission telemetry.

- [ ] **Step 5: Run start lifecycle tests**

Run: `python3 -m pytest flight_cycle_manager/test/test_manager_start.py -q`

Expected: PASS.

- [ ] **Step 6: Commit start lifecycle**

```bash
git add flight_cycle_manager/flight_cycle_manager/manager.py \
  flight_cycle_manager/test/test_manager_start.py \
  flight_cycle_manager/test/vehicle_fakes.py
git commit -m "feat: add validated flight cycle start"
```

---

### Task 9: Implement Dock Request, Handoff, Search, Backends, and FC Fallback

**Files:**
- Modify: `flight_cycle_manager/flight_cycle_manager/manager.py`
- Create: `flight_cycle_manager/test/test_manager_docking.py`

**Interfaces:**
- Consumes: validated sequence numbers, mission progress, Task 3 search decisions, and Tasks 6-7 backend decisions.
- Produces: `~/request_dock`, normal/early convergence, GUIDED takeover, both backend integrations, deterministic fallback handback, and direct LAND fallback.

- [ ] **Step 1: Write failing request and handoff tests**

```python
def test_request_dock_is_idempotent_and_queues_do_land_start(auto_manager):
    first = call_request_dock(auto_manager)
    second = call_request_dock(auto_manager)
    assert first.success and second.success
    assert auto_manager.vehicle.request_dock_calls == 1


def test_handoff_sequence_switches_to_guided_and_starts_search(auto_manager):
    auto_manager.vehicle.state = auto_active_state(seq=auto_manager.contract.handoff_seq)
    auto_manager.tick(now=10.0)
    finish_mode_change(auto_manager, GUIDED_MODE)
    assert auto_manager.state is CycleState.SEARCH_DOCK
    assert auto_manager.search_controller.center_enu_m == pytest.approx((0.0, 0.0, 4.0))
```

- [ ] **Step 2: Write failing acquisition/backend/fallback tests**

```python
@pytest.mark.parametrize("backend_name", ["pid", "precise_landing"])
def test_shared_search_acquisition_starts_selected_backend(manager_for_backend, backend_name):
    manager = manager_for_backend(backend_name)
    enter_search(manager)
    manager.search_controller.queue_outcome(SearchOutcome.ACQUIRED)
    manager.tick(now=12.0)
    assert manager.state is CycleState.LANDING_BACKEND
    assert manager.backend.starts == 1


def test_search_timeout_hands_back_to_validated_nav_land(searching_manager):
    searching_manager.search_controller.queue_outcome(SearchOutcome.TIMED_OUT)
    searching_manager.tick(now=40.0)
    assert searching_manager.vehicle.set_mission_current_calls[-1] == (
        searching_manager.contract.fallback_land_seq,
        True,
    )


def test_failed_handback_commands_direct_land(returning_manager):
    returning_manager.vehicle.reject_pending("mission current rejected")
    returning_manager.tick(now=41.0)
    assert returning_manager.vehicle.land_calls == 1
```

Add tests for request rejection outside AUTO, DO_LAND_START ACK failure, backend velocity routing, landing-target streaming only on fresh pose generations, native LAND request/notification, PID disarm request correlation, precommit backend handback, postcommit direct LAND, and fallback AUTO/current-sequence confirmation.

- [ ] **Step 3: Run docking-manager tests to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_manager_docking.py -q`

Expected: FAIL because docking states are not wired.

- [ ] **Step 4: Implement normal and early docking convergence**

During `AUTO_MISSION`, accept an asynchronous dock request and send `request_dock_async()` once. Set `WAIT_FOR_HANDOFF` after acceptance, but also detect the handoff sequence during normal mission progression without a service request. Treat AUTO and ArduCopter AUTO_RTL mode values as expected while waiting.

When `mission_seq == contract.handoff_seq`, enter GUIDED, capture the fresh local ENU pose, and start `DockSearchController`. Convert each `(east, north, up)` search decision through `send_velocity(VelocityFrame.MAP_ENU, east, north, up, now=now)`.

- [ ] **Step 5: Implement backend routing and fallback transactions**

On `ACQUIRED`, create the configured backend and call `start()`. Route decisions as follows:

```python
if decision.velocity is not None:
    command = decision.velocity
    self._vehicle.send_velocity(
        command.frame,
        command.velocity_x,
        command.velocity_y,
        command.velocity_z,
        now=now,
        yaw_rate_rad_s=command.yaw_rate_rad_s,
    )
if decision.stream_landing_target:
    self._stream_latest_landing_target_once()
if decision.request_land:
    self._begin("native_land", self._vehicle.land_async(now))
if decision.request_disarm:
    self._begin_pid_disarm(decision.disarm_request_id, now)
if decision.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT:
    self._begin_fc_handback(now)
if decision.outcome is LandingBackendOutcome.FAIL_AFTER_COMMIT:
    self._begin("direct_land", self._vehicle.land_async(now))
```

FC handback must select `contract.fallback_land_seq` with reset true, start the mission, and verify AUTO plus the expected sequence. Any rejection or confirmation timeout sends direct LAND once.

- [ ] **Step 6: Run docking and prior manager suites**

Run: `python3 -m pytest flight_cycle_manager/test/test_manager_start.py flight_cycle_manager/test/test_manager_docking.py -q`

Expected: PASS.

- [ ] **Step 7: Commit docking orchestration**

```bash
git add flight_cycle_manager/flight_cycle_manager/manager.py \
  flight_cycle_manager/test/test_manager_docking.py
git commit -m "feat: orchestrate docking handoff and fallback"
```

---

### Task 10: Latch Manual Authority, Link Loss, and Completion

**Files:**
- Modify: `flight_cycle_manager/flight_cycle_manager/manager.py`
- Create: `flight_cycle_manager/test/test_manager_authority.py`

**Interfaces:**
- Consumes: current lifecycle state, pending expected mode transitions, heartbeat freshness, landed state, and backend completion.
- Produces: mode-allowlist enforcement, latched `PILOT_OVERRIDE`, passive `LINK_LOSS`, touchdown/disarm completion, and explicit next-cycle reset from `DOCKED`.

- [ ] **Step 1: Write failing manual takeover tests across every active mode**

```python
@pytest.mark.parametrize("state,expected_mode,new_mode", [
    (CycleState.AUTO_MISSION, AUTO_MODE, LOITER_MODE),
    (CycleState.WAIT_FOR_HANDOFF, AUTO_RTL_MODE, STABILIZE_MODE),
    (CycleState.SEARCH_DOCK, GUIDED_MODE, LOITER_MODE),
    (CycleState.LANDING_BACKEND, GUIDED_MODE, ALT_HOLD_MODE),
    (CycleState.WAIT_LANDED, LAND_MODE, LOITER_MODE),
])
def test_unexpected_mode_latches_passive_override(manager_in_state, state, expected_mode, new_mode):
    manager = manager_in_state(state, expected_mode)
    manager.vehicle.clear_recorded_output()
    manager.vehicle.state = replace(manager.vehicle.state, mode=new_mode)
    manager.tick(now=20.0)
    assert manager.state is CycleState.PILOT_OVERRIDE
    assert manager.vehicle.flight_command_count == 0

    manager.vehicle.state = replace(manager.vehicle.state, mode=expected_mode)
    manager.tick(now=21.0)
    assert manager.state is CycleState.PILOT_OVERRIDE
    assert manager.vehicle.flight_command_count == 0
```

Add a test proving a mode change that matches an outstanding manager command is not treated as override. Add a test that a new start request while still armed is rejected and that disarm plus a new accepted start clears the previous-flight latch.

Add a mission-integrity test that changes `MISSION_CURRENT.total` or a
supported nonzero `mission_id` after takeoff, expects one `rtl_async()` call,
and expects `land_async()` if the RTL command is rejected.

- [ ] **Step 2: Write failing link-loss and completion tests**

```python
def test_stale_heartbeat_enters_passive_link_loss_without_recovery_commands(airborne_manager):
    airborne_manager.vehicle.clear_recorded_output()
    airborne_manager.tick(now=airborne_manager.vehicle.state.heartbeat_at + 3.0)
    assert airborne_manager.state is CycleState.LINK_LOSS
    assert airborne_manager.vehicle.flight_command_count == 0


def test_touchdown_and_disarm_finish_docked_without_repeat(landing_manager):
    landing_manager.vehicle.state = on_ground_armed_state()
    landing_manager.tick(now=30.0)
    assert landing_manager.state is CycleState.WAIT_DISARM
    landing_manager.vehicle.state = on_ground_disarmed_state()
    landing_manager.tick(now=31.0)
    assert landing_manager.state is CycleState.DOCKED
    landing_manager.tick(now=100.0)
    assert landing_manager.vehicle.arm_calls == []
```

- [ ] **Step 3: Run authority tests to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_manager_authority.py -q`

Expected: FAIL because mode authority and terminal completion are not implemented.

- [ ] **Step 4: Implement expected-mode policy before state output**

Define `_expected_modes()` by state. Run mode-authority checks after polling/synchronizing telemetry and before streaming targets or setpoints. Exempt only a mode matching the manager's currently pending mode command. On unexpected departure, clear pending application operations, stop invoking backends/search, enter `PILOT_OVERRIDE`, and return from every future tick before outbound commands.

Do not send a final zero setpoint after observing the new pilot/failsafe mode; ArduPilot already changed controllers. Stop `LANDING_TARGET` immediately.

On stale heartbeat, enter `LINK_LOSS` and likewise return without attempting MAVLink recovery. Use fresh `ON_GROUND` followed by `armed=false` to reach `DOCKED`. Remove every timer-driven restart path.

Store the validated item count and nonzero mission identity for the active
cycle. A changed count or changed supported identity while airborne is a
contract violation: request RTL once, accept RTL/LAND as FC fallback modes, and
send direct LAND if RTL is rejected or times out. Do not use the previously
validated handoff or fallback sequence after such a change.

- [ ] **Step 5: Run all manager suites**

Run: `python3 -m pytest flight_cycle_manager/test/test_manager_start.py flight_cycle_manager/test/test_manager_docking.py flight_cycle_manager/test/test_manager_authority.py -q`

Expected: PASS.

- [ ] **Step 6: Commit authority behavior**

```bash
git add flight_cycle_manager/flight_cycle_manager/manager.py \
  flight_cycle_manager/test/test_manager_authority.py
git commit -m "feat: latch pilot and failsafe authority"
```

---

### Task 11: Add Split Configuration, Launch Integration, and Package Tests

**Files:**
- Create: `flight_cycle_manager/config/flight_cycle_manager.yaml`
- Create: `flight_cycle_manager/config/pid_landing.yaml`
- Create: `flight_cycle_manager/launch/flight_cycle.launch.py`
- Create: `flight_cycle_manager/README.md`
- Create: `flight_cycle_manager/test/test_launch_and_config.py`
- Modify: `flight_cycle_manager/setup.py`
- Modify: `flight_cycle_manager/package.xml`

**Interfaces:**
- Consumes: all manager parameter names and ROS interfaces from Tasks 8-10.
- Produces: installable launch/configuration with PID settings conditionally loaded and no mission selector.

- [ ] **Step 1: Write failing configuration ownership tests**

```python
def test_common_and_pid_configuration_do_not_overlap():
    common = load_parameters(COMMON_CONFIG)
    pid = load_parameters(PID_CONFIG)
    assert set(common).isdisjoint(pid)
    assert "search_radius_m" in common
    assert "takeoff_altitude_m" in common
    assert "pid_x_kp" in pid
    assert not any("mission_file" in key or "repeat" in key for key in common | pid)


def test_launch_has_no_companion_mission_selector():
    text = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '"mission"' not in text
    assert '"mission_file"' not in text
    assert "flight_cycle_manager" in text
```

Add launch-context tests proving `landing_backend=pid` loads both YAML files, `precise_landing` loads only common YAML, and `auto_start=true` with `simulation_mode=false` raises `ValueError` before a node is created.

- [ ] **Step 2: Run launch/config tests to verify failure**

Run: `python3 -m pytest flight_cycle_manager/test/test_launch_and_config.py -q`

Expected: FAIL because config and launch files do not exist.

- [ ] **Step 3: Create common configuration with explicit defaults**

Include these common groups as flat ROS parameters under `flight_cycle_manager.ros__parameters`:

```yaml
simulation_mode: false
auto_start: false
landing_backend: pid
update_rate_hz: 20.0
takeoff_altitude_m: 4.0
takeoff_altitude_tolerance_m: 0.25
takeoff_vertical_speed_tolerance_m_s: 0.15
takeoff_stable_time_s: 1.0
mission_handoff_min_loiter_s: 5.0
mission_landing_location_tolerance_m: 2.0
mission_download_timeout_s: 10.0
search_radius_m: 1.0
search_tangential_speed_m_s: 0.30
search_radial_gain: 0.70
search_altitude_gain: 0.70
search_max_horizontal_speed_m_s: 0.50
search_max_vertical_speed_m_s: 0.50
search_target_stable_time_s: 0.50
search_timeout_s: 30.0
search_max_displacement_m: 2.0
```

Retain the current direct MAVLink IDs/connection defaults, dock topics, pose timeout, landing-target camera offsets, native warmup/yaw configuration, and landing/disarm warning thresholds. Do not include route coordinates or repeat settings.

Move every `pid_*` gain, alignment, descent, recovery, commit, touchdown, and disarm parameter into `pid_landing.yaml`. Remove PID initial-search parameters because Task 3 owns search.

- [ ] **Step 4: Implement backend-aware launch and README**

`flight_cycle.launch.py` starts the localizer by default, selects `pid` or `precise_landing`, loads the common file, and appends the PID file only for `pid`. Declare MAVLink, camera/localizer, `simulation_mode`, and `auto_start` arguments. Validate the simulation-only auto-start rule in the OpaqueFunction before creating the node.

Document the required onboard mission pattern, Mission Planner upload prerequisite, start/dock services, state/reason topics, RC takeover latch, backend selection, and the absence of automatic repeat.

- [ ] **Step 5: Run launch/config and package unit suites**

Run: `python3 -m pytest flight_cycle_manager/test -q`

Expected: PASS.

- [ ] **Step 6: Build and test the new package while the old package still exists**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select flight_cycle_manager
colcon test --packages-select flight_cycle_manager --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build succeeds and the package reports zero failed tests.

- [ ] **Step 7: Commit launch and configuration**

```bash
git add flight_cycle_manager
git commit -m "feat: configure and launch flight cycle manager"
```

---

### Task 12: Migrate Project References and Remove the Old Package

**Files:**
- Modify: `README.md`
- Modify: `dock_apriltag_localizer/README.md`
- Modify if references are found: `drone_bringup/launch/*.launch.py`
- Delete: `drone_mission_manager/`
- Test: repository-wide `rg`, pytest, and colcon checks below.

**Interfaces:**
- Consumes: a passing installable `flight_cycle_manager` package from Task 11.
- Produces: one canonical manager package with no runtime compatibility alias or obsolete companion mission files.

- [ ] **Step 1: Record the pre-removal reference set**

Run:

```bash
rg -n "drone_mission_manager|mission_file|simple_circle|dock_circle|repeat\.count|repeat\.delay_s" \
  . -g '!docs/superpowers/**' -g '!**/__pycache__/**' -g '!*.pyc'
```

Expected: matches are limited to the old package and project documentation that this task will migrate. Investigate any runtime reference outside that set before deleting the package.

- [ ] **Step 2: Update project documentation and launch dependencies**

Replace build/run examples with:

```bash
colcon build --symlink-install --packages-select \
  flight_cycle_manager dock_apriltag_localizer drone_description drone_bringup
ros2 launch flight_cycle_manager flight_cycle.launch.py \
  landing_backend:=pid simulation_mode:=true
ros2 service call /flight_cycle_manager/start_cycle std_srvs/srv/Trigger "{}"
```

Explain that a valid mission must already be stored in ArduPilot. Update the localizer README's single-MAVLink-owner wording to name `flight_cycle_manager`.

- [ ] **Step 3: Remove the old package and verify no runtime references remain**

Delete the tracked `drone_mission_manager` tree. Do not touch `.AGENTS`, build/install/log artifacts, or historical files under `docs/superpowers/`.

Run:

```bash
rg -n "drone_mission_manager|mission_file|simple_circle|dock_circle|repeat\.count|repeat\.delay_s" \
  . -g '!docs/superpowers/**' -g '!**/__pycache__/**' -g '!*.pyc'
```

Expected: no output.

- [ ] **Step 4: Run package and workspace-owned build gates**

Run:

```bash
python3 -m pytest flight_cycle_manager/test dock_apriltag_localizer/test -q
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  flight_cycle_manager dock_apriltag_localizer drone_description drone_bringup
colcon test --packages-select flight_cycle_manager dock_apriltag_localizer \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all commands succeed with zero failed tests.

- [ ] **Step 5: Commit the migration**

```bash
git add README.md dock_apriltag_localizer/README.md drone_bringup \
  drone_mission_manager flight_cycle_manager
git commit -m "refactor: replace drone mission manager"
```

---

### Task 13: Automate SITL Mission Contract and Acceptance Scenarios

**Files:**
- Create: `flight_cycle_manager/test/data/docking_contract.waypoints`
- Create: `flight_cycle_manager/test/sitl_mission_fixture.py`
- Create: `flight_cycle_manager/test/test_sitl_flight_cycle.py`
- Modify: `flight_cycle_manager/README.md`
- Modify: `flight_cycle_manager/setup.cfg`

**Interfaces:**
- Consumes: ArduCopter SITL/Gazebo bringup, direct pymavlink endpoint, AprilTag localizer, and the public services/topics.
- Produces: test-only mission upload fixture and repeatable end-to-end acceptance tests; runtime manager still never uploads missions.

- [ ] **Step 1: Create a test-only valid onboard mission fixture**

Use this exact QGC WPL 110 fixture at the simulated dock reference. Command 16
is `NAV_WAYPOINT`, 189 is `DO_LAND_START`, 19 is `NAV_LOITER_TIME`, and 21 is
`NAV_LAND`:

```text
QGC WPL 110
0	1	3	16	0	0	0	0	45.3821944	12.0006033	4.0	1
1	0	3	189	0	0	0	0	45.3821944	12.0005833	4.0	1
2	0	3	16	0	0	0	0	45.3822044	12.0005833	4.0	1
3	0	3	16	0	0	0	0	45.3821944	12.0005833	4.0	1
4	0	3	19	10	0	0	0	45.3821944	12.0005833	4.0	1
5	0	3	21	0	0	0	0	45.3821944	12.0005833	0.0	1
```

Keep the fixture under `test/data`; do not install it with the package.

`sitl_mission_fixture.py` uses a separate preflight pymavlink connection to
upload the fixture before `flight_cycle_manager` starts, waits for
`MISSION_ACK=MAV_MISSION_ACCEPTED`, closes the uploader connection, and only
then launches the manager so exactly one process owns the SITL TCP endpoint.
Register the `sitl` pytest marker under `[tool:pytest]` in `setup.cfg` so normal
unit runs can exclude these tests without unknown-marker warnings.

- [ ] **Step 2: Write an invalid-contract SITL smoke test first**

```python
@pytest.mark.sitl
def test_invalid_onboard_mission_refuses_to_arm(sitl_stack):
    sitl_stack.upload_mission(items_without_nav_land())
    sitl_stack.start_manager(backend="pid")
    sitl_stack.call_start_cycle()
    sitl_stack.wait_state("INVALID_ONBOARD_MISSION", timeout=15.0)
    assert sitl_stack.vehicle_armed() is False
```

Run the single test against the headless docking stack. Expected before fixture/harness completion: FAIL because the helper is absent; after implementation: PASS without arming.

- [ ] **Step 3: Add parameterized successful backend scenarios**

```python
@pytest.mark.sitl
@pytest.mark.parametrize("backend", ["pid", "precise_landing"])
def test_stored_mission_handoff_search_and_dock(sitl_stack, backend):
    sitl_stack.upload_valid_docking_mission()
    sitl_stack.start_manager(backend=backend)
    sitl_stack.call_start_cycle()
    sitl_stack.wait_states(
        ["WAIT_TAKEOFF_STABLE", "AUTO_MISSION", "SEARCH_DOCK", "LANDING_BACKEND", "DOCKED"],
        timeout=180.0,
    )
    assert sitl_stack.vehicle_armed() is False
```

Add scenarios for `request_dock`, visual-target suppression causing search timeout and AUTO `NAV_LAND`, rejected handback causing direct LAND, and RC mode changes during GUIDED PID/native LAND that produce persistent `PILOT_OVERRIDE` without manager reacquisition.

- [ ] **Step 4: Run SITL scenarios separately from unit CI**

Use the repository's headless simulation launch with `initial_point:=dock`,
`use_dds_agent:=false`, `use_mavproxy:=false`, and
`use_gz_sim_gui:=false`. Run:

```bash
python3 -m pytest flight_cycle_manager/test/test_sitl_flight_cycle.py \
  -m sitl -v --tb=short
```

Expected: all seven acceptance categories pass; capture state/reason and
ArduPilot mode transitions in the pytest output on failure.

- [ ] **Step 5: Document exact SITL preparation and safety fallback behavior**

Add README commands for uploading the test mission, starting the simulation,
launching each backend, calling both services, observing state/reason, and
performing RC/mode takeover. State explicitly that test fixture upload is not a
runtime manager capability.

- [ ] **Step 6: Run the final verification matrix**

Run:

```bash
python3 -m pytest flight_cycle_manager/test -m "not sitl" -q
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  flight_cycle_manager dock_apriltag_localizer drone_description drone_bringup
colcon test --packages-select flight_cycle_manager dock_apriltag_localizer \
  --event-handlers console_direct+
colcon test-result --verbose
git diff --check
git status --short
```

Expected: unit/package gates pass, `colcon test-result` reports zero failures,
`git diff --check` reports no whitespace errors, and `git status` contains only
intentional changes plus the pre-existing untracked `.AGENTS` file.

- [ ] **Step 7: Commit SITL acceptance coverage**

```bash
git add flight_cycle_manager/test flight_cycle_manager/README.md
git commit -m "test: cover flight cycle manager in SITL"
```

---

## Completion Gate

Before declaring implementation complete:

1. Run every non-SITL unit and colcon command in Tasks 11-13.
2. Run both successful backend SITL scenarios, invalid-contract refusal,
   request-dock, search-timeout fallback, failed-handback direct LAND, and RC
   takeover scenarios.
3. Confirm no installed/runtime reference to `drone_mission_manager` or
   companion mission profiles remains.
4. Confirm the manager never uploads a mission in production code.
5. Confirm no airborne failure path remains in GUIDED while only emitting zero
   velocity.
6. Invoke `superpowers:verification-before-completion` before reporting success.

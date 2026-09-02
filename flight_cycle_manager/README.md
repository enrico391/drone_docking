# Flight Cycle Manager

`flight_cycle_manager` owns one explicit takeoff-to-dock cycle at a time. It
does not upload missions, choose mission profiles, or repeat cycles on its own.
ArduPilot must already hold a valid docking mission before startup.

## Required onboard mission

Upload the mission from Mission Planner before launching this node. The stored
mission must contain exactly:

1. one `DO_LAND_START`;
2. one later `NAV_LOITER_TIME` with at least the configured
   `mission_handoff_min_loiter_s`; and
3. an immediate following `NAV_LAND` at the same landing site within
   `mission_landing_location_tolerance_m`.

`~/start_cycle` always re-downloads and re-validates the onboard mission before
arming. If the mission is invalid, the manager stays passive and publishes the
failure reason.

The checked-in QGC WPL fixture under `test/data/` and the upload helper in
`test/sitl_mission_fixture.py` are test-only. Runtime `flight_cycle_manager`
never uploads or mutates missions on the vehicle.

## Services and topics

- `/flight_cycle_manager/start_cycle`: validate the stored mission and begin one
  takeoff cycle when the vehicle is disarmed.
- `/flight_cycle_manager/request_dock`: while the validated mission is active,
  request docking at the mission's `DO_LAND_START` handoff point.
- `/flight_cycle_manager/state`: transient-local lifecycle state such as
  `READY`, `AUTO_MISSION`, `SEARCH_DOCK`, `LANDING_BACKEND`, `DOCKED`,
  `PILOT_OVERRIDE`, or `LINK_LOSS`.
- `/flight_cycle_manager/reason`: transient-local explanation for the latest
  transition, rejection, warning, or failure.

The manager latches manual RC takeover as `PILOT_OVERRIDE` when an unexpected
airborne mode change occurs. It does not resume automatically if the pilot
switches back; the latch clears only after fresh landed telemetry, disarm, and
the next explicit `/flight_cycle_manager/start_cycle`.

## Backends and fallback

`landing_backend:=pid` keeps ArduPilot in `GUIDED` after shared dock search and
uses the checked-in `pid_landing.yaml` gains for alignment, descent, touchdown
commit, and disarm retry behavior.

`landing_backend:=precise_landing` streams `LANDING_TARGET` updates and lets
ArduPilot complete the descent in `LAND`. The common configuration still owns
the dock-pose freshness limits, target warmup, yaw-correction enable flag, and
touchdown/disarm warning thresholds.

If docking fails before touchdown commit, the manager can hand control back to
the validated onboard `NAV_LAND`. If state integrity is lost or no validated
fallback mission remains, it escalates to a direct FC `LAND`. There is no
automatic repeat after `DOCKED`; each new cycle requires another service call.

## Build and run

Start the simulator without a competing MAVLink owner:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_bringup docking_sim.launch.py \
  initial_point:=dock use_dds_agent:=false \
  use_mavproxy:=false use_gz_sim_gui:=false
```

In another sourced terminal:

```bash
colcon build --symlink-install --packages-select flight_cycle_manager
source install/setup.bash
ros2 launch flight_cycle_manager flight_cycle.launch.py \
  landing_backend:=pid simulation_mode:=true
```

For the native backend:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch flight_cycle_manager flight_cycle.launch.py \
  landing_backend:=precise_landing simulation_mode:=true
```

`auto_start:=true` is accepted only with `simulation_mode:=true`. Production
and hardware runs should keep `auto_start:=false` and use the explicit service:

```bash
ros2 service call /flight_cycle_manager/start_cycle std_srvs/srv/Trigger "{}"
ros2 service call /flight_cycle_manager/request_dock std_srvs/srv/Trigger "{}"
```

Start the localizer by default. Disable it only when another launch already
owns `/dock/pose` and `/dock/mode`:

```bash
ros2 launch flight_cycle_manager flight_cycle.launch.py \
  start_localizer:=false landing_backend:=precise_landing
```

Useful diagnostics:

```bash
ros2 topic echo /flight_cycle_manager/state
ros2 topic echo /flight_cycle_manager/reason
ros2 topic echo /dock/mode
ros2 topic echo /dock/pose
```

To force a manual mode takeover during SITL, use a separate MAVLink client on
the observer port (`5762`). Replace `mode_id` with `3` (`AUTO`), `4`
(`GUIDED`), or `9` (`LAND`):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 - <<'PY'
from flight_cycle_manager.pymavlink_vehicle import load_mavutil

mavutil = load_mavutil()
mode_id = 9
connection = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5762",
    source_system=253,
    source_component=1,
    dialect="ardupilotmega",
    autoreconnect=False,
    robust_parsing=True,
)
heartbeat = connection.wait_heartbeat(timeout=20)
connection.mav.command_long_send(
    heartbeat.get_srcSystem(),
    heartbeat.get_srcComponent(),
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id,
    0,
    0,
    0,
    0,
    0,
)
connection.close()
PY
```

The SITL acceptance suite owns the preflight mission upload and startup
sequencing automatically:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest flight_cycle_manager/test/test_sitl_flight_cycle.py \
  -m sitl -v --tb=short
```

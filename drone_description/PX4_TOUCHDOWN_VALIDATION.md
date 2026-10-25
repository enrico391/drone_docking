# PX4 touchdown timeout diagnosis and SITL validation

Date: 2026-10-24. Firmware: the repository's pinned RC1 SITL build.

The reported failure occurred before any disarm request. The original manager
log, `/home/enri-ubuntu/.ros/log/python3_25787_1792833501481.log`, reports
`FC_FALLBACK_LAND: touchdown was not confirmed before timeout`, followed by
`DOCKED` after native LAND.

The matching PX4 log is
`/home/enri-ubuntu/drone_ws/.worktrees/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-10-24/09_18_25.ulg`.
It records:

- Final downward velocity setpoint: 0.15 m/s.
- `LNDMC_Z_VEL_MAX`: 0.25 m/s; the detector's `in_descend` flag stays false
  during final approach.
- No external disarm command; native LAND is requested at PX4 time 86.829 s.
- Landing detection at 88.259 s, then automatic disarm at 90.259 s.

`MulticopterLandDetector::_get_ground_contact_state()` in this firmware
requires commanded downward velocity >= `1.1 * LNDMC_Z_VEL_MAX` before ground
contact can latch. Thus the stock threshold requires 0.275 m/s, which the
configured 0.15 m/s descent cannot satisfy. Low thrust, limited motion and
landing hysteresis must also pass; matching the speed gate alone does not
immediately establish landing.

The paired SITL profile now sets `LNDMC_Z_VEL_MAX=0.10`, preserving the
0.15 m/s descent. The manager's touchdown deadline increases from 15 to 30
seconds to accommodate post-contact thrust reduction and landing confirmation.
Normal disarm still requires fresh, stable PX4 landed telemetry.

## Verification

The cross-profile regression test failed before the correction with
`0.15 >= 1.1 * 0.25` and passed afterward. The manager was rebuilt using
`colcon build --symlink-install --packages-select flight_cycle_manager`.

An isolated headless run used ROS domain 42 and Gazebo partition
`codex_disarm_validation`:

```bash
ros2 launch drone_description px4_docking.launch.py \
  use_gz_sim_gui:=false use_flight_cycle_manager:=true \
  pid_tuner:=false initial_point:=dock
```

The validation driver called `start_cycle`, waited for `HOVER`, then called
`request_dock`. State receipt times relative to the driver start were:

| State | Wall time (s) |
| --- | ---: |
| HOVER | 10.337 |
| DESCEND_CENTRAL | 27.057 |
| TOUCHDOWN_APPROACH | 37.397 |
| DISARM | 53.842 |
| DOCKED | 53.897 |

No fallback occurred. PX4 logged `Landing detected` followed by
`Disarmed by external command`. The 16.445-second interval from approach to
disarm exceeds the previous deadline. Local evidence is in
`/tmp/codex_disarm_sitl.log`, `/tmp/codex_disarm_validation.json` and PX4 log
`2026-10-24/09_24_10.ulg` in the same rootfs log directory.

The two package suites produced **456 passed, 2 failed**, using the source
flight-cycle package on `PYTHONPATH` for subprocess imports. Remaining failures:

- `test_executable_shuts_down_cleanly_on_interrupt`: existing camera-latency
  startup/interrupt race, outside the changed landing configuration.
- `test_recovery_climb_times_out_if_altitude_does_not_increase`: expects the
  nominal 4.5-second climb deadline, while the existing committed controller
  enforces a temporary 10-second minimum. Its fixture does not load the
  modified YAML.

Full test output: `/tmp/codex_disarm_tests.log`. This validates one nominal
SITL cycle starting on the dock; hardware and disturbance cases remain untested.

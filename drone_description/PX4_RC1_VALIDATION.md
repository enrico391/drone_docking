# PX4 v1.18.0-rc1 SITL update — 2026-10-18

## Firmware and build

- Default source tree:
  `/home/enri-ubuntu/drone_ws/.worktrees/PX4-Autopilot`.
- Tag: `v1.18.0-rc1`.
- Commit: `fca3df865af36124a28c9d607e850f111dbaaea9`, matching the seven
  bundled ROS interfaces and their snapshot manifest.
- All 46 recursive submodules match the tag's pinned revisions.
- A clean `px4_sitl_default` build completed with Gazebo Harmonic 8.15.0,
  GCC 11.4 and the default Micro XRCE-DDS v2 client for ROS 2 Humble.
- The firmware build dependencies were updated only in
  `/home/enri-ubuntu/drone_ws/.worktrees/px4-venv`.
- The local `4099_gz_x500_docking` airframe and its CMake registration were
  preserved. The normal launch still selects stock airframe 4001 and applies
  the docking parameter profile before controllers start.
- `MPC_USE_HTE` was removed from the profile and custom airframe because RC1
  removed that parameter and automatically uses a valid hover-thrust estimate.
  All remaining profile and custom-airframe parameters exist in RC1 metadata.

## Backup

The previous firmware revisions, custom-airframe patch and original airframe,
build, saved parameters and 32 flight logs are preserved under:

`/home/enri-ubuntu/drone_ws/.worktrees/px4-backup-v1.16.2-20261018-133931`.

RC1 was built into the normal build directory with a new rootfs; the old
parameters were not imported into the new firmware.

## Disarmed startup and DDS check

The existing docking launch was started headless with the custom docking world,
X500 model and DART physics. The ROS-Gazebo bridge, localizer, flight-cycle
manager and disturbances were disabled for this focused firmware/DDS check.
The check used ROS domain 211, a separate Gazebo partition, DDS UDP port 8899
and namespace `rc1check` to avoid the normal application endpoints.

Verified:

- The running binary reports the RC1 commit and firmware version 1.18.0 RC1.
- `MPC_THR_HOVER=0.38` and `THR_MDL_FAC=0.74` are applied at startup.
- The EKF produces valid horizontal and vertical local position.
- ROS receives `VehicleStatus` on `/rc1check/fmu/out/vehicle_status_v4`,
  `VehicleLocalPosition` on `/rc1check/fmu/out/vehicle_local_position_v1`,
  and `VehicleLandDetected` on `/rc1check/fmu/out/vehicle_land_detected`.
- PX4 discovers subscribers for all three input message types:
  `OffboardControlMode`, `TrajectorySetpoint` and `VehicleCommand`.
- A `VEHICLE_CMD_RUN_PREARM_CHECKS` command sent over DDS receives an accepted,
  correctly addressed `VehicleCommandAck` on
  `/rc1check/fmu/out/vehicle_command_ack_v1`.
- The vehicle stays landed and disarmed throughout. No flight setpoints,
  arming commands or mode changes are sent.

For localhost-only DDS testing, `ROS_LOCALHOST_ONLY=1` must be paired with
`PX4_PARAM_UXRCE_DDS_PTCFG=1`. A localhost-only ROS participant did not discover
PX4's default unrestricted participant; this is not a message layout mismatch.
The check also passed with normal unrestricted DDS using
`ROS_LOCALHOST_ONLY=0` and `PX4_PARAM_UXRCE_DDS_PTCFG=0`. The final run restored
the participant configuration to its normal default; all check processes stopped.

Temporary check artifacts (not committed):

- `/tmp/px4_rc1_build.log`
- `/tmp/px4_rc1_ros_build.log`
- `/tmp/px4_rc1_smoke.log`
- `/tmp/px4_rc1_smoke_launch.log`
- `/tmp/px4_rc1_smoke_result.json`

## Limits and known issues

- This validates startup, estimation and DDS connectivity, not takeoff, flight
  handling, Offboard tracking or precision touchdown on RC1.
- PX4 exits cleanly after the check. The installed Gazebo server reports a
  segmentation fault during signal-driven shutdown; runtime startup and DDS
  checks complete before shutdown. The agent is intentionally interrupted too.
- The focused PX4/interface/metadata tests pass (64 tests). The broader retained
  manager and PX4 launch suite has 275 passes and the pre-existing failure
  `test_recovery_climb_times_out_if_altitude_does_not_increase`.
# PX4 DDS message snapshot

This package contains the seven **unmodified** uORB message definitions needed
by the docking manager, copied from PX4-Autopilot **v1.18.0-rc1**, commit
`fca3df865af36124a28c9d607e850f111dbaaea9`. It is a minimal subset of the
standard `px4_msgs` package, not a new wire protocol. Versioned messages retain
their `MESSAGE_VERSION` constants. `firmware_snapshot.json` records the exact
upstream paths and SHA-256 checksums and is installed with the package.

ROS package versions require three numeric components, so `package.xml` uses
`1.18.0`; the description and snapshot manifest identify the RC1 firmware pin.

Do not build this alongside another package named `px4_msgs`. A full upstream
`px4_msgs` package matching the running firmware can replace this subset.
When upgrading firmware, update these definitions from that firmware's `msg/`
and `msg/versioned/` directories together, then regenerate the snapshot manifest
and rebuild the interfaces and dependent packages. The tests always verify the
checksums and compare against the pinned Git commit when it is available locally,
regardless of which firmware version is checked out.

The manager uses these topics with RC1 (before any PX4 namespace prefix):

| Direction | Message | Topic |
| --- | --- | --- |
| Input | `OffboardControlMode` | `/fmu/in/offboard_control_mode` |
| Input | `TrajectorySetpoint` | `/fmu/in/trajectory_setpoint` |
| Input | `VehicleCommand` | `/fmu/in/vehicle_command` |
| Output | `VehicleCommandAck` | `/fmu/out/vehicle_command_ack_v1` |
| Output | `VehicleLandDetected` | `/fmu/out/vehicle_land_detected` |
| Output | `VehicleLocalPosition` | `/fmu/out/vehicle_local_position_v1` |
| Output | `VehicleStatus` | `/fmu/out/vehicle_status_v4` |

These interfaces no longer directly match the old v1.16.2 SITL binary. Use an
RC1 firmware build, or an appropriate PX4 message translation node. Updating this
package does not switch or rebuild the separate PX4 firmware checkout.

The definitions and accompanying LICENSE originate from PX4-Autopilot.

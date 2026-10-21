# Monotonic GUI scene broadcaster

These files are vendored from Gazebo Sim tag `gz-sim8_8.15.0`:

- `src/systems/scene_broadcaster/SceneBroadcaster.cc`
- `src/systems/scene_broadcaster/SceneBroadcaster.hh`
- Repository `LICENSE` (Apache-2.0).

Upstream: <https://github.com/gazebosim/gz-sim/tree/gz-sim8_8.15.0/src/systems/scene_broadcaster>

Original SHA-256 checksums:

```text
579badac1909cb29a8d8bd25d5ea0d24139be29d8fafa184cdaef6cb483a4309  SceneBroadcaster.cc
4bf58ed09dc878fe6662cfbc218f6c6f5dd16fe3d126eb9f872d36a06b0f50a2  SceneBroadcaster.hh
```

Local changes are deliberately limited to:

1. Use `std::chrono::steady_clock` for GUI state publication scheduling instead
   of `system_clock`. WSL UTC corrections can jump backwards by several seconds;
   the original throttle then suppresses GUI updates while physics and rotor
   joints continue running normally.
2. Move the plugin into `gz::sim::docking_systems` and register the unique alias
   `drone_description::MonotonicSceneBroadcaster`, avoiding collisions with the
   installed Gazebo SceneBroadcaster.

The docking world loads `libdrone_scene_broadcaster.so` from this package's
plugin directory. No system library is overwritten. Physics, sensors, state
message timestamps, scene services and publication rates retain upstream behavior.
Both ArduPilot and PX4 launches use this world and receive the fix.

The regression test uses a process-local fake `CLOCK_REALTIME` to reproduce
backwards UTC steps without changing the host clock. It checks that the installed
plugin stalls while this plugin continues sending changing rotor poses in GUI
state messages.

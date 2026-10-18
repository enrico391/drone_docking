# PX4 DDS message snapshot

This package contains the seven **unmodified** uORB message definitions needed
by the docking manager, copied from the workspace's PX4-Autopilot v1.16.2
checkout. It is a minimal subset of the standard `px4_msgs` package, not a new
wire protocol. Versioned messages retain their `MESSAGE_VERSION` constants.

Do not build this alongside another package named `px4_msgs`. A full upstream
`px4_msgs` package matching the running firmware can replace this subset.
When upgrading firmware, update these definitions from that firmware's `msg/`
and `msg/versioned/` directories together. The tests compare this snapshot to
the local firmware when that checkout is available.

The definitions and accompanying LICENSE originate from PX4-Autopilot.

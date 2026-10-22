"""PX4 Offboard velocity sysID node.

Owns one explicit one-shot run: `start` -> airborne check -> warmup-hold
-> Offboard -> chirp/PRBS velocity excitation -> continuous position hold.
Never run alongside flight_cycle_manager (both own Offboard).

Fit offline with the unchanged ArduPilot fitter in FC timebase:
  python3 src/drone_docking/drone_sysid/fit_plant.py runs/x_px4_01 --timebase fc --out models/x_px4_01
"""
import argparse
import math
import os
import queue
import select
import sys
import threading
import time

from .excitation import SysIdConfig, Excitation
from .policy import Phase, OffboardPolicy, HealthSnapshot
from .recorder import CsvRecorder

# Minimum height above the EKF origin to consider the vehicle airborne. Used
# instead of VehicleLandDetected, which reports landed while hovering on this
# airframe (documented in drone_description/config/px4_sitl_docking.parm).
AIRBORNE_MIN_HEIGHT_M = 0.5
# Warn when the hold stream goes quiet this long while PX4 is in Offboard.
STREAM_GAP_WARN_S = 0.4

try:
    from px4_msgs.msg import (
        OffboardControlMode, TrajectorySetpoint, VehicleCommand,
        VehicleCommandAck, VehicleLandDetected, VehicleLocalPosition,
        VehicleStatus,
    )
    HAVE_PX4_MSGS = True
except Exception:
    OffboardControlMode = TrajectorySetpoint = VehicleCommand = None
    VehicleCommandAck = VehicleLandDetected = VehicleLocalPosition = VehicleStatus = None
    HAVE_PX4_MSGS = False


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--axis", choices=["x", "y", "z"], default="x")
    p.add_argument("--out", required=True)
    p.add_argument("--px4-namespace", default="")
    p.add_argument("--amp", type=float, default=None)
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--chirp", type=float, default=50.0)
    p.add_argument("--prbs", type=float, default=45.0)
    p.add_argument("--hover", type=float, default=5.0)
    p.add_argument("--f0", type=float, default=0.1)
    p.add_argument("--f1", type=float, default=2.0)
    p.add_argument("--switch-period", type=float, default=0.2)
    p.add_argument("--radius", type=float, default=15.0)
    p.add_argument("--source-system", type=int, default=254)
    p.add_argument("--source-component", type=int, default=191)
    p.add_argument("--no-stdin", action="store_true")
    p.add_argument("--verbose", action="store_true",
                   help="Print per-second stream/state diagnostics while holding")
    return p.parse_args(argv)


class MixinA:
    def _now(self):
        return time.monotonic()

    def _on_status(self, msg):
        try:
            self._handle_status(msg)
        except Exception as exc:
            self._callback_error = repr(exc)
            print(f"[status callback error] {exc!r}", flush=True)

    def _handle_status(self, msg):
        now = self._now()
        self.snap.status_at = now
        self.snap.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.snap.failsafe = bool(msg.failsafe)
        offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.mode_name = str(msg.nav_state)
        self.offboard = offboard
        if getattr(self, "_conflict_reason", None) is None:
            self._check_conflict()
        self.policy.on_mode(self.snap, offboard,
                            blocked=bool(getattr(self, "_conflict_reason", None)))
        if self.recorder and self.mode_name != getattr(self, "_last_mode", None):
            self.recorder.event(now, "mode", mode=self.mode_name, armed=self.snap.armed)
            self._last_mode = self.mode_name

    def _on_local(self, msg):
        now = self._now()
        s = self.snap
        fc_s = msg.timestamp / 1e6
        if self.last_local_stamp is not None and msg.timestamp <= self.last_local_stamp:
            backward_s = (self.last_local_stamp - msg.timestamp) / 1e6
            # microRTPS can republish the same sample or DDS can swap two
            # datagrams. Drop those; they carry no new information.
            if backward_s < 0.05:
                self._dropped_local = getattr(self, "_dropped_local", 0) + 1
                return
            # A large step backwards is a real clock reset (FC reboot): abort.
            if self.policy.stage not in (Phase.WAIT.value, Phase.DONE.value):
                self.policy.finish("FC position clock reset")
            self.last_local_stamp = msg.timestamp
            return
        self._dropped_local = 0
        self.last_local_stamp = msg.timestamp
        s.local_at, s.local_fc_s = now, fc_s
        s.x, s.y, s.z, s.vx, s.vy, s.vz = msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
        s.xy_valid, s.z_valid = msg.xy_valid, msg.z_valid
        s.v_xy_valid, s.v_z_valid = msg.v_xy_valid, msg.v_z_valid
        s.heading_good = bool(msg.heading_good_for_control)
        s.reset_counters = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        # Height above the EKF origin is the reliable in-air signal here; the
        # land detector is unreliable on this airframe.
        s.airborne = math.isfinite(msg.z) and -msg.z >= AIRBORNE_MIN_HEIGHT_M
        if getattr(self, "_run_counters", None) is not None and self.policy.stage not in (Phase.WAIT.value, Phase.DONE.value):
            if tuple(s.reset_counters) != tuple(self._run_counters):
                self.policy.finish("PX4 estimator reference reset during run")
                return
        self.last_local_msg = msg
        if self.recorder and self.policy.stage == Phase.RUN.value:
            rel_alt = (self.policy.origin_alt - s.z) if self.policy.origin_alt is not None else -s.z
            self.recorder.telemetry(now, fc_s, s.x, s.y, s.z, s.vx, s.vy, s.vz,
                                    self.mode_name, s.armed, rel_alt,
                                    bool(s.xy_valid and s.z_valid and s.v_xy_valid and s.v_z_valid))

    def _on_land(self, msg):
        self.snap.land_at = self._now()
        self.snap.landed = bool(msg.landed)

    def _on_ack(self, msg):
        # Surface PX4's verdict on our Offboard request (e.g. rejected=4
        # usually means no setpoint stream yet or preconditions unmet).
        if msg.command == 176:  # VEHICLE_CMD_DO_SET_MODE
            self.last_mode_ack = (int(msg.result), self._now())
            if self.recorder is not None:
                self.recorder.event(self._now(), "offboard_ack", result=int(msg.result))

    def command(self, text, now):
        text = text.strip().lower()
        if text in ("offboard", "start"):
            if self.pub_cmd is None:
                return "No DDS publishers (test mode)"
            self._check_conflict()
            if self._conflict_reason:
                return self._conflict_reason
            if text == "start" and self.recorder is None:
                return "Recorder not ready yet"
            # The experiment and CSV fitter require PX4 boot time. With DDS
            # timestamp synchronization enabled, telemetry is translated into
            # the Agent's UTC domain, whose corrections can jump either way.
            # 1e14 us is over three years of uptime, below modern epoch time.
            if self.last_local_msg is not None and self.last_local_msg.timestamp >= 100_000_000_000_000:
                return ("Acquisition rejected: DDS timestamps use wall time; set "
                        "UXRCE_DDS_SYNCT=0 and restart PX4 before identification")
            previous = self.policy.stage
            request = self.policy.request_start if text == "start" else self.policy.request_offboard
            err = request(self.snap, now, self.offboard)
            if err:
                return err
            if previous == Phase.WAIT.value:
                self.offboard_cmd_at = None
                self.last_mode_ack = None
                self._run_counters = tuple(self.snap.reset_counters)
                self._hold_position = tuple(self.policy.origin)
                self._hold_yaw = float(self.last_local_msg.heading)
            if self.policy.stage == Phase.RUN.value:
                self._record_start(now)
            return self.policy.reason
        if text in ("quit", "stop"):
            self.policy.finish("operator stopped script")
            return "stopping; stream halted"
        return "unknown command: use offboard | start | quit"

    def _record_start(self, now):
        self.recorder.meta["start_host_mono_s"] = now
        self.recorder.meta["start_fc_boot_s"] = self.policy.start_fc_s
        self.recorder.meta["origin_ned_m"] = list(self.policy.origin)
        self.recorder.meta["outcome"] = "running"
        self.recorder.save_meta()
        self.recorder.event(now, "start", axis=self.args.axis)

    def _on_start(self, request, response):
        was_pending = self.policy.start_pending
        previous_start = self.policy.start_t
        response.message = self.command("start", self._now())
        response.success = ((self.policy.start_pending and not was_pending) or
                            (self.policy.stage == Phase.RUN.value and previous_start is None))
        return response

    def _fc_timestamp(self):
        if self.last_local_msg is not None:
            elapsed = max(0.0, self._now() - self.snap.local_at)
            return int(self.last_local_msg.timestamp + elapsed * 1000000)
        return int(time.time() * 1000000)

    def _publish_hold(self):
        ts = self._fc_timestamp()
        mode = OffboardControlMode(timestamp=ts)
        mode.position, mode.velocity = True, False
        sp = TrajectorySetpoint(timestamp=ts)
        hold = getattr(self, "_hold_position", None) or self.policy.origin
        sp.position = [float(hold[0]), float(hold[1]), float(hold[2])]
        nan3 = [float("nan")] * 3
        sp.velocity = list(nan3)
        sp.acceleration = list(nan3)
        sp.jerk = list(nan3)
        last = self.last_local_msg
        sp.yaw = getattr(self, "_hold_yaw", float(last.heading) if last is not None else float("nan"))
        sp.yawspeed = float("nan")
        self.pub_mode.publish(mode)
        self.pub_sp.publish(sp)

    def _publish_velocity(self, u):
        ts = self._fc_timestamp()
        mode = OffboardControlMode(timestamp=ts)
        mode.position, mode.velocity = False, True
        sp = TrajectorySetpoint(timestamp=ts)
        sp.position = [float("nan")] * 3
        vel = [0.0, 0.0, 0.0]
        vel[self.axis_index] = float(u)
        sp.velocity = vel
        sp.acceleration = [float("nan")] * 3
        sp.jerk = [float("nan")] * 3
        sp.yaw = float("nan")
        sp.yawspeed = 0.0
        self.pub_mode.publish(mode)
        self.pub_sp.publish(sp)

    def _tick_hold(self, now):
        """Stream a fixed position through acquisition, READY and completion.

        The setpoint stream must never stop while PX4 is in Offboard:
        PX4 treats a gap longer than COM_OF_LOSS_T as Offboard loss and
        applies COM_OBL_RC_ACT (LAND in the docking profile). So the hold
        is published on every tick, including the tick that confirms entry
        and every tick while waiting for `start`.
        """
        last_publish = getattr(self, "_last_publish_at", None)
        if (self.policy.stage == Phase.WARMUP.value and self.offboard_cmd_at is None and
                (last_publish is None or now - last_publish > STREAM_GAP_WARN_S)):
            # Measure uninterrupted publishing, not time since a button press
            # that might have been followed by a stalled executor.
            self.policy.warmup_t = now
        if self.offboard and last_publish is not None and now - last_publish > STREAM_GAP_WARN_S:
            print(
                f"[stream gap] {now - last_publish:.2f} s without a setpoint while in "
                "Offboard; PX4 will apply COM_OBL_RC_ACT after COM_OF_LOSS_T",
                flush=True)
        # Keep the stream alive on every tick, before any state change.
        try:
            self._publish_hold()
        except Exception as exc:  # never let one bad tick kill the stream
            self._publish_error = repr(exc)
            self.policy.reason = f"hold publish failed: {exc!r}"
            if self.policy.stage == Phase.WARMUP.value and self.offboard_cmd_at is None:
                self.policy.warmup_t = now
            return
        self._published = getattr(self, "_published", 0) + 1
        self._last_publish_at = now
        self._log_stream(now)
        if self.policy.stage != Phase.WARMUP.value or self.offboard:
            return
        # Exactly one mode request after a continuous warmup. Never fight LAND
        # or a pilot override with recurring Offboard requests.
        if (self.policy.warmup_t is not None and
                now - self.policy.warmup_t >= self.policy.warmup_s and
                self.offboard_cmd_at is None):
            self._send_offboard_request()
            self.offboard_cmd_at = now
            self.policy.reason = "Offboard requested once; waiting for mode confirmation"

    def _tick(self):
        try:
            self._tick_impl()
        except Exception as exc:  # a raising timer callback would kill the stream
            self._tick_error = repr(exc)
            print(f"[tick error] {exc!r}", flush=True)

    def _tick_impl(self):
        now = self._now()
        # Stdin is read on a worker, but all control mutations run on the ROS
        # executor, like the start service and telemetry/timer callbacks.
        pending = getattr(self, "_commands", None)
        while pending is not None and not pending.empty():
            print(self.command(pending.get_nowait(), now), flush=True)
        if self.policy.stage == Phase.DONE.value or self.pub_mode is None:
            return
        self.policy.tick(self.snap, now, self.offboard)
        if self.policy.stage == Phase.DONE.value:
            return
        if self.policy.stage == Phase.WAIT.value:
            return
        if self.policy.stage == Phase.READY.value and self.policy.start_pending:
            self.policy.begin_run(self.snap, now)
            self._record_start(now)
        if self.policy.stage in (Phase.WARMUP.value, Phase.READY.value, Phase.HOLD.value):
            self._tick_hold(now)
            return
        if self.policy.stage == Phase.RUN.value:
            if not self.policy.can_send(self.snap, now, self.offboard):
                return
            elapsed = self.policy.elapsed(self.snap, now)
            phase, u = self.excitation.value(elapsed)
            if phase == "COMPLETE":
                # Hold HERE, not at the takeoff/start point. Publish on this
                # very tick, and keep the node/executor alive indefinitely.
                self._hold_position = (self.snap.x, self.snap.y, self.snap.z)
                self._hold_yaw = float(self.last_local_msg.heading)
                self.policy.complete()
                self._tick_hold(now)
                if self.recorder:
                    self.recorder.event(now, "complete", hold_ned_m=list(self._hold_position))
                    self.recorder.close("complete")
                    self.recorder = None
                return
            if phase != self.last_phase:
                if self.recorder:
                    self.recorder.event(now, "phase", phase=phase)
                self.last_phase = phase
            last_publish = getattr(self, "_last_publish_at", None)
            if last_publish is not None and now - last_publish > 0.2:
                self.policy.finish("setpoint scheduler overrun")
                return
            # The steady ROS timer already sets the rate. A second deadline
            # gate skipped ticks arriving just before a nominal deadline and
            # reduced the 50 Hz stream to roughly 40 Hz in recorded runs.
            self._publish_velocity(u)
            self._last_publish_at = now
            self._published = getattr(self, "_published", 0) + 1
            self._log_stream(now)
            if self.recorder:
                vel = [0.0, 0.0, 0.0]
                vel[self.axis_index] = u
                self.recorder.command(now, elapsed, phase, vel[0], vel[1], vel[2], self.snap.local_fc_s)

    def _log_stream(self, now):
        """Emit one diagnostic line per second (--verbose) to expose stream gaps.

        A gap longer than PX4's COM_OF_LOSS_T makes PX4 leave Offboard and apply
        COM_OBL_RC_ACT (LAND). Printing the actual publish rate makes that gap
        visible instead of looking like PX4 landing on its own.
        """
        if not getattr(getattr(self, "args", None), "verbose", False):
            return
        last = getattr(self, "_verbose_at", None)
        if last is not None and now - last < 1.0:
            return
        self._verbose_at = now
        published = getattr(self, "_published", 0)
        previous = getattr(self, "_verbose_count", 0)
        self._verbose_count = published
        rate = (published - previous) / max(1e-6, now - last) if last is not None else float("nan")
        print(
            f"[stream] stage={self.policy.stage} mode={self.mode_name} "
            f"offboard={self.offboard} armed={self.snap.armed} landed={self.snap.landed} "
            f"published={published} rate={rate:.1f}Hz",
            flush=True)

    def _check_conflict(self):
        """Detect a second publisher on the Offboard setpoint topic.

        Two publishers on /fmu/in/trajectory_setpoint (e.g. this node plus
        flight_cycle_manager) deliver alternating setpoints to PX4. PX4 sees
        inconsistent input, trips its Offboard failsafe and applies
        COM_OBL_RC_ACT, which looks like an unexplained switch to LAND.
        Streaming is blocked until this node is the only publisher.
        """
        if self.pub_sp is None:
            return
        try:
            topic = self.pub_sp.topic_name
            publishers = self.count_publishers(topic)
        except Exception:
            return
        previous = self._conflict_reason
        if publishers > 1:
            self._conflict_reason = (
                f"BLOCKED: {publishers} publishers on {topic}; "
                "another node (flight_cycle_manager?) owns Offboard. "
                "Stop it before starting identification.")
            if self.policy.stage not in (Phase.WAIT.value, Phase.DONE.value):
                self.policy.finish(self._conflict_reason)
        else:
            self._conflict_reason = None
        if self.recorder is not None and previous != self._conflict_reason:
            self.recorder.event(self._now(), "offboard_conflict",
                                publishers=publishers, topic=topic)

    def _send_offboard_request(self):
        if self.pub_cmd is None or self.last_local_msg is None:
            return
        msg = VehicleCommand()
        msg.timestamp = self._fc_timestamp()
        msg.command = 176
        msg.param1, msg.param2 = 1.0, 6.0
        msg.target_system, msg.target_component = 1, 1
        msg.source_system = self.args.source_system
        msg.source_component = self.args.source_component
        msg.from_external = True
        self.pub_cmd.publish(msg)


def build_node(args, recorder=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    class SysIdNode(Node, MixinA):
        def __init__(self):
            Node.__init__(self, "px4_sysid")
            self.args = args
            amp = args.amp if args.amp is not None else (0.2 if args.axis == "z" else 0.3)
            self.cfg = SysIdConfig(amplitude=amp, rate=args.rate, chirp_s=args.chirp,
                                   prbs_s=args.prbs, hover_s=args.hover, f0=args.f0,
                                   f1=args.f1, switch_s=args.switch_period, radius_m=args.radius)
            self.excitation = Excitation(self.cfg)
            self.policy = OffboardPolicy(config=self.cfg)
            self.snap = HealthSnapshot()
            self.offboard = False
            self.mode_name = "UNKNOWN"
            self.recorder = recorder
            self.last_phase = None
            self.last_local_msg = None
            self.last_local_stamp = None
            self._run_counters = None
            self.axis_index = {"x": 0, "y": 1, "z": 2}[args.axis]
            self._reported = ""
            self._conflict_reason = None
            self._commands = queue.SimpleQueue()
            prefix = args.px4_namespace.strip("/")
            pfx = f"/{prefix}/" if prefix else "/"
            sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            if HAVE_PX4_MSGS:
                self.pub_mode = self.create_publisher(OffboardControlMode, f"{pfx}fmu/in/offboard_control_mode", 10)
                self.pub_sp = self.create_publisher(TrajectorySetpoint, f"{pfx}fmu/in/trajectory_setpoint", 10)
                self.pub_cmd = self.create_publisher(VehicleCommand, f"{pfx}fmu/in/vehicle_command", 10)
                self.create_subscription(VehicleStatus, f"{pfx}fmu/out/vehicle_status_v4", self._on_status, sensor_qos)
                self.create_subscription(VehicleLocalPosition, f"{pfx}fmu/out/vehicle_local_position_v1",
                                         self._on_local, sensor_qos)
                self.create_subscription(VehicleLandDetected, f"{pfx}fmu/out/vehicle_land_detected",
                                         self._on_land, sensor_qos)
                try:
                    from px4_msgs.msg import VehicleCommandAck as _Ack
                    self.create_subscription(_Ack, f"{pfx}fmu/out/vehicle_command_ack_v1",
                                             self._on_ack, sensor_qos)
                except Exception:
                    pass
            else:
                self.pub_mode = self.pub_sp = self.pub_cmd = None
            # Default ROS/system-time timers pause after backwards UTC steps
            # (observed under WSL). PX4 still runs and declares Offboard loss.
            self._stream_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(1.0 / self.cfg.rate, self._tick,
                                          clock=self._stream_clock)
            self.offboard_cmd_at = None
            self.last_mode_ack = None
            self._last_mode = None
            from std_srvs.srv import Trigger
            self.create_service(Trigger, "~/start", self._on_start)
            if HAVE_PX4_MSGS:
                # Refuse to stream if another node already publishes Offboard
                # setpoints. Two publishers on /fmu/in/* make PX4 see
                # inconsistent setpoints and trip its Offboard failsafe.
                self.create_timer(1.0, self._check_conflict, clock=self._stream_clock)

    return SysIdNode()


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    try:
        SysIdConfig(amplitude=args.amp if args.amp is not None else (0.2 if args.axis == "z" else 0.3),
                    rate=args.rate, chirp_s=args.chirp, prbs_s=args.prbs, hover_s=args.hover,
                    f0=args.f0, f1=args.f1, switch_s=args.switch_period, radius_m=args.radius)
    except ValueError as exc:
        raise SystemExit(f"Invalid config: {exc}")
    from pathlib import Path
    out = Path(args.out)
    if out.exists():
        raise SystemExit("Output directory already exists; choose a new --out")
    import rclpy
    rclpy.init()
    node = build_node(args)
    try:
        node.recorder = CsvRecorder(out, args.axis, node.cfg, extra_meta={"px4_namespace": args.px4_namespace})
    except Exception as exc:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"Cannot create output: {exc}")
    print("Take off, hold in Position/Hold, then type `start` (or call /px4_sysid/start).", flush=True)
    print("After completion, position hold continues. Switch to Position/Hold before quitting.", flush=True)
    print("NEVER run alongside flight_cycle_manager: both own Offboard.", flush=True)

    def stdin_loop():
        buf = ""
        try:
            while rclpy.ok() and node.policy.stage != Phase.DONE.value:
                ready = select.select([sys.stdin], [], [], 0.2)[0]
                if ready:
                    chunk = os.read(sys.stdin.fileno(), 4096).decode(errors="replace")
                    if not chunk:
                        break
                    for char in chunk:
                        if char in "\r\n":
                            if buf.strip():
                                node._commands.put(buf)
                                buf = ""
                        elif len(buf) < 80:
                            buf += char
        except Exception:
            pass

    if not args.no_stdin:
        threading.Thread(target=stdin_loop, daemon=True).start()
    try:
        while rclpy.ok() and node.policy.stage != Phase.DONE.value:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.policy.reason != node._reported:
                print(node.policy.reason, flush=True)
                node._reported = node.policy.reason
    except KeyboardInterrupt:
        node.policy.finish("operator stopped script")
    finally:
        outcome = node.policy.reason
        try:
            if node.recorder is not None:
                node.recorder.close(outcome if node.policy.stage == Phase.DONE.value else "interrupted")
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    print(f"Saved {args.out}: reason={outcome}", flush=True)


if __name__ == "__main__":
    main()

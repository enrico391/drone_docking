"""Unit tests for px4_sysid (no ROS required)."""
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from px4_sysid.excitation import SysIdConfig, Excitation, prbs_sequence
from px4_sysid.policy import OffboardPolicy, HealthSnapshot, Phase
from px4_sysid.recorder import CsvRecorder
import px4_sysid.px4_sysid_node as node_mod


def healthy(now):
    return HealthSnapshot(offboard=True, armed=True, landed=False, airborne=True,
                          failsafe=False,
                          xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
                          heading_good=True, x=0.0, y=0.0, z=-5.0, vx=0.0, vy=0.0, vz=0.0,
                          roll=0.0, pitch=0.0, local_at=now, local_fc_s=100.0,
                          status_at=now, land_at=now)


class TestExcitation(unittest.TestCase):
    def test_prbs_period(self):
        seq = prbs_sequence()
        self.assertEqual(len(seq), 255)
        self.assertTrue(all(v in (1.0, -1.0) for v in seq))

    def test_schedule_boundaries(self):
        c = SysIdConfig()
        e = Excitation(c)
        self.assertEqual(e.value(0.0)[0], "PRE_HOVER")
        self.assertEqual(e.value(c.hover_s + 0.01)[0], "CHIRP")
        self.assertEqual(e.value(2 * c.hover_s + c.chirp_s + 0.01)[0], "PRBS")
        self.assertEqual(e.value(e.total + 1.0)[0], "COMPLETE")

    def test_bad_config(self):
        with self.assertRaises(ValueError):
            SysIdConfig(amplitude=2.0)


class TestPolicy(unittest.TestCase):
    def test_start_automatically_acquires_offboard(self):
        p = OffboardPolicy(config=SysIdConfig())
        err = p.request_start(healthy(10.0), 10.0, offboard=False)
        self.assertIsNone(err)
        self.assertEqual(p.stage, Phase.WARMUP.value)
        self.assertTrue(p.start_pending)
        self.assertIsNone(p.start_fc_s)

    def test_start_on_ground_is_rejected(self):
        p = OffboardPolicy(config=SysIdConfig())
        s = healthy(10.0)
        s.airborne = False
        self.assertIn("not airborne", p.request_start(s, 10.0, offboard=False))
        self.assertEqual(p.stage, Phase.WAIT.value)

    def test_start_and_override_latch(self):
        p = OffboardPolicy(config=SysIdConfig())
        s = healthy(10.0)
        self.assertIsNone(p.request_start(s, 10.0, offboard=True))
        self.assertEqual(p.stage, Phase.RUN.value)
        p.on_mode(s, offboard=False)
        self.assertEqual(p.stage, Phase.DONE.value)

    def test_health_gates(self):
        p = OffboardPolicy(config=SysIdConfig())
        s = healthy(10.0)
        # The land detector is unreliable on this airframe; airborne comes from
        # height above the EKF origin instead, so it is what must gate control.
        s.airborne = False
        s.landed = True
        self.assertIn("airborne", p.health_reason(s, 10.0))
        s.airborne = True
        s.landed = False
        s.armed = False
        self.assertIn("disarmed", p.health_reason(s, 10.0))

    def test_elapsed_uses_fc_clock(self):
        p = OffboardPolicy(config=SysIdConfig())
        s = healthy(10.0)
        p.request_start(s, 10.0, offboard=True)
        s.local_fc_s = 101.5
        self.assertAlmostEqual(p.elapsed(s, 11.0), 1.5)


class FakePublisher:
    topic_name = "/fmu/in/trajectory_setpoint"

    def __init__(self):
        self.messages = []

    @property
    def count(self):
        return len(self.messages)

    def publish(self, msg):
        self.messages.append(msg)


class TestNode(unittest.TestCase):
    """Exercise real control/publish code with ROS-independent message doubles."""

    def setUp(self):
        # Explicit doubles work both with and without a sourced ROS overlay.
        self.messages = patch.multiple(
            node_mod, OffboardControlMode=types.SimpleNamespace,
            TrajectorySetpoint=types.SimpleNamespace, VehicleCommand=types.SimpleNamespace,
            VehicleStatus=types.SimpleNamespace(ARMING_STATE_ARMED=2, NAVIGATION_STATE_OFFBOARD=14))
        self.messages.start()
        self.addCleanup(self.messages.stop)
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)

        class Host(node_mod.MixinA):
            def count_publishers(self, topic):
                return 1

        self.h = h = Host()
        h.cfg = SysIdConfig()
        h.policy = OffboardPolicy(config=h.cfg)
        h.snap = healthy(20.0)
        h.snap.x, h.snap.y = 1.0, 2.0
        h.last_local_msg = types.SimpleNamespace(timestamp=100_000_000, heading=0.3)
        h.last_local_stamp = h.last_local_msg.timestamp
        h.recorder = CsvRecorder(Path(temp.name) / "run", "x", h.cfg)
        self.recorder = h.recorder
        self.addCleanup(lambda: h.recorder.close("test cleanup") if h.recorder else None)
        h.mode_name = "4"
        h._run_counters = None
        h.offboard = False
        h.pub_mode, h.pub_sp, h.pub_cmd = FakePublisher(), FakePublisher(), FakePublisher()
        h._now = lambda: 20.0
        h.axis_index = 0
        h._conflict_reason = None
        h.offboard_cmd_at = None
        h.last_mode_ack = None
        h.last_phase = None
        h.args = types.SimpleNamespace(axis="x", verbose=False, source_system=254, source_component=191)
        h.excitation = Excitation(h.cfg)

    def tick(self, now):
        h = self.h
        h._now = lambda: now
        h.snap.local_at = h.snap.status_at = h.snap.land_at = now
        h._tick_impl()

    def stream_until(self, now):
        last = getattr(self.h, "_last_publish_at", 20.0)
        while last + 0.02 < now - 1e-9:
            last += 0.02
            self.tick(last)
        self.tick(now)

    def test_wait_is_passive_even_if_external_offboard_is_active(self):
        self.h.offboard = True
        self.tick(20.0)
        self.assertEqual(self.h.policy.stage, Phase.WAIT.value)
        self.assertEqual(self.h.pub_mode.count, 0)
        self.assertEqual(self.h.pub_cmd.count, 0)

    def test_start_rejected_on_ground_without_publishing(self):
        self.h.snap.airborne = False
        self.assertIn("not airborne", self.h.command("start", 20.0))
        self.tick(20.0)
        self.assertEqual(self.h.pub_mode.count, 0)
        self.assertEqual(self.h.pub_cmd.count, 0)

    def test_start_rejects_dds_wall_clock_before_streaming(self):
        h = self.h
        h.last_local_msg.timestamp = 1_792_693_194_497_836
        self.assertIn("UXRCE_DDS_SYNCT", h.command("start", 20.0))
        self.tick(20.0)
        self.assertEqual(h.policy.stage, Phase.WAIT.value)
        self.assertEqual(h.pub_sp.count, 0)
        self.assertEqual(h.pub_cmd.count, 0)

    def test_start_warms_up_requests_once_then_runs_after_confirmation(self):
        h = self.h
        h.command("start", 20.0)
        self.tick(20.0)
        self.stream_until(21.49)
        self.assertEqual(h.pub_cmd.count, 0)
        self.tick(21.5)
        self.assertEqual(h.pub_cmd.count, 1)
        self.assertEqual(h.pub_cmd.messages[0].param2, 6.0)
        self.stream_until(22.6)
        self.assertEqual(h.pub_cmd.count, 1)
        self.assertIsNone(h.policy.start_fc_s)
        h._handle_status(types.SimpleNamespace(arming_state=2, nav_state=14, failsafe=False))
        self.assertEqual(h.policy.stage, Phase.READY.value)
        h.snap.local_fc_s = 102.6
        before = h.pub_sp.count
        self.tick(22.62)
        self.assertEqual(h.policy.stage, Phase.RUN.value)
        self.assertEqual(h.policy.start_fc_s, 102.6)
        self.assertEqual(h.pub_sp.count, before + 1)
        self.assertEqual(h.pub_sp.messages[-1].velocity, [0.0, 0.0, 0.0])
        self.assertEqual(h.recorder.meta["start_fc_boot_s"], 102.6)

    def test_no_mode_retry_after_timeout(self):
        h = self.h
        h.command("start", 20.0)
        self.tick(20.0)
        self.stream_until(21.5)
        self.tick(32.0)
        self.assertEqual(h.policy.stage, Phase.DONE.value)
        self.assertIn("timed out", h.policy.reason)
        self.assertEqual(h.pub_cmd.count, 1)

    def test_warmup_requires_real_continuous_stream(self):
        h = self.h
        h.command("start", 20.0)
        self.tick(22.0)
        self.assertEqual(h.pub_cmd.count, 0)
        self.tick(23.0)
        self.assertEqual(h.policy.warmup_t, 23.0)
        self.assertEqual(h.pub_cmd.count, 0)
        self.stream_until(24.5)
        self.assertEqual(h.pub_cmd.count, 1)

    def test_mode_loss_in_ready_run_and_hold_is_latched(self):
        h = self.h
        for stage in (Phase.READY.value, Phase.RUN.value, Phase.HOLD.value):
            with self.subTest(stage=stage):
                h.policy = OffboardPolicy(config=h.cfg, stage=stage, offboard_acquired=True)
                h.offboard = False
                before = h.pub_mode.count
                self.tick(20.0)
                self.assertEqual(h.policy.stage, Phase.DONE.value)
                h.offboard = True
                self.tick(20.02)
                self.assertEqual(h.pub_mode.count, before)
                self.assertEqual(h.pub_cmd.count, 0)

    def test_duplicate_start_cannot_reset_the_run(self):
        h = self.h
        h.offboard = True
        h.command("start", 20.0)
        self.assertIn("unavailable", h.command("start", 21.0))
        self.assertIn("unavailable", h.command("offboard", 21.0))
        self.assertEqual(h.policy.start_t, 20.0)
        self.assertEqual(h.policy.stage, Phase.RUN.value)

    def test_start_service_accepts_pending_but_rejects_duplicate(self):
        response = self.h._on_start(None, types.SimpleNamespace())
        self.assertTrue(response.success)
        self.assertFalse(self.h._on_start(None, types.SimpleNamespace()).success)

    def test_heading_loss_during_acquisition_cancels_pending_start(self):
        h = self.h
        h.command("start", 20.0)
        h.snap.heading_good = False
        h.offboard = True
        self.tick(20.02)
        self.assertEqual(h.policy.stage, Phase.DONE.value)
        self.assertFalse(h.policy.start_pending)
        self.assertIn("heading", h.policy.reason)
        self.assertEqual(h.pub_sp.count, 0)

    def test_velocity_ticks_update_stream_timestamp(self):
        h = self.h
        h.offboard = True
        h.command("start", 20.0)
        self.tick(20.0)
        self.tick(20.02)
        self.assertEqual(h._last_publish_at, 20.02)

    def test_velocity_stream_does_not_skip_early_timer_ticks(self):
        h = self.h
        h.offboard = True
        h.command("start", 20.0)
        for now in (20.0, 20.0199, 20.0401, 20.0599, 20.0801):
            self.tick(now)
        self.assertEqual(h.pub_sp.count, 5)

    def test_velocity_stream_aborts_on_executor_stall(self):
        h = self.h
        h.offboard = True
        h.command("start", 20.0)
        self.tick(20.0)
        self.tick(20.3)
        self.assertEqual(h.policy.stage, Phase.DONE.value)
        self.assertIn("overrun", h.policy.reason)
        self.assertEqual(h.pub_sp.count, 1)

    def test_conflicting_publisher_blocks_start(self):
        h = self.h
        h.count_publishers = lambda topic: 2
        self.assertIn("BLOCKED", h.command("start", 20.0))
        self.assertEqual(h.policy.stage, Phase.WAIT.value)
        self.tick(20.0)
        self.assertEqual(h.pub_mode.count, 0)

    def test_success_holds_finishing_position_and_closes_recording(self):
        import json
        h = self.h
        h.offboard = True
        h.command("start", 20.0)
        h.snap.x, h.snap.y, h.snap.z = 3.0, 4.0, -5.5
        h.snap.local_fc_s = h.policy.start_fc_s + h.excitation.total
        self.tick(20.02)
        self.assertEqual(h.policy.stage, Phase.HOLD.value)
        self.assertEqual(h.pub_sp.count, 1)  # no gap on the completion tick
        self.assertEqual(h.pub_sp.messages[-1].position, [3.0, 4.0, -5.5])
        self.assertTrue(h.pub_mode.messages[-1].position)
        self.assertTrue(all(math.isnan(v) for v in h.pub_sp.messages[-1].velocity))
        self.assertIsNone(h.recorder)
        self.assertEqual(json.loads((self.recorder.out / "metadata.json").read_text())["outcome"], "complete")
        self.assertTrue(all(f.closed for f in self.recorder.files.values()))
        # Changes in measured position must NOT move the hold target.
        h.snap.x, h.snap.y = 3.1, 4.1
        h.last_local_msg.heading = 0.5
        for i in range(100):
            self.tick(20.04 + i * 0.02)
        self.assertEqual(h.pub_sp.count, 101)
        self.assertEqual(h.pub_sp.messages[-1].position, [3.0, 4.0, -5.5])
        self.assertEqual(h.pub_sp.messages[-1].yaw, 0.3)
        self.assertEqual(h.pub_cmd.count, 0)
        self.assertEqual(h.policy.stage, Phase.HOLD.value)

    def test_stale_telemetry_stops_hold(self):
        h = self.h
        h.offboard = True
        h.command("start", 20.0)
        h.policy.complete()
        h._now = lambda: 23.0
        h._tick_impl()
        self.assertEqual(h.policy.stage, Phase.DONE.value)
        self.assertIn("stale", h.policy.reason)

    def test_hold_stream_never_stops_in_offboard(self):
        """PX4 lands after COM_OF_LOSS_T if the setpoint stream stops.

        The hold must keep publishing every tick while waiting for `start`,
        including the tick that confirms Offboard entry.
        """
        h = self.h
        h.command("offboard", 20.0)
        # Entering Offboard must not skip the hold publish on that same tick.
        h.offboard = True
        self.tick(20.0)
        self.assertEqual(h.policy.stage, Phase.READY.value)
        self.assertEqual(h.pub_mode.count, 1)
        self.assertEqual(h.pub_sp.count, 1)
        # And it must keep streaming while idle in READY, indefinitely.
        for i in range(50):
            self.tick(20.0 + i * 0.02)
        self.assertEqual(h.policy.stage, Phase.READY.value)
        self.assertEqual(h.pub_mode.count, 51)
        self.assertEqual(h.pub_sp.count, 51)

    def test_hold_streams_even_when_land_detector_is_wrong(self):
        """VehicleLandDetected reports landed while hovering on this airframe.

        Gating the hold stream on it stopped the setpoints, so PX4 declared
        Offboard loss after COM_OF_LOSS_T and applied COM_OBL_RC_ACT (LAND).
        The stream must depend on armed, not on the land detector.
        """
        h = self.h
        # Land detector says landed while the vehicle is hovering.
        h.snap.landed = True
        h.offboard = True
        h.command("offboard", 20.0)

        for i in range(25):
            self.tick(20.0 + i * 0.02)
        self.assertEqual(h.pub_mode.count, 25)
        self.assertEqual(h.pub_sp.count, 25)

    def test_single_swapped_sample_is_dropped_not_fatal(self):
        import types
        import px4_sysid.px4_sysid_node as node_mod

        class FakeMsg:
            def __init__(self, ts):
                self.timestamp = ts
                self.x = self.y = 0.0
                self.z = -5.0
                self.vx = self.vy = self.vz = 0.0
                self.xy_valid = self.z_valid = True
                self.v_xy_valid = self.v_z_valid = True
                self.heading_good_for_control = True
                self.xy_reset_counter = self.z_reset_counter = 0
                self.heading_reset_counter = 0

        h = types.SimpleNamespace(
            snap=HealthSnapshot(), policy=OffboardPolicy(config=SysIdConfig()),
            last_local_stamp=None, last_local_msg=None, recorder=None,
            mode_name="14", _run_counters=None,
            _now=lambda: 10.0,
        )
        h.snap.status_at = h.snap.local_at = h.snap.land_at = 10.0
        h.policy.stage = Phase.RUN.value
        node_mod.MixinA._on_local(h, FakeMsg(1_000_000))
        node_mod.MixinA._on_local(h, FakeMsg(1_000_000))  # duplicate
        node_mod.MixinA._on_local(h, FakeMsg(999_999))    # swapped
        self.assertEqual(h.policy.stage, Phase.RUN.value)
        node_mod.MixinA._on_local(h, FakeMsg(1_020_000))
        self.assertEqual(h.snap.local_fc_s, 1.02)
        # A real reset (large step backwards) still aborts.
        node_mod.MixinA._on_local(h, FakeMsg(100_000))
        self.assertEqual(h.policy.stage, Phase.DONE.value)


if __name__ == "__main__":
    unittest.main()

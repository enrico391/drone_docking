import tempfile
import csv
import unittest
from pathlib import Path
from types import SimpleNamespace

from sysid_core import Config
from log_flight import Recorder, parser


class Message(SimpleNamespace):
    def get_type(self): return self.kind
    def get_srcSystem(self): return getattr(self, "sys", 1)
    def get_srcComponent(self): return getattr(self, "comp", 1)
    def to_dict(self): return vars(self).copy()


class FakeMav:
    def __init__(self): self.sent = []
    def __getattr__(self, name):
        def send(*args): self.sent.append((name, args))
        return send


class FakeLink:
    target_system = 1
    target_component = 1
    def __init__(self): self.mav = FakeMav()


def hb(mode):
    return Message(kind="HEARTBEAT", custom_mode={"STABILIZE":0,"GUIDED":4,"LOITER":5,"LAND":9}[mode],
                   base_mode=128, system_status=4)


class RunnerTests(unittest.TestCase):
    def sim_config(self):
        return Config(sim_time=True, hover_s=.5, chirp_s=1., prbs_s=1.)

    def sim_fresh(self, r, t, fc, rate=.5):
        # FC advances at half wall-clock speed; replies arrive in 4 ms.
        sent = t-.004
        stamp = int(sent*1e9)
        r.pending_sync[stamp] = sent
        r.ingest(Message(kind="TIMESYNC", ts1=stamp, tc1=int((fc-rate*.002)*1e9)), t)
        r.ingest(hb("GUIDED"), t)
        r.ingest(Message(kind="EXTENDED_SYS_STATE", landed_state=2), t)
        r.ingest(Message(kind="EKF_STATUS_REPORT", flags=1|2|4|16|32), t)
        r.ingest(Message(kind="GLOBAL_POSITION_INT", time_boot_ms=round(fc*1000), relative_alt=5000), t)
        r.ingest(Message(kind="ATTITUDE", time_boot_ms=round(fc*1000), roll=0., pitch=0.,
                         yaw=0., rollspeed=0., pitchspeed=0., yawspeed=0.), t)
        r.ingest(Message(kind="LOCAL_POSITION_NED", time_boot_ms=round(fc*1000),
                         x=0., y=0., z=-5., vx=0., vy=0., vz=0.), t)

    def sim_advance(self, r, t, fc):
        for wall, boot in [(t-.2,fc-.1),(t-.1,fc-.05),(t,fc)]:
            self.sim_fresh(r, wall, boot)

    def test_simulation_starts_and_excites_in_fc_seconds(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, self.sim_config())
            for t, fc in [(100., 50.), (100.1, 50.05), (100.2, 50.1)]:
                self.sim_fresh(r, t, fc)
            self.assertIn("accepted", r.command("start", 100.2))
            r.step(100.2)
            self.sim_advance(r, 102., 51.)
            r.next_send = 102.
            r.step(102.)
            r.close()
            with (r.out/"commands.csv").open() as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(rows[-1]["phase"], "CHIRP")
            self.assertAlmostEqual(float(rows[-1]["elapsed_s"]), .9, places=6)
            self.assertAlmostEqual(float(rows[-1]["fc_boot_s"]), 51., places=6)

    def test_simulation_completes_after_simulated_duration(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, self.sim_config())
            for t, fc in [(100., 50.), (100.1, 50.05), (100.2, 50.1)]:
                self.sim_fresh(r, t, fc)
            self.assertIn("accepted", r.command("start", 100.2))
            r.step(100.2)
            self.sim_advance(r, 107.3, 53.65)
            r.step(107.3)
            self.assertEqual(r.policy.reason, "complete")
            self.assertEqual(r.link.mav.sent[-1], ("set_mode_send", (1, 1, 5)))
            r.close()

    def test_simulation_still_rejects_backlog_and_stale_clock(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, self.sim_config())
            for t, fc in [(100., 50.), (100.1, 50.05), (100.2, 50.1)]:
                self.sim_fresh(r, t, fc)
            r.state.pos_boot_s = 49.
            self.assertIn("sample age", r.command("start", 100.2))
            self.assertIn("synchronization", r.command("start", 100.8))
            self.assertFalse(r.link.mav.sent)
            r.close()

    def test_simulation_clock_reset_and_pause_stop_the_run(self):
        for fc in (49., 50.1):
            with self.subTest(fc=fc), tempfile.TemporaryDirectory() as d:
                r = self.make(d, self.sim_config())
                for t, stamp in [(100., 50.), (100.1, 50.05), (100.2, 50.1)]:
                    self.sim_fresh(r, t, stamp)
                self.assertIn("accepted", r.command("start", 100.2))
                r.step(100.2)
                self.sim_fresh(r, 100.3, fc)
                r.step(100.3)
                self.assertEqual(r.policy.stage, "DONE")
                self.assertEqual(r.link.mav.sent[-1], ("set_mode_send", (1,1,5)))
                self.assertEqual(len(r.link.mav.sent), 2)
                r.close()

    def test_simulation_clock_needs_recent_acquisition_samples(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, self.sim_config())
            for t, fc in [(100.,50.),(100.1,50.05),(110.,55.)]:
                self.sim_fresh(r,t,fc)
            self.assertIn('synchronization', r.command('start',110.))
            self.assertFalse(r.link.mav.sent)
            r.close()

    def test_simulation_pilot_override_is_latched(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, self.sim_config())
            for t, fc in [(100., 50.), (100.1, 50.05), (100.2, 50.1)]:
                self.sim_fresh(r, t, fc)
            self.assertIn("accepted", r.command("start", 100.2))
            r.step(100.2)
            r.ingest(hb("LOITER"), 100.3)
            r.ingest(hb("GUIDED"), 100.3)
            r.step(100.3)
            r.shutdown("exit", 100.3)
            self.assertEqual(len(r.link.mav.sent), 1)
            r.close()

    def test_simulation_clock_failure_before_queued_override_sends_no_cleanup(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, self.sim_config())
            for t, fc in [(100.,50.),(100.1,50.05),(100.2,50.1)]:
                self.sim_fresh(r,t,fc)
            self.assertIn('accepted',r.command('start',100.2))
            r.step(100.2)
            self.sim_advance(r,102.,51.)
            r.next_send=102.
            r.step(102.)
            self.assertGreater(abs(r.link.mav.sent[-1][1][8]),.01)
            sends=len(r.link.mav.sent)
            self.sim_fresh(r,102.1,51.)
            r.ingest(hb('LOITER'),102.1)
            r.ingest(hb('GUIDED'),102.1)
            r.step(102.1)
            self.assertEqual(len(r.link.mav.sent),sends)
            self.assertIn('override',r.policy.reason)
            r.close()

    def test_simulation_backward_command_clock_requests_loiter(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, self.sim_config())
            for t, fc in [(100.,50.),(100.1,50.05),(100.2,50.1)]:
                self.sim_fresh(r,t,fc)
            self.assertIn('accepted',r.command('start',100.2))
            r.step(100.2)
            self.sim_advance(r,102.,51.)
            r.next_send=102.
            r.step(102.)
            self.assertGreater(abs(r.link.mav.sent[-1][1][8]),.01)
            r.state.clock_offset_s-=.05
            r.send_velocity(102.001,'CHIRP',0.)
            self.assertEqual(r.policy.reason,'FC command clock moved backwards')
            self.assertEqual(r.link.mav.sent[-1],('set_mode_send',(1,1,5)))
            r.close()

    def test_hardware_clock_still_rejects_simulation_drift(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, Config())
            for t, fc in [(100., 50.), (100.1, 50.05), (100.2, 50.1)]:
                self.sim_fresh(r, t, fc)
            self.assertIn("synchronization", r.command("start", 100.2))
            self.assertFalse(r.link.mav.sent)
            r.close()

    def test_simulation_retains_flight_health_checks(self):
        for change in ({'armed':False}, {'in_air':False}, {'ekf_ok':False},
                       {'relative_alt':2.}, {'roll':1.}, {'failsafe':True}):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as d:
                r = self.make(d, self.sim_config())
                for t, fc in [(100., 50.), (100.1, 50.05), (100.2, 50.1)]:
                    self.sim_fresh(r, t, fc)
                for key, value in change.items(): setattr(r.state, key, value)
                self.assertIn('rejected', r.command('start',100.2))
                self.assertFalse(r.link.mav.sent)
                r.close()

    def test_simulation_faster_clock_keeps_fc_sample_age_valid(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.make(d, self.sim_config())
            for t, fc in [(100., 50.), (100.1, 50.2), (100.2, 50.4)]:
                self.sim_fresh(r, t, fc, rate=2.)
            self.assertIn('accepted', r.command('start',100.2))
            self.assertAlmostEqual(r.state.fc_now(100.3), 50.6, places=6)
            r.close()

    def test_terminal_start_when_already_guided_sends_only_after_start(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d,Config())
            self.fresh(r)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            self.assertFalse(r.link.mav.sent)
            self.assertIn("accepted",r.command("start",10))
            r.step(10)
            self.assertEqual([n for n,_ in r.link.mav.sent],["set_position_target_local_ned_send"])
            r.close()

    def test_terminal_requests_guided_once_and_requires_start_after_confirmation(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d,Config())
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            self.assertIn("rejected",r.command("start",10))
            self.assertFalse(r.link.mav.sent)
            self.assertIn("requested",r.command("guided",10))
            self.assertEqual(r.link.mav.sent,[("set_mode_send",(1,1,4))])
            r.step(10)
            self.assertNotIn("requested",r.command("guided",10))
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            self.assertEqual(r.policy.stage,"READY")
            self.assertEqual(len(r.link.mav.sent),1)
            self.assertIn("accepted",r.command("start",10))
            r.step(10)
            self.assertEqual(len(r.link.mav.sent),2)
            r.close()

    def test_terminal_override_before_start_does_not_send(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d,Config())
            self.fresh(r)
            r.ingest(hb("GUIDED"),10)
            r.ingest(hb("LAND"),10.1)
            r.step(10.1)
            self.assertIn("rejected",r.command("start",10.1))
            self.assertFalse(r.link.mav.sent)
            r.close()

    def make(self, directory, config=None):
        r = Recorder(FakeLink(), Config(trigger_mode="GUIDED") if config is None else config,
                     "x", Path(directory)/"run", {"STABILIZE":0,"GUIDED":4,"LOITER":5,"LAND":9},clock=lambda:10.)
        for f in [*r.files.values(), r.events]: self.addCleanup(f.close)
        return r

    def fresh(self, r, t=10):
        r.ingest(Message(kind="EXTENDED_SYS_STATE", landed_state=2), t)
        r.ingest(Message(kind="EKF_STATUS_REPORT", flags=1|2|4|16|32), t)
        r.ingest(Message(kind="GLOBAL_POSITION_INT", time_boot_ms=int(t*1000),relative_alt=5000), t)
        r.ingest(Message(kind="ATTITUDE", time_boot_ms=int(t*1000), roll=0., pitch=0.,
                         yaw=0., rollspeed=0., pitchspeed=0., yawspeed=0.),t)
        r.ingest(Message(kind="LOCAL_POSITION_NED", time_boot_ms=int(t*1000),
                         x=0.,y=0.,z=-5.,vx=0.,vy=0.,vz=0.),t)
        for i in range(3):
            sent=t-.01+i*.002
            stamp=int(sent*1e9)
            r.pending_sync[stamp]=sent
            r.ingest(Message(kind="TIMESYNC",ts1=stamp,tc1=int((sent+t)/2*1e9)),t)

    def test_stabilize_to_guided_starts_once_when_airborne(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(hb("STABILIZE"),10)
            r.step(10)
            self.assertEqual(r.policy.stage,"WAIT")
            self.assertFalse(r.link.mav.sent)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            self.assertEqual(r.policy.stage,"RUN")
            self.assertEqual([name for name,_ in r.link.mav.sent],
                             ["set_position_target_local_ned_send"])
            r.close()

    def test_completion_requests_loiter_once_and_stops_setpoints(self):
        with tempfile.TemporaryDirectory() as d:
            c=Config(trigger_mode="GUIDED",hover_s=.5,chirp_s=1.,prbs_s=1.)
            r=self.make(d,c)
            self.fresh(r)
            r.ingest(hb("STABILIZE"),10)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            self.assertEqual(r.link.mav.sent[0][0],"set_position_target_local_ned_send")
            finish=10+r.excitation.total
            r.clock=lambda:finish
            self.fresh(r,finish)
            r.ingest(hb("GUIDED"),finish)
            r.step(finish)
            self.assertEqual(r.policy.stage,"DONE")
            self.assertEqual(r.policy.reason,"complete")
            self.assertTrue(r.exit_requested)
            self.assertEqual(r.link.mav.sent[-1],("set_mode_send",(1,1,5)))
            count=len(r.link.mav.sent)
            r.step(finish)
            self.assertEqual(len(r.link.mav.sent),count)
            r.ingest(hb("LOITER"),finish+.01)
            self.assertFalse(r.exit_requested)
            r.step(finish+.01)
            self.assertEqual(len(r.link.mav.sent),count)
            r.close()
            import json
            events=[json.loads(line) for line in (Path(d)/"run/events.jsonl").read_text().splitlines()]
            self.assertTrue(any(e["event"]=="exit_mode_observed" and e["confirmed"]
                                for e in events))
            self.assertEqual(json.loads((Path(d)/"run/metadata.json").read_text())["outcome"],
                             "complete")

    def test_override_then_guided_in_same_batch_has_no_cleanup_send(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            self.assertEqual(len(r.link.mav.sent), 1)
            r.ingest(hb("LOITER"),10.1)
            r.ingest(hb("GUIDED"),10.1)
            r.step(10.1)
            r.shutdown("exit",10.1)
            self.assertEqual(len(r.link.mav.sent), 1)
            self.assertEqual(r.link.mav.sent[0][1][4],3527)
            r.close()

    def test_handoff_one_request_then_confirm_before_send(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d, Config(trigger_mode="LAND"))
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("LAND"),10)
            r.step(10)
            r.step(10.1)
            self.assertEqual([x[0] for x in r.link.mav.sent], ["set_mode_send"])
            self.assertEqual(r.link.mav.sent[0][1],(1,1,4))
            r.ingest(hb("GUIDED"),10.2)
            r.step(10.2)
            self.assertEqual(r.link.mav.sent[-1][0],"set_position_target_local_ned_send")
            r.close()

    def test_default_land_handoff_completes_in_loiter(self):
        with tempfile.TemporaryDirectory() as d:
            c=Config(trigger_mode="LAND",hover_s=.5,chirp_s=1.,prbs_s=1.)
            r=self.make(d,c)
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("LAND"),10)
            r.step(10)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            self.assertEqual([name for name,_ in r.link.mav.sent],
                             ["set_mode_send","set_position_target_local_ned_send"])
            finish=10+r.excitation.total
            r.clock=lambda:finish
            self.fresh(r,finish)
            r.ingest(hb("GUIDED"),finish)
            r.step(finish)
            self.assertEqual(r.policy.reason,"complete")
            self.assertEqual(r.link.mav.sent[-1],("set_mode_send",(1,1,5)))
            r.ingest(hb("LOITER"),finish+.01)
            self.assertFalse(r.exit_requested)
            r.close()

    def test_land_handoff_timeout_sends_no_velocity_or_repeat_request(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d,Config(trigger_mode="LAND"))
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("LAND"),10)
            r.step(10)
            r.clock=lambda:14.
            r.step(14)
            self.assertEqual(r.policy.stage,"DONE")
            self.assertEqual(r.link.mav.sent,[("set_mode_send",(1,1,4))])
            r.close()

    def test_default_cli_terminal_and_legacy_trigger_opt_in(self):
        self.assertEqual(parser().get_default("trigger_mode"),"TERMINAL")
        self.assertNotIn("allow_land_handoff",[a.dest for a in parser()._actions])

    def test_land_handoff_never_reenters_after_pilot_override(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d,Config(trigger_mode="LAND"))
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("LAND"),10)
            r.step(10)
            self.assertEqual([n for n,_ in r.link.mav.sent],["set_mode_send"])
            r.ingest(hb("STABILIZE"),10.1)
            r.ingest(hb("GUIDED"),10.1)
            r.step(10.1)
            self.assertEqual(r.policy.stage,"DONE")
            self.assertEqual([n for n,_ in r.link.mav.sent],["set_mode_send"])
            r.close()

    def test_default_land_trigger_rejects_ground_entry_without_mode_command(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d,Config(trigger_mode="LAND"))
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.state.in_air=False
            r.ingest(hb("LAND"),10)
            r.step(10)
            self.assertEqual(r.policy.stage,"WAIT")
            self.assertFalse(r.link.mav.sent)
            r.close()

    def test_stale_heartbeat_stops_without_any_mode_or_zero_command(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            count=len(r.link.mav.sent)
            r.step(14)
            self.assertEqual(len(r.link.mav.sent),count)
            self.assertEqual(r.policy.stage,"DONE")
            r.close()

    def test_other_component_cannot_override_fc(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("GUIDED"),10)
            h=hb("LAND");h.comp=191
            r.ingest(h,10.1)
            self.assertEqual(r.state.mode,"GUIDED")
            r.close()

    def test_fc_critical_status_yields_without_requesting_mode(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            count=len(r.link.mav.sent)
            h=hb("GUIDED");h.system_status=5
            r.ingest(h,10.1)
            r.step(10.1)
            self.assertEqual(len(r.link.mav.sent),count)
            self.assertEqual(r.policy.stage,"DONE")
            r.close()

    def test_duplicate_local_position_is_not_logged_twice(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(Message(kind="LOCAL_POSITION_NED", time_boot_ms=10000,
                             x=0.,y=0.,z=-5.,vx=0.,vy=0.,vz=0.),10.1)
            r.close()
            self.assertEqual(len((Path(d)/"run/telemetry.csv").read_text().splitlines()),2)

    def test_send_rechecks_clock_after_logging_stall(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("GUIDED"),10)
            original_save=r.save_meta
            def stalled_save():
                original_save()
                r.clock=lambda:10.6
            r.save_meta=stalled_save
            r.step(10)
            self.assertFalse(any(n=="set_position_target_local_ned_send" for n,_ in r.link.mav.sent))
            self.assertEqual(r.policy.stage,"DONE")
            r.close()

    def test_command_records_actual_post_io_send_time(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("GUIDED"),10)
            original_save=r.save_meta
            def slow_save():
                original_save()
                r.clock=lambda:10.04
            r.save_meta=slow_save
            r.step(10)
            r.close()
            import csv
            with (Path(d)/"run/commands.csv").open() as f:
                rows=list(csv.DictReader(f))
            self.assertAlmostEqual(float(rows[0]["host_mono_s"]),10.04)

    def test_unsynchronized_clock_prevents_trigger(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.state.clock_offset_s=float("nan")
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            self.assertEqual(r.policy.stage,"WAIT")
            self.assertFalse(r.link.mav.sent)
            r.close()

    def test_monotone_backlog_cannot_renew_fc_age(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(hb("LOITER"),10)
            r.ingest(hb("GUIDED"),10)
            r.step(10)
            count=len(r.link.mav.sent)
            r.ingest(Message(kind="LOCAL_POSITION_NED",time_boot_ms=10020,
                             x=0.,y=0.,z=-5.,vx=0.,vy=0.,vz=0.),11)
            self.assertFalse(r.policy.can_send(r.state,11))
            self.assertEqual(len(r.link.mav.sent),count)
            r.close()

    def test_duplicate_altitude_does_not_renew_receipt_age(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.make(d)
            self.fresh(r)
            r.ingest(Message(kind="GLOBAL_POSITION_INT",time_boot_ms=10000,relative_alt=5000),11)
            self.assertEqual(r.state.alt_t,10)
            r.close()


if __name__ == "__main__": unittest.main()

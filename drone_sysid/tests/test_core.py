import unittest
from dataclasses import replace

from sysid_core import Config, State, Policy, Excitation, prbs_sequence


def state(now=10.0, mode="LOITER", **kw):
    d = dict(mode=mode, armed=True, in_air=True, ekf_ok=True,
             x=0., y=0., z=-5., relative_alt=5., roll=0., pitch=0.,
             hb_t=now, pos_t=now, alt_t=now, air_t=now, ekf_t=now, att_t=now,
             clock_offset_s=0.,sync_t=now,pos_boot_s=now,att_boot_s=now,alt_boot_s=now)
    d.update(kw)
    return State(**d)


class CoreTests(unittest.TestCase):
    def test_terminal_guided_start_requires_explicit_command_and_health(self):
        p = Policy(Config())
        p.heartbeat(state(mode="GUIDED"),10)
        self.assertEqual(p.stage,"WAIT")
        self.assertFalse(p.can_send(state(mode="GUIDED"),10))
        self.assertIn("rejected",p.start(state(mode="GUIDED",in_air=False),10))
        self.assertIsNone(p.start(state(mode="GUIDED"),10))
        self.assertEqual(p.stage,"RUN")
        p.heartbeat(state(mode="LAND"),10.1)
        self.assertEqual(p.stage,"DONE")

    def test_terminal_handoff_waits_for_separate_start_and_cancels_override(self):
        p = Policy(Config())
        self.assertIsNone(p.request_guided(state(),10))
        self.assertEqual(p.stage,"HANDOFF")
        self.assertIsNotNone(p.start(state(mode="GUIDED"),10))
        p.heartbeat(state(mode="GUIDED"),10.1)
        self.assertEqual(p.stage,"READY")
        self.assertFalse(p.can_send(state(mode="GUIDED"),10.1))
        p.heartbeat(state(mode="LAND"),10.2)
        self.assertEqual(p.stage,"DONE")

    def test_terminal_handoff_timeout_and_rejected_request(self):
        p = Policy(Config())
        self.assertIn("rejected",p.request_guided(state(armed=False),10))
        self.assertEqual(p.stage,"WAIT")
        self.assertIsNone(p.request_guided(state(),10))
        p.tick(state(now=14),14)
        self.assertEqual(p.stage,"DONE")

    def test_default_land_trigger_requires_airborne_entry_and_guided_confirmation(self):
        p = Policy(Config(trigger_mode="LAND"))
        p.heartbeat(state(mode="LAND"), 10)
        self.assertEqual(p.stage, "WAIT")
        p.heartbeat(state(mode="LOITER"), 10.1)
        p.heartbeat(state(mode="LAND", in_air=False), 10.2)
        self.assertEqual(p.stage, "WAIT")
        p.heartbeat(state(mode="LAND"), 10.3)
        self.assertEqual(p.stage, "WAIT")
        p.heartbeat(state(mode="LOITER"), 10.4)
        p.heartbeat(state(mode="LAND"), 10.5)
        self.assertEqual(p.stage, "HANDOFF")
        self.assertFalse(p.can_send(state(mode="LAND"), 10.5))
        p.heartbeat(state(now=10.6, mode="GUIDED"), 10.6)
        self.assertEqual(p.stage, "RUN")

    def test_startup_in_trigger_does_not_start(self):
        p = Policy(Config(trigger_mode="GUIDED"))
        p.heartbeat(state(mode="GUIDED"), 10)
        self.assertEqual(p.stage, "WAIT")

    def test_entry_and_override_are_terminal(self):
        p = Policy(Config(trigger_mode="GUIDED"))
        p.heartbeat(state(), 10)
        p.heartbeat(state(mode="GUIDED"), 10)
        self.assertEqual(p.stage, "RUN")
        p.heartbeat(state(mode="LOITER"), 10)
        p.heartbeat(state(mode="GUIDED"), 10)
        self.assertEqual(p.stage, "DONE")
        self.assertFalse(p.can_send(state(mode="GUIDED"), 10))

    def test_landed_entry_is_consumed_without_delayed_start(self):
        p = Policy(Config(trigger_mode="GUIDED"))
        p.heartbeat(state(), 10)
        p.heartbeat(state(mode="GUIDED", in_air=False), 10)
        p.heartbeat(state(mode="GUIDED"), 10)
        self.assertEqual(p.stage, "WAIT")

    def test_land_handoff_and_pilot_override(self):
        p = Policy(Config(trigger_mode="LAND"))
        p.heartbeat(state(), 10)
        p.heartbeat(state(mode="LAND"), 10)
        self.assertEqual(p.stage, "HANDOFF")
        self.assertFalse(p.can_send(state(mode="LAND"), 10))
        p.heartbeat(state(mode="ALT_HOLD"), 10.1)
        p.heartbeat(state(mode="GUIDED"), 10.2)
        self.assertEqual(p.stage, "DONE")

    def test_land_handoff_confirmed(self):
        p = Policy(Config(trigger_mode="LAND"))
        p.heartbeat(state(), 10)
        p.heartbeat(state(mode="LAND"), 10)
        p.heartbeat(state(now=10.2, mode="GUIDED"), 10.2)
        self.assertEqual(p.stage, "RUN")

    def test_stale_disarm_ekf_and_bounds_block_setpoints(self):
        for kw in [dict(pos_t=8), dict(hb_t=7), dict(armed=False),
                   dict(ekf_ok=False), dict(in_air=False), dict(x=16),
                   dict(relative_alt=1), dict(relative_alt=12), dict(x=float('nan'))]:
            with self.subTest(kw=kw):
                p = Policy(Config(trigger_mode="GUIDED"))
                p.heartbeat(state(), 10)
                p.heartbeat(state(mode="GUIDED"), 10)
                self.assertFalse(p.can_send(state(mode="GUIDED", **kw), 10))

    def test_handoff_timeout(self):
        p = Policy(Config(trigger_mode="LAND"))
        p.heartbeat(state(), 10)
        p.heartbeat(state(mode="LAND"), 10)
        p.tick(state(now=14, mode="LAND"), 14)
        self.assertEqual(p.stage, "DONE")

    def test_prbs_period_balance_and_seed(self):
        s = prbs_sequence(0x5a)
        self.assertEqual(len(s), 255)
        self.assertEqual(len(set(tuple(s[i:]+s[:i]) for i in range(255))),255)
        self.assertEqual(abs(sum(s)), 1)
        with self.assertRaises(ValueError):
            prbs_sequence(0)

    def test_excitation_phase_boundaries_and_repeatability(self):
        e = Excitation(Config())
        self.assertEqual(e.value(0), ("PRE_HOVER", 0.))
        self.assertEqual(e.value(5)[0], "CHIRP")
        self.assertEqual(e.value(55), ("MID_HOVER", 0.))
        self.assertEqual(e.value(60)[0], "PRBS")
        self.assertEqual(e.value(e.total), ("COMPLETE", 0.))
        self.assertEqual(e.value(60.5), e.value(60.5))


if __name__ == "__main__":
    unittest.main()

"""Pure experiment policy and deterministic excitation; no MAVLink dependency."""
import math
from dataclasses import dataclass

VELOCITY_MASK = 3527


@dataclass
class Config:
    trigger_mode: str = "TERMINAL"
    sim_time: bool = False
    amplitude: float = 0.3
    rate: float = 50.
    hover_s: float = 5.
    chirp_s: float = 50.
    prbs_s: float = 45.
    f0: float = 0.1
    f1: float = 2.
    switch_s: float = 0.2
    radius_m: float = 15.
    min_alt_m: float = 3.
    max_alt_m: float = 10.
    heartbeat_timeout: float = 2.5
    state_timeout: float = 0.5
    status_timeout: float = 2.5
    handoff_timeout: float = 3.
    max_tilt_rad: float = math.radians(25.)

    def __post_init__(self):
        numeric = [v for v in vars(self).values() if isinstance(v, (int, float))
                   and not isinstance(v, bool)]
        if not all(math.isfinite(v) and v > 0 for v in numeric):
            raise ValueError("All numeric settings must be finite and positive")
        if self.min_alt_m >= self.max_alt_m or self.f0 >= self.f1:
            raise ValueError("Require min altitude < max altitude and f0 < f1")
        if self.rate < 10*self.f1 or self.rate > 100:
            raise ValueError("Rate must be at least 10*f1 and at most 100 Hz")
        if self.amplitude > 1. or self.trigger_mode not in ("TERMINAL", "GUIDED", "LAND"):
            raise ValueError("Amplitude must be <=1 m/s; trigger must be TERMINAL, GUIDED or LAND")


@dataclass
class State:
    mode: str = "UNKNOWN"
    armed: bool = False
    in_air: bool = False
    ekf_ok: bool = False
    x: float = math.nan
    y: float = math.nan
    z: float = math.nan
    relative_alt: float = math.nan
    roll: float = math.nan
    pitch: float = math.nan
    hb_t: float = -math.inf
    pos_t: float = -math.inf
    alt_t: float = -math.inf
    air_t: float = -math.inf
    ekf_t: float = -math.inf
    att_t: float = -math.inf
    failsafe: bool = False
    clock_offset_s: float = math.nan
    clock_rate: float = 1.
    sync_t: float = -math.inf
    pos_boot_s: float = math.nan
    att_boot_s: float = math.nan
    alt_boot_s: float = math.nan

    def fc_now(self, now):
        return now*self.clock_rate+self.clock_offset_s


class Policy:
    """Latch an override even if GUIDED returns in the same receive batch."""
    def __init__(self, config):
        self.c = config
        self.stage = "WAIT"
        self.reason = ("Waiting for terminal start in GUIDED" if config.trigger_mode == "TERMINAL"
                       else "Waiting for a non-" + config.trigger_mode + " mode, then " + config.trigger_mode + " entry")
        self.previous_mode = None
        self.seen_nontrigger = False
        self.origin = None
        self.start_t = None
        self.start_fc_t = None
        self.handoff_t = None

    def health_reason(self, s, now):
        if now-s.hb_t > self.c.heartbeat_timeout:
            return "heartbeat stale"
        if not s.armed:
            return "disarmed"
        if s.failsafe:
            return "autopilot critical/emergency status"
        sync_timeout = .5 if self.c.sim_time else 5.
        if (not math.isfinite(s.clock_offset_s) or not math.isfinite(s.clock_rate)
                or s.clock_rate <= 0 or now-s.sync_t > sync_timeout):
            return "FC clock synchronization unavailable/stale"
        for stamp in (s.pos_boot_s,s.att_boot_s,s.alt_boot_s):
            age=s.fc_now(now)-stamp
            if not math.isfinite(age) or age > self.c.state_timeout or age < -.1:
                return "FC sample age/clock invalid"
        if now-s.pos_t > self.c.state_timeout or now-s.att_t > self.c.state_timeout:
            return "position/attitude stale"
        if now-s.alt_t > self.c.state_timeout:
            return "home-relative altitude stale"
        if now-s.air_t > self.c.status_timeout or not s.in_air:
            return "not confirmed airborne"
        if now-s.ekf_t > self.c.status_timeout or not s.ekf_ok:
            return "EKF unhealthy/stale"
        if not all(math.isfinite(v) for v in
                   [s.x, s.y, s.z, s.relative_alt, s.roll, s.pitch]):
            return "nonfinite state"
        if not self.c.min_alt_m <= s.relative_alt <= self.c.max_alt_m:
            return "altitude bound"
        if max(abs(s.roll), abs(s.pitch)) > self.c.max_tilt_rad:
            return "tilt bound"
        if self.origin is not None and math.hypot(
                s.x-self.origin[0], s.y-self.origin[1]) > self.c.radius_m:
            return "horizontal radius bound"
        return None

    def heartbeat(self, s, now):
        prior = self.previous_mode
        self.previous_mode = s.mode
        if self.stage == "DONE":
            return
        if self.stage == "RUN":
            if s.mode != "GUIDED":
                self.finish("mode override: " + s.mode)
            elif not s.armed:
                self.finish("disarmed")
            return
        if self.stage == "HANDOFF":
            if s.mode == "GUIDED":
                why = self.health_reason(s, now)
                if why:
                    self.finish("handoff unsafe: " + why)
                elif self.c.trigger_mode == "TERMINAL":
                    self.stage = "READY"
                    self.reason = "GUIDED confirmed; type start to begin"
                else:
                    self.start_t = now
                    self.start_fc_t = s.fc_now(now)
                    self.stage = "RUN"
                    self.reason = "GUIDED confirmed"
            elif self.c.trigger_mode == "TERMINAL":
                if s.mode != self.handoff_mode:
                    self.finish("mode override during handoff: " + s.mode)
            elif s.mode != self.c.trigger_mode:
                self.finish("mode override during handoff: " + s.mode)
            return
        if self.c.trigger_mode == "TERMINAL":
            if self.stage == "READY" and s.mode != "GUIDED":
                self.finish("mode override before start: " + s.mode)
            return
        if s.mode != self.c.trigger_mode:
            self.seen_nontrigger = True
        elif self.seen_nontrigger and prior != s.mode:
            why = self.health_reason(s, now)
            if why:
                self.reason = "Trigger rejected: " + why + "; leave and re-enter trigger"
                return
            self.origin = (s.x, s.y, s.z)
            if s.mode == "GUIDED":
                self.stage, self.start_t = "RUN", now
                self.start_fc_t = s.fc_now(now)
                self.reason = "trigger accepted"
            else:
                self.stage, self.handoff_t = "HANDOFF", now
                self.reason = "waiting for GUIDED handoff"

    def tick(self, s, now):
        if self.stage == "HANDOFF":
            why = self.health_reason(s, now)
            if why:
                self.finish("handoff unsafe: " + why)
            elif now-self.handoff_t > self.c.handoff_timeout:
                self.finish("GUIDED handoff timeout")

    def request_guided(self, s, now):
        if self.c.trigger_mode != "TERMINAL" or self.stage != "WAIT":
            return "GUIDED request unavailable"
        if s.mode == "GUIDED":
            return "Already in GUIDED; type start to begin"
        why = self.health_reason(s, now)
        if why:
            return "GUIDED request rejected: " + why
        self.stage, self.handoff_t = "HANDOFF", now
        self.handoff_mode = s.mode
        self.reason = "waiting for GUIDED handoff"
        return None

    def start(self, s, now):
        if self.c.trigger_mode != "TERMINAL" or self.stage not in ("WAIT", "READY"):
            return "Start unavailable"
        if s.mode != "GUIDED":
            return "Start rejected: FC is not in GUIDED (type guided to request it)"
        why = self.health_reason(s, now)
        if why:
            return "Start rejected: " + why
        self.origin = (s.x, s.y, s.z)
        self.start_t, self.stage = now, "RUN"
        self.start_fc_t = s.fc_now(now)
        self.reason = "terminal start accepted"
        return None

    def elapsed(self, s, now):
        return s.fc_now(now)-self.start_fc_t if self.c.sim_time else now-self.start_t

    def can_send(self, s, now):
        return (self.stage == "RUN" and s.mode == "GUIDED"
                and self.health_reason(s, now) is None)

    def finish(self, reason):
        self.stage, self.reason = "DONE", reason


def prbs_sequence(seed=0x5a):
    """255 chips from x^8+x^6+x^5+x^4+1; seed cannot be zero."""
    if not 1 <= seed <= 255:
        raise ValueError("PRBS seed must be 1..255")
    reg, result = seed, []
    for _ in range(255):
        result.append(1. if reg & 1 else -1.)
        bit = ((reg >> 7) ^ (reg >> 5) ^ (reg >> 4) ^ (reg >> 3)) & 1
        reg = ((reg << 1) & 255) | bit
    assert reg == seed
    return result


class Excitation:
    def __init__(self, config):
        self.c = config
        self.prbs = prbs_sequence()
        self.total = 3*config.hover_s + config.chirp_s + config.prbs_s

    def value(self, t):
        c = self.c
        if t < c.hover_s:
            return "PRE_HOVER", 0.
        t -= c.hover_s
        if t < c.chirp_s:
            a = math.log(c.f1/c.f0)/c.chirp_s
            phase = 2*math.pi*c.f0*math.expm1(a*t)/a
            return "CHIRP", c.amplitude*math.sin(phase)
        t -= c.chirp_s
        if t < c.hover_s:
            return "MID_HOVER", 0.
        t -= c.hover_s
        if t < c.prbs_s:
            return "PRBS", c.amplitude*self.prbs[int(t/c.switch_s) % 255]
        t -= c.prbs_s
        if t < c.hover_s:
            return "POST_HOVER", 0.
        return "COMPLETE", 0.

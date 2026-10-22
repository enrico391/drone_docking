"""Deterministic chirp/PRBS excitation, ported from drone_sysid/sysid_core.py.

Vendored here so px4_sysid has no dependency on the ArduPilot-only
drone_sysid package (MAVLink GUIDED transport, TIMESYNC).
Schedule/timing contract is identical so fit_plant.py works unchanged.
"""

import math
from dataclasses import dataclass


@dataclass
class SysIdConfig:
    """One-shot experiment settings (mirrors drone_sysid Config)."""

    amplitude: float = 0.3
    rate: float = 50.0
    hover_s: float = 5.0
    chirp_s: float = 50.0
    prbs_s: float = 45.0
    f0: float = 0.1
    f1: float = 2.0
    switch_s: float = 0.2
    radius_m: float = 15.0
    min_alt_m: float = 3.0
    max_alt_m: float = 10.0
    telemetry_timeout_s: float = 2.0
    command_timeout_s: float = 10.0
    max_tilt_rad: float = math.radians(25.0)

    def __post_init__(self):
        numeric = [v for v in vars(self).values() if isinstance(v, (int, float)) and not isinstance(v, bool)]
        if not all(math.isfinite(v) and v > 0 for v in numeric):
            raise ValueError("All numeric settings must be finite and positive")
        if self.min_alt_m >= self.max_alt_m or self.f0 >= self.f1:
            raise ValueError("Require min altitude < max altitude and f0 < f1")
        if self.rate < 10 * self.f1 or self.rate > 100:
            raise ValueError("Rate must be at least 10*f1 and at most 100 Hz")
        if self.amplitude > 1.0:
            raise ValueError("Amplitude must be <=1 m/s")


def prbs_sequence(seed=0x5A):
    """255 chips from x^8+x^6+x^5+x^4+1; seed cannot be zero."""
    if not 1 <= seed <= 255:
        raise ValueError("PRBS seed must be 1..255")
    reg, result = seed, []
    for _ in range(255):
        result.append(1.0 if reg & 1 else -1.0)
        bit = ((reg >> 7) ^ (reg >> 5) ^ (reg >> 4) ^ (reg >> 3)) & 1
        reg = ((reg << 1) & 255) | bit
    assert reg == seed
    return result


class Excitation:
    """PRE_HOVER / CHIRP / MID_HOVER / PRBS / POST_HOVER schedule."""

    def __init__(self, config):
        self.c = config
        self.prbs = prbs_sequence()
        self.total = 3 * config.hover_s + config.chirp_s + config.prbs_s

    def value(self, t):
        c = self.c
        if t < c.hover_s:
            return "PRE_HOVER", 0.0
        t -= c.hover_s
        if t < c.chirp_s:
            a = math.log(c.f1 / c.f0) / c.chirp_s
            phase = 2 * math.pi * c.f0 * math.expm1(a * t) / a
            return "CHIRP", c.amplitude * math.sin(phase)
        t -= c.chirp_s
        if t < c.hover_s:
            return "MID_HOVER", 0.0
        t -= c.hover_s
        if t < c.prbs_s:
            return "PRBS", c.amplitude * self.prbs[int(t / c.switch_s) % 255]
        t -= c.prbs_s
        if t < c.hover_s:
            return "POST_HOVER", 0.0
        return "COMPLETE", 0.0

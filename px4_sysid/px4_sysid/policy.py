"""Offboard experiment policy: stages, freshness gates, latched override.

Mirrors drone_sysid Policy health checks but uses PX4 DDS state:
NAVIGATION_STATE_OFFBOARD instead of GUIDED, VehicleLocalPosition
validity/reset counters, VehicleStatus failsafe, VehicleLandDetected.
"""

import math
from dataclasses import dataclass, field
from enum import Enum


class Phase(str, Enum):
    WAIT = "WAIT"            # passive until an explicit start/offboard request
    WARMUP = "WARMUP"        # streaming hold before requesting Offboard once
    READY = "READY"          # Offboard confirmed, waiting for `start`
    RUN = "RUN"              # excitation active
    HOLD = "HOLD"            # experiment complete; keep a fixed position hold
    DONE = "DONE"            # terminal; never restarts


@dataclass
class HealthSnapshot:
    offboard: bool = False
    armed: bool = False
    landed: bool = True
    # Robust in-air signal: height above the EKF origin. VehicleLandDetected
    # reports landed while hovering on this airframe (see
    # drone_description/config/px4_sitl_docking.parm), so it must not gate
    # control or the setpoint stream.
    airborne: bool = False
    failsafe: bool = False
    xy_valid: bool = False
    z_valid: bool = False
    v_xy_valid: bool = False
    v_z_valid: bool = False
    heading_good: bool = False
    x: float = math.nan
    y: float = math.nan
    z: float = math.nan
    vx: float = math.nan
    vy: float = math.nan
    vz: float = math.nan
    roll: float = math.nan
    pitch: float = math.nan
    local_at: float = -math.inf       # monotonic receipt time of last local position
    local_fc_s: float = math.nan      # msg.timestamp/1e6 (FC clock, the fit timebase)
    status_at: float = -math.inf
    land_at: float = -math.inf
    reset_counters: tuple = field(default_factory=lambda: (0, 0, 0))


@dataclass
class OffboardPolicy:
    """Latch pilot override; never re-enter Offboard automatically."""

    config: object
    stage: str = Phase.WAIT.value
    reason: str = "waiting for airborne vehicle; type start"
    origin: tuple | None = None
    start_t: float | None = None
    start_fc_s: float | None = None
    warmup_t: float | None = None
    warmup_s: float = 1.5
    offboard_acquired: bool = False
    start_pending: bool = False
    origin_alt: float | None = None  # NED z at start; bounds use height above it

    def health_reason(self, s: HealthSnapshot, now: float) -> str | None:
        c = self.config
        if now - s.status_at > c.telemetry_timeout_s:
            return "status stale"
        if now - s.local_at > c.telemetry_timeout_s:
            return "local position stale"
        if now - s.land_at > c.telemetry_timeout_s:
            return "land detection stale"
        if not s.armed:
            return "disarmed"
        if not s.airborne:
            return "not airborne (height above EKF origin below minimum)"
        if s.failsafe:
            return "PX4 failsafe"
        if not all((s.xy_valid, s.z_valid, s.v_xy_valid, s.v_z_valid)):
            return "EKF position/velocity invalid"
        if not all(math.isfinite(v) for v in (s.x, s.y, s.z, s.vx, s.vy, s.vz)):
            return "nonfinite state"
        if not math.isfinite(s.local_fc_s):
            return "FC timestamp invalid"
        if self.origin_alt is not None and math.isfinite(s.z):
            height_above_origin = self.origin_alt - s.z  # NED: up is -z
            if not (c.min_alt_m - 5.0 <= height_above_origin <= c.max_alt_m + 10.0):
                # Wide guard only; tight hover window is the operator's job.
                return "altitude far outside run window"
        if max(abs(s.roll), abs(s.pitch)) > c.max_tilt_rad:
            return "tilt bound"
        if self.origin is not None and math.hypot(s.x - self.origin[0], s.y - self.origin[1]) > c.radius_m:
            return "horizontal radius bound"
        return None

    def note_offboard(self, offboard: bool):
        self.offboard_acquired = self.offboard_acquired or offboard

    def request_offboard(self, s: HealthSnapshot, now: float, offboard: bool) -> str | None:
        """Explicit acquisition only; never override a completed/active run."""
        if self.stage != Phase.WAIT.value:
            return "Offboard unavailable: acquisition/run already active or finished"
        why = self.health_reason(s, now)
        if why:
            return "Offboard rejected: " + why
        if not s.heading_good:
            return "Offboard rejected: heading not good for control"
        self.origin = (s.x, s.y, s.z)
        self.origin_alt = s.z
        self.warmup_t = now
        self.stage = Phase.WARMUP.value
        self.reason = "streaming hold before one Offboard request"
        self.on_mode(s, offboard)
        return None

    def request_start(self, s: HealthSnapshot, now: float, offboard: bool) -> str | None:
        """Validate in-air health, acquire Offboard, then begin exactly once."""
        if self.stage not in (Phase.WAIT.value, Phase.WARMUP.value, Phase.READY.value) or self.start_pending:
            return "Start unavailable: run already active, requested or finished"
        why = self.health_reason(s, now)
        if why:
            return "Start rejected: " + why
        if not s.heading_good:
            return "Start rejected: heading not good for control"
        if self.stage == Phase.WAIT.value:
            self.request_offboard(s, now, offboard)
        self.start_pending = True
        self.reason = "start accepted; waiting for Offboard confirmation"
        if offboard:
            self.begin_run(s, now)
        return None

    def begin_run(self, s: HealthSnapshot, now: float):
        self.origin = (s.x, s.y, s.z)
        self.origin_alt = s.z
        self.start_t, self.start_fc_s = now, s.local_fc_s
        self.start_pending = False
        self.note_offboard(True)
        self.stage = Phase.RUN.value
        self.reason = "Offboard confirmed; identification running"

    def complete(self):
        self.stage, self.reason = Phase.HOLD.value, "complete; holding finishing position in Offboard"

    def on_mode(self, s: HealthSnapshot, offboard: bool, blocked: bool = False):
        """Latch mode loss in every phase after explicit Offboard acquisition.

        `blocked` means another node owns the Offboard topics, so entry
        confirmation is not meaningful and must not overwrite the warning.
        """
        if self.stage in (Phase.WAIT.value, Phase.DONE.value):
            return
        if self.offboard_acquired and not offboard:
            self.finish("mode override: left Offboard")
            return
        if not blocked:
            self.note_offboard(offboard)
        if self.stage in (Phase.WARMUP.value, Phase.READY.value) and offboard:
            if self.stage != Phase.READY.value and not blocked:
                self.stage = Phase.READY.value
                self.reason = "Offboard confirmed; type start to begin"

    def tick(self, s: HealthSnapshot, now: float, offboard: bool):
        self.on_mode(s, offboard)
        if self.stage not in (Phase.WAIT.value, Phase.DONE.value):
            why = self.health_reason(s, now)
            if why:
                self.finish(why if "override" in why else "safety abort: " + why)
            elif self.start_pending and not s.heading_good:
                self.finish("safety abort: heading not good for control")
            elif self.stage == Phase.WARMUP.value and self.warmup_t is not None:
                if now - self.warmup_t > self.warmup_s + self.config.command_timeout_s:
                    self.finish("Offboard acquisition timed out; no automatic retry")

    def elapsed(self, s: HealthSnapshot, now: float) -> float:
        # FC timebase: msg.timestamp is FC clock, shared by cmd + feedback.
        return s.local_fc_s - self.start_fc_s

    def can_send(self, s: HealthSnapshot, now: float, offboard: bool) -> bool:
        return self.stage == Phase.RUN.value and offboard and self.health_reason(s, now) is None

    def finish(self, reason: str):
        self.start_pending = False
        self.stage, self.reason = Phase.DONE.value, reason

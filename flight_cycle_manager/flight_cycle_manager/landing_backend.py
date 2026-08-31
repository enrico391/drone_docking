"""Transport-independent observations and landing backend decisions."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple


class VelocityFrame(Enum):
    """Coordinate frame used by a landing velocity command."""

    MAP_ENU = auto()
    BODY_FLU = auto()


class DockTargetMode(Enum):
    """AprilTag layout currently observed by the camera."""

    OUTER = auto()
    CENTRAL_ACQUIRING = auto()
    CENTRAL = auto()


@dataclass(frozen=True)
class DockObservation:
    forward_m: float
    left_m: float
    distance_m: float
    yaw_error_rad: float
    mode: DockTargetMode
    source_stamp_s: float
    received_at: float


@dataclass(frozen=True)
class VehicleObservation:
    position_enu_m: Tuple[float, float, float]
    velocity_enu_m_s: Tuple[float, float, float]
    armed: Optional[bool]
    flying: Optional[bool]
    mode: Optional[int]
    landed_at: float


@dataclass(frozen=True)
class VelocityCommand:
    frame: VelocityFrame
    velocity_x: float
    velocity_y: float
    velocity_z: float
    yaw_rate_rad_s: float = 0.0


class LandingBackendOutcome(Enum):
    RUNNING = auto()
    COMPLETE = auto()
    FAIL_BEFORE_COMMIT = auto()
    FAIL_AFTER_COMMIT = auto()


@dataclass(frozen=True)
class LandingBackendDecision:
    outcome: LandingBackendOutcome
    substate: str
    velocity: Optional[VelocityCommand] = None
    stream_landing_target: bool = False
    request_land: bool = False
    request_disarm: bool = False
    disarm_request_id: Optional[int] = None
    yaw_correction_rad: Optional[float] = None
    reason: str = ""

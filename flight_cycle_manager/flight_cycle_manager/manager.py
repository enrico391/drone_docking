"""Non-blocking validated flight-cycle start and AUTO mission handoff."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Mapping, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .mission_contract import (
    MissionContract,
    MissionContractError,
    MissionValidationConfig,
    validate_docking_mission,
)
from .pymavlink_vehicle import MavlinkVehicle, VehicleCommandResult, VehicleState


# ArduCopter custom modes used by the start lifecycle.
AUTO_MODE = 3
GUIDED_MODE = 4
LAND_MODE = 9
MISSION_STATE_ACTIVE = 3
MISSION_TYPE_MISSION = 0


class CycleState(Enum):
    """All lifecycle states, including the later docking/authority slices."""

    READY = auto()
    VALIDATING_MISSION = auto()
    INVALID_ONBOARD_MISSION = auto()
    PREARM = auto()
    SET_GUIDED = auto()
    ARM = auto()
    TAKEOFF = auto()
    WAIT_TAKEOFF_STABLE = auto()
    RESET_ONBOARD_MISSION = auto()
    START_AUTO = auto()
    AUTO_MISSION = auto()
    WAIT_FOR_HANDOFF = auto()
    GUIDED_TAKEOVER = auto()
    SEARCH_DOCK = auto()
    LANDING_BACKEND = auto()
    RETURN_TO_FC = auto()
    FC_FALLBACK_LAND = auto()
    WAIT_LANDED = auto()
    WAIT_DISARM = auto()
    DOCKED = auto()
    PILOT_OVERRIDE = auto()
    LINK_LOSS = auto()
    FAILED_DISARMED = auto()


@dataclass
class _PendingOperation:
    """One correlated MAVLink transaction owned by the timer state machine."""

    name: str
    future: Any
    deadline: float


class FlightCycleManager(Node):
    """Own one explicit takeoff-to-stored-mission transition at a time.

    MAVLink transactions are initiated in ``tick`` and completed by later
    ticks. The service callback only admits a new request and begins a read-
    only mission download; it neither waits nor sends flight commands.
    """

    def __init__(
        self,
        *,
        vehicle: Optional[MavlinkVehicle] = None,
        parameter_values: Optional[Mapping[str, object]] = None,
    ) -> None:
        overrides = [
            Parameter(name=name, value=value)
            for name, value in (parameter_values or {}).items()
        ]
        super().__init__("flight_cycle_manager", parameter_overrides=overrides)
        self._declare_parameters()

        self._landing_backend = str(
            self.get_parameter("landing_backend").value
        ).strip().lower()
        if self._landing_backend not in {"pid", "precise_landing"}:
            self.destroy_node()
            raise ValueError("landing_backend must be 'pid' or 'precise_landing'")
        self._simulation_mode = bool(
            self.get_parameter("simulation_mode").value
        )
        self._auto_start = bool(self.get_parameter("auto_start").value)
        if self._auto_start and not self._simulation_mode:
            self.destroy_node()
            raise ValueError("auto_start=true requires simulation_mode=true")

        self._vehicle = vehicle or MavlinkVehicle(
            connection=str(self.get_parameter("mavlink_connection").value),
            baud=int(self.get_parameter("mavlink_baud").value),
            target_system=int(
                self.get_parameter("mavlink_target_system").value
            ),
            target_component=int(
                self.get_parameter("mavlink_target_component").value
            ),
            source_system=int(
                self.get_parameter("mavlink_source_system").value
            ),
            source_component=int(
                self.get_parameter("mavlink_source_component").value
            ),
        )
        update_rate_hz = self._positive_parameter("update_rate_hz")
        self._state_publisher = self.create_publisher(
            String,
            "~/state",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._reason_publisher = self.create_publisher(
            String,
            "~/reason",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._start_service = self.create_service(
            Trigger, "~/start_cycle", self._start_cycle_callback
        )
        self._timer = self.create_timer(1.0 / update_rate_hz, self._timer_callback)
        self._state = CycleState.READY
        self._reason = "initializing"
        self._pending: Optional[_PendingOperation] = None
        self._contract: Optional[MissionContract] = None
        self._start_requested = False
        self._stable_takeoff_since: Optional[float] = None
        self._mission_confirmation_deadline: Optional[float] = None
        self._auto_heartbeat_baseline: Optional[float] = None
        self._mission_current_baseline: Optional[float] = None
        self._closed = False

        # Startup validation is deliberately read-only. A later cycle request
        # always starts another download, so it never trusts this result.
        self._begin_validation(time.monotonic(), start_requested=False)

    @property
    def vehicle(self) -> MavlinkVehicle:
        """Expose the boundary for inspection and deterministic unit tests."""
        return self._vehicle

    @property
    def state(self) -> CycleState:
        """Current authoritative lifecycle state."""
        return self._state

    @property
    def reason(self) -> str:
        """Reason associated with the latest material lifecycle transition."""
        return self._reason

    @property
    def landing_backend(self) -> str:
        """Configured landing backend selected for this node instance."""
        return self._landing_backend

    def _declare_parameters(self) -> None:
        """Declare only values required by the first lifecycle slice."""
        defaults = {
            "simulation_mode": False,
            "auto_start": False,
            "landing_backend": "pid",
            "update_rate_hz": 20.0,
            "takeoff_altitude_m": 4.0,
            "takeoff_altitude_tolerance_m": 0.25,
            "takeoff_vertical_speed_tolerance_m_s": 0.15,
            "takeoff_stable_time_s": 1.0,
            "mission_handoff_min_loiter_s": 5.0,
            "mission_landing_location_tolerance_m": 2.0,
            "mission_download_timeout_s": 10.0,
            "mavlink_operation_timeout_s": 10.0,
            "mission_start_confirmation_timeout_s": 5.0,
            "mavlink_heartbeat_timeout_s": 2.5,
            "pose_timeout_s": 1.0,
            "landed_state_timeout_s": 1.0,
            "mavlink_connection": "tcp:127.0.0.1:5760",
            "mavlink_baud": 921600,
            "mavlink_target_system": 1,
            "mavlink_target_component": 1,
            "mavlink_source_system": 254,
            "mavlink_source_component": 191,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _start_cycle_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Queue only a fresh mission validation for one explicit cycle."""
        del request
        if self._vehicle.state.armed is True:
            return self._reject_start(response, "cycle request rejected: vehicle is armed")
        if self._state not in {
            CycleState.READY,
            CycleState.INVALID_ONBOARD_MISSION,
        }:
            return self._reject_start(
                response,
                f"cycle request rejected: lifecycle is {self._state.name}",
            )
        self._begin_validation(time.monotonic(), start_requested=True)
        response.success = True
        response.message = "cycle request accepted for validation"
        return response

    def tick(self, now: float) -> None:
        """Advance at most one acknowledged transaction or lifecycle action."""
        self._vehicle.poll(now)
        if self._closed:
            return
        if self._advance_pending(now):
            return
        if self._state is CycleState.PREARM:
            if not self._fresh_ground_telemetry(now):
                self._route_failure(
                    "fresh heartbeat, position, and landed state telemetry "
                    "are required before arming",
                    grounded_state=CycleState.FAILED_DISARMED,
                    now=now,
                )
                return
            self._begin_operation("prearm", self._vehicle.prearm_async(now), now)
            return
        if self._state is CycleState.SET_GUIDED:
            self._begin_operation(
                "set_guided",
                self._vehicle.set_mode_async(GUIDED_MODE, now),
                now,
            )
            return
        if self._state is CycleState.ARM:
            if not self._fresh_ground_telemetry(now):
                self._route_failure(
                    "fresh heartbeat, position, and landed state telemetry "
                    "are required before arming",
                    grounded_state=CycleState.FAILED_DISARMED,
                    now=now,
                )
                return
            self._begin_operation("arm", self._vehicle.arm_async(True, now), now)
            return
        if self._state is CycleState.TAKEOFF:
            self._begin_operation(
                "takeoff",
                self._vehicle.takeoff_async(
                    self._positive_parameter("takeoff_altitude_m"), now
                ),
                now,
            )
            return
        if self._state is CycleState.WAIT_TAKEOFF_STABLE:
            self._update_takeoff_stability(now)
            return
        if self._state is CycleState.RESET_ONBOARD_MISSION:
            self._begin_operation(
                "reset_mission",
                self._vehicle.set_mission_current_async(0, reset=True, now=now),
                now,
            )
            return
        if self._state is CycleState.START_AUTO:
            if self._mission_confirmation_deadline is None:
                self._begin_operation(
                    "start_mission", self._vehicle.start_mission_async(now), now
                )
                return
            if self._auto_mission_confirmed(now):
                self._transition(
                    CycleState.AUTO_MISSION,
                    "AUTO heartbeat and active onboard mission confirmed",
                )
                return
            if now > self._mission_confirmation_deadline:
                self._command_direct_land("mission confirmation timed out")

    def close(self) -> None:
        """Cancel timer, release MAVLink once, then destroy the ROS node."""
        if self._closed:
            return
        self._closed = True
        self._timer.cancel()
        self._vehicle.close()
        self.destroy_node()

    def _timer_callback(self) -> None:
        self.tick(time.monotonic())

    def _begin_validation(self, now: float, *, start_requested: bool) -> None:
        self._start_requested = start_requested
        self._contract = None
        self._stable_takeoff_since = None
        self._mission_confirmation_deadline = None
        self._auto_heartbeat_baseline = None
        self._mission_current_baseline = None
        self._transition(
            CycleState.VALIDATING_MISSION,
            "downloading onboard mission for validation",
        )
        try:
            self._begin_operation(
                "mission_download",
                self._vehicle.download_mission_async(now),
                now,
                timeout_name="mission_download_timeout_s",
            )
        except Exception as error:
            self._invalid_mission(
                f"onboard mission download could not start: {error}", now
            )

    def _begin_operation(
        self,
        name: str,
        future: Any,
        now: float,
        *,
        timeout_name: str = "mavlink_operation_timeout_s",
    ) -> None:
        if self._pending is not None:
            raise RuntimeError("manager attempted to overlap MAVLink operations")
        timeout = self._positive_parameter(timeout_name)
        self._pending = _PendingOperation(name, future, now + timeout)

    def _advance_pending(self, now: float) -> bool:
        pending = self._pending
        if pending is None:
            return False
        if not pending.future.done():
            if now > pending.deadline:
                self._pending = None
                self._operation_failed(
                    pending.name, "MAVLink operation timed out", now
                )
            return True
        self._pending = None
        try:
            result = pending.future.result()
        except Exception as error:
            self._operation_failed(
                pending.name, f"MAVLink operation failed: {error}", now
            )
            return True
        if not getattr(result, "success", False):
            self._operation_failed(
                pending.name,
                getattr(result, "message", "MAVLink operation was rejected"),
                now,
            )
            return True
        self._operation_succeeded(pending.name, result, now)
        return True

    def _operation_succeeded(self, name: str, result: Any, now: float) -> None:
        if name == "mission_download":
            self._validate_download(result, now)
        elif name == "native_landing_parameters":
            self._validate_native_landing_parameters(result, now)
        elif name == "prearm":
            self._transition(CycleState.SET_GUIDED, "pre-arm checks passed")
        elif name == "set_guided":
            command = self._command_result(result)
            if command.success or command.current_mode == GUIDED_MODE:
                self._transition(CycleState.ARM, "GUIDED mode acknowledged")
            else:
                self._operation_failed("set_guided", command.message, now)
        elif name == "arm":
            self._transition(CycleState.TAKEOFF, "arm command acknowledged")
        elif name == "takeoff":
            self._stable_takeoff_since = None
            self._transition(
                CycleState.WAIT_TAKEOFF_STABLE,
                "vertical takeoff acknowledged; waiting for stable altitude",
            )
        elif name == "reset_mission":
            self._transition(
                CycleState.START_AUTO,
                "onboard mission reset to sequence zero",
            )
        elif name == "start_mission":
            state = self._vehicle.state
            self._auto_heartbeat_baseline = state.heartbeat_at
            self._mission_current_baseline = state.mission_current_at
            self._mission_confirmation_deadline = now + self._positive_parameter(
                "mission_start_confirmation_timeout_s"
            )
            self._transition(
                CycleState.START_AUTO,
                "onboard mission start acknowledged; awaiting AUTO confirmation",
                force=True,
            )
        elif name == "direct_land":
            self._transition(
                CycleState.FC_FALLBACK_LAND,
                "direct LAND command acknowledged; waiting for flight controller landing",
                force=True,
            )

    def _operation_failed(self, name: str, message: str, now: float) -> None:
        reason = f"{name.replace('_', ' ')} failed: {message or 'rejected'}"
        if name in {"mission_download", "native_landing_parameters"}:
            self._invalid_mission(reason, now)
            return
        if name == "direct_land":
            self._transition(CycleState.FC_FALLBACK_LAND, reason, force=True)
            return
        self._route_failure(
            reason,
            grounded_state=CycleState.FAILED_DISARMED,
            now=now,
        )

    def _validate_download(self, result: Any, now: float) -> None:
        try:
            contract = validate_docking_mission(
                result.items,
                MissionValidationConfig(
                    min_handoff_loiter_s=self._positive_parameter(
                        "mission_handoff_min_loiter_s", allow_zero=True
                    ),
                    landing_location_tolerance_m=self._positive_parameter(
                        "mission_landing_location_tolerance_m", allow_zero=True
                    ),
                ),
            )
        except (MissionContractError, TypeError, ValueError) as error:
            self._invalid_mission(
                f"onboard mission contract is invalid: {error}", now
            )
            return
        self._contract = contract
        if self._landing_backend == "precise_landing":
            self._transition(
                CycleState.VALIDATING_MISSION,
                "validating native precision landing parameters",
                force=True,
            )
            try:
                self._begin_operation(
                    "native_landing_parameters",
                    self._vehicle.get_parameters_async(
                        ("PLND_ENABLED", "PLND_TYPE"), now
                    ),
                    now,
                )
            except Exception as error:
                self._invalid_mission(
                    f"native precision landing parameter read could not start: {error}",
                    now,
                )
            return
        self._validation_succeeded(now)

    def _validate_native_landing_parameters(self, result: Any, now: float) -> None:
        parameters = self._command_result(result).parameters
        enabled = parameters.get("PLND_ENABLED")
        landing_type = parameters.get("PLND_TYPE")
        if enabled != 1.0 or landing_type != 1.0:
            self._invalid_mission(
                "ArduPilot native precision landing is not configured: "
                f"PLND_ENABLED={enabled}, PLND_TYPE={landing_type}",
                now,
            )
            return
        self._validation_succeeded(now)

    def _validation_succeeded(self, now: float) -> None:
        if self._start_requested:
            self._transition(
                CycleState.PREARM,
                "onboard mission validation passed; waiting for fresh pre-arm telemetry",
            )
            return
        if self._auto_start:
            self._auto_start = False
            self._begin_validation(now, start_requested=True)
            return
        self._transition(CycleState.READY, "onboard mission contract is valid")

    def _update_takeoff_stability(self, now: float) -> None:
        state = self._vehicle.state
        if not self._fresh_airborne_telemetry(now):
            self._command_direct_land(
                "takeoff telemetry became stale before stable altitude confirmation"
            )
            return
        assert state.position_enu_m is not None
        assert state.velocity_enu_m_s is not None
        altitude_error = abs(
            state.position_enu_m[2]
            - self._positive_parameter("takeoff_altitude_m")
        )
        vertical_speed = abs(state.velocity_enu_m_s[2])
        is_stable = (
            altitude_error <= self._positive_parameter(
                "takeoff_altitude_tolerance_m", allow_zero=True
            )
            and vertical_speed <= self._positive_parameter(
                "takeoff_vertical_speed_tolerance_m_s", allow_zero=True
            )
        )
        if not is_stable:
            self._stable_takeoff_since = None
            return
        if self._stable_takeoff_since is None:
            self._stable_takeoff_since = now
            return
        if now - self._stable_takeoff_since < self._positive_parameter(
            "takeoff_stable_time_s", allow_zero=True
        ):
            return
        self._transition(
            CycleState.RESET_ONBOARD_MISSION,
            "takeoff altitude and vertical speed are stable",
        )
        self._begin_operation(
            "reset_mission",
            self._vehicle.set_mission_current_async(0, reset=True, now=now),
            now,
        )

    def _auto_mission_confirmed(self, now: float) -> bool:
        state = self._vehicle.state
        heartbeat_baseline = self._auto_heartbeat_baseline
        mission_current_baseline = self._mission_current_baseline
        if heartbeat_baseline is None or mission_current_baseline is None:
            return False
        mission_mode_matches = (
            state.mission_mode is None
            or state.mission_mode == MISSION_TYPE_MISSION
        )
        mission_state_matches = (
            state.mission_state is None
            or state.mission_state == MISSION_STATE_ACTIVE
        )
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.mode == AUTO_MODE
            and state.heartbeat_at > heartbeat_baseline
            and state.mission_seq == 0
            and state.mission_current_at > mission_current_baseline
            and mission_state_matches
            and mission_mode_matches
            and 0.0 <= now - state.mission_current_at
            <= self._positive_parameter("mavlink_heartbeat_timeout_s")
        )

    def _fresh_ground_telemetry(self, now: float) -> bool:
        state = self._vehicle.state
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.armed is False
            and state.flying is False
            and self._fresh_position(now, state)
            and self._fresh_landed_state(now, state)
        )

    def _fresh_airborne_telemetry(self, now: float) -> bool:
        state = self._vehicle.state
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.armed is True
            and state.flying is True
            and state.position_enu_m is not None
            and state.velocity_enu_m_s is not None
            and self._fresh_position(now, state)
        )

    def _fresh_position(self, now: float, state: VehicleState) -> bool:
        return (
            state.position_enu_m is not None
            and 0.0 <= now - state.position_at
            <= self._positive_parameter("pose_timeout_s")
        )

    def _fresh_landed_state(self, now: float, state: VehicleState) -> bool:
        return (
            state.flying is not None
            and 0.0 <= now - state.landed_at
            <= self._positive_parameter("landed_state_timeout_s")
        )

    def _vehicle_is_confirmed_disarmed_on_ground(self, now: float) -> bool:
        state = self._vehicle.state
        return (
            self._vehicle.ready(
                now,
                heartbeat_timeout_s=self._positive_parameter(
                    "mavlink_heartbeat_timeout_s"
                ),
            )
            and state.armed is False
            and state.flying is False
            and self._fresh_landed_state(now, state)
        )

    def _command_direct_land(self, reason: str) -> None:
        self._transition(CycleState.FC_FALLBACK_LAND, reason)
        now = time.monotonic()
        try:
            self._begin_operation("direct_land", self._vehicle.land_async(now), now)
        except Exception as error:
            self._transition(
                CycleState.FC_FALLBACK_LAND,
                f"{reason}; direct LAND could not start: {error}",
                force=True,
            )

    def _invalid_mission(self, reason: str, now: float) -> None:
        self._contract = None
        self._route_failure(
            reason,
            grounded_state=CycleState.INVALID_ONBOARD_MISSION,
            now=now,
        )

    def _route_failure(
        self,
        reason: str,
        *,
        grounded_state: CycleState,
        now: float,
    ) -> None:
        """Land unless live telemetry confirms a disarmed vehicle on ground."""
        self._start_requested = False
        if self._vehicle_is_confirmed_disarmed_on_ground(now):
            if grounded_state is CycleState.FAILED_DISARMED:
                self._fail_disarmed(reason)
            else:
                self._transition(grounded_state, reason)
            return
        self._command_direct_land(reason)

    def _fail_disarmed(self, reason: str) -> None:
        self._start_requested = False
        self._transition(CycleState.FAILED_DISARMED, reason)

    def _reject_start(self, response: Trigger.Response, reason: str) -> Trigger.Response:
        self._transition(self._state, reason, force=True)
        response.success = False
        response.message = reason
        return response

    def _transition(self, state: CycleState, reason: str, *, force: bool = False) -> None:
        if not force and state is self._state and reason == self._reason:
            return
        self._state = state
        self._reason = reason
        state_message = String()
        state_message.data = state.name
        reason_message = String()
        reason_message.data = reason
        self._state_publisher.publish(state_message)
        self._reason_publisher.publish(reason_message)
        self.get_logger().info(f"Flight cycle state: {state.name}: {reason}")

    def _positive_parameter(self, name: str, *, allow_zero: bool = False) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0 or (not allow_zero and value == 0.0):
            comparator = "non-negative" if allow_zero else "positive"
            raise ValueError(f"{name} must be finite and {comparator}")
        return value

    @staticmethod
    def _command_result(result: Any) -> VehicleCommandResult:
        if not isinstance(result, VehicleCommandResult):
            raise TypeError("vehicle returned an invalid command result")
        return result


def main(args: Optional[list[str]] = None) -> None:
    """Construct, spin, and always close the MAVLink-owning ROS node."""
    rclpy.init(args=args)
    node: Optional[FlightCycleManager] = None
    try:
        node = FlightCycleManager()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.close()
        rclpy.shutdown()

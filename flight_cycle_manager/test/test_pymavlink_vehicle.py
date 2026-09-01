"""Behavioral tests for the lightweight direct pymavlink vehicle adapter."""

import pytest
from pymavlink import mavutil

from flight_cycle_manager.landing_backend import (
    DockObservation,
    DockTargetMode,
    VehicleObservation,
)
from flight_cycle_manager.mission_contract import MissionItem
from flight_cycle_manager.pid import PIDConfig
from flight_cycle_manager.pid_landing import PidLandingConfig
from flight_cycle_manager.pid_landing_backend import PidLandingBackend
from flight_cycle_manager.pymavlink_vehicle import (
    MavlinkVehicle,
    VehicleState,
    VelocityFrame,
)


MAVLINK = mavutil.mavlink


class FakeMessage:
    """MAVLink-shaped message with source metadata."""

    def __init__(self, message_type, src_system=1, src_component=1, **fields):
        self._message_type = message_type
        self._src_system = src_system
        self._src_component = src_component
        self.__dict__.update(fields)

    def get_type(self):
        return self._message_type

    def get_srcSystem(self):
        return self._src_system

    def get_srcComponent(self):
        return self._src_component


class FakeMav:
    """Record generated pymavlink send calls without opening a transport."""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        if not name.endswith("_send"):
            raise AttributeError(name)

        def record(*args):
            self.calls.append((name, args))

        return record

    def calls_for(self, name):
        return [args for method, args in self.calls if method == name]


class FakeConnection:
    """In-memory connection boundary used by the real adapter."""

    def __init__(self):
        self.mav = FakeMav()
        self.incoming = []
        self.closed = False

    def recv_match(self, blocking=False):
        assert blocking is False
        return self.incoming.pop(0) if self.incoming else None

    def close(self):
        self.closed = True


@pytest.fixture
def connection():
    return FakeConnection()


@pytest.fixture
def vehicle(connection):
    return MavlinkVehicle(
        connection="tcp:127.0.0.1:5760",
        target_system=1,
        target_component=1,
        source_system=254,
        source_component=191,
        connection_factory=lambda *args, **kwargs: connection,
    )


def push_heartbeat(connection, *, mode=4, armed=True):
    connection.incoming.append(
        FakeMessage(
            "HEARTBEAT",
            base_mode=(
                MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
            ),
            custom_mode=mode,
            system_status=MAVLINK.MAV_STATE_ACTIVE,
        )
    )


def push_ack(connection, command, result=MAVLINK.MAV_RESULT_ACCEPTED):
    connection.incoming.append(
        FakeMessage(
            "COMMAND_ACK",
            command=command,
            result=result,
            target_system=254,
            target_component=191,
        )
    )


def controller_pid_config(output_limit=1.0):
    return PIDConfig(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        integral_limit=0.5,
        output_limit=output_limit,
        derivative_filter_time_constant_s=0.1,
    )


def pid_config(**overrides):
    values = dict(
        tag_stable_time_s=0.5,
        central_transition_distance_m=1.0,
        central_acquire_timeout_s=3.0,
        central_loss_grace_s=0.3,
        recovery_climb_m=0.3,
        recovery_altitude_tolerance_m=0.05,
        max_recovery_attempts=3,
        touchdown_commit_height_m=0.20,
        touchdown_speed_m_s=0.05,
        touchdown_timeout_s=15.0,
        landed_state_timeout_s=1.0,
        landed_stable_time_s=0.5,
        disarm_retry_period_s=1.0,
        alignment_stable_time_s=1.0,
        coarse_horizontal_tolerance_m=0.2,
        fine_horizontal_tolerance_m=0.07,
        coarse_yaw_tolerance_rad=0.2,
        fine_yaw_tolerance_rad=0.1,
        max_horizontal_speed_m_s=0.5,
        outer_descent_speed_m_s=0.3,
        central_descent_speed_m_s=0.12,
        recovery_speed_m_s=0.2,
        max_yaw_rate_rad_s=0.35,
        x_pid=controller_pid_config(output_limit=0.5),
        y_pid=controller_pid_config(output_limit=0.5),
        z_pid=controller_pid_config(output_limit=0.7),
        yaw_pid=controller_pid_config(output_limit=0.35),
    )
    values.update(overrides)
    return PidLandingConfig(**values)


def dock_observation(received_at=1.0):
    return DockObservation(
        forward_m=0.1,
        left_m=-0.1,
        distance_m=2.0,
        yaw_error_rad=0.05,
        mode=DockTargetMode.OUTER,
        source_stamp_s=received_at,
        received_at=received_at,
    )


def vehicle_observation(*, altitude_m=4.0, armed=True, flying=True, landed_at=0.0):
    return VehicleObservation(
        position_enu_m=(4.0, -2.0, altitude_m),
        velocity_enu_m_s=(0.0, 0.0, 0.0),
        armed=armed,
        flying=flying,
        mode=4,
        landed_at=landed_at,
    )


def mission_item_message(
    *,
    seq,
    command,
    mission_type=MAVLINK.MAV_MISSION_TYPE_MISSION,
    src_system=1,
    src_component=1,
):
    """Build a complete MISSION_ITEM_INT-shaped response from the FC."""
    return FakeMessage(
        "MISSION_ITEM_INT",
        src_system=src_system,
        src_component=src_component,
        target_system=254,
        target_component=191,
        seq=seq,
        frame=MAVLINK.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=command,
        current=0,
        autocontinue=1,
        param1=1.0,
        param2=2.0,
        param3=3.0,
        param4=4.0,
        x=450000000,
        y=90000000,
        z=4.0,
        mission_type=mission_type,
    )


def test_telemetry_maps_vehicle_state_and_ned_pose_to_enu(vehicle, connection):
    """Break caught: direct telemetry retains NED axes or loses flight state."""
    push_heartbeat(connection, mode=3, armed=True)
    connection.incoming.extend(
        [
            FakeMessage(
                "LOCAL_POSITION_NED",
                x=12.0,
                y=-4.0,
                z=-3.5,
                vx=1.0,
                vy=2.0,
                vz=-0.5,
            ),
            FakeMessage(
                "EXTENDED_SYS_STATE",
                landed_state=MAVLINK.MAV_LANDED_STATE_IN_AIR,
            ),
        ]
    )

    vehicle.poll(now=10.0)

    assert vehicle.state.connected is True
    assert vehicle.state.armed is True
    assert vehicle.state.mode == 3
    assert vehicle.state.flying is True
    assert vehicle.state.position_enu_m == pytest.approx((-4.0, 12.0, 3.5))
    assert vehicle.state.velocity_enu_m_s == pytest.approx((2.0, 1.0, 0.5))
    assert vehicle.state.position_at == 10.0


def test_vehicle_state_has_neutral_mission_progress_defaults():
    """Break caught: future mission state is initialized as active progress."""
    state = VehicleState()

    assert state.mission_seq is None
    assert state.mission_total is None
    assert state.mission_state is None
    assert state.mission_mode is None
    assert state.mission_id is None
    assert state.mission_current_at == 0.0


def test_telemetry_ignores_other_vehicle_sources(vehicle, connection):
    """Break caught: a second MAVLink system can overwrite mission state."""
    connection.incoming.append(
        FakeMessage(
            "HEARTBEAT",
            src_system=2,
            base_mode=MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED,
            custom_mode=9,
            system_status=MAVLINK.MAV_STATE_ACTIVE,
        )
    )

    vehicle.poll(now=10.0)

    assert vehicle.state.connected is False
    assert vehicle.state.mode is None


def test_mode_future_requires_ack_and_observed_heartbeat(vehicle, connection):
    """Break caught: command delivery is mistaken for completed mode change."""
    push_heartbeat(connection, mode=0, armed=False)
    vehicle.poll(now=10.0)
    future = vehicle.set_mode_async(4, now=10.1)

    vehicle.poll(now=10.1)
    push_ack(connection, MAVLINK.MAV_CMD_DO_SET_MODE)
    vehicle.poll(now=10.2)
    assert future.done() is False

    push_heartbeat(connection, mode=4, armed=False)
    vehicle.poll(now=10.3)

    assert future.done() is True
    assert future.result().success is True
    command = connection.mav.calls_for("command_long_send")[-1]
    assert command[:5] == (
        1,
        1,
        MAVLINK.MAV_CMD_DO_SET_MODE,
        0,
        MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    )
    assert command[5] == 4.0


def test_pending_command_retries_at_retry_interval(vehicle, connection):
    """Break caught: an unacknowledged command is never resent at 0.5 s."""
    future = vehicle.takeoff_async(altitude_m=3.0, now=11.0)

    vehicle.poll(now=11.49)
    assert len(connection.mav.calls_for("command_long_send")) == 1

    vehicle.poll(now=11.5)

    assert future.done() is False
    assert len(connection.mav.calls_for("command_long_send")) == 2


def test_receive_telemetry_without_service_does_not_retry_pending_command(
    vehicle, connection
):
    """Break caught: ingesting telemetry resends an overdue command before authority can gate it."""
    future = vehicle.takeoff_async(altitude_m=3.0, now=11.0)

    vehicle.receive_telemetry(now=11.5)

    assert future.done() is False
    assert len(connection.mav.calls_for("command_long_send")) == 1

    vehicle.service_pending(now=11.5)

    assert future.done() is False
    assert len(connection.mav.calls_for("command_long_send")) == 2


def test_pending_command_times_out_with_terminal_failure(vehicle):
    """Break caught: a command remains pending past the three-second limit."""
    future = vehicle.takeoff_async(altitude_m=3.0, now=12.0)

    vehicle.poll(now=15.01)

    assert future.done() is True
    assert future.result().success is False
    assert future.result().message == "MAVLink operation timed out"


def test_condition_yaw_encodes_relative_signed_correction(vehicle, connection):
    """Break caught: dock yaw correction loses sign or is sent as absolute."""
    sent = vehicle.send_condition_yaw(yaw_error_rad=-1.5707963267948966)

    assert sent is True
    call = connection.mav.calls_for("command_long_send")[-1]
    assert call == pytest.approx(
        (
            1,
            1,
            MAVLINK.MAV_CMD_CONDITION_YAW,
            0,
            90.0,
            0.0,
            -1.0,
            1.0,
            0.0,
            0.0,
            0.0,
        )
    )


def test_companion_heartbeat_emits_once_per_second(vehicle, connection):
    """Break caught: companion heartbeats are absent or emitted too often."""
    vehicle.poll(now=13.0)
    vehicle.poll(now=13.99)
    vehicle.poll(now=14.0)

    heartbeats = connection.mav.calls_for("heartbeat_send")
    assert heartbeats == [
        (
            MAVLINK.MAV_TYPE_ONBOARD_CONTROLLER,
            MAVLINK.MAV_AUTOPILOT_INVALID,
            0,
            0,
            MAVLINK.MAV_STATE_ACTIVE,
            3,
        ),
        (
            MAVLINK.MAV_TYPE_ONBOARD_CONTROLLER,
            MAVLINK.MAV_AUTOPILOT_INVALID,
            0,
            0,
            MAVLINK.MAV_STATE_ACTIVE,
            3,
        ),
    ]


def test_second_pending_operation_is_rejected(vehicle, connection):
    """Break caught: a new operation overwrites an active MAVLink operation."""
    first = vehicle.takeoff_async(altitude_m=3.0, now=16.0)

    with pytest.raises(RuntimeError, match="another MAVLink operation is already active"):
        vehicle.arm_async(True, now=16.1)

    assert first.done() is False
    assert len(connection.mav.calls_for("command_long_send")) == 1


def test_rejected_command_ack_finishes_future_with_failure(vehicle, connection):
    """Break caught: ArduPilot's terminal command rejection is ignored."""
    future = vehicle.takeoff_async(altitude_m=3.0, now=17.0)
    push_ack(
        connection,
        MAVLINK.MAV_CMD_NAV_TAKEOFF,
        result=MAVLINK.MAV_RESULT_DENIED,
    )

    vehicle.poll(now=17.1)

    assert future.done() is True
    assert future.result().success is False
    assert future.result().message == (
        "ArduPilot rejected MAVLink command "
        f"{MAVLINK.MAV_CMD_NAV_TAKEOFF} with result {MAVLINK.MAV_RESULT_DENIED}"
    )


def test_in_progress_ack_waits_for_observed_mode_before_success(
    vehicle, connection
):
    """Break caught: an in-progress mode ACK is treated as terminal success."""
    push_heartbeat(connection, mode=0, armed=False)
    vehicle.poll(now=18.0)
    future = vehicle.set_mode_async(4, now=18.1)

    push_ack(
        connection,
        MAVLINK.MAV_CMD_DO_SET_MODE,
        result=MAVLINK.MAV_RESULT_IN_PROGRESS,
    )
    vehicle.poll(now=18.2)
    assert future.done() is False

    push_ack(connection, MAVLINK.MAV_CMD_DO_SET_MODE)
    vehicle.poll(now=18.3)
    assert future.done() is False

    push_heartbeat(connection, mode=4, armed=False)
    vehicle.poll(now=18.4)

    assert future.done() is True
    assert future.result().success is True


def test_parameter_future_collects_requested_values_out_of_order(
    vehicle, connection
):
    """Break caught: parameter responses are matched by arrival order."""
    future = vehicle.get_parameters_async(
        ("PLND_ENABLED", "PLND_TYPE"), now=20.0
    )
    connection.incoming.extend(
        [
            FakeMessage(
                "PARAM_VALUE",
                param_id=b"PLND_TYPE\0\0\0\0\0\0\0",
                param_value=1.0,
            ),
            FakeMessage(
                "PARAM_VALUE",
                param_id=b"UNRELATED\0\0\0\0\0\0\0",
                param_value=99.0,
            ),
            FakeMessage(
                "PARAM_VALUE",
                param_id=b"PLND_ENABLED\0\0\0\0",
                param_value=1.0,
            ),
        ]
    )

    vehicle.poll(now=20.1)

    assert future.done() is True
    assert future.result().parameters == {
        "PLND_ENABLED": 1.0,
        "PLND_TYPE": 1.0,
    }
    requests = connection.mav.calls_for("param_request_read_send")
    assert [request[2] for request in requests] == [
        b"PLND_ENABLED",
        b"PLND_TYPE",
    ]


def test_pid_backend_velocity_frame_routes_through_send_velocity(vehicle, connection):
    """Break caught: PID backend frames are rejected by the MAVLink transport."""
    backend = PidLandingBackend(pid_config())
    backend.start(
        now=1.0,
        dock=dock_observation(1.0),
        vehicle=vehicle_observation(),
    )
    decision = backend.update(
        now=1.1,
        dock=dock_observation(1.1),
        vehicle=vehicle_observation(),
    )

    assert decision.velocity is not None
    assert vehicle.send_velocity(
        decision.velocity.frame,
        decision.velocity.velocity_x,
        decision.velocity.velocity_y,
        decision.velocity.velocity_z,
        yaw_rate_rad_s=decision.velocity.yaw_rate_rad_s,
        now=30.0,
    ) is True

    call = connection.mav.calls_for("set_position_target_local_ned_send")[-1]
    assert call[3] == MAVLINK.MAV_FRAME_BODY_NED


@pytest.mark.parametrize(
    ("frame", "velocity", "expected_frame", "expected_ned"),
    [
        (
            VelocityFrame.MAP_ENU,
            (1.5, -2.0, 0.25),
            MAVLINK.MAV_FRAME_LOCAL_NED,
            (-2.0, 1.5, -0.25),
        ),
        (
            VelocityFrame.BODY_FLU,
            (1.5, -2.0, 0.25),
            MAVLINK.MAV_FRAME_BODY_NED,
            (1.5, 2.0, -0.25),
        ),
    ],
)
def test_velocity_uses_expected_mavlink_frame_and_axis_conversion(
    vehicle, connection, frame, velocity, expected_frame, expected_ned
):
    """Break caught: ROS ENU/FLU velocity is sent using MAVLink axes."""
    vehicle.send_velocity(frame, *velocity, now=30.0)

    call = connection.mav.calls_for("set_position_target_local_ned_send")[-1]
    assert call[3] == expected_frame
    assert call[8:11] == pytest.approx(expected_ned)
    assert call[4] == 3527


def test_body_velocity_with_yaw_rate_uses_flu_to_ned_conversion(vehicle, connection):
    vehicle.send_velocity(
        VelocityFrame.BODY_FLU,
        1.0,
        -2.0,
        0.25,
        yaw_rate_rad_s=0.4,
        now=30.0,
    )

    call = connection.mav.calls_for("set_position_target_local_ned_send")[-1]
    assert call[3] == MAVLINK.MAV_FRAME_BODY_NED
    assert call[4] == 1479
    assert call[8:11] == pytest.approx((1.0, 2.0, -0.25))
    assert call[15] == pytest.approx(-0.4)


@pytest.mark.parametrize(
    "values",
    [
        (float("nan"), 0.0, 0.0, 0.0),
        (0.0, float("inf"), 0.0, 0.0),
        (0.0, 0.0, float("-inf"), 0.0),
        (0.0, 0.0, 0.0, float("nan")),
    ],
)
def test_velocity_rejects_non_finite_velocity_and_yaw_rate(vehicle, values):
    with pytest.raises(ValueError, match="finite"):
        vehicle.send_velocity(
            VelocityFrame.MAP_ENU,
            values[0],
            values[1],
            values[2],
            yaw_rate_rad_s=values[3],
            now=30.0,
        )


def test_landing_target_uses_generated_pymavlink_message(vehicle, connection):
    """Break caught: precision target bypasses the shared pymavlink link."""
    sent = vehicle.send_landing_target(
        time_usec=123456,
        forward_m=1.25,
        right_m=-0.5,
        down_m=3.0,
    )

    assert sent is True
    call = connection.mav.calls_for("landing_target_send")[-1]
    assert call[0] == 123456
    assert call[2] == MAVLINK.MAV_FRAME_BODY_FRD
    assert call[8:11] == pytest.approx((1.25, -0.5, 3.0))
    assert call[-1] == 1


def test_prearm_result_reflects_sys_status_health(vehicle, connection):
    """Break caught: an unhealthy pre-arm bit is reported as successful."""
    future = vehicle.prearm_async(now=40.0)
    vehicle.poll(now=40.0)
    push_ack(connection, MAVLINK.MAV_CMD_RUN_PREARM_CHECKS)
    connection.incoming.append(
        FakeMessage(
            "SYS_STATUS",
            onboard_control_sensors_present=MAVLINK.MAV_SYS_STATUS_PREARM_CHECK,
            onboard_control_sensors_enabled=MAVLINK.MAV_SYS_STATUS_PREARM_CHECK,
            onboard_control_sensors_health=0,
        )
    )

    vehicle.poll(now=40.1)

    assert future.done() is True
    assert future.result().success is False
    assert "pre-arm" in future.result().message


def test_close_stops_vehicle_with_zero_velocity(vehicle, connection):
    """Break caught: shutdown closes the link without first clearing motion."""
    vehicle.send_velocity(VelocityFrame.MAP_ENU, 1.0, 2.0, 3.0, now=50.0)

    vehicle.close()

    call = connection.mav.calls_for("set_position_target_local_ned_send")[-1]
    assert call[8:11] == (0.0, 0.0, 0.0)
    assert connection.closed is True


def test_close_without_output_skips_final_zero_velocity(vehicle, connection):
    """Break caught: passive shutdown emits a final velocity command after authority is lost."""
    vehicle.send_velocity(VelocityFrame.MAP_ENU, 1.0, 2.0, 3.0, now=50.0)

    vehicle.close_without_output()

    calls = connection.mav.calls_for("set_position_target_local_ned_send")
    assert len(calls) == 1
    assert calls[0][8:11] == (2.0, 1.0, -3.0)
    assert connection.closed is True


def test_download_mission_requests_every_int_item_and_acks(vehicle, connection):
    """Break caught: a complete FC mission is not downloaded as INT items."""
    future = vehicle.download_mission_async(now=10.0)

    assert connection.mav.calls_for("mission_request_list_send")[-1] == (
        1,
        1,
        MAVLINK.MAV_MISSION_TYPE_MISSION,
    )
    connection.incoming.append(
        FakeMessage(
            "MISSION_COUNT",
            target_system=254,
            target_component=191,
            count=2,
            mission_type=MAVLINK.MAV_MISSION_TYPE_MISSION,
        )
    )
    vehicle.poll(10.1)
    assert connection.mav.calls_for("mission_request_int_send")[-1] == (1, 1, 0, 0)

    connection.incoming.append(
        mission_item_message(seq=0, command=MAVLINK.MAV_CMD_DO_LAND_START)
    )
    vehicle.poll(10.2)
    connection.incoming.append(
        mission_item_message(seq=1, command=MAVLINK.MAV_CMD_NAV_LAND)
    )
    vehicle.poll(10.3)

    assert future.done()
    result = future.result()
    assert [item.seq for item in result.items] == [0, 1]
    assert result.items[0].param4 == 4.0
    assert connection.mav.calls_for("mission_ack_send")[-1] == (
        1,
        1,
        MAVLINK.MAV_MISSION_ACCEPTED,
        MAVLINK.MAV_MISSION_TYPE_MISSION,
    )


def test_download_mission_re_requests_next_missing_sequence_for_out_of_order_items(
    vehicle, connection
):
    """Break caught: an early later item skips the lower missing sequence."""
    future = vehicle.download_mission_async(now=11.0)
    connection.incoming.append(
        FakeMessage("MISSION_COUNT", count=2, mission_type=0)
    )
    vehicle.poll(11.1)

    connection.incoming.append(mission_item_message(seq=1, command=21))
    vehicle.poll(11.2)

    assert future.done() is False
    assert connection.mav.calls_for("mission_request_int_send")[-1] == (1, 1, 0, 0)

    connection.incoming.append(mission_item_message(seq=0, command=189))
    vehicle.poll(11.3)

    assert future.done()
    assert [item.seq for item in future.result().items] == [0, 1]


def test_download_mission_duplicate_item_does_not_advance_completion(
    vehicle, connection
):
    """Break caught: a duplicate is treated as the next requested item."""
    future = vehicle.download_mission_async(now=12.0)
    connection.incoming.append(FakeMessage("MISSION_COUNT", count=2, mission_type=0))
    vehicle.poll(12.1)
    connection.incoming.append(mission_item_message(seq=0, command=189))
    vehicle.poll(12.2)
    assert connection.mav.calls_for("mission_request_int_send")[-1] == (1, 1, 1, 0)

    connection.incoming.append(mission_item_message(seq=0, command=189))
    vehicle.poll(12.3)

    assert future.done() is False
    assert connection.mav.calls_for("mission_request_int_send")[-1] == (1, 1, 1, 0)


def test_download_mission_ignores_wrong_source_and_mission_type(vehicle, connection):
    """Break caught: another vehicle or mission type can complete a download."""
    future = vehicle.download_mission_async(now=13.0)
    connection.incoming.extend(
        [
            FakeMessage("MISSION_COUNT", src_system=2, count=1, mission_type=0),
            FakeMessage("MISSION_COUNT", count=1, mission_type=1),
        ]
    )
    vehicle.poll(13.1)
    assert connection.mav.calls_for("mission_request_int_send") == []

    connection.incoming.append(FakeMessage("MISSION_COUNT", count=1, mission_type=0))
    vehicle.poll(13.2)
    connection.incoming.extend(
        [
            mission_item_message(seq=0, command=189, mission_type=1),
            mission_item_message(seq=0, command=189, src_system=2),
        ]
    )
    vehicle.poll(13.3)

    assert future.done() is False
    assert connection.mav.calls_for("mission_ack_send") == []


def test_download_mission_empty_count_acks_and_returns_empty_mission(
    vehicle, connection
):
    """Break caught: an empty FC mission hangs or is rejected."""
    future = vehicle.download_mission_async(now=14.0)
    connection.incoming.append(FakeMessage("MISSION_COUNT", count=0, mission_type=0))
    vehicle.poll(14.1)

    assert future.done()
    assert future.result().items == ()
    assert connection.mav.calls_for("mission_ack_send")[-1][2] == MAVLINK.MAV_MISSION_ACCEPTED


def test_download_mission_retries_the_missing_item_request(vehicle, connection):
    """Break caught: a lost indexed item request is never resent."""
    future = vehicle.download_mission_async(now=15.0)
    connection.incoming.append(FakeMessage("MISSION_COUNT", count=1, mission_type=0))
    vehicle.poll(15.1)
    assert len(connection.mav.calls_for("mission_request_int_send")) == 1

    vehicle.poll(15.59)
    assert len(connection.mav.calls_for("mission_request_int_send")) == 1
    vehicle.poll(15.6)

    assert future.done() is False
    assert connection.mav.calls_for("mission_request_int_send")[-1] == (1, 1, 0, 0)


def test_download_mission_times_out_if_fc_never_replies(vehicle):
    """Break caught: an unanswered mission request remains active forever."""
    future = vehicle.download_mission_async(now=16.0)

    vehicle.poll(19.01)

    assert future.done()
    assert future.result().success is False
    assert future.result().message == "MAVLink operation timed out"


def test_download_mission_re_requests_when_an_item_sequence_is_absent(
    vehicle, connection
):
    """Break caught: an out-of-range item is accepted in place of a missing one."""
    future = vehicle.download_mission_async(now=17.0)
    connection.incoming.append(FakeMessage("MISSION_COUNT", count=2, mission_type=0))
    vehicle.poll(17.1)
    connection.incoming.append(mission_item_message(seq=2, command=189))
    vehicle.poll(17.2)

    assert future.done() is False
    assert connection.mav.calls_for("mission_request_int_send")[-1] == (1, 1, 0, 0)


def test_download_mission_preserves_nonzero_opaque_id_and_ignores_absent_extension(
    vehicle, connection
):
    """Break caught: opaque mission identity is mishandled when extensions vary."""
    future = vehicle.download_mission_async(now=18.0)
    connection.incoming.append(
        FakeMessage("MISSION_COUNT", count=0, mission_type=0, opaque_id=12345)
    )
    vehicle.poll(18.1)
    assert future.result().opaque_id == 12345

    future = vehicle.download_mission_async(now=18.2)
    connection.incoming.append(
        FakeMessage("MISSION_COUNT", count=0, mission_type=0, opaque_id=0)
    )
    vehicle.poll(18.3)
    assert future.result().opaque_id is None

    future = vehicle.download_mission_async(now=18.4)
    connection.incoming.append(FakeMessage("MISSION_COUNT", count=0, mission_type=0))
    vehicle.poll(18.5)
    assert future.result().opaque_id is None


def test_mission_current_updates_persistent_progress_and_absent_extensions(
    vehicle, connection
):
    """Break caught: MISSION_CURRENT progress is dropped or requires extensions."""
    connection.incoming.append(
        FakeMessage(
            "MISSION_CURRENT",
            seq=7,
            total=9,
            mission_state=MAVLINK.MISSION_STATE_ACTIVE,
            mission_mode=1,
            mission_id=2468,
        )
    )
    vehicle.poll(20.0)

    assert vehicle.state.mission_seq == 7
    assert vehicle.state.mission_total == 9
    assert vehicle.state.mission_state == MAVLINK.MISSION_STATE_ACTIVE
    assert vehicle.state.mission_mode == 1
    assert vehicle.state.mission_id == 2468
    assert vehicle.state.mission_current_at == 20.0

    connection.incoming.append(
        FakeMessage("MISSION_CURRENT", seq=8, total=9, mission_id=0)
    )
    vehicle.poll(20.1)
    assert vehicle.state.mission_seq == 8
    assert vehicle.state.mission_state is None
    assert vehicle.state.mission_mode is None
    assert vehicle.state.mission_id is None
    assert vehicle.state.mission_current_at == 20.1

    connection.incoming.append(FakeMessage("MISSION_CURRENT", seq=9, total=9))
    vehicle.poll(20.2)
    assert vehicle.state.mission_id is None
    assert vehicle.state.mission_current_at == 20.2


def test_mission_current_without_extensions_preserves_sequence_zero_progress(
    vehicle, connection
):
    """Break caught: runtime MAVLink v1 progress at sequence zero is discarded."""
    connection.incoming.append(FakeMessage("MISSION_CURRENT", seq=0, total=3))
    vehicle.poll(21.0)

    assert vehicle.state.mission_seq == 0
    assert vehicle.state.mission_total == 3
    assert vehicle.state.mission_state is None
    assert vehicle.state.mission_current_at == 21.0


@pytest.mark.parametrize(
    ("method", "command", "params", "confirmed_mode"),
    [
        (
            lambda v: v.set_mission_current_async(0, reset=True, now=21.0),
            MAVLINK.MAV_CMD_DO_SET_MISSION_CURRENT,
            (0.0, 1.0),
            None,
        ),
        (
            lambda v: v.start_mission_async(21.0),
            MAVLINK.MAV_CMD_MISSION_START,
            (0.0, 0.0),
            None,
        ),
        (
            lambda v: v.request_dock_async(21.0),
            MAVLINK.MAV_CMD_DO_LAND_START,
            (0.0, 0.0),
            None,
        ),
        (
            lambda v: v.rtl_async(21.0),
            MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            (0.0, 0.0),
            27,
        ),
        (
            lambda v: v.land_async(21.0),
            MAVLINK.MAV_CMD_NAV_LAND,
            (0.0, 0.0),
            9,
        ),
    ],
)
def test_acknowledged_cycle_commands_send_exact_parameters(
    method, command, params, confirmed_mode, vehicle, connection
):
    """Break caught: a cycle command has the wrong MAVLink command or parameters."""
    future = method(vehicle)
    sent = connection.mav.calls_for("command_long_send")[-1]

    assert sent == (1, 1, command, 0, *params, 0.0, 0.0, 0.0, 0.0, 0.0)
    push_ack(connection, command)
    vehicle.poll(21.1)
    if confirmed_mode is None:
        assert future.result().success
        return

    assert future.done() is False
    push_heartbeat(connection, mode=confirmed_mode, armed=True)
    vehicle.poll(21.2)
    assert future.result().success


@pytest.mark.parametrize(
    ("method", "command", "confirmed_mode"),
    [
        (lambda v: v.rtl_async(22.0), MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH, 27),
        (lambda v: v.land_async(22.0), MAVLINK.MAV_CMD_NAV_LAND, 9),
    ],
)
def test_rtl_and_land_wait_for_mode_confirmation_after_ack(
    method, command, confirmed_mode, vehicle, connection
):
    """Break caught: ACK success is mistaken for a completed flight-mode transition."""
    future = method(vehicle)

    push_ack(connection, command)
    vehicle.poll(22.1)

    assert future.done() is False

    push_heartbeat(connection, mode=confirmed_mode, armed=True)
    vehicle.poll(22.2)

    assert future.done() is True
    assert future.result().success is True
    assert future.result().current_mode == confirmed_mode


def test_cancel_pending_operation_ignores_late_ack_and_frees_transport(
    vehicle, connection
):
    """Break caught: a canceled operation can still complete from a stale ACK or block the next command."""
    stale = vehicle.land_async(now=23.0)

    vehicle.cancel_pending_operation()
    push_ack(connection, MAVLINK.MAV_CMD_NAV_LAND)
    push_heartbeat(connection, mode=9, armed=True)
    vehicle.receive_telemetry(now=23.1)
    vehicle.service_pending(now=23.1)
    replacement = vehicle.rtl_async(now=23.2)

    assert stale.done() is False
    assert replacement.done() is False
    assert connection.mav.calls_for("command_long_send")[-1][2] == (
        MAVLINK.MAV_CMD_NAV_RETURN_TO_LAUNCH
    )


@pytest.mark.parametrize("sequence", (-1, 32768, 1.5, True))
def test_set_mission_current_rejects_invalid_sequence(vehicle, sequence):
    """Break caught: invalid sequence numbers are encoded into control commands."""
    with pytest.raises(ValueError, match="mission sequence"):
        vehicle.set_mission_current_async(sequence, reset=False, now=22.0)


def test_command_and_mission_download_share_the_single_pending_operation(
    vehicle, connection
):
    """Break caught: a mission operation can overwrite an unacknowledged command."""
    command = vehicle.land_async(now=23.0)

    with pytest.raises(RuntimeError, match="another MAVLink operation is already active"):
        vehicle.download_mission_async(now=23.1)

    assert command.done() is False
    assert connection.mav.calls_for("mission_request_list_send") == []


def test_mission_download_and_command_share_the_single_pending_operation(
    vehicle, connection
):
    """Break caught: a command can overwrite an incomplete mission download."""
    mission = vehicle.download_mission_async(now=24.0)

    with pytest.raises(RuntimeError, match="another MAVLink operation is already active"):
        vehicle.land_async(now=24.1)

    assert mission.done() is False
    assert connection.mav.calls_for("command_long_send") == []


@pytest.mark.parametrize("count", (32769, 65535))
def test_download_mission_accepts_full_uint16_mission_counts(
    vehicle, connection, count
):
    """Break caught: a valid large uint16 count is discarded before item zero."""
    future = vehicle.download_mission_async(now=60.0)
    connection.incoming.append(FakeMessage("MISSION_COUNT", count=count, mission_type=0))

    vehicle.poll(60.1)

    assert future.done() is False
    assert connection.mav.calls_for("mission_request_int_send") == [(1, 1, 0, 0)]


def test_download_mission_ignores_count_outside_uint16_range(vehicle, connection):
    """Break caught: an invalid count is accepted and produces an unsafe request."""
    future = vehicle.download_mission_async(now=61.0)
    connection.incoming.append(FakeMessage("MISSION_COUNT", count=65536, mission_type=0))

    vehicle.poll(61.1)

    assert future.done() is False
    assert connection.mav.calls_for("mission_request_int_send") == []


def test_download_mission_accepts_high_uint16_item_sequence_after_prior_items(
    vehicle, connection
):
    """Break caught: a high valid item is rejected using the set-current limit."""
    count = 32769
    future = vehicle.download_mission_async(now=62.0)
    connection.incoming.append(FakeMessage("MISSION_COUNT", count=count, mission_type=0))
    vehicle.poll(62.1)

    # This controlled pending state represents the already received lower
    # items without creating tens of thousands of fake MAVLink messages.
    pending = vehicle._pending
    assert pending is not None
    pending.mission_items = {
        sequence: MissionItem(
            sequence, MAVLINK.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 16,
            0.0, 0.0, 0.0, 0.0, 450000000, 90000000, 4.0,
        )
        for sequence in range(32768)
    }
    connection.incoming.append(mission_item_message(seq=32768, command=21))

    vehicle.poll(62.2)

    assert future.done() is True
    assert future.result().items[-1].seq == 32768
    assert connection.mav.calls_for("mission_ack_send")[-1][2] == MAVLINK.MAV_MISSION_ACCEPTED

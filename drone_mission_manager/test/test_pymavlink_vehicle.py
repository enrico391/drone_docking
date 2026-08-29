"""Behavioral tests for the lightweight direct pymavlink vehicle adapter."""

import pytest
from pymavlink import mavutil

from drone_mission_manager.pymavlink_vehicle import (
    MavlinkVehicle,
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

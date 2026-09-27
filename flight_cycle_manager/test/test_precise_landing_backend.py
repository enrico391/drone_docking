"""Tests for the pure native ArduPilot precision-landing backend."""

from flight_cycle_manager.landing_backend import (
    DockObservation,
    DockTargetMode,
    LandingBackendOutcome,
    VehicleObservation,
)
from flight_cycle_manager.precise_landing_backend import (
    PreciseLandingBackend,
    PreciseLandingConfig,
)


def dock_observation(received_at=0.0, *, yaw_error_rad=0.0):
    return DockObservation(
        forward_m=0.1,
        left_m=-0.1,
        distance_m=2.0,
        yaw_error_rad=yaw_error_rad,
        mode=DockTargetMode.OUTER,
        source_stamp_s=received_at,
        received_at=received_at,
    )


def vehicle_observation(
    *, mode=4, armed=True, flying=True, landed_at=0.0
):
    return VehicleObservation(
        position_enu_m=(0.0, 0.0, 4.0),
        velocity_enu_m_s=(0.0, 0.0, 0.0),
        armed=armed,
        flying=flying,
        mode=mode,
        landed_at=landed_at,
    )


def guided_vehicle():
    return vehicle_observation(mode=4)


def land_vehicle(**overrides):
    return vehicle_observation(mode=9, **overrides)


def backend_config(**overrides):
    values = dict(
        warmup_s=1.0,
        dock_fresh_timeout_s=0.25,
        landed_state_timeout_s=0.5,
        touchdown_warn_after_s=30.0,
        disarm_warn_after_s=20.0,
    )
    values.update(overrides)
    return PreciseLandingConfig(**values)


def started_native_backend(config=None):
    backend = PreciseLandingBackend(config or backend_config())
    backend.start(now=0.0, dock=dock_observation(), vehicle=guided_vehicle())
    return backend


def waiting_for_land_result(backend):
    decision = backend.update(
        now=1.0, dock=dock_observation(1.0), vehicle=guided_vehicle()
    )
    assert decision.request_land is True
    return backend


def native_backend_waiting_landed():
    backend = waiting_for_land_result(started_native_backend())
    backend.notify_land_result(success=True, now=1.1)
    return backend


def test_native_backend_streams_target_during_continuous_warmup_then_requests_land():
    """Stopping the fresh stream resets the handoff instead of issuing LAND."""
    backend = started_native_backend()

    warming = backend.update(
        now=0.5, dock=dock_observation(0.5), vehicle=guided_vehicle()
    )
    decision = backend.update(
        now=1.0, dock=dock_observation(1.0), vehicle=guided_vehicle()
    )
    repeated = backend.update(
        now=1.1, dock=dock_observation(1.1), vehicle=guided_vehicle()
    )

    assert warming.stream_landing_target is True
    assert warming.substate == "WARMUP"
    assert decision.stream_landing_target is True
    assert decision.request_land is True
    assert decision.substate == "REQUEST_LAND"
    assert repeated.request_land is False
    assert repeated.substate == "REQUEST_LAND"


def test_target_loss_before_land_is_precommit_failure():
    """Removing the visual target before handoff must stop the landing path."""
    backend = started_native_backend()

    decision = backend.update(now=0.5, dock=None, vehicle=guided_vehicle())

    assert decision.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT
    assert decision.reason == "dock target lost before LAND was accepted"


def test_stale_target_before_land_is_precommit_failure():
    """An old observation must not keep the LANDING_TARGET warmup alive."""
    backend = started_native_backend()

    decision = backend.update(
        now=0.5, dock=dock_observation(0.0), vehicle=guided_vehicle()
    )

    assert decision.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT


def test_native_backend_requests_one_optional_yaw_correction():
    """A repeated observation cannot turn a one-shot yaw request into a loop."""
    backend = started_native_backend(
        backend_config(yaw_correction_threshold_rad=0.1)
    )

    first = backend.update(
        now=0.1,
        dock=dock_observation(0.1, yaw_error_rad=0.2),
        vehicle=guided_vehicle(),
    )
    repeated = backend.update(
        now=0.2,
        dock=dock_observation(0.2, yaw_error_rad=0.3),
        vehicle=guided_vehicle(),
    )

    assert first.yaw_correction_rad == 0.2
    assert repeated.yaw_correction_rad is None


def test_land_request_waits_for_manager_notification_before_commitment():
    """Only the manager's acknowledged LAND result commits native landing."""
    backend = waiting_for_land_result(started_native_backend())

    waiting = backend.update(
        now=1.1, dock=dock_observation(1.1), vehicle=guided_vehicle()
    )
    backend.notify_land_result(success=True, now=1.2)
    accepted = backend.update(
        now=1.3, dock=dock_observation(1.3), vehicle=land_vehicle()
    )

    assert waiting.substate == "REQUEST_LAND"
    assert waiting.request_land is False
    assert accepted.substate == "WAIT_LANDED"
    assert accepted.outcome is LandingBackendOutcome.RUNNING


def test_rejected_land_result_is_precommit_failure():
    """A rejected LAND command has not transferred descent authority."""
    backend = waiting_for_land_result(started_native_backend())

    backend.notify_land_result(success=False, now=1.1)
    decision = backend.update(
        now=1.2, dock=dock_observation(1.2), vehicle=guided_vehicle()
    )

    assert decision.outcome is LandingBackendOutcome.FAIL_BEFORE_COMMIT
    assert decision.reason == "LAND command was rejected"


def test_post_land_target_loss_pauses_streaming_until_a_fresh_target_returns():
    """PLND_STRICT recovery owns visual loss after LAND has been accepted."""
    backend = native_backend_waiting_landed()

    missing = backend.update(now=1.2, dock=None, vehicle=land_vehicle())
    recovered = backend.update(
        now=1.3, dock=dock_observation(1.3), vehicle=land_vehicle()
    )

    assert missing.outcome is LandingBackendOutcome.RUNNING
    assert missing.stream_landing_target is False
    assert recovered.stream_landing_target is True
    assert recovered.substate == "WAIT_LANDED"


def test_mode_departure_is_not_recovered_by_backend():
    """A post-commit pilot or failsafe mode change is manager-authority failure."""
    backend = native_backend_waiting_landed()

    decision = backend.update(
        now=2.0, dock=dock_observation(2.0), vehicle=vehicle_observation(mode=5)
    )

    assert decision.reason == "vehicle left LAND"
    assert decision.outcome is LandingBackendOutcome.FAIL_AFTER_COMMIT


def test_stale_landed_state_does_not_count_as_touchdown():
    """An old landed flag cannot advance a still-airborne LAND operation."""
    backend = native_backend_waiting_landed()

    decision = backend.update(
        now=2.0,
        dock=dock_observation(2.0),
        vehicle=land_vehicle(flying=False, landed_at=0.0),
    )

    assert decision.substate == "WAIT_LANDED"
    assert decision.outcome is LandingBackendOutcome.RUNNING


def test_fresh_touchdown_waits_for_auto_disarm_then_completes():
    """A fresh touchdown and later disarm heartbeat complete the backend."""
    backend = native_backend_waiting_landed()

    touchdown = backend.update(
        now=1.2,
        dock=dock_observation(1.2),
        vehicle=land_vehicle(flying=False, landed_at=1.2),
    )
    complete = backend.update(
        now=1.3,
        dock=dock_observation(1.3),
        vehicle=land_vehicle(armed=False, flying=False, landed_at=1.3),
    )

    assert touchdown.substate == "WAIT_DISARM"
    assert touchdown.request_disarm is False
    assert complete.substate == "COMPLETE"
    assert complete.outcome is LandingBackendOutcome.COMPLETE


def test_warning_thresholds_do_not_terminate_accepted_land():
    """Slow landing and auto-disarm are warnings, never post-commit failure."""
    backend = PreciseLandingBackend(
        backend_config(touchdown_warn_after_s=0.1, disarm_warn_after_s=0.1)
    )
    backend.start(now=0.0, dock=dock_observation(), vehicle=guided_vehicle())
    waiting_for_land_result(backend)
    backend.notify_land_result(success=True, now=1.1)

    landing_warning = backend.update(
        now=1.3, dock=dock_observation(1.3), vehicle=land_vehicle()
    )
    touchdown = backend.update(
        now=1.4,
        dock=dock_observation(1.4),
        vehicle=land_vehicle(flying=False, landed_at=1.4),
    )
    disarm_warning = backend.update(
        now=1.6,
        dock=dock_observation(1.6),
        vehicle=land_vehicle(flying=False, landed_at=1.6),
    )

    assert landing_warning.reason == "landing has not completed"
    assert landing_warning.outcome is LandingBackendOutcome.RUNNING
    assert touchdown.substate == "WAIT_DISARM"
    assert disarm_warning.reason == "automatic disarm has not completed"
    assert disarm_warning.outcome is LandingBackendOutcome.RUNNING

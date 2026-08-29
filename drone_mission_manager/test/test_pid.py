import math

import pytest

from drone_mission_manager.pid import PID, PIDConfig


def config(**overrides):
    values = dict(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        integral_limit=1.0,
        output_limit=1.0,
        derivative_filter_time_constant_s=0.0,
    )
    values.update(overrides)
    return PIDConfig(**values)


def test_pid_rejects_invalid_limits():
    with pytest.raises(ValueError, match="output_limit"):
        PIDConfig(kp=1.0, ki=0.0, kd=0.0, integral_limit=1.0,
                  output_limit=0.0, derivative_filter_time_constant_s=0.1)


def test_pid_proportional_output_is_symmetric_and_clamped():
    controller = PID(config(output_limit=0.5, kp=2.0))
    assert controller.update(0.1, now=1.0) == pytest.approx(0.2)
    assert controller.update(-1.0, now=1.1) == pytest.approx(-0.5)


def test_integral_stops_growing_while_output_is_saturated():
    controller = PID(config(kp=0.0, ki=1.0, integral_limit=10.0,
                            output_limit=0.2, max_dt_s=2.0))
    controller.update(1.0, now=1.0)
    assert controller.update(1.0, now=2.0) == pytest.approx(0.2)
    assert controller.update(-1.0, now=3.0) < 0.2


def test_reset_removes_integral_and_derivative_history():
    controller = PID(config(kp=0.0, ki=1.0, kd=1.0))
    controller.update(1.0, now=1.0)
    controller.update(2.0, now=1.1)
    controller.reset()
    assert controller.update(0.0, now=5.0) == 0.0


def test_filtered_derivative_follows_error_direction():
    controller = PID(config(kp=0.0, kd=1.0,
                            derivative_filter_time_constant_s=0.2))
    assert controller.update(0.0, now=1.0) == 0.0
    assert controller.update(1.0, now=1.1) > 0.0
    assert controller.update(0.0, now=1.2) < 0.0


def test_integral_is_clamped():
    controller = PID(config(kp=0.0, ki=1.0, integral_limit=0.1,
                            output_limit=10.0))
    controller.update(1.0, now=1.0)
    assert controller.update(1.0, now=1.1) == pytest.approx(0.1)


def test_repeated_timestamp_resets_derivative_without_spike():
    controller = PID(config(kp=0.0, kd=1.0))
    controller.update(0.0, now=1.0)
    controller.update(1.0, now=1.1)
    assert controller.update(2.0, now=1.1) == 0.0
    assert controller.update(2.0, now=1.2) == 0.0


def test_time_jump_resets_derivative_without_spike():
    controller = PID(config(kp=0.0, kd=1.0))
    controller.update(0.0, now=1.0)
    controller.update(1.0, now=1.1)
    assert controller.update(2.0, now=2.0) == 0.0


@pytest.mark.parametrize("error, now", [(math.nan, 1.0), (math.inf, 1.0),
                                         (1.0, math.nan), (1.0, math.inf)])
def test_non_finite_inputs_raise(error, now):
    with pytest.raises(ValueError):
        PID(config()).update(error, now)


def test_config_rejects_non_finite_and_invalid_values():
    for field, value in (("kp", math.nan), ("ki", -1.0), ("kd", math.inf),
                         ("integral_limit", -1.0),
                         ("derivative_filter_time_constant_s", -1.0),
                         ("min_dt_s", 0.0), ("max_dt_s", 0.0)):
        values = dict(config().__dict__)
        values[field] = value
        with pytest.raises(ValueError, match=field):
            PIDConfig(**values)
    with pytest.raises(ValueError, match="min_dt_s"):
        PIDConfig(**dict(config().__dict__, min_dt_s=1.0, max_dt_s=1.0))

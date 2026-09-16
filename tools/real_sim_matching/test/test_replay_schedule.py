import csv

import pytest

from replay_schedule import flight_mode_id, load_rc_calibration, load_schedule, mavlink_source_system, takeoff_altitude


def test_load_schedule_returns_relative_integer_rc_samples(tmp_path):
    path = tmp_path / "rc.csv"
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["time_s", "C1", "C2", "C3", "C4"])
        writer.writeheader()
        writer.writerows([
            {"time_s": "10.0", "C1": "1500", "C2": "1501", "C3": "1499", "C4": "1502"},
            {"time_s": "10.1", "C1": "1510", "C2": "1490", "C3": "1500", "C4": "1500"},
        ])
    assert load_schedule(path) == [
        (0.0, (1500, 1501, 1499, 1502)),
        (0.1, (1510, 1490, 1500, 1500)),
    ]


def test_load_schedule_rejects_non_monotonic_time(tmp_path):
    path = tmp_path / "rc.csv"
    path.write_text("time_s,C1,C2,C3,C4\n1,1500,1500,1500,1500\n0,1500,1500,1500,1500\n")
    with pytest.raises(ValueError, match="strictly increasing"):
        load_schedule(path)


def test_rc_replay_uses_the_configured_gcs_system_id():
    assert mavlink_source_system(replaying_rc=True) == 255
    assert mavlink_source_system(replaying_rc=False) == 250


def test_load_rc_calibration_selects_only_primary_input_scaling(tmp_path):
    path = tmp_path / "real.param"
    path.write_text(
        "RC1_MIN,1068\nRC1_MAX,1931\nRC1_TRIM,1496\nRC1_DZ,20\n"
        "RC2_REVERSED,0\nRC5_MIN,1000\nATC_RAT_RLL_P,0.13\n"
    )
    assert load_rc_calibration(path) == {
        "RC1_MIN": 1068.0, "RC1_MAX": 1931.0,
        "RC1_TRIM": 1496.0, "RC1_DZ": 20.0,
        "RC2_REVERSED": 0.0,
    }


def test_replay_flight_mode_ids_match_arducopter():
    assert flight_mode_id("althold") == 2
    assert flight_mode_id("loiter") == 5


def test_takeoff_altitude_uses_override_or_replay_default():
    assert takeoff_altitude(replaying=True, override=None) == 12.0
    assert takeoff_altitude(replaying=False, override=None) == 2.0
    assert takeoff_altitude(replaying=True, override=4.73) == 4.73

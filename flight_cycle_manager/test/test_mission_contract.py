import math

import pytest

from flight_cycle_manager.mission_contract import (
    MissionContractError,
    MissionItem,
    MissionValidationConfig,
    validate_docking_mission,
)


DO_LAND_START = 189
NAV_WAYPOINT = 16
NAV_LOITER_TIME = 19
NAV_LAND = 21

GLOBAL = 0
GLOBAL_RELATIVE_ALT = 3
GLOBAL_INT = 5
GLOBAL_RELATIVE_ALT_INT = 6


def item(seq, command, *, frame=GLOBAL_RELATIVE_ALT_INT, p1=0.0,
         x=450000000, y=90000000, z=4.0, p2=0.0, p3=0.0, p4=0.0):
    return MissionItem(seq, frame, command, p1, p2, p3, p4, x, y, z)


def config():
    return MissionValidationConfig(
        min_handoff_loiter_s=5.0,
        landing_location_tolerance_m=2.0,
    )


def contract_mission(*, frame=GLOBAL_RELATIVE_ALT_INT, fallback=None):
    landing = item(4, NAV_LAND, frame=frame)
    if fallback is not None:
        landing = fallback
    return [
        item(0, NAV_WAYPOINT, frame=frame),
        item(1, DO_LAND_START, frame=frame),
        item(2, NAV_WAYPOINT, frame=frame),
        item(3, NAV_LOITER_TIME, frame=frame, p1=10.0),
        landing,
    ]


def test_extracts_unique_docking_contract_from_sorted_items():
    contract = validate_docking_mission(list(reversed(contract_mission())), config())

    assert contract.item_count == 5
    assert contract.land_start_seq == 1
    assert contract.handoff_seq == 3
    assert contract.fallback_land_seq == 4


def test_accepts_supported_global_and_global_relative_frames():
    for frame in (GLOBAL, GLOBAL_RELATIVE_ALT, GLOBAL_INT, GLOBAL_RELATIVE_ALT_INT):
        assert validate_docking_mission(contract_mission(frame=frame), config()).handoff_seq == 3


def test_accepts_different_supported_frames_for_handoff_and_fallback():
    mission = contract_mission(frame=GLOBAL_RELATIVE_ALT_INT)
    mission[-1] = item(4, NAV_LAND, frame=GLOBAL_INT)

    assert validate_docking_mission(mission, config()).fallback_land_seq == 4


def test_rejects_missing_land_start():
    with pytest.raises(MissionContractError, match="exactly one DO_LAND_START"):
        validate_docking_mission([item(0, NAV_WAYPOINT)], config())


def test_rejects_duplicate_land_start():
    mission = contract_mission()
    mission[2] = item(2, DO_LAND_START)

    with pytest.raises(MissionContractError, match="exactly one DO_LAND_START"):
        validate_docking_mission(mission, config())


def test_rejects_non_contiguous_sequence_numbers():
    mission = contract_mission()
    mission[3] = item(7, NAV_LOITER_TIME, p1=10.0)

    with pytest.raises(MissionContractError, match="contiguous"):
        validate_docking_mission(mission, config())


def test_rejects_loiter_before_land_start_as_contract_candidate():
    mission = [
        item(0, NAV_LOITER_TIME, p1=10.0),
        item(1, DO_LAND_START),
        item(2, NAV_WAYPOINT),
        item(3, NAV_LAND),
    ]

    with pytest.raises(MissionContractError, match="exactly one NAV_LOITER_TIME"):
        validate_docking_mission(mission, config())


def test_rejects_multiple_candidate_loiters_after_marker():
    mission = contract_mission()
    mission.insert(3, item(3, NAV_LOITER_TIME, p1=10.0))
    mission[4] = item(4, NAV_LOITER_TIME, p1=10.0)
    mission[5] = item(5, NAV_LAND)

    with pytest.raises(MissionContractError, match="exactly one NAV_LOITER_TIME"):
        validate_docking_mission(mission, config())


def test_rejects_short_handoff_loiter():
    mission = contract_mission()
    mission[3] = item(3, NAV_LOITER_TIME, p1=4.999)

    with pytest.raises(MissionContractError, match="shorter than the minimum"):
        validate_docking_mission(mission, config())


def test_accepts_handoff_loiter_at_minimum_duration():
    mission = contract_mission()
    mission[3] = item(3, NAV_LOITER_TIME, p1=5.0)

    assert validate_docking_mission(mission, config()).handoff_seq == 3


def test_rejects_non_adjacent_fallback_land():
    mission = [
        item(0, DO_LAND_START),
        item(1, NAV_LOITER_TIME, p1=10.0),
        item(2, NAV_WAYPOINT),
        item(3, NAV_LAND),
    ]

    with pytest.raises(MissionContractError, match="immediately follow"):
        validate_docking_mission(mission, config())


def test_rejects_landing_location_beyond_tolerance():
    mission = contract_mission()
    mission[-1] = item(4, NAV_LAND, x=450000000, y=90000300)

    with pytest.raises(MissionContractError, match="location"):
        validate_docking_mission(mission, config())


def test_accepts_nav_land_current_location_fallback():
    mission = contract_mission()
    mission[-1] = item(4, NAV_LAND, x=0, y=0)

    assert validate_docking_mission(mission, config()).fallback_land_seq == 4


@pytest.mark.parametrize("x,y", [(0, 90000000), (450000000, 0)])
def test_rejects_single_zero_nav_land_coordinate(x, y):
    mission = contract_mission()
    mission[-1] = item(4, NAV_LAND, x=x, y=y)

    with pytest.raises(MissionContractError, match="coordinates"):
        validate_docking_mission(mission, config())


def test_rejects_unsupported_coordinate_frame():
    mission = contract_mission()
    mission[-1] = item(4, NAV_LAND, frame=11)

    with pytest.raises(MissionContractError, match="coordinate frame"):
        validate_docking_mission(mission, config())


def test_rejects_fractional_e7_coordinates():
    mission = contract_mission()
    mission[3] = item(3, NAV_LOITER_TIME, p1=10.0, x=450000000.5)
    mission[4] = item(4, NAV_LAND, x=450000000.5)

    with pytest.raises(MissionContractError, match="integer"):
        validate_docking_mission(mission, config())


def test_rejects_boolean_e7_coordinates():
    mission = contract_mission()
    mission[3] = item(3, NAV_LOITER_TIME, p1=10.0, x=True)
    mission[4] = item(4, NAV_LAND, x=True)

    with pytest.raises(MissionContractError, match="integer"):
        validate_docking_mission(mission, config())


@pytest.mark.parametrize(
    "field,value",
    [
        ("p1", math.nan),
        ("p2", math.inf),
        ("p3", -math.inf),
        ("p4", math.nan),
        ("x", math.nan),
        ("y", math.inf),
        ("z", -math.inf),
    ],
)
def test_rejects_non_finite_mission_fields(field, value):
    mission = contract_mission()
    mission[3] = item(3, NAV_LOITER_TIME, p1=10.0)
    values = {
        "p1": mission[3].param1,
        "p2": mission[3].param2,
        "p3": mission[3].param3,
        "p4": mission[3].param4,
        "x": mission[3].x,
        "y": mission[3].y,
        "z": mission[3].z,
    }
    values[field] = value
    mission[3] = MissionItem(
        mission[3].seq,
        mission[3].frame,
        mission[3].command,
        values["p1"], values["p2"], values["p3"], values["p4"],
        values["x"], values["y"], values["z"],
    )

    with pytest.raises(MissionContractError, match="finite|integer"):
        validate_docking_mission(mission, config())

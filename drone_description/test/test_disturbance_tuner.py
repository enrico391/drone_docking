import pytest

from drone_description.disturbance_tuner import parse_change


def test_parse_change_accepts_vector_and_enable_switch():
    assert parse_change({"name": "constant.force", "value": [0.2, -0.1, 0]}) == (
        "constant.force", [0.2, -0.1, 0.0]
    )
    assert parse_change({"name": "enabled", "value": True}) == ("enabled", True)


@pytest.mark.parametrize("payload", [
    {"name": "unknown", "value": "x500"},
    {"name": "enabled", "value": "false"},
    {"name": "limits.force", "value": [1, 2]},
    {"name": "gust.interval", "value": float("inf")},
    {"name": "update_rate", "value": 0},
])
def test_parse_change_rejects_unsupported_or_invalid_values(payload):
    with pytest.raises(ValueError):
        parse_change(payload)

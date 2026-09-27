from extract_reference import mode_windows, shift_and_clip


def test_mode_windows_close_at_next_mode_and_log_end():
    events = [(0.0, 0), (10.0, 2), (20.0, 5), (30.0, 2)]
    assert mode_windows(events, 35.0, 2) == [(10.0, 20.0), (30.0, 35.0)]


def test_shift_and_clip_uses_requested_window_origin():
    rows = [{"time_s": 9.9, "value": 1}, {"time_s": 10.0, "value": 2},
            {"time_s": 11.5, "value": 3}, {"time_s": 12.1, "value": 4}]
    assert shift_and_clip(rows, 10.0, 12.0) == [
        {"time_s": 0.0, "value": 2}, {"time_s": 1.5, "value": 3}
    ]

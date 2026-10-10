"""Timing must distinguish capture age from callback processing duration."""

import json
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import Image

from dock_apriltag_localizer import dock_localizer


@pytest.mark.parametrize('outcome', ['accepted', 'outlier_rejected', 'no_target'])
def test_callback_reports_timing_even_without_published_pose(monkeypatch, outcome):
    durations = iter([10.0, 10.01, 10.08, 10.10, 10.12])
    monkeypatch.setattr(dock_localizer.time, 'perf_counter', lambda: next(durations))
    reports, poses = [], []
    transform = np.eye(4)
    node = SimpleNamespace(
        K=np.eye(3),
        bridge=SimpleNamespace(imgmsg_to_cv2=lambda *a, **k: np.zeros((2, 2, 3), np.uint8)),
        detector=SimpleNamespace(detect=lambda *a, **k: []),
        ids_pub=SimpleNamespace(publish=lambda msg: None),
        mode_pub=SimpleNamespace(publish=lambda msg: None),
        timing_pub=SimpleNamespace(publish=lambda msg: reports.append(json.loads(msg.data))),
        debug_pub=None,
        _select_pose=lambda candidates: (None, 'NO_TARGET') if outcome == 'no_target'
        else (transform, 'OUTER'),
        pose_filter=SimpleNamespace(update=lambda *a: (
            transform if outcome == 'accepted' else None, outcome)),
        _publish_pose=lambda *a: poses.append(a),
        get_logger=lambda: SimpleNamespace(warning=lambda *a, **k: None),
    )
    ages = iter([0.05, 0.15])
    node._frame_age_s = lambda stamp: next(ages)
    msg = Image()
    msg.header.stamp.sec = 100
    dock_localizer.DockLocalizer._image_cb(node, msg)
    data = reports[0]
    assert data['source_stamp_s'] == 100.0
    assert data['image_age_at_callback_s'] == 0.05
    assert data['detection_s'] == pytest.approx(0.07)
    assert data['callback_processing_s'] == pytest.approx(0.12)
    assert data['filter_status'] == outcome
    assert data['pose_age_at_publish_s'] == (0.15 if outcome == 'accepted' else None)
    assert len(poses) == (1 if outcome == 'accepted' else 0)


@pytest.mark.parametrize('stamp_ns, expected', [(100_000_000_000, 0.2),
                                              (101_000_000_000, -0.8), (0, None)])
def test_age_preserves_clock_errors_and_marks_missing_stamp(stamp_ns, expected):
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_200_000_000)))
    stamp = SimpleNamespace(sec=stamp_ns // 1_000_000_000,
                            nanosec=stamp_ns % 1_000_000_000)
    age = dock_localizer.DockLocalizer._frame_age_s(node, stamp)
    assert age == (pytest.approx(expected) if expected is not None else None)

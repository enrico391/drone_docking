import numpy as np

from compare_response import compare_signals


def test_compare_signals_recovers_delay_and_gain():
    time = np.arange(0.0, 20.0, 0.01)
    real = np.sin(2 * np.pi * 0.4 * time) + 0.3 * np.sin(2 * np.pi * 0.9 * time)
    simulated = 1.2 * np.interp(time - 0.12, time, real, left=real[0])

    metrics = compare_signals(time, real, time, simulated, max_delay_s=0.4)

    assert abs(metrics["delay_s"] - 0.12) <= 0.02
    assert abs(metrics["gain_ratio"] - 1.2) <= 0.03
    assert metrics["correlation"] > 0.99
    assert metrics["normalized_rmse"] < 0.03

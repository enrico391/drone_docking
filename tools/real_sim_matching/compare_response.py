#!/usr/bin/env python3
"""Align and compare real and simulated movement responses."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def compare_signals(real_t, real_y, sim_t, sim_y, max_delay_s=0.5, dt=0.02):
    real_t, real_y = np.asarray(real_t), np.asarray(real_y)
    sim_t, sim_y = np.asarray(sim_t), np.asarray(sim_y)
    start, stop = max(real_t[0], sim_t[0]), min(real_t[-1], sim_t[-1])
    grid = np.arange(start, stop, dt)
    real_grid = np.interp(grid, real_t, real_y)
    sim_grid = np.interp(grid, sim_t, sim_y)
    real_zero = real_grid - np.mean(real_grid)
    sim_zero = sim_grid - np.mean(sim_grid)
    max_lag = int(max_delay_s / dt)
    correlation = np.correlate(sim_zero, real_zero, mode="full")
    lags = np.arange(-len(grid) + 1, len(grid))
    keep = np.abs(lags) <= max_lag
    lag = lags[keep][np.argmax(correlation[keep])]
    delay = lag * dt
    aligned_start = max(real_t[0], sim_t[0] - delay)
    aligned_stop = min(real_t[-1], sim_t[-1] - delay)
    aligned = np.arange(aligned_start, aligned_stop, dt)
    reference = np.interp(aligned, real_t, real_y)
    simulation = np.interp(aligned + delay, sim_t, sim_y)
    ref_zero = reference - np.mean(reference)
    sim_zero = simulation - np.mean(simulation)
    ref_std = np.std(ref_zero)
    sim_std = np.std(sim_zero)
    gain = sim_std / ref_std if ref_std else float("nan")
    corr = np.corrcoef(reference, simulation)[0, 1] if ref_std and sim_std else float("nan")
    span = np.ptp(reference)
    raw_nrmse = np.sqrt(np.mean((simulation - reference) ** 2)) / span if span else float("nan")
    fitted = np.mean(reference) + sim_zero / gain if gain else simulation
    shape_nrmse = np.sqrt(np.mean((fitted - reference) ** 2)) / span if span else float("nan")
    return {"delay_s": round(float(delay), 4), "gain_ratio": round(float(gain), 4),
            "correlation": round(float(corr), 4),
            "normalized_rmse": round(float(shape_nrmse), 4),
            "raw_normalized_rmse": round(float(raw_nrmse), 4),
            "real_peak_to_peak": round(float(span), 4),
            "sim_peak_to_peak": round(float(np.ptp(simulation)), 4)}


def _read(path):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}


def _window(data, start, stop, field):
    mask = (data["time_s"] >= start) & (data["time_s"] <= stop)
    return data["time_s"][mask], data[field][mask]


def analyze(reference_dir, sim_dir, output_dir):
    reference_dir, sim_dir, output_dir = map(Path, (reference_dir, sim_dir, output_dir))
    output_dir.mkdir(parents=True, exist_ok=True)
    real_rc = _read(reference_dir / "real_rcin.csv")
    origin = real_rc["time_s"][0]
    real_rate = _read(reference_dir / "real_rate.csv")
    real_att = _read(reference_dir / "real_att.csv")
    real_ctun = _read(reference_dir / "real_ctun.csv")
    for data in (real_rate, real_att, real_ctun):
        data["time_s"] -= origin
    sim_att = _read(sim_dir / "sim_attitude.csv")
    sim_pos = _read(sim_dir / "sim_position.csv")
    definitions = {
        "vertical": (5 - origin, 20 - origin, real_ctun, "CRt", sim_pos, "vz_m_s"),
        "roll": (65 - origin, 80 - origin, real_rate, "R", sim_att, "R"),
        "pitch": (85 - origin, 115 - origin, real_rate, "P", sim_att, "P"),
        "yaw": (120 - origin, 149.8 - origin, real_rate, "Y", sim_att, "Y"),
    }
    metrics = {}
    figure, axes = plt.subplots(4, 1, figsize=(11, 10), constrained_layout=True)
    for axis, (name, (start, stop, real, real_field, sim, sim_field)) in zip(axes, definitions.items()):
        rt, ry = _window(real, start, stop, real_field)
        st, sy = _window(sim, start, stop, sim_field)
        metrics[name] = compare_signals(rt, ry, st, sy)
        axis.plot(rt, ry, label="real", linewidth=1.1)
        axis.plot(st, sy, label="simulation", linewidth=1.0, alpha=.8)
        axis.set(title=f"{name}: real vs simulation", ylabel="m/s" if name == "vertical" else "deg/s")
        axis.grid(True, alpha=.25); axis.legend()
    axes[-1].set_xlabel("replay time (s)")
    figure.savefig(output_dir / "response_overlay.png", dpi=150)
    (output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("reference")
    parser.add_argument("simulation")
    parser.add_argument("output")
    args = parser.parse_args()
    print(json.dumps(analyze(args.reference, args.simulation, args.output), indent=2))


if __name__ == "__main__":
    main()

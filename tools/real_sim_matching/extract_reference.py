#!/usr/bin/env python3
"""Extract a compact, replayable movement window from an ArduPilot log."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from pymavlink import mavutil


FIELDS = {
    "RCIN": ["C1", "C2", "C3", "C4"],
    "RCOU": ["C1", "C2", "C3", "C4"],
    "ATT": ["DesRoll", "Roll", "DesPitch", "Pitch", "DesYaw", "Yaw"],
    "RATE": ["RDes", "R", "PDes", "P", "YDes", "Y", "ADes", "A"],
    "CTUN": ["ThO", "DAlt", "Alt", "DCRt", "CRt"],
    "POS": ["Lat", "Lng", "Alt", "RelHomeAlt"],
}


def mode_windows(events, end_s, wanted_mode):
    windows = []
    for index, (start, mode) in enumerate(events):
        stop = events[index + 1][0] if index + 1 < len(events) else end_s
        if mode == wanted_mode:
            windows.append((start, stop))
    return windows


def shift_and_clip(rows, start_s, stop_s):
    return [dict(row, time_s=round(row["time_s"] - start_s, 6))
            for row in rows if start_s <= row["time_s"] <= stop_s]


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def extract_reference(log_path, output_dir, start_s=390.0, stop_s=540.0):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    streams = {name: [] for name in FIELDS}
    modes = []
    end_s = 0.0
    log = mavutil.mavlink_connection(str(log_path))
    while True:
        msg = log.recv_match(type=[*FIELDS, "MODE"])
        if msg is None:
            break
        time_s = msg.TimeUS / 1e6
        end_s = max(end_s, time_s)
        kind = msg.get_type()
        if kind == "MODE":
            modes.append((time_s, int(msg.ModeNum)))
        elif start_s <= time_s <= stop_s:
            row = {"time_s": round(time_s - start_s, 6)}
            row.update({field: getattr(msg, field) for field in FIELDS[kind]})
            streams[kind].append(row)
    for name, rows in streams.items():
        _write_csv(output_dir / f"real_{name.lower()}.csv", rows)
    manifest = {
        "source_log": str(Path(log_path).resolve()),
        "source_sha256": _sha256(log_path),
        "window_boot_s": [start_s, stop_s],
        "window_duration_s": stop_s - start_s,
        "flight_mode": "AltHold",
        "althold_windows_boot_s": mode_windows(modes, end_s, 2),
        "sample_counts": {name: len(rows) for name, rows in streams.items()},
        "maneuvers_boot_s": {
            "vertical": [395.0, 410.0],
            "roll": [455.0, 470.0],
            "pitch": [475.0, 505.0],
            "yaw": [510.0, 540.0],
        },
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("output")
    parser.add_argument("--start", type=float, default=390.0)
    parser.add_argument("--stop", type=float, default=540.0)
    args = parser.parse_args()
    print(json.dumps(extract_reference(args.log, args.output, args.start, args.stop), indent=2))


if __name__ == "__main__":
    main()

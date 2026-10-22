"""Incremental CSV recorder in fit_plant.py-compatible format.

commands.csv: host_mono_s, elapsed_s, phase, vx_cmd, vy_cmd, vz_cmd, fc_boot_s
telemetry.csv: host_mono_s, fc_boot_s, x, y, z, vx, vy, vz, mode, armed, relative_alt_m, ekf_ok
events.jsonl: run events.
fc_boot_s = VehicleLocalPosition.timestamp/1e6 (FC clock = fit timebase).
"""

import csv
import json
import time
from pathlib import Path


class CsvRecorder:
    def __init__(self, out: Path, axis: str, config, extra_meta: dict | None = None):
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=False)
        self.axis = axis
        headers = {
            "commands": ["host_mono_s", "elapsed_s", "phase", "vx_cmd", "vy_cmd", "vz_cmd", "fc_boot_s"],
            "telemetry": ["host_mono_s", "fc_boot_s", "x", "y", "z", "vx", "vy", "vz",
                          "mode", "armed", "relative_alt_m", "ekf_ok"],
        }
        self.files, self.writers = {}, {}
        for name, header in headers.items():
            f = (self.out / (name + ".csv")).open("w", newline="", buffering=1)
            self.files[name] = f
            self.writers[name] = csv.writer(f)
            self.writers[name].writerow(header)
        self.events = (self.out / "events.jsonl").open("w", buffering=1)
        try:
            from dataclasses import asdict
            cfg = asdict(config)
        except Exception:
            cfg = {"repr": repr(config)}
        self.meta = {
            "format_version": 1, "axis": axis, "frame": "LOCAL_NED",
            "transport": "px4_offboard_dds",
            "config": cfg,
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "start_host_mono_s": None, "outcome": "waiting",
        }
        if extra_meta:
            self.meta.update(extra_meta)
        self.save_meta()

    def save_meta(self):
        temp = self.out / "metadata.tmp"
        temp.write_text(json.dumps(self.meta, indent=2) + "\n")
        temp.replace(self.out / "metadata.json")

    def event(self, now: float, name: str, **fields):
        self.events.write(json.dumps({"host_mono_s": now, "event": name, **fields}) + "\n")

    def command(self, now: float, elapsed: float, phase: str, vx: float, vy: float, vz: float, fc_s: float):
        self.writers["commands"].writerow([now, elapsed, phase, vx, vy, vz, fc_s])

    def telemetry(self, now: float, fc_s: float, x: float, y: float, z: float,
                  vx: float, vy: float, vz: float, mode: str, armed: bool,
                  relative_alt_m: float, ekf_ok: bool):
        self.writers["telemetry"].writerow(
            [now, fc_s, x, y, z, vx, vy, vz, mode, int(armed), relative_alt_m, int(ekf_ok)])

    def close(self, outcome: str):
        self.meta["outcome"] = outcome
        self.save_meta()
        for f in self.files.values():
            f.close()
        self.events.close()

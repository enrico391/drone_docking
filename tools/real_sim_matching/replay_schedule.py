"""Load RC input schedules used by the real/simulation replay."""
import csv
import re


_PRIMARY_RC_PARAMETER = re.compile(r"RC[1-4]_(?:MIN|MAX|TRIM|DZ|REVERSED)$")


def load_rc_calibration(path):
    """Return the real transmitter scaling needed for raw-PWM replay."""
    calibration = {}
    with open(path) as stream:
        for line in stream:
            fields = line.split("#", 1)[0].replace(",", " ").split()
            if len(fields) >= 2 and _PRIMARY_RC_PARAMETER.fullmatch(fields[0]):
                calibration[fields[0]] = float(fields[1])
    return calibration


def mavlink_source_system(replaying_rc):
    return 255 if replaying_rc else 250


def flight_mode_id(name):
    return {"althold": 2, "loiter": 5}[name]


def takeoff_altitude(replaying, override):
    if override is not None:
        return float(override)
    return 12.0 if replaying else 2.0


def load_schedule(path):
    samples = []
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            samples.append((float(row["time_s"]), tuple(int(row[f"C{i}"]) for i in range(1, 5))))
    if not samples:
        raise ValueError("RC schedule is empty")
    if any(b[0] <= a[0] for a, b in zip(samples, samples[1:])):
        raise ValueError("RC schedule times must be strictly increasing")
    origin = samples[0][0]
    return [(round(time_s - origin, 6), channels) for time_s, channels in samples]

"""Allocate ordered log names without depending on wall time."""

from pathlib import Path
import re
from typing import TextIO, Tuple


def open_numbered_log(directory: Path, prefix: str, suffix: str) -> Tuple[int, TextIO]:
    """Continue after the highest saved number, reserving each file exclusively."""
    directory.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(re.escape(prefix) + r"_([0-9]+)" + re.escape(suffix))
    highest = 0
    for path in directory.glob(f"{prefix}_*{suffix}"):
        match = pattern.fullmatch(path.name)
        if match is not None:
            highest = max(highest, int(match.group(1)))
    number = highest + 1
    while True:
        try:
            return number, (directory / f"{prefix}_{number:06d}{suffix}").open(
                "x", encoding="utf-8")
        except FileExistsError:
            number += 1

"""Optional per-update JSONL trace of the final PID approach."""

import json
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, TextIO


class TouchdownTrace:
    """One independently flushed trace per approach; recording must never affect flight."""

    def __init__(self, *, enabled: bool, directory: str,
                 warn: Callable[[str], None]) -> None:
        self._enabled = enabled
        self._directory = directory
        self._warn = warn
        self._file: Optional[TextIO] = None

    @property
    def active(self) -> bool:
        return self._file is not None

    def start(self) -> None:
        if not self._enabled or self.active:
            return
        try:
            directory = Path(self._directory).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
            for index in range(1000):
                try:
                    self._file = (directory / f"touchdown_{stamp}_{index:03d}.jsonl").open(
                        "x", encoding="utf-8")
                    return
                except FileExistsError:
                    continue
            raise RuntimeError("could not allocate a unique touchdown trace name")
        except Exception as error:
            self._safe_warn(f"Could not create touchdown trace: {error}")
            self._enabled = False

    def write(self, data: dict) -> None:
        if not self.active:
            return
        try:
            self._file.write(json.dumps(data, allow_nan=False, separators=(",", ":")) + "\n")
            self._file.flush()
        except (OSError, ValueError, TypeError) as error:
            self._safe_warn(f"Could not write touchdown trace: {error}")
            self._enabled = False
            self.close()

    def finish(self, outcome: str, reason: str) -> None:
        if self.active:
            self.write({"event": "END", "outcome": outcome, "reason": reason,
                        "wall_time": datetime.now().astimezone().isoformat()})
            self.close()

    def close(self) -> None:
        if self._file is None:
            return
        try:
            self._file.close()
        finally:
            self._file = None

    def disable(self) -> None:
        """Stop attempting disk writes after an unexpected recording failure."""
        self._enabled = False
        self.close()

    def _safe_warn(self, message: str) -> None:
        try:
            self._warn(message)
        except Exception:
            pass
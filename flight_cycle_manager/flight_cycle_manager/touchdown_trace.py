"""Optional per-update JSONL trace of the complete PID docking procedure."""

import json
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, TextIO

from .numbered_logs import open_numbered_log


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

    @property
    def trace_id(self) -> Optional[str]:
        return Path(self._file.name).stem if self._file is not None else None

    def start(self) -> None:
        if not self._enabled or self.active:
            return
        try:
            directory = Path(self._directory).expanduser()
            _, self._file = open_numbered_log(directory, "touchdown", ".jsonl")
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
        trace_file = self._file
        self._file = None
        try:
            trace_file.close()
        except Exception as error:
            self._safe_warn(f"Could not close touchdown trace: {error}")

    def disable(self) -> None:
        """Stop attempting disk writes after an unexpected recording failure."""
        self._enabled = False
        self.close()

    def _safe_warn(self, message: str) -> None:
        try:
            self._warn(message)
        except Exception:
            pass

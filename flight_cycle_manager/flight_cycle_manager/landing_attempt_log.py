"""Small, failure-tolerant text logger for individual landing attempts."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, TextIO


class LandingAttemptLog:
    """Write one flushed text file per landing approach."""

    def __init__(
        self,
        *,
        enabled: bool,
        directory: str,
        warn: Callable[[str], None],
    ) -> None:
        self._enabled = enabled
        self._directory = directory
        self._warn = warn
        self._file: Optional[TextIO] = None
        self._attempt_number = 0

    @property
    def active(self) -> bool:
        return self._file is not None

    def start(self, state: str, reason: str) -> None:
        if not self._enabled or self.active:
            return
        self._attempt_number += 1
        try:
            directory = Path(self._directory).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
            path = directory / (
                f"landing_attempt_{stamp}_{self._attempt_number:03d}.txt"
            )
            collision = 1
            while True:
                try:
                    self._file = path.open("x", encoding="utf-8")
                    break
                except FileExistsError:
                    path = directory / (
                        f"landing_attempt_{stamp}_{self._attempt_number:03d}"
                        f"_{collision}.txt"
                    )
                    collision += 1
            self._file.write(f"Landing attempt {self._attempt_number}\n")
            self.write(f"ATTEMPT_START state={state} reason={reason}")
        except Exception as error:
            self._file = None
            self._safe_warn(f"Could not create landing attempt log: {error}")

    def write(self, message: str) -> None:
        if self._file is None:
            return
        try:
            timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
            self._file.write(f"{timestamp} {message}\n")
            self._file.flush()
        except Exception as error:
            self._safe_warn(f"Could not write landing attempt log: {error}")
            self.close()

    def finish(self, outcome: str, state: str, reason: str) -> None:
        if self._file is None:
            return
        self.write(f"ATTEMPT_END outcome={outcome} state={state} reason={reason}")
        self.close()

    def close(self) -> None:
        if self._file is None:
            return
        try:
            self._file.close()
        except Exception as error:
            self._safe_warn(f"Could not close landing attempt log: {error}")
        finally:
            self._file = None

    def _safe_warn(self, message: str) -> None:
        try:
            self._warn(message)
        except Exception:
            pass

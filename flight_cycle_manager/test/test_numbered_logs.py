"""Log identities must follow saved files rather than an unreliable wall clock."""

from pathlib import Path

import pytest

from flight_cycle_manager.landing_attempt_log import LandingAttemptLog
from flight_cycle_manager.touchdown_trace import TouchdownTrace


@pytest.fixture(params=["touchdown", "landing_attempt"])
def logger_factory(request, tmp_path):
    prefix = request.param
    suffix = ".jsonl" if prefix == "touchdown" else ".txt"

    def create():
        logger_type = TouchdownTrace if prefix == "touchdown" else LandingAttemptLog
        return logger_type(enabled=True, directory=str(tmp_path), warn=lambda message: None)

    def start(logger):
        if prefix == "touchdown":
            logger.start()
        else:
            logger.start("GUIDED_TAKEOVER", "LAND trigger")

    return create, start, prefix, suffix


def test_log_number_continues_after_restart_and_ignores_legacy_dates(tmp_path, logger_factory):
    """Catch clock-based names, number reuse or old timestamps treated as counters."""
    create, start, prefix, suffix = logger_factory
    legacy = tmp_path / f"{prefix}_19700101_000000_000000_000{suffix}"
    legacy.write_text("old log")
    first = create()
    start(first)
    first.close()
    assert (tmp_path / f"{prefix}_000001{suffix}").exists()
    (tmp_path / f"{prefix}_000007{suffix}").write_text("saved seventh log")
    restarted = create()
    start(restarted)
    restarted.close()
    assert (tmp_path / f"{prefix}_000008{suffix}").exists()
    start(restarted)
    restarted.close()
    assert (tmp_path / f"{prefix}_000009{suffix}").exists()
    assert legacy.read_text() == "old log"
    assert (tmp_path / f"{prefix}_000007{suffix}").read_text() == "saved seventh log"


def test_numbered_log_allocation_preserves_a_concurrent_writer(tmp_path, monkeypatch, logger_factory):
    """Catch overwrites or allocation failure if another writer reserves the next number."""
    create, start, prefix, suffix = logger_factory
    open_file = Path.open
    candidate = tmp_path / f"{prefix}_000001{suffix}"
    raced = False

    def competing_open(path, mode="r", *args, **kwargs):
        nonlocal raced
        if path == candidate and mode == "x" and not raced:
            raced = True
            with open_file(path, "x", encoding="utf-8") as saved:
                saved.write("other writer")
        return open_file(path, mode, *args, **kwargs)

    monkeypatch.setattr(Path, "open", competing_open)
    logger = create()
    start(logger)
    logger.close()
    assert raced
    assert candidate.read_text() == "other writer"
    assert (tmp_path / f"{prefix}_000002{suffix}").exists()


def test_attempt_header_uses_the_persistent_file_number(tmp_path):
    """Catch a restarted logger describing attempt 1 inside attempt file 13."""
    (tmp_path / "landing_attempt_000012.txt").touch()
    logger = LandingAttemptLog(enabled=True, directory=str(tmp_path), warn=lambda message: None)
    logger.start("GUIDED_TAKEOVER", "LAND trigger")
    logger.close()
    assert (tmp_path / "landing_attempt_000013.txt").read_text().startswith("Landing attempt 13\n")

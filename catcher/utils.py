"""로깅 + 타이밍."""

from __future__ import annotations

import logging
import time

_LOG_PATH = "run.log"

_logger = logging.getLogger("catcher")
_logger.setLevel(logging.INFO)
if not _logger.handlers:
    _fmt = logging.Formatter("%(asctime)s %(message)s")

    _file = logging.FileHandler(_LOG_PATH, mode="a", encoding="utf-8")
    _file.setFormatter(_fmt)
    _logger.addHandler(_file)

    _console = logging.StreamHandler()
    _console.setFormatter(_fmt)
    _logger.addHandler(_console)


def log(message: str) -> None:
    _logger.info(message)


def now() -> float:
    """단조 시계(s). 벽시계가 아니라 경과시간 측정용."""
    return time.monotonic()

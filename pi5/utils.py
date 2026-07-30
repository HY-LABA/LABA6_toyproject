"""단일 파일(run.log) append 로그 + 타이밍."""

from __future__ import annotations

import logging
import time

_LOG_PATH = "run.log"

_logger = logging.getLogger("catch_robot")
_logger.setLevel(logging.INFO)
if not _logger.handlers:
    _fmt = logging.Formatter("%(asctime)s %(message)s")

    _file_handler = logging.FileHandler(_LOG_PATH, mode="a", encoding="utf-8")
    _file_handler.setFormatter(_fmt)
    _logger.addHandler(_file_handler)

    _console_handler = logging.StreamHandler()
    _console_handler.setFormatter(_fmt)
    _logger.addHandler(_console_handler)


def log(message: str) -> None:
    _logger.info(message)


def log_detection(class_name: str, confidence: float, bbox: tuple[float, float, float, float]) -> None:
    cx, cy, w, h = bbox
    log(f"detect class={class_name} conf={confidence:.2f} bbox_center=({cx:.1f},{cy:.1f}) bbox_size=({w:.1f}x{h:.1f})")


def log_position(x: float, y: float, z: float) -> None:
    log(f"position x={x:.3f} y={y:.3f} z={z:.3f}")


def timestamp() -> float:
    return time.monotonic()

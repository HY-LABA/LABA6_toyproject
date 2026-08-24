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


_detection_console_quiet = False


def set_detection_console_quiet(quiet: bool) -> None:
    """검출마다 찍히는 log_detection()의 콘솔 출력만 끈다 (파일 run.log에는 계속 남음).

    60fps로 계속 뭔가 잡히면 화면이 그 로그로 다 덮여서 정작 중요한 착지 예측
    출력을 못 보게 된다 (live_predict.py가 씀). log()/log_cycle() 같은 다른
    로그(트랙 시작/종료 등)는 그대로 콘솔에 찍힌다 — 이건 검출 로그만 조용히 한다.
    """
    global _detection_console_quiet
    _detection_console_quiet = quiet


def log_detection(confidence: float, bbox, uv) -> None:
    """bbox는 왜곡 보정 전 원본, uv는 보정 후 중심.

    bbox 크기는 이제 궤적 추정에 안 쓰지만 로그에는 남긴다 — 검출이 흔들릴 때
    "물체가 실제로 작아진 건지 검출이 튄 건지" 구분하는 데 쓸 수 있다.
    """
    cx, cy, w, h = bbox
    msg = (f"detect conf={confidence:.2f} center=({cx:.1f},{cy:.1f})->"
           f"({uv[0]:.1f},{uv[1]:.1f}) size=({w:.1f}x{h:.1f})")
    if _detection_console_quiet:
        _file_handler.emit(_logger.makeRecord(
            _logger.name, logging.INFO, __file__, 0, msg, None, None))
    else:
        log(msg)


def log_cycle(fit, landing_xy, time_remaining: float, odom_xy, cmd) -> None:
    """한 사이클의 결정 근거를 한 줄로.

    잔차(resid)와 관측 수(n)를 같이 찍는 게 중요하다. 예측이 이상할 때
    "궤적이 안 맞는 건지(resid 큼) 관측이 부족한 건지(n 작음)"를 로그만 보고
    구분할 수 있어야 한다.
    """
    log(f"fit n={fit.n} resid={fit.residual_px:.2f}px z={fit.p0[2]:.2f}m "
        f"| landing=({landing_xy[0]:+.3f},{landing_xy[1]:+.3f}) t_rem={time_remaining:.3f}s "
        f"| odom=({odom_xy[0]:+.3f},{odom_xy[1]:+.3f}) "
        f"| cmd=({cmd.target_vx:+.2f},{cmd.target_vy:+.2f}) {cmd.speed:.2f}m/s")


def timestamp() -> float:
    return time.monotonic()

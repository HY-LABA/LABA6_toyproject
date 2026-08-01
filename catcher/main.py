"""SEARCH / TRACK 상태 기계 — 4단계 파이프라인을 매 프레임 한 번씩 돈다.

    ① 인식      vision.detect_full / detect_roi
    ② 좌표변환  frames  (estimator.update 안에서)
    ③ 궤적추정  estimator
    ④ 목표지점  trajectory -> control -> link

별도의 "확정 단계"도 "재보정 루프"도 없다. 한 번의 반복이 관측·추정·예측·명령을 모두 한다.
(../docs/pi5-algorithm.md 4장)

vision/link은 의존성 주입이다. 시뮬레이터의 가짜 객체를 넘기면 하드웨어 없이 그대로 돈다.
"""

from __future__ import annotations

import config
import control
import trajectory
import utils
import vision
from datatypes import State
from estimator import ProjectileEstimator
from link import Link
from vision import Camera, Detector


def _termination_reason(est: ProjectileEstimator, now: float, cycle_start: float) -> str | None:
    """사이클을 끝내야 하는 이유. 없으면 None."""
    if est.initialized and est.z <= config.Z_CATCH_M:
        return "landed"
    if est.stale(now):
        return "lost"
    if now - cycle_start > config.CYCLE_TIMEOUT_S:
        return "timeout"
    return None


def run(cam: Camera, detector: Detector, lnk: Link, max_frames: int | None = None) -> None:
    """메인 루프. max_frames는 테스트용 (None이면 무한)."""
    est = ProjectileEstimator()
    state = State.SEARCH
    cycle_start = utils.now()
    frames_done = 0

    while max_frames is None or frames_done < max_frames:
        frames_done += 1

        frame, t_cap = cam.capture()
        pose = lnk.latest_odometry()
        if pose is None:
            continue                                   # 오도메트리 없이는 world 변환 불가

        # ── ① 인식 ────────────────────────────────────────────────────
        if state is State.SEARCH:
            det = vision.detect_full(detector, frame)
            sigma_px = config.SIGMA_BBOX_PX_SEARCH
        else:
            uv = est.predict_image_position(t_cap, pose)
            det = vision.detect_roi(detector, frame, uv)
            sigma_px = config.SIGMA_BBOX_PX
            if det is None:
                state = State.SEARCH                   # 재획득 — 칼만 상태는 유지한다
                continue

        if det is None:
            if est.stale(t_cap):
                _end_cycle(lnk, est, "lost")
                state, cycle_start = State.SEARCH, utils.now()
            continue

        # ── ②③ 좌표변환 + 궤적추정 ─────────────────────────────────────
        if not est.initialized:
            cycle_start = utils.now()
        est.update(det, pose, t_cap, sigma_px)
        state = State.TRACK

        # ── ④ 목표지점 → 명령 ──────────────────────────────────────────
        if est.confident:
            landing = trajectory.predict_landing(
                est.position, est.velocity,
                config.gravity_for(est.class_name), config.Z_CATCH_M,
            )
            if landing.valid:
                if not control.is_reachable(landing, pose):
                    utils.log(f"unreachable: landing=({landing.x:.2f},{landing.y:.2f}) "
                              f"t={landing.t_land:.3f}")
                lnk.send(control.to_velocity_command(landing, pose, est.confidence))

        # ── 종료 판정 ──────────────────────────────────────────────────
        reason = _termination_reason(est, t_cap, cycle_start)
        if reason is not None:
            _end_cycle(lnk, est, reason)
            state, cycle_start = State.SEARCH, utils.now()


def _end_cycle(lnk: Link, est: ProjectileEstimator, reason: str) -> None:
    """정지 명령을 보내고 추정기를 초기화한다. 프로그램은 끝내지 않는다 — 다음 물체를 기다린다."""
    lnk.send(control.stop_command())
    utils.log(f"cycle end: {reason}")
    est.reset()


if __name__ == "__main__":
    raise SystemExit(
        "vision.Picamera2Camera / vision.HailoDetector / link.SerialLink 구현이 필요합니다.\n"
        "구현체를 주입해 run(cam, detector, lnk)을 호출하세요."
    )

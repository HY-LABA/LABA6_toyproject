"""피코 없이 카메라+YOLO+궤적만 라이브로 돌려서 터미널에 착지 x,y를 찍는다.

피코가 아직 준비 안 된 동안 카메라·YOLO·궤적 피팅만 먼저 확인하려고 만든
임시 진입점이다. `main.py`와 프레임 루프 로직은 같은데 `communication.py`
(피코 시리얼 통신)·`control.py`(속도 명령 계산) 부분만 뺐다 — 그래서 로봇 구동
없이 "지금 던지면 어디로 떨어질지"만 콘솔에 확인할 수 있다.

피코 연결되면 이 파일은 그만 쓰고 `main.py`로 넘어가면 된다 (로직 중복 남겨두는
용도가 아니라 딱 이 과도기용).

    python live_predict.py
"""

from __future__ import annotations

import config
import trajectory
import utils


def run() -> None:
    cam = vision_open()
    tracker = trajectory.Tracker()
    last_seen_t: float | None = None
    n_predictions = 0

    utils.log("live predict 시작 (피코 없음 — 터미널 출력만, main.py 아님)")
    try:
        while True:
            det = vision.observe(cam)

            # ── 트랙 유지/리셋 판단 (main.py와 동일 로직) ───────────────
            if det is None:
                if last_seen_t is not None and tracker.times:
                    gap = utils.timestamp() - last_seen_t
                    if gap > config.TRACK_MAX_GAP_S:
                        utils.log(f"트랙 유실(gap={gap:.3f}s) — 리셋")
                        tracker.reset()
                        last_seen_t = None
                continue

            last_seen_t = det.t

            # ── 관측 누적 + 재피팅 ───────────────────────────────────────
            fit = tracker.add(det.u, det.v, det.t)
            if fit is None:
                continue

            landing = tracker.landing()
            if landing is None:
                utils.log(f"착지 해 없음: {fit}")
                continue

            landing_x, landing_y, time_remaining = landing
            n_predictions += 1
            print(f"[resid={fit.residual_px:5.2f}px n={fit.n:2d}] "
                  f"착지 예측 (x={landing_x:+.3f}, y={landing_y:+.3f})  "
                  f"남은시간={time_remaining:.3f}s")

            if time_remaining <= 0.0:
                utils.log(f"착지 시점 통과 — 트랙 종료 (이번 던지기 예측 {n_predictions}회)")
                tracker.reset()
                last_seen_t = None
                n_predictions = 0
    except KeyboardInterrupt:
        utils.log("중단됨 (Ctrl+C)")
    finally:
        cam.close()
        utils.log("live predict 종료")


def vision_open():
    """vision을 늦게 import한다 — Picamera2/HailoRT가 없는 PC에서도 이 파일을
    읽고 문법 검사할 수 있게 하기 위함 (main.py와 동일한 이유)."""
    global vision
    import vision as _vision

    vision = _vision
    return vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)


if __name__ == "__main__":
    run()

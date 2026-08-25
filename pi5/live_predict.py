"""피코 없이 카메라+YOLO+궤적만 라이브로 돌려서 터미널에 착지 x,y를 찍는다.

피코가 아직 준비 안 된 동안 카메라·YOLO·궤적 피팅만 먼저 확인하려고 만든
임시 진입점이다. `main.py`와 프레임 루프 로직은 같은데 `communication.py`
(피코 시리얼 통신)·`control.py`(속도 명령 계산) 부분만 뺐다 — 그래서 로봇 구동
없이 "지금 던지면 어디로 떨어질지"만 콘솔에 확인할 수 있다.

피코 연결되면 이 파일은 그만 쓰고 `main.py`로 넘어가면 된다 (로직 중복 남겨두는
용도가 아니라 딱 이 과도기용).

    python live_predict.py
    python live_predict.py --record

실행할 때마다 live_sessions/session_YYYYMMDD_HHMMSS/ 폴더가 자동으로 생기고,
검출될 때마다 debug_frames/에 bbox+confidence 그린 사진이 항상 저장된다 (SSH로
화면을 직접 띄우기 번거로우니(X11 문제, prep/TROUBLESHOOTING.md 참고) 나중에
scp로 받아 눈으로 확인하는 용도).

--record를 주면 같은 세션 폴더에 recording.mp4로 시작부터 Ctrl+C까지 전체
영상도 남는다. **기본은 꺼져 있다** — 매 프레임 cv2.VideoWriter로 인코딩하는
게 라즈베리파이5 CPU에 부담이라(소프트웨어 인코딩, 하드웨어 가속 없음) 검출
루프가 느려지고 트랙 유실/사진 누락이 늘어나는 게 확인됐다. 정말 영상이
필요할 때만 켤 것.

⚠ recording.mp4는 Picamera2 하드웨어 인코더가 아니라 **검출에 쓴 프레임을 그대로**
cv2.VideoWriter로 받아쓴다 — 인코더를 따로 붙이면(Picamera2.start_recording) 같은
스트림에서 capture_array()로 프레임을 읽는 쪽이 막혀버려서 검출/궤적 예측이 통째로
멈추는 문제가 있었다(그래서 이 방식을 씀). 대신 "라이브가 실제로 본 그 프레임"이
그대로 영상이 되므로 나중에 debug_hef_on_video.py로 재현할 때 정확하다 — 다만
그 대가가 위의 성능 저하다.

검출될 때마다 confidence/좌표는 콘솔에도 실시간으로 찍힌다 (vision.detect가 부르는
utils.log_detection). 다만 사진이 항상 저장되므로 그 60fps짜리 텍스트 로그는
꺼두고(run.log에는 남음), 착지 예측 결과만 콘솔에 보이게 한다.
"""

from __future__ import annotations

import argparse
import pathlib
from datetime import datetime

import config
import trajectory
import utils


def _save_debug_frame(out_dir: pathlib.Path, frame, det, idx: int) -> None:
    """검출 프레임에 bbox 그려서 저장 (눈으로 확인용)."""
    import cv2

    cx, cy, w, h = det.bbox   # 원본 픽셀, 왜곡보정 전 — 실제 찍힌 화면과 맞는 값
    x1, y1 = int(cx - w / 2), int(cy - h / 2)
    x2, y2 = int(cx + w / 2), int(cy + h / 2)
    img = frame.copy()
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(img, f"conf={det.confidence:.2f}", (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    out_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_dir / f"{idx:05d}_t{det.t:.3f}.jpg"), img)


def run(record: bool) -> None:
    cam = vision_open()
    tracker = trajectory.Tracker()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = pathlib.Path("live_sessions") / f"session_{stamp}"
    debug_dir = session_dir / "debug_frames"
    debug_dir.mkdir(parents=True, exist_ok=True)
    video_path = session_dir / "recording.mp4"

    # 사진이 항상 저장되므로 콘솔 검출 로그(60fps로 계속 찍혀서 화면 도배함)는
    # 끈다 — 착지 예측 출력이 묻히지 않게. run.log 파일에는 계속 남는다.
    utils.set_detection_console_quiet(True)

    writer = None  # --record면 첫 프레임이 와야 실제 크기를 알 수 있어 그때 연다
    last_seen_t: float | None = None
    n_predictions = 0
    n_saved = 0

    utils.log(f"live predict 시작 (피코 없음 — 터미널 출력만, main.py 아님)  "
              f"세션: {session_dir}")
    try:
        while True:
            frame, t = cam.capture()

            if record:
                if writer is None:
                    import cv2
                    h, w = frame.shape[:2]
                    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"),
                                              config.CAMERA_FPS, (w, h))
                    utils.log(f"영상을 {video_path}에 녹화한다 (Ctrl+C까지 계속) — "
                              f"검출 루프가 느려질 수 있다")
                writer.write(frame)

            det = vision.detect(frame, t)

            # ── 트랙 유지/리셋 판단 (main.py와 동일 로직) ───────────────
            if det is None:
                if last_seen_t is not None and tracker.times:
                    gap = utils.timestamp() - last_seen_t
                    if gap > config.TRACK_MAX_GAP_S:
                        utils.log(f"트랙 유실(gap={gap:.3f}s) — 리셋")
                        tracker.reset()
                        last_seen_t = None
                continue

            _save_debug_frame(debug_dir, frame, det, n_saved)
            n_saved += 1

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
        if writer is not None:
            writer.release()
        cam.close()
        utils.log(f"live predict 종료 (세션: {session_dir}, 저장된 사진 {n_saved}장)")


def vision_open():
    """vision을 늦게 import한다 — Picamera2/HailoRT가 없는 PC에서도 이 파일을
    읽고 문법 검사할 수 있게 하기 위함 (main.py와 동일한 이유)."""
    global vision
    import vision as _vision

    vision = _vision
    return vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--record", action="store_true",
                     help="세션 폴더에 recording.mp4로 전체 영상도 녹화한다. 기본 꺼짐 — "
                          "매 프레임 소프트웨어 인코딩이 라즈베리파이5 CPU에 부담이라 "
                          "검출 루프가 느려질 수 있다. 영상이 꼭 필요할 때만 켤 것.")
    args = ap.parse_args()
    run(record=args.record)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

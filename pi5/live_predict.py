"""피코 없이 카메라+YOLO+궤적만 라이브로 돌려서 터미널에 착지 x,y를 찍는다.

피코가 아직 준비 안 된 동안 카메라·YOLO·궤적 피팅만 먼저 확인하려고 만든
임시 진입점이다. `main.py`와 프레임 루프 로직은 같은데 `communication.py`
(피코 시리얼 통신)·`control.py`(속도 명령 계산) 부분만 뺐다 — 그래서 로봇 구동
없이 "지금 던지면 어디로 떨어질지"만 콘솔에 확인할 수 있다.

피코 연결되면 이 파일은 그만 쓰고 `main.py`로 넘어가면 된다 (로직 중복 남겨두는
용도가 아니라 딱 이 과도기용).

    python live_predict.py
    python live_predict.py --save-debug
    python live_predict.py --no-record

실행할 때마다 live_sessions/session_YYYYMMDD_HHMMSS/ 폴더가 하나 생기고, 그 안에
전체 실행 구간을 담은 recording.mp4가 기본으로 남는다(--no-record로 끌 수 있음).
--save-debug를 같이 주면 검출될 때마다 bbox 그린 사진도 같은 세션 폴더 안
debug_frames/에 쌓인다.

검출될 때마다 confidence/좌표는 콘솔에 실시간으로 찍힌다 (vision.detect가 부르는
utils.log_detection). 그것만으론 "진짜 쓰레기를 본 건지 오탐인지" 눈으로 확인이
안 되는데, SSH로 화면을 직접 띄우기는 번거로우니(X11 문제, prep/TROUBLESHOOTING.md
참고) 대신 --save-debug로 bbox 그린 사진을 저장해서 나중에 scp로 받아 눈으로
확인하는 방식을 쓴다. recording.mp4는 그와 별개로 매 프레임 라이브가 실제로 본
화면 자체를 남긴다 — live_predict.py와 같은 카메라 스트림 설정(단일 스트림)으로
찍히므로, capture_video.py로 따로 찍은 영상과 달리 debug_hef_on_video.py에 그대로
넣어서 라이브 결과와 신뢰성 있게 비교할 수 있다.
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


def run(save_debug: bool, record: bool) -> None:
    cam = vision_open()
    tracker = trajectory.Tracker()

    # 실행할 때마다 세션 폴더 하나 — 그 안에 녹화 영상 + (있으면) 디버그 사진을
    # 같이 모아둔다. capture_video.py의 session_YYYYMMDD_HHMMSS 관례와 맞춤.
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = pathlib.Path("live_sessions") / f"session_{stamp}"
    session_dir.mkdir(parents=True, exist_ok=True)
    debug_dir = session_dir / "debug_frames" if save_debug else None
    video_path = session_dir / "recording.mp4"

    if debug_dir:
        # 사진으로 저장되니 콘솔 검출 로그(60fps로 계속 찍혀서 화면 도배함)는
        # 끈다 — 착지 예측 출력이 묻히지 않게. run.log 파일에는 계속 남는다.
        utils.set_detection_console_quiet(True)

    last_seen_t: float | None = None
    n_predictions = 0
    n_saved = 0

    utils.log(f"live predict 시작 (피코 없음 — 터미널 출력만, main.py 아님)  "
              f"세션: {session_dir}")
    if debug_dir:
        utils.log(f"검출 프레임을 {debug_dir}에 저장한다")
    if record:
        cam.start_recording(str(video_path))
        utils.log(f"영상을 {video_path}에 녹화한다 (Ctrl+C까지 계속)")

    try:
        while True:
            frame, t = cam.capture()
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

            if debug_dir:
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
        if record:
            cam.stop_recording()
        cam.close()
        utils.log(f"live predict 종료 (세션: {session_dir}" +
                  (f", 저장된 사진 {n_saved}장" if debug_dir else "") + ")")


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
    ap.add_argument("--save-debug", action="store_true",
                     help="검출될 때마다 bbox 그린 사진을 세션 폴더 안 debug_frames/에 "
                          "저장한다 (눈으로 확인용). 안 주면 저장 안 하고 콘솔 로그만 나온다.")
    ap.add_argument("--record", action=argparse.BooleanOptionalAction, default=True,
                     help="시작부터 Ctrl+C까지 전체 영상을 세션 폴더에 녹화한다 (기본 켜짐, "
                          "끄려면 --no-record). live_predict가 실제로 쓰는 것과 같은 카메라 "
                          "스트림 설정(단일 스트림)으로 찍히므로, capture_video.py로 따로 "
                          "찍은 영상과 달리 debug_hef_on_video.py로 재현/비교할 때 믿을 수 "
                          "있다.")
    args = ap.parse_args()
    run(save_debug=args.save_debug, record=args.record)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

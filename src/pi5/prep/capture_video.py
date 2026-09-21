"""물체 던지는 걸 영상으로만 녹화한다 — 검출/저장은 나중에 따로.

캡처 중엔 아무 처리도 안 하고 Picamera2 하드웨어 인코더로 영상만 찍는다 —
매 프레임 검출+저장을 캡처 루프 안에서 동기로 하면 그 처리 시간만큼 프레임을
놓치는데, 이 방식은 그런 병목이 구조적으로 없다. 라벨링은 녹화가 끝난 뒤
`extract_from_video.py`가 오프라인으로 한다.

    python capture_video.py --label trash --continuous 60   # 60초 켜두고 계속 던지기
    python capture_video.py --label trash                    # Enter로 한 번씩 녹화

continuous 모드는 재시작 없이 여러 번 던질 수 있어서, clip 모드보다 매 던지기마다
드는 오버헤드(카운트다운 등)가 없다 — 그냥 계속 던지면 된다.

⚠ 기본 코덱은 mjpeg다 — 파이5는 하드웨어 H.264 인코더가 없어서(Pi4까지는 있었음)
h264는 60fps를 못 따라갈 위험이 있고, 압축 아티팩트가 배경차분에 노이즈로 잡힐
수도 있다. mjpeg는 계산이 가볍고 프레임 단위 JPEG 압축이라 나중에 저장할 개별
사진과 압축 특성이 같다. 파일 용량이 더 크지만 짧은 클립이라 문제 안 될 것으로
판단해 기본으로 걸어뒀다. 용량이 아쉬우면 `--codec h264`로 바꿔도 된다(fps 자동
검증이 같이 찍히니 문제 있으면 바로 보임).

⚠ ffmpeg가 시스템에 설치돼 있어야 한다: `sudo apt install ffmpeg`

출력: capture_sessions/session_YYYYMMDD_HHMMSS/clips/*.mp4 + session.json
      (session.json에 그때 쓴 exposure/gain 등 카메라 설정이 같이 남는다 —
      extract_from_video.py는 안 쓰지만 나중에 재현/디버깅할 때 필요하다)
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time
from datetime import datetime

import numpy as np

import camera as camlib

ROOT = pathlib.Path("capture_sessions")

# 클래스가 하나다 (2026-08-10). 궤적 추정이 중력 기반으로 바뀌면서 물체의 실제
# 크기를 알 필요가 없어졌고, 그러면 종류를 구분할 이유도 함께 사라졌다.
# extract_from_video.py도 여기서 CLASSES/BlobFinder를 가져다 쓴다 — 원래
# capture_dataset.py에 있던 걸, 그 도구를 정리(2026-09-01, extract_from_video.py
# 방식이 더 잘 모아서 대체됨)하면서 이쪽으로 옮겼다.
CLASSES = ["trash"]


def beep() -> None:
    """저장될 때마다(버저 켜져 있으면) 소리로 알린다."""
    sys.stdout.write("\a")
    sys.stdout.flush()


class BlobFinder:
    """MOG2 전경 마스크에서 '물체 하나'를 찾아낸다. extract_from_video.py가 오프라인
    추출에 재사용한다 (여기 capture_video.py 자체는 무처리 녹화만 하므로 안 씀)."""

    def __init__(self, cv2, args, frame_area: int) -> None:
        self.cv2 = cv2
        self.a = args
        self.min_area = max(args.min_area, int(frame_area * 1e-5))
        self.max_area = int(frame_area * args.max_area_frac)
        # detectShadows=False: 그림자를 회색(127)으로 표시하는 기능. 천장 배경에는 불필요하고
        # 마스크만 지저분해진다.
        self.mog = cv2.createBackgroundSubtractorMOG2(
            history=args.history, varThreshold=args.var_threshold, detectShadows=False)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def __call__(self, frame, learning_rate: float):
        cv2 = self.cv2
        mask = self.mog.apply(frame, learningRate=learning_rate)
        # 열림 -> 점 노이즈 제거, 닫힘 -> 물체 내부 구멍 메우기
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = [c for c in contours if self.min_area <= cv2.contourArea(c) <= self.max_area]
        if not big:
            return mask, None, "none"
        if len(big) > 1:
            # 손 + 물체처럼 덩어리가 여럿이면 어느 쪽이 물체인지 확신할 수 없다
            return mask, None, f"multi({len(big)})"

        x, y, w, h = cv2.boundingRect(big[0])
        H, W = mask.shape[:2]
        m = self.a.edge_margin
        if x <= m or y <= m or x + w >= W - m or y + h >= H - m:
            return mask, None, "edge"          # 잘린 물체는 bbox 폭이 의미 없다
        ar = w / h if h else 0
        if not (self.a.min_aspect <= ar <= self.a.max_aspect):
            return mask, None, f"aspect({ar:.2f})"
        if cv2.contourArea(big[0]) / (w * h) < self.a.min_fill:
            return mask, None, "sparse"        # 길쭉한 노이즈 제거
        return mask, (x, y, w, h), "ok"


def _record_with_buzzer(cam, seconds: float, enabled: bool, threshold: float = 6.0,
                        check_hz: float = 15.0) -> None:
    """녹화 시간만큼 대기한다. buzzer가 켜져 있으면 저해상도 보조 스트림(lores)으로
    가볍게 움직임을 체크해서, 물체가 지나가는 동안 터미널 벨(\\a)을 울린다.

    메인 녹화(인코더가 쓰는 스트림)와는 완전히 다른 경로라 녹화 자체엔 영향이
    없다 — 매 프레임 MOG2를 돌리는 게 아니라, 320x240 흑백 프레임 하나만
    초당 몇 번 diff 떠보는 정도라 훨씬 가볍다.
    """
    if not enabled:
        time.sleep(seconds)
        return

    period = 1.0 / check_hz
    t_end = time.monotonic() + seconds
    bg = None
    was_moving = False
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        gray = cam.capture_lores().astype(np.float32)
        if bg is None:
            bg = gray
        diff = float(np.abs(gray - bg).mean())
        moving = diff > threshold
        if moving and not was_moving:
            beep()
        was_moving = moving
        bg = bg * 0.9 + gray * 0.1   # 서서히 배경 갱신 (지수이동평균)
        elapsed = time.monotonic() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


def _check_fps(path: pathlib.Path, expected_seconds: float, expected_fps: int,
               codec: str) -> None:
    """녹화가 실제로 설정 fps를 유지했는지 파일을 다시 열어 확인한다.

    인코더가 못 따라가면 그 자리에서 에러를 내는 게 아니라 그냥 프레임을 덜
    써서 조용히 fps가 낮아진다 — 재생해보기 전엔 알 방법이 없어서 여기서 대신
    확인해준다.
    """
    import cv2

    cap = cv2.VideoCapture(str(path))
    n = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    cap.release()
    expected = expected_seconds * expected_fps
    if n <= 0:
        print("  ⚠ 프레임 수를 못 읽었다 — 파일이 제대로 안 만들어졌을 수 있다.")
        return
    actual_fps = n / expected_seconds
    print(f"  {n:.0f}장 기록됨 (기대 {expected:.0f}장 근처, 실효 ~{actual_fps:.0f}fps)")
    if n < expected * 0.8:
        other = "mjpeg" if codec == "h264" else "h264"
        print(f"  ⚠ 설정({expected_fps}fps)보다 많이 낮다 — 인코더가 못 따라간 것 같다. "
              f"--codec {other}로 바꿔서 비교해볼 것.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    camlib.add_profile_arg(ap)
    ap.add_argument("--label", default=CLASSES[0], choices=CLASSES,
                    help="이번 세션에서 떨어뜨릴 물체")
    ap.add_argument("--seconds", type=float, default=2.0, help="던지기 1회당 클립 길이(초)")
    ap.add_argument("--continuous", type=float, default=None, metavar="SECS",
                    help="영상 하나로 길게 녹화 — 재시작 없이 계속 던지기")
    ap.add_argument("--bitrate", type=int, default=20_000_000,
                    help="인코더 비트레이트. 낮추면 빠른 움직임에 압축 아티팩트가 "
                         "생겨 배경차분이 노이즈로 오인할 수 있다.")
    ap.add_argument("--codec", choices=["h264", "mjpeg"], default="mjpeg",
                    help="기본 mjpeg — 파이5는 하드웨어 H.264 인코더가 없어 h264는 "
                         "60fps를 못 따라갈 위험이 있고, 압축 아티팩트가 배경차분에 "
                         "노이즈로 잡힐 수도 있다(Jay capture_pipeline에서도 같은 "
                         "우려). mjpeg는 계산이 가볍고 기존에 검증된 프레임 단위 "
                         "JPEG 저장과 압축 특성이 같다. 파일 용량이 커도 괜찮으면 "
                         "h264로 바꿔서 더 작게 받을 수 있다.")
    ap.add_argument("--buzzer", action=argparse.BooleanOptionalAction, default=True,
                    help="물체가 지나가는 동안 터미널 벨을 울린다 (저해상도 보조 "
                         "스트림으로 가볍게 체크 — 메인 녹화엔 영향 없음). "
                         "안 들리거나 끄고 싶으면 --no-buzzer.")
    ap.add_argument("--buzzer-threshold", type=float, default=6.0,
                    help="움직임 감지 민감도 (낮을수록 민감, 오탐 늘어남)")
    args = ap.parse_args()

    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                              auto_lock=args.auto_lock_exposure,
                              max_exposure_us=args.max_exposure_us, max_gain=args.max_gain,
                              want_lores=args.buzzer)
    if cam.backend != "picamera2":
        print("[에러] 영상 녹화는 Picamera2 전용이다 (webcam 프로파일로는 안 됨).")
        cam.close()
        return 1

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"session_{stamp}"
    outdir = ROOT / session
    clipdir = outdir / "clips"
    clipdir.mkdir(parents=True, exist_ok=True)

    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "camera": args.camera,
        "exposure_us": cam.spec.exposure_us,
        "gain": cam.spec.gain,
        "resolution": [cam.spec.width, cam.spec.height],
        "fps": cam.spec.fps,
        "codec": args.codec,
        "clips": [],
    }

    try:
        if args.continuous:
            name = f"{args.label}_continuous.mp4"
            print(f"\n{args.continuous:.0f}초 녹화 시작 — 자유롭게 던지세요.\n")
            cam.start_recording(str(clipdir / name), bitrate=args.bitrate, codec=args.codec)
            _record_with_buzzer(cam, args.continuous, args.buzzer, args.buzzer_threshold)
            cam.stop_recording()
            meta["clips"].append({"file": name, "label": args.label, "mode": "continuous",
                                  "seconds": args.continuous})
            print(f"저장: {name}")
            _check_fps(clipdir / name, args.continuous, cam.spec.fps, args.codec)
        else:
            print("\n[조작] Enter=녹화 시작(카운트다운 후)   q, Enter=종료\n")
            n = 0
            while True:
                cmd = input(f"[{n}개 클립] > ").strip().lower()
                if cmd in ("q", "quit", "exit"):
                    break
                n += 1
                name = f"{args.label}_{n:04d}.mp4"
                for sec in (3, 2, 1):
                    print(f"  {sec}...")
                    time.sleep(1)
                print("  던져!")
                cam.start_recording(str(clipdir / name), bitrate=args.bitrate, codec=args.codec)
                _record_with_buzzer(cam, args.seconds, args.buzzer, args.buzzer_threshold)
                cam.stop_recording()
                meta["clips"].append({"file": name, "label": args.label, "mode": "clip",
                                      "seconds": args.seconds})
                print(f"  저장: {name}")
                _check_fps(clipdir / name, args.seconds, cam.spec.fps, args.codec)
                print()
    except KeyboardInterrupt:
        print("\n중단 — 지금까지 녹화된 클립은 그대로 남아있다.")
    finally:
        (outdir / "session.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        cam.close()
        print(f"\n{len(meta['clips'])}개 클립 -> {outdir.resolve()}")
        print(f"다음: python extract_from_video.py {outdir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

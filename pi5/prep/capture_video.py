"""물체 던지는 걸 영상으로만 녹화한다 — 검출/저장은 나중에 따로.

`capture_dataset.py`는 매 프레임 MOG2 검출 + JPEG 저장을 캡처 루프 안에서 동기로
하기 때문에, 그 처리 시간만큼 프레임을 놓친다. 이 스크립트는 캡처 중엔 아무 처리도
안 하고 Picamera2 하드웨어 인코더로 영상만 찍는다 — 병목이 구조적으로 없다.
라벨링은 녹화가 끝난 뒤 `extract_from_video.py`가 오프라인으로 한다.

    python capture_video.py --label trash --continuous 60   # 60초 켜두고 계속 던지기
    python capture_video.py --label trash                    # Enter로 한 번씩 녹화

continuous 모드는 재시작 없이 여러 번 던질 수 있어서, clip 모드보다 매 던지기마다
드는 오버헤드(카운트다운 등)가 없다 — 그냥 계속 던지면 된다.

⚠ 파이5는 하드웨어 H.264 인코더가 없다. `--codec h264`(기본)가 60fps를 못 따라가면
`--codec mjpeg`로 바꿔볼 것 — 계산이 가벼워 fps 유지에 유리하고, 압축 방식도
capture_dataset.py가 쓰는 프레임 단위 JPEG와 같아 검출 노이즈 부담도 적다.
대신 파일 용량은 더 크다. 어느 쪽이 나은지는 실제로 비교해봐야 한다.

⚠ ffmpeg가 시스템에 설치돼 있어야 한다: `sudo apt install ffmpeg`

출력: capture_sessions/session_YYYYMMDD_HHMMSS/clips/*.mp4 + session.json
      (session.json에 그때 쓴 exposure/gain 등 카메라 설정이 같이 남는다 —
      extract_from_video.py는 안 쓰지만 나중에 재현/디버깅할 때 필요하다)
"""

from __future__ import annotations

import argparse
import json
import pathlib
import time
from datetime import datetime

import camera as camlib
from capture_dataset import CLASSES

ROOT = pathlib.Path("capture_sessions")


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
    ap.add_argument("--codec", choices=["h264", "mjpeg"], default="h264",
                    help="h264는 파일이 작지만 파이5엔 하드웨어 인코더가 없어 "
                         "60fps를 못 따라갈 수 있다. 그러면 mjpeg로 바꿀 것 "
                         "(계산이 가볍고 압축 특성도 기존 프레임 저장 방식과 같음).")
    args = ap.parse_args()

    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                              auto_lock=args.auto_lock_exposure,
                              max_exposure_us=args.max_exposure_us, max_gain=args.max_gain)
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
            time.sleep(args.continuous)
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
                time.sleep(args.seconds)
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

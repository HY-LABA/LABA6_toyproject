"""검출 없이 **모든 프레임**을 그대로 녹화한다. (라파이에서 실행)

    python record_raw.py --no-display                 # 기본 60초
    python record_raw.py --seconds 120 --note "복도 형광등"

capture_dataset.py 와의 차이
----------------------------
capture_dataset.py 는 **검출에 성공한 프레임만** 저장한다. 그래서 파라미터를 한 번
바꾸려면 방에 가서 물체를 다시 던져야 하고, "지금 파라미터가 버린 프레임"은 애초에
디스크에 없으니 **더 느슨한 방향으로는 튜닝 자체가 불가능**하다.

이 스크립트는 판단을 전혀 하지 않고 전부 저장한다. 한 번 녹화해두면
`tune_params.py` 가 노트북에서 파라미터 조합 수백 개를 재생해볼 수 있다.

절차
----
    1. (라파이)  python record_raw.py --seconds 120 --no-display
    2. (PC로 복사) rsync -av pi@raspberrypi:~/LABA6_toyproject/pi5/prep/raw/ ./raw/
    3. (PC)      python label_gt.py --session <세션명>      # 정답 박스 손으로
    4. (PC)      python tune_params.py --session <세션명>   # 최적 파라미터 탐색

녹화 요령
---------
- **앞의 --warmup 프레임 동안은 화면을 비운다.** 그게 배경 기준이 된다
- 물체가 없는 구간도 충분히 넣는다 — 그게 있어야 **오검출률**을 잴 수 있다
- 조명·배경·물체를 바꿔가며 여러 세션을 녹화한다

용량 주의: 1456x1088 품질 85 면 장당 약 150~250 KB. 60fps 로 60초면 3600장,
**약 700 MB** 다. --every 로 솎아 저장할 수 있다.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import statistics
import time

import camera as camlib
from frame_saver import FrameSaver

ROOT = pathlib.Path("raw")


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description="원본 프레임 전체 녹화 (파라미터 튜닝용)")
    camlib.add_profile_arg(ap)
    ap.add_argument("--seconds", type=float, default=60.0, help="녹화 길이(초)")
    ap.add_argument("--warmup", type=int, default=60,
                    help="배경 기준용. 이 프레임 수만큼은 화면을 비워둘 것")
    ap.add_argument("--every", type=int, default=1,
                    help="N프레임마다 1장만 저장(용량 절약). 궤적을 보려면 1을 쓸 것")
    ap.add_argument("--jpeg-quality", type=int, default=85)
    ap.add_argument("--note", default="", help="조명·배경 메모")
    ap.add_argument("--no-display", dest="display", action="store_false",
                    help="창을 띄우지 않는다. SSH/VNC 접속이면 반드시 붙일 것")
    args = ap.parse_args()

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"{args.camera}_{stamp}"
    out_dir = ROOT / session / "frames"
    out_dir.mkdir(parents=True, exist_ok=True)

    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                             auto_lock=args.auto_lock_exposure,
                             max_exposure_us=args.max_exposure_us, max_gain=args.max_gain)
    frame, _ = cam.read()
    H, W = frame.shape[:2]
    saver = FrameSaver(cv2, out_dir, None, args.jpeg_quality)

    print(f"\n세션 {session}   {W}x{H}   {args.seconds:.0f}초 녹화")
    print(f"[순서]  ① 카메라 고정 ② 워밍업 {args.warmup}프레임 화면 비우기 ③ 투척 반복")
    print("[조작]  " + ("Q 종료" if args.display else "Ctrl+C 종료") + "\n")

    frames: list[dict] = []
    n = 0
    fps_log: list[float] = []
    t_start = None
    fps_t0 = time.monotonic()

    try:
        while True:
            frame, t_cap = cam.read()
            if t_start is None:
                t_start = t_cap
            elapsed = t_cap - t_start
            if elapsed > args.seconds:
                break

            if n % args.every == 0:
                name = f"{len(frames):06d}"
                if saver.submit(name, frame):
                    frames.append({"name": f"{name}.jpg",
                                   "t": round(t_cap - t_start, 6),
                                   "warmup": n < args.warmup})
            n += 1

            if args.display:
                view = frame.copy()
                tag = "WARMUP - 화면 비우기" if n <= args.warmup else "REC"
                cv2.putText(view, f"{tag}  {elapsed:5.1f}s / {args.seconds:.0f}s  "
                                  f"{len(frames)}장", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 200, 255) if n <= args.warmup else (0, 255, 0), 2)
                cv2.imshow("record", view)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

            if n % 30 == 0:
                now = time.monotonic()
                fps = 30.0 / max(1e-6, now - fps_t0)
                fps_t0 = now
                fps_log.append(fps)
                print(f"  {elapsed:5.1f}s  {fps:5.1f} fps  저장 {len(frames)}장")
    except KeyboardInterrupt:
        print("\n  Ctrl+C — 중단한다.")
    finally:
        cam.close()
        saver.close()
        if args.display:
            cv2.destroyAllWindows()

    n_warm = sum(1 for f in frames if f["warmup"])
    (ROOT / session / "meta.json").write_text(json.dumps({
        "session": session, "camera": args.camera, "note": args.note,
        "image_size": [W, H], "fps_setting": cam.spec.fps,
        "exposure_us": cam.spec.exposure_us, "gain": cam.spec.gain,
        "every": args.every, "warmup_frames": n_warm,
        "frames": frames,
    }, indent=1, ensure_ascii=False), encoding="utf-8")

    print(f"\n{len(frames)}장 (워밍업 {n_warm}장 포함) -> {(ROOT / session).resolve()}")
    if saver.dropped:
        print(f"⚠ 저장 큐가 넘쳐 {saver.dropped}장을 버렸다 — --every 를 올리거나 "
              f"--jpeg-quality 를 낮출 것.")
    if fps_log:
        avg = statistics.mean(fps_log)
        print(f"실효 프레임률: 평균 {avg:.1f} fps (설정 {cam.spec.fps} fps)")
        if avg < cam.spec.fps * 0.7:
            print("⚠ 프레임을 놓치고 있다. --no-display 또는 --every 2 를 검토할 것.")
    if n_warm == 0:
        print("⚠ 워밍업 프레임이 없다 — ref 검출기가 배경 기준을 못 만든다.")

    print(f"\n다음: (PC에서) python label_gt.py --session {session}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

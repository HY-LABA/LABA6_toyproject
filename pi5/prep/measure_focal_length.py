"""렌즈 초점거리 확인 — "벽 재기" 방법 (pi5/TODO.md 1번, 렌즈 각인이 안 보일 때).

원리: 벽에 줄자를 수평으로 붙이고 카메라를 벽에서 정확히 1m 띄운 채 줄자와
평행하게 놓는다. 화면 좌우 끝에 걸리는 줄자 눈금 두 개를 읽으면 그 차이 S(m)가
"1m 거리에서 카메라가 담는 실제 폭"이고, f_px = 이미지폭_px × 거리 ÷ S.

절차:
  1. 줄자를 벽에 수평으로 테이프로 붙인다 (0점 위치는 상관없다 — 아무 데서나 시작해도 됨).
  2. 카메라를 벽에서 정확히 1.000m 띄우고, 렌즈가 벽을 똑바로(비스듬하지 않게)
     정면으로 향하게 놓는다 (비스듬하면 S가 실제보다 크게 읽혀 f_px가 작게 나온다).
     줄자가 화면 안에 넉넉히 들어오기만 하면, 화면 중앙에 안 와도 된다.
  3. 라이브 프리뷰를 보며 SPACE로 스냅샷 저장, Q로 종료.
  4. **저장된 사진을 열어서, 사진의 맨 왼쪽 끝과 맨 오른쪽 끝에 각각 줄자가
     몇 cm를 가리키는지 읽는다.** (특별한 표시선 없음 — 그냥 사진 가장자리 자체가
     카메라가 담는 화면의 경계다.) 그 두 숫자의 차이가 S(m).
  5. S를 --width-m 로 다시 실행하면 f_px를 계산해준다.

    python measure_focal_length.py --camera gs               # 1~3단계: 라이브 프리뷰+저장
    python measure_focal_length.py --camera gs --no-display  # SSH 대역폭 부족하면 (TROUBLESHOOTING.md)
    python measure_focal_length.py --width-m 0.84             # 4단계: 계산만
"""

from __future__ import annotations

import argparse
import pathlib
import time

import camera as camlib

OUT_DIR = pathlib.Path("focal_length_check")


def compute_f_px(image_width_px: int, distance_m: float, width_m: float) -> float:
    return image_width_px * distance_m / width_m


def cmd_live(args) -> int:
    import cv2

    OUT_DIR.mkdir(exist_ok=True)
    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                              auto_lock=args.auto_lock_exposure,
                              max_exposure_us=args.max_exposure_us, max_gain=args.max_gain)
    print(f"\n카메라를 벽에서 정확히 {args.distance:.2f}m 띄우고 줄자와 평행하게 놓을 것.")
    if args.no_display:
        print(f"[--no-display] {args.interval:.1f}초마다 자동 저장, Ctrl+C로 종료")
    else:
        print("[조작] SPACE 저장 / Q 종료")

    n = 0
    last_save = 0.0
    try:
        while True:
            frame, _ = cam.read()
            h, w = frame.shape[:2]
            if not args.no_display:
                view = frame.copy()
                cv2.putText(view, f"saved {n}  (read tape at LEFT/RIGHT edge of this image)",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("focal length check (사진 좌우 '끝'의 줄자 눈금을 읽을 것)", view)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    path = OUT_DIR / f"{n:02d}.jpg"
                    cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    print(f"저장: {path}")
                    n += 1
            else:
                now = time.monotonic()
                if now - last_save >= args.interval:
                    path = OUT_DIR / f"{n:02d}.jpg"
                    cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    print(f"저장: {path}")
                    n += 1
                    last_save = now
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        if not args.no_display:
            cv2.destroyAllWindows()

    print(f"\n{n}장 저장 -> {OUT_DIR}/")
    print("저장된 사진을 열어서, 사진 맨 왼쪽 끝 / 맨 오른쪽 끝의 줄자 눈금을 각각 읽고,")
    print("그 차이 S(m)를 --width-m 로 넣어 다시 실행할 것:")
    print("  python measure_focal_length.py --width-m <S>")
    return 0


def cmd_compute(args) -> int:
    f_px = compute_f_px(args.image_width_px, args.distance, args.width_m)
    print(f"\nf_px = {args.image_width_px} × {args.distance:.3f} ÷ {args.width_m:.4f} = {f_px:.1f}")
    print("\npi5/config.py에 반영할 것:")
    print(f"  CAMERA_FX = {f_px:.1f}")
    print(f"  CAMERA_FY = {f_px:.1f}  # 정사각 픽셀이라 fx와 같다")
    if not (600 <= f_px <= 5000):
        print("\n⚠ 흔한 렌즈 범위(2.8~16mm, f_px 812~4638)를 벗어난다 — "
              "거리를 1m로 정확히 뒀는지, 줄자가 광축과 평행했는지 다시 확인할 것.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    camlib.add_profile_arg(ap)
    ap.add_argument("--distance", type=float, default=1.0, help="카메라~벽 거리(m)")
    ap.add_argument("--no-display", action="store_true",
                     help="X11 프리뷰 없이 --interval초마다 자동 저장 (TROUBLESHOOTING.md 참고)")
    ap.add_argument("--interval", type=float, default=3.0, help="--no-display일 때 저장 간격(초)")
    ap.add_argument("--image-width-px", type=int, default=None,
                     help="--width-m 계산 시 이미지 폭(기본: --camera 프로파일 폭)")
    ap.add_argument("--width-m", type=float, default=None,
                     help="줄자로 실측한 화면 좌우 끝 폭(m) — 주면 캡처 없이 f_px만 계산")
    args = ap.parse_args()

    if args.width_m is not None:
        if args.image_width_px is None:
            args.image_width_px = camlib.SPECS[args.camera].width
        return cmd_compute(args)
    return cmd_live(args)


if __name__ == "__main__":
    raise SystemExit(main())

"""카메라 세팅(노출/게인) 실시간 튜닝 + 블러 확인용 도구.

로직(trajectory.py 등) 고칠 때마다 카메라 설정 바꾸겠다고 코드 수정 + 재실행을
반복하기 귀찮아서 만들었다. 슬라이더로 노출/게인을 실시간으로 바꾸면서 화면으로
바로 확인하고, 물체를 빠르게 흔들면서 블러가 어느 정도인지도 같이 본다.

    python tune_camera.py --camera gs

[조작]
  슬라이더 exposure_us / gain_x10 — 움직이면 바로 카메라에 반영된다
  SPACE — 지금 프레임을 파일로 저장 (물체를 빠르게 움직인 순간에 눌러서 블러 확인)
  R     — 슬라이더를 --camera 프로파일 초기값으로 되돌림
  Q     — 종료. 마지막 값을 콘솔에 출력한다 — 맘에 들면 그대로 camera.py SPECS에 넣으면 됨

화면 좌상단에 지금 exposure_us/gain/실측 밝기(mean)/선명도 점수(라플라시안 분산)를
띄운다. 선명도 점수는 절대치가 아니라 **같은 장면·같은 물체 위치에서 값이 왜
떨어지는지 상대 비교**로 쓸 것 — 화면이 비어있으면 원래도 낮게 나온다.

⚠ 블러 확인은 반드시 SPACE로 저장된 파일을 그대로 열어서 볼 것. 화면을 폰카메라로
다시 찍어서 보면 폰카메라 자체의 블러/모아레가 섞여 판단을 그르친다 (실제로 이전에
그렇게 확인하다가 화질 문제를 과대평가한 적이 있다).

원격 접속(SSH/VNC)이면 슬라이더 반응이 느릴 수 있다 — TROUBLESHOOTING.md 3번과
같은 원인이니, 가능하면 모니터를 직접 연결해서 쓸 것.
"""

from __future__ import annotations

import argparse
import pathlib

import camera as camlib

OUT_DIR = pathlib.Path("tune_camera_snapshots")


def _sharpness(cv2, gray) -> float:
    """라플라시안 분산 — 값이 클수록 에지가 선명하다(블러가 적다)는 신호."""
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    camlib.add_profile_arg(ap)
    ap.add_argument("--exposure-max-us", type=int, default=10000,
                    help="슬라이더 상한(µs). 더 밝게 보고 싶으면 늘릴 것")
    ap.add_argument("--gain-max", type=float, default=32.0, help="슬라이더 상한")
    args = ap.parse_args()

    OUT_DIR.mkdir(exist_ok=True)
    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                              auto_lock=args.auto_lock_exposure,
                              max_exposure_us=args.max_exposure_us, max_gain=args.max_gain)
    if cam.backend != "picamera2":
        print(f"[경고] backend={cam.backend} — 실시간 노출/게인 조절은 Picamera2 전용이다. "
              f"슬라이더를 움직여도 반영되지 않는다.")

    win = "tune_camera  (SPACE=저장  R=초기화  Q=종료)"
    cv2.namedWindow(win)
    # ⚠ Qt 백엔드는 namedWindow 직후 바로 createTrackbar를 부르면 아직 창 핸들이
    # 등록되기 전이라 "Null pointer ... icvFindTrackBarByName" 에러를 낸다. 프레임을
    # 한 번 먼저 보여주고(imshow) 이벤트 루프를 한 틱 돌려야(waitKey) 창이 확정된다.
    first_frame, _ = cam.read()
    cv2.imshow(win, first_frame)
    cv2.waitKey(1)

    init_exp = int(cam.spec.exposure_us or 1000)
    init_gain10 = int(round((cam.spec.gain or 4.0) * 10))
    cv2.createTrackbar("exposure_us", win, init_exp, args.exposure_max_us, lambda v: None)
    cv2.createTrackbar("gain_x10", win, init_gain10, int(args.gain_max * 10), lambda v: None)

    print(f"\n초기값: exposure_us={init_exp}  gain={init_gain10 / 10:.1f}")
    print("[조작] 슬라이더로 실시간 조절 / SPACE 저장 / R 초기화 / Q 종료\n")

    last_exp, last_gain = None, None

    n = 0
    try:
        while True:
            frame, _ = cam.read()
            exp = max(1, cv2.getTrackbarPos("exposure_us", win))
            gain = max(0.1, cv2.getTrackbarPos("gain_x10", win) / 10.0)
            if (exp, gain) != (last_exp, last_gain):
                cam.set_manual(exposure_us=exp, gain=gain)
                last_exp, last_gain = exp, gain

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mean = float(frame.mean())
            sharp = _sharpness(cv2, gray)

            view = frame.copy()
            cv2.putText(view, f"exposure_us={exp}  gain={gain:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(view, f"mean={mean:.1f}  sharpness={sharp:.1f}  saved={n}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow(win, view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                path = OUT_DIR / f"{n:03d}_exp{exp}_gain{gain:.1f}.jpg"
                cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                print(f"저장: {path}  (mean={mean:.1f}, sharpness={sharp:.1f})")
                n += 1
            if key == ord("r"):
                cv2.setTrackbarPos("exposure_us", win, init_exp)
                cv2.setTrackbarPos("gain_x10", win, init_gain10)
    finally:
        cam.close()
        cv2.destroyAllWindows()

    print(f"\n최종값: exposure_us={last_exp}  gain={last_gain:.1f}")
    print("맘에 들면 pi5/prep/camera.py의 SPECS[...] 에 그대로 반영할 것:")
    print(f"  exposure_us={last_exp}, gain={last_gain:.1f},")
    print(f"\n{n}장 저장 -> {OUT_DIR.resolve()}/")
    print("블러 확인은 이 파일들을 직접 열어서 볼 것 (폰카메라로 화면 재촬영 금지).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

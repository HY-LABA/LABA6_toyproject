"""카메라 세팅(노출/게인) 튜닝 도구 — GUI 창 없이 터미널로.

`cv2.imshow`/`createTrackbar`는 이 라즈베리파이의 OpenCV 4.6.0 Qt 빌드에서
"Null pointer ... icvFindTrackBarByName" 에러로 항상 죽는 게 확인됐다
(namedWindow 직후든, imshow로 창을 먼저 띄운 뒤든 동일하게 실패). 그래서 이 버전은
GUI를 아예 안 쓴다 — 터미널에서 값을 입력해 노출/게인을 바꾸고, 그때마다 사진을
한 장 찍어서 파일로 저장한다. 확인은 그 파일을 이미지 뷰어로 열어서 하면 된다
(저장 관련 기능만 쓰므로 헤드리스 SSH에서도 항상 동작한다 — GUI 창을 전혀 안 띄움).

    python tune_camera.py --camera gs

[명령] (엔터로 구분해서 입력)
  e <값>   — exposure_us 설정 (예: e 1500)
  g <값>   — gain 설정 (예: g 20)
  c        — 3초 카운트다운 후 촬영·저장 (그 사이에 물체를 빠르게 흔들어서 블러 테스트)
  s        — 카운트다운 없이 바로 촬영·저장
  q        — 종료 (마지막 값 출력)

매 촬영마다 밝기(mean)·선명도(라플라시안 분산 — 클수록 에지가 선명하다는 신호)를
같이 찍어준다. 절대치보다 **같은 장면에서 값이 왜 변하는지 상대 비교**로 볼 것.

⚠ 블러/색 확인은 반드시 저장된 파일을 직접 열어서 볼 것. 화면을 폰카메라로 다시
찍어서 보면 폰카메라 자체의 블러/모아레가 섞여 판단을 그르친다.
"""

from __future__ import annotations

import argparse
import pathlib
import time

import camera as camlib

OUT_DIR = pathlib.Path("tune_camera_snapshots")


def _sharpness(cv2, gray) -> float:
    """라플라시안 분산 — 값이 클수록 에지가 선명하다(블러가 적다)는 신호."""
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def _snap(cv2, cam, exp: int, gain: float, n: int) -> int:
    frame, _ = cam.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mean = float(frame.mean())
    sharp = _sharpness(cv2, gray)
    path = OUT_DIR / f"{n:03d}_exp{exp}_gain{gain:.1f}.jpg"
    cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(f"  저장: {path}  mean={mean:.1f}  sharpness={sharp:.1f}")
    return n + 1


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    camlib.add_profile_arg(ap)
    args = ap.parse_args()

    OUT_DIR.mkdir(exist_ok=True)
    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                              auto_lock=args.auto_lock_exposure,
                              max_exposure_us=args.max_exposure_us, max_gain=args.max_gain)
    if cam.backend != "picamera2":
        print(f"[경고] backend={cam.backend} — 실시간 노출/게인 조절은 Picamera2 전용이다. "
              f"명령을 넣어도 반영되지 않는다.")

    exp = int(cam.spec.exposure_us or 1000)
    gain = float(cam.spec.gain or 4.0)
    n = 0

    print(f"\n초기값: exposure_us={exp}  gain={gain:.1f}")
    print("[명령]  e <값>  g <값>  c(카운트다운 촬영)  s(바로 촬영)  q(종료)\n")

    try:
        while True:
            try:
                raw = input(f"[exp={exp} gain={gain:.1f}] > ").strip()
            except EOFError:
                break
            if not raw:
                continue
            head, *rest = raw.split()

            if head == "q":
                break
            elif head == "e" and rest:
                try:
                    exp = max(1, int(rest[0]))
                except ValueError:
                    print("  숫자를 넣을 것 (예: e 1500)")
                    continue
                cam.set_manual(exposure_us=exp)
            elif head == "g" and rest:
                try:
                    gain = max(0.1, float(rest[0]))
                except ValueError:
                    print("  숫자를 넣을 것 (예: g 20)")
                    continue
                cam.set_manual(gain=gain)
            elif head == "c":
                for sec in (3, 2, 1):
                    print(f"  {sec}...")
                    time.sleep(1)
                print("  촬영!")
                n = _snap(cv2, cam, exp, gain, n)
            elif head == "s":
                n = _snap(cv2, cam, exp, gain, n)
            else:
                print("  e <값> / g <값> / c / s / q 중 하나를 입력할 것")
    finally:
        cam.close()

    print(f"\n최종값: exposure_us={exp}  gain={gain:.1f}")
    print("맘에 들면 pi5/prep/camera.py의 SPECS[...] 에 반영할 것:")
    print(f"  exposure_us={exp}, gain={gain:.1f},")
    print(f"\n{n}장 저장 -> {OUT_DIR.resolve()}/")
    print("이미지 뷰어로 직접 열어서 밝기·색·블러를 확인할 것.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

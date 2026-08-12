"""카메라 캘리브레이션 — CAMERA_FX/FY/CX/CY 와 왜곡계수를 실측한다.

    python calibrate.py capture          # 체커보드 촬영
    python calibrate.py solve            # 계산 -> JSON + config.py 스니펫

**이게 `pi5/TODO.md` 의 ★ 1번 항목("렌즈 초점거리 확인")을 끝낸다.**
지금 `config.CAMERA_FX = 1739`는 "6mm 렌즈"라고 가정한 이론값이다. CS 마운트는
백포커스를 나사로 돌려 맞추는 구조라 초점을 맞추는 과정에서 실효 초점거리가
달라지고, 주점도 화면 정중앙이 아니다. **f가 10% 틀리면 깊이도 10% 틀어진다.**
`solve`가 실측 f_px에서 렌즈 mm를 역산해주므로 렌즈 각인을 못 읽어도 확정된다.

**카메라가 바뀌어도 코드는 동일하다.** 바뀌는 건 `--camera` 프로파일 하나뿐이고,
그 안의 `calib_model`이 pinhole/fisheye를 자동으로 고른다 (camera.py 표 참고).

왜 모델 구분이 중요한가:
  화각이 90°를 넘으면 핀홀(r = f·tanθ)이 발산한다. 지금 config는 6mm(대각 55°)를
  가정해 pinhole로 잡혀 있는데, 만약 실제 렌즈가 번들 2.8mm(대각 140°)라면
  fisheye로 캘리브레이션해야 하고 `trajectory.py`의 선형 해법도 그대로는 못 쓴다
  (uv를 먼저 핀홀 등가로 펴야 한다 — `pi5/fisheye.py` 참고).
  근거: ../../docs/physics.md 7.2장

체커보드: A4에 인쇄해 평평한 판에 붙인다. 기본값은 9x6 내부 코너, 25mm 격자.
**데이터 수집을 막지 않는다** — capture_dataset.py는 초점거리를 쓰지 않는다.
"""

from __future__ import annotations

import argparse
import json
import pathlib

import numpy as np

import camera as camlib

OUT_DIR = pathlib.Path("calib")
MIN_SHOTS = 12          # 이보다 적으면 신뢰도가 떨어진다
TARGET_SHOTS = 20

# IMX296 픽셀 피치(mm). f_px -> 렌즈 mm 역산에 쓴다.
# 센서가 바뀌면 이 값도 바뀐다 — IMX219는 1.12µm, IMX708은 1.4µm.
DEFAULT_PIXEL_MM = 3.45e-3

# pi5/TODO.md 의 렌즈 후보표 (IMX296 기준)
LENS_TABLE = [(2.8, 812), (4.0, 1159), (6.0, 1739), (8.0, 2319), (12.0, 3478)]


def _corners(gray, pattern, cv2):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, pts = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, pts, (11, 11), (-1, -1), crit)


def cmd_capture(args) -> int:
    import cv2

    out = OUT_DIR / args.camera
    out.mkdir(parents=True, exist_ok=True)
    pattern = (args.cols, args.rows)

    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                             auto_lock=args.auto_lock_exposure,
                             max_exposure_us=args.max_exposure_us, max_gain=args.max_gain)
    print("\n[조작]  SPACE 저장   Q 종료")
    print("[요령]  화면 **가장자리와 모서리**를 반드시 채워라. 화각이 넓을수록 중요하다.")
    print("        보드를 기울여 여러 각도로 찍어야 초점거리와 왜곡이 분리된다.")
    print("        체커보드는 정지 상태라 노출을 길게 줘도 된다 — 블러 걱정 없이 밝게 찍어라.\n")

    saved = 0
    # 화면을 3x3으로 나눠 어느 칸을 채웠는지 추적 (커버리지 부족이 가장 흔한 실패 원인)
    covered = np.zeros((3, 3), dtype=int)
    try:
        while True:
            frame, _ = cam.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            pts = _corners(gray, pattern, cv2)

            view = frame.copy()
            if pts is not None:
                cv2.drawChessboardCorners(view, pattern, pts, True)
            h, w = frame.shape[:2]
            cv2.putText(view, f"saved {saved}/{TARGET_SHOTS}   cells {int((covered>0).sum())}/9",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            for i in range(1, 3):
                cv2.line(view, (w * i // 3, 0), (w * i // 3, h), (60, 60, 60), 1)
                cv2.line(view, (0, h * i // 3), (w, h * i // 3), (60, 60, 60), 1)
            cv2.imshow("calibrate", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" ") and pts is not None:
                cv2.imwrite(str(out / f"shot_{saved:03d}.png"), frame)
                c = pts.reshape(-1, 2).mean(axis=0)
                covered[min(2, int(c[1] * 3 // h)), min(2, int(c[0] * 3 // w))] += 1
                saved += 1
                print(f"  저장 {saved}  (중심 {c[0]:.0f},{c[1]:.0f})")
    finally:
        cam.close()
        cv2.destroyAllWindows()

    print(f"\n{saved}장 저장 -> {out}")
    if saved < MIN_SHOTS:
        print(f"⚠ {MIN_SHOTS}장 이상 권장")
    empty = [(r, c) for r in range(3) for c in range(3) if covered[r, c] == 0]
    if empty:
        print(f"⚠ 비어있는 칸 {empty} — 가장자리 커버리지가 부족하면 왜곡계수가 부정확해진다")
    return 0


def cmd_solve(args) -> int:
    import cv2

    spec = camlib.SPECS[args.camera]
    model = args.model or spec.calib_model
    src = OUT_DIR / args.camera
    shots = sorted(src.glob("shot_*.png"))
    if len(shots) < MIN_SHOTS:
        print(f"이미지가 부족하다 ({len(shots)}장). capture를 먼저 돌려라.")
        return 1

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_mm

    obj_pts, img_pts, size = [], [], None
    for p in shots:
        img = cv2.imread(str(p))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        pts = _corners(gray, pattern, cv2)
        if pts is None:
            print(f"  코너 검출 실패, 건너뜀: {p.name}")
            continue
        obj_pts.append(objp)
        img_pts.append(pts)
    print(f"\n{len(obj_pts)}장 사용, 해상도 {size}, 모델 = {model}")

    if model == "fisheye":
        # cv2.fisheye는 (N,1,3)/(N,1,2) 형태를 요구한다
        objf = [o.reshape(-1, 1, 3) for o in obj_pts]
        imgf = [i.reshape(-1, 1, 2) for i in img_pts]
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                 | cv2.fisheye.CALIB_FIX_SKEW)
        rms, K, D, _, _ = cv2.fisheye.calibrate(
            objf, imgf, size, K, D, flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
        )
        dist = D.ravel().tolist()
    else:
        rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
        dist = D.ravel().tolist()

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    f_px = (fx + fy) / 2

    result = {
        "camera": args.camera, "model": model,
        "image_size": list(size), "rms_reproj_px": float(rms),
        "fx": fx, "fy": fy, "cx": cx, "cy": cy, "f_px": f_px,
        "dist": dist, "n_images": len(obj_pts),
        "square_mm": args.square_mm, "pattern": [args.cols, args.rows],
    }
    dst = src / "calibration.json"
    dst.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"  RMS 재투영 오차 = {rms:.3f} px  "
          f"{'(양호)' if rms < 1.0 else '(⚠ 1px 초과 — 재촬영 권장)'}")
    print(f"  fx={fx:.1f}  fy={fy:.1f}  주점=({cx:.1f}, {cy:.1f})")
    print(f"  왜곡계수 = {[round(d, 5) for d in dist]}")
    print(f"  -> {dst}")

    # ── 화각 (모델에 맞게) ──────────────────────────────────────────────
    w, h = size
    half = float(np.hypot(w / 2, h / 2))
    if model == "fisheye":
        # cv2.fisheye는 등거리(r = f·θ) 기준이다
        diag = 2 * np.degrees(half / f_px)
        print(f"  대각 화각 ≈ {diag:.1f}°  (등거리 모델 기준)")
    else:
        diag = 2 * np.degrees(np.arctan(half / f_px))
        print(f"  대각 화각 ≈ {diag:.1f}°")

    # ── 렌즈 확정 (pi5/TODO.md ★ 1번) ──────────────────────────────────
    lens_mm = f_px * args.pixel_mm
    best = min(LENS_TABLE, key=lambda t: abs(t[1] - f_px))
    print(f"\n--- 렌즈 확정 (TODO.md ★ 1번) ---")
    print(f"  실측 f_px = {f_px:.0f}  ->  렌즈 초점거리 ≈ {lens_mm:.2f} mm")
    print(f"  가장 가까운 표준 렌즈: {best[0]}mm (이론 f_px {best[1]})")
    if abs(best[1] - f_px) / best[1] > 0.15:
        print(f"  ⚠ 표준값과 15% 이상 차이난다. --pixel-mm({args.pixel_mm*1000:.2f}µm)가 "
              f"이 센서에 맞는지 확인하라.")

    if model == "pinhole" and diag > 90:
        print(f"\n  ⚠⚠ 대각 화각 {diag:.0f}°인데 pinhole로 풀었다. 90°를 넘으면 핀홀이 "
              f"성립하지 않는다.\n"
              f"     --model fisheye 로 다시 풀고, trajectory.py 입력에 "
              f"pi5/fisheye.py 를 끼워야 한다.")

    # ── config.py 스니펫 ───────────────────────────────────────────────
    print("\n--- pi5/config.py 에 반영할 값 ---")
    print(f"CAMERA_FX = {fx:.1f}")
    print(f"CAMERA_FY = {fy:.1f}")
    print(f"CAMERA_CX = {cx:.1f}")
    print(f"CAMERA_CY = {cy:.1f}")
    print(f'CAMERA_MODEL = "{model}"')
    print(f"CAMERA_DISTORTION = {tuple(round(d, 6) for d in dist)}")
    print("\n반영 후 pi5/TODO.md 의 '렌즈 초점거리 확인'과 'CAMERA_FX/FY/CX/CY' 항목을 닫을 것.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="카메라 캘리브레이션")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn in (("capture", cmd_capture), ("solve", cmd_solve)):
        p = sub.add_parser(name)
        camlib.add_profile_arg(p)
        p.add_argument("--cols", type=int, default=9, help="체커보드 내부 코너 열 수")
        p.add_argument("--rows", type=int, default=6, help="체커보드 내부 코너 행 수")
        p.add_argument("--square-mm", type=float, default=25.0, help="격자 한 칸 mm")
        if name == "solve":
            p.add_argument("--model", choices=["pinhole", "fisheye"], default=None,
                           help="기본값은 카메라 프로파일의 calib_model")
            p.add_argument("--pixel-mm", type=float, default=DEFAULT_PIXEL_MM,
                           help="센서 픽셀 피치(mm). f_px -> 렌즈 mm 역산용. "
                                "기본값은 IMX296의 3.45µm")
        p.set_defaults(func=fn)
    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())

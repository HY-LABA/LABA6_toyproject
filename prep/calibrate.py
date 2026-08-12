"""카메라 캘리브레이션 — f_px, 주점, 왜곡계수를 구한다.

    python calibrate.py capture --camera csi          # 체커보드 촬영
    python calibrate.py solve   --camera csi          # 계산 -> JSON + config 스니펫

**카메라가 바뀌어도 코드는 동일하다.** 바뀌는 건 `--camera` 프로파일 하나뿐이고,
그 안의 `calib_model`이 pinhole/fisheye를 자동으로 고른다 (camera.py 표 참고).

왜 모델 구분이 중요한가:
  화각이 90°를 넘으면 핀홀(r = f·tanθ)이 발산한다. GS 카메라의 2.8mm 렌즈는 대각 140°라
  반드시 fisheye 모델을 써야 하고, 기본 CSI(~62°)는 pinhole로 충분하다.
  (../docs/physics.md 7.2장)

체커보드: A4에 인쇄해 평평한 판에 붙인다. 기본값은 9x6 내부 코너, 25mm 격자.
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

    cam = camlib.open_from_args(args)
    print("\n[조작]  SPACE 저장   Q 종료")
    print("[요령]  화면 **가장자리와 모서리**를 반드시 채워라. 어안일수록 중요하다.")
    print("        보드를 기울여 여러 각도로 찍어야 초점거리와 왜곡이 분리된다.\n")

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

    print(f"  RMS 재투영 오차 = {rms:.3f} px  {'(양호)' if rms < 1.0 else '(⚠ 1px 초과 — 재촬영 권장)'}")
    print(f"  fx={fx:.1f}  fy={fy:.1f}  주점=({cx:.1f}, {cy:.1f})")
    print(f"  왜곡계수 = {[round(d, 5) for d in dist]}")
    print(f"  -> {dst}")

    # 화각 (모델에 맞게)
    w, h = size
    if model == "fisheye":
        # cv2.fisheye는 등거리(r = f·θ) 기준이다. frames.py의 등입체각과 다르므로 주의.
        half = float(np.hypot(w / 2, h / 2))
        diag = 2 * np.degrees(half / f_px)
        print(f"  대각 화각 ≈ {diag:.1f}°  (등거리 모델 기준)")
    else:
        diag = 2 * np.degrees(np.arctan(float(np.hypot(w / 2, h / 2)) / f_px))
        print(f"  대각 화각 ≈ {diag:.1f}°")

    print("\n--- catcher/config.py 에 반영할 값 ---")
    print(f"FOCAL_LENGTH_PX = {f_px:.1f}")
    print(f"PRINCIPAL_POINT_PX = ({cx:.1f}, {cy:.1f})")
    print(f"DISTORTION_COEFFS = {tuple(round(d, 6) for d in dist)}")
    if model == "fisheye":
        print('PROJECTION_MODEL = "equisolid"   # ⚠ 아래 주의사항 확인')
        print("\n⚠ cv2.fisheye는 **등거리**(r=f·θ) 모델이고 frames.py는 **등입체각**")
        print("   (r=2f·sin(θ/2))을 쓴다. 위 대각 화각이 사양(140°)과 크게 다르면")
        print("   frames.py의 PROJECTION_MODEL을 실제에 맞게 조정해야 한다.")
        print("   판단 근거: ../docs/physics.md 7.2장의 모델별 f_eff 표")
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
        p.set_defaults(func=fn)
    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())

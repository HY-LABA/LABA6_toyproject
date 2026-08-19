"""yolo_to_csv.py가 뽑은 CSV로 trajectory.py를 실측 검증한다. (로컬 PC에서 실행, 가벼움)

무거운 YOLO 추론은 이미 yolo_to_csv.py(Colab)에서 끝났다고 가정한다. 여기서는
그 CSV의 (frame, t, u, v)를 `trajectory.Tracker`에 실시간과 똑같은 순서로 먹여서
착지 예측이 실제로 어떻게 나오는지만 본다. main.py의 판단 흐름(놓치면 얼마나
기다리다 트랙을 끊는지 등)을 그대로 재현한다.

    python predict_trajectory.py out.csv
    python predict_trajectory.py out.csv --actual-xy 0.12 0.85
    python predict_trajectory.py out.csv --min-time-span 0.15 --depth-ratio 10

`--actual-xy`로 실제로 잰 착지점(카메라 원점 기준, 미터)을 주면 예측과의 오차까지
계산해준다 — **카메라를 기준으로 좌우(x)/앞뒤(y)를 잰 값**이어야 한다 (trajectory.py
좌표계 참고, ../trajectory.py 27행). **주의: trajectory.py의 "착지"는 바닥이 아니라
카메라 높이 평면(CATCH_HEIGHT_M=0.0)을 통과하는 순간이다.** 바닥에 떨어진 지점을
그대로 --actual-xy에 넣으면 좌표계가 안 맞아서 오차가 가짜로 커진다.

`--min-time-span`/`--depth-ratio`는 config.py의 MIN_TIME_SPAN_S/DEPTH_STABILITY_RATIO를
이 실행에서만 임시로 덮어쓴다 (파이프라인이 도는지만 빨리 확인할 때 게이트를
느슨하게 풀어보는 용도 — config.py 자체는 안 건드림).

⚠ config.py의 CAMERA_FX 등이 확정된 값이 아니면 --fx/--fy/--cx/--cy로 그 자리에서
덮어써서 테스트할 수 있다 (config.py를 안 고쳐도 됨).
"""

from __future__ import annotations

import argparse
import csv as csv_module
import math
import sys

import numpy as np

import config
import trajectory

# 윈도우 기본 콘솔은 cp949라 "⚠" 같은 문자에 UnicodeEncodeError로 죽는다.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def _undistort(u: float, v: float) -> tuple[float, float]:
    """vision.py의 _undistort()와 동일한 로직 — 실제 로봇 파이프라인과 일치시키려고
    복붙했다 (vision.py는 utils.py의 로그 파일 부작용이 딸려와서 여기선 import 안 함).

    렌즈 왜곡을 보정해 이상적인 핀홀 좌표로 옮긴다. CAMERA_DISTORTION이 None이면
    (아직 캘리브레이션 전) 원본 좌표를 그대로 돌려준다.
    """
    if config.CAMERA_DISTORTION is None:
        return u, v

    import cv2

    K = np.array([[config.CAMERA_FX, 0.0, config.CAMERA_CX],
                  [0.0, config.CAMERA_FY, config.CAMERA_CY],
                  [0.0, 0.0, 1.0]])
    d = np.asarray(config.CAMERA_DISTORTION, dtype=float)
    pts = np.array([[[float(u), float(v)]]], dtype=np.float64)

    if config.CAMERA_MODEL == "fisheye":
        out = cv2.fisheye.undistortPoints(pts, K, d.reshape(4, 1), P=K)
    else:
        out = cv2.undistortPoints(pts, K, d, P=K)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="yolo_to_csv.py가 뽑은 CSV (frame,t,u,v,conf)")
    ap.add_argument("--actual-xy", type=float, nargs=2, default=None, metavar=("X", "Y"),
                    help="실측 착지점(카메라 원점 기준, m, 카메라 높이 평면 통과 시점) "
                         "— 주면 예측 오차까지 계산")
    ap.add_argument("--fx", type=float, default=None, help="config.CAMERA_FX 덮어쓰기")
    ap.add_argument("--fy", type=float, default=None, help="config.CAMERA_FY 덮어쓰기")
    ap.add_argument("--cx", type=float, default=None, help="config.CAMERA_CX 덮어쓰기")
    ap.add_argument("--cy", type=float, default=None, help="config.CAMERA_CY 덮어쓰기")
    ap.add_argument("--min-time-span", type=float, default=None,
                    help="config.MIN_TIME_SPAN_S 덮어쓰기 (이 실행에서만)")
    ap.add_argument("--depth-ratio", type=float, default=None,
                    help="config.DEPTH_STABILITY_RATIO 덮어쓰기 (이 실행에서만)")
    args = ap.parse_args()

    if args.fx is not None:
        config.CAMERA_FX = args.fx
    if args.fy is not None:
        config.CAMERA_FY = args.fy
    if args.cx is not None:
        config.CAMERA_CX = args.cx
    if args.cy is not None:
        config.CAMERA_CY = args.cy
    if args.min_time_span is not None:
        config.MIN_TIME_SPAN_S = args.min_time_span
    if args.depth_ratio is not None:
        config.DEPTH_STABILITY_RATIO = args.depth_ratio
    print(f"[intrinsics] fx={config.CAMERA_FX:.1f} fy={config.CAMERA_FY:.1f} "
          f"cx={config.CAMERA_CX:.1f} cy={config.CAMERA_CY:.1f}")
    print(f"[gate] MIN_TIME_SPAN_S={config.MIN_TIME_SPAN_S:.3f} "
          f"DEPTH_STABILITY_RATIO={config.DEPTH_STABILITY_RATIO:.2f}\n")

    with open(args.csv, newline="", encoding="utf-8") as f:
        rows = list(csv_module.DictReader(f))
    if not rows:
        print(f"CSV가 비어있다: {args.csv}")
        return 1
    print(f"[csv] {args.csv}  {len(rows)}행\n")

    tracker = trajectory.Tracker()
    last_seen_t: float | None = None
    last_landing = None
    n_detected = 0
    n_predictions = 0
    first_pred_frame: int | None = None

    for row in rows:
        frame_idx = int(row["frame"])
        t = float(row["t"])
        if row["u"] == "" or row["v"] == "":
            # main.py와 동일한 놓침 판단: 너무 오래 놓치면 트랙을 끊는다.
            if last_seen_t is not None and tracker.times and (t - last_seen_t) > config.TRACK_MAX_GAP_S:
                print(f"[frame {frame_idx:4d} t={t:.3f}s] 트랙 유실(gap) — 리셋")
                tracker.reset()
                last_seen_t = None
            continue

        n_detected += 1
        u_raw, v_raw = float(row["u"]), float(row["v"])
        u, v = _undistort(u_raw, v_raw)
        last_seen_t = t

        fit = tracker.add(u, v, t)
        if fit is not None:
            landing = tracker.landing()
            if landing is not None:
                n_predictions += 1
                if first_pred_frame is None:
                    first_pred_frame = frame_idx
                last_landing = landing
                print(f"[frame {frame_idx:4d} t={t:.3f}s] resid={fit.residual_px:5.2f}px "
                      f"n={fit.n:2d}  landing=({landing[0]:+.3f},{landing[1]:+.3f})  "
                      f"remain={landing[2]:.3f}s")

    print(f"\n총 {len(rows)}프레임, 검출 {n_detected}프레임, 착지 예측 {n_predictions}회")
    if first_pred_frame is not None:
        first_t = next(float(r["t"]) for r in rows if int(r["frame"]) == first_pred_frame)
        print(f"첫 예측: {first_pred_frame}프레임째 (t={first_t:.3f}s)")
    if last_landing is None:
        print("\n⚠ 착지 예측이 한 번도 안 나왔다. 원인 후보:")
        print("   · MIN_TIME_SPAN_S 이상 연속 관측이 안 됨 — 검출이 자주 끊긴다 "
              "(--min-time-span 으로 낮춰서 재시도해볼 것)")
        print("   · 재투영 잔차/조건수 게이트에 계속 걸림 — config.MAX_RESIDUAL_PX 등 확인")
        print("   · 깊이 수렴 판정(DEPTH_STABILITY_RATIO) 통과 못 함 — 관측 스팬이 짧다 "
              "(--depth-ratio 으로 풀어서 재시도해볼 것)")
        return 0

    print(f"\n최종 예측 착지점: ({last_landing[0]:+.3f}, {last_landing[1]:+.3f})  "
          f"[카메라 원점 기준, m]")

    if args.actual_xy:
        ax, ay = args.actual_xy
        err = math.hypot(last_landing[0] - ax, last_landing[1] - ay)
        print(f"실측 착지점: ({ax:+.3f}, {ay:+.3f})")
        print(f"오차: {err * 100:.1f}cm")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

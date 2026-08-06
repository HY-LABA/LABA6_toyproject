"""σ_w 측정 — 이 파이프라인의 **합격 기준**을 확인한다.

    # ① 정지 물체를 알려진 거리에서 촬영 (라파이)
    python measure_sigma_w.py capture --camera csi --label can --distance 2.0

    # ② 학습된 모델로 bbox 폭 분산 측정 (PC)
    python measure_sigma_w.py measure --weights runs/detect/catcher/weights/best.pt

왜 mAP가 아니라 이걸 보는가:
    z = f_px · W_real / w_px    이므로 bbox 폭 오차가 그대로 거리 오차다.
    σ_w = 2 px 가정 위에 ../docs/physics.md 의 모든 정확도 계산이 서 있다.
    mAP 0.99여도 폭이 5 px씩 흔들리면 시스템은 성립하지 않는다.
    (../docs/vision-pipeline.md 1장, 7장)

**양자화 전후로 각각 측정하라.** Hailo INT8 변환이 bbox 회귀를 떨어뜨릴 수 있다.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import statistics

import numpy as np

import camera as camlib

ROOT = pathlib.Path("sigma_w")


def cmd_capture(args) -> int:
    import cv2

    out = ROOT / f"{args.label}_{args.distance:.2f}m_{args.camera}"
    out.mkdir(parents=True, exist_ok=True)
    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                              auto_lock=args.auto_lock_exposure)
    print(f"\n물체를 카메라에서 정확히 {args.distance} m 에 **정지**시켜라 (줄자로 실측).")
    print(f"[조작] SPACE 촬영 시작 / Q 종료   목표 {args.frames}장\n")

    n = 0
    recording = False
    try:
        while n < args.frames:
            frame, _ = cam.read()
            view = frame.copy()
            cv2.putText(view, f"{'REC ' if recording else 'ready '}{n}/{args.frames}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 0, 255) if recording else (0, 255, 0), 2)
            cv2.imshow("sigma_w capture", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                recording = not recording
            if recording:
                cv2.imwrite(str(out / f"{n:04d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                n += 1
    finally:
        cam.close()
        cv2.destroyAllWindows()

    (out / "meta.json").write_text(json.dumps(
        {"label": args.label, "distance_m": args.distance,
         "camera": args.camera, "frames": n}, indent=2), encoding="utf-8")
    print(f"\n{n}장 저장 -> {out}")
    print("여러 거리(1.0 / 1.5 / 2.0 / 2.5 m)에서 반복하면 거리 의존성까지 볼 수 있다.")
    return 0


def cmd_measure(args) -> int:
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics 가 없다:  pip install ultralytics")
        return 1

    sets = sorted(d for d in ROOT.iterdir() if d.is_dir() and (d / "meta.json").exists())
    if not sets:
        print(f"{ROOT} 에 측정 세트가 없다. capture 를 먼저 돌려라.")
        return 1

    model = YOLO(args.weights)
    print(f"\n{'세트':<28} {'N':>4} {'평균폭':>8} {'σ_w':>7} {'σ_w/w':>7} "
          f"{'추정z':>8} {'실제z':>7} {'편향':>7}")
    print("-" * 82)

    overall = []
    for d in sets:
        meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
        widths = []
        for img in sorted(d.glob("*.jpg")):
            res = model.predict(str(img), imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
            if len(res.boxes) == 0:
                continue
            # 가장 신뢰도 높은 하나
            i = int(np.argmax(res.boxes.conf.cpu().numpy()))
            x1, _, x2, _ = res.boxes.xyxy.cpu().numpy()[i]
            widths.append(float(x2 - x1))
        if len(widths) < 5:
            print(f"{d.name:<28} {len(widths):>4}  검출 부족 — 모델이나 촬영을 확인")
            continue

        mean_w = statistics.mean(widths)
        sigma = statistics.stdev(widths)
        z_true = meta["distance_m"]
        z_est = args.f_px * args.object_w_m / mean_w
        overall.append(sigma)
        print(f"{d.name:<28} {len(widths):>4} {mean_w:8.1f} {sigma:7.2f} "
              f"{sigma/mean_w*100:6.1f}% {z_est:8.3f} {z_true:7.2f} "
              f"{(z_est-z_true)/z_true*100:+6.1f}%")

    if not overall:
        return 1
    worst = max(overall)
    print("-" * 82)
    print(f"\n최대 σ_w = {worst:.2f} px   합격선 2.0 px")
    if worst <= 2.0:
        print("  ✅ 통과 — physics.md 의 정확도 계산이 그대로 성립한다")
    else:
        print(f"  ❌ 초과 — σ_z 가 {worst/2:.1f}배로 늘어난다. 되돌아가서 확인할 것:")
        print("     · 라벨 일관성 (review_labels.py 로 폭이 들쭉날쭉한지)")
        print("     · 작은 물체(먼 거리) 학습 샘플이 충분한지")
        print("     · 모션 블러가 섞이지 않았는지")
    print("\n'편향' 열이 한 방향으로 치우쳐 있으면 계통 오차다.")
    print("catcher/config.py 의 OBJECT_SIZE_M 을 그만큼 보정하면 흡수된다 —")
    print("일관된 편향은 괜찮지만 들쭉날쭉한 산포(σ_w)는 못 없앤다.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="σ_w 측정 (파이프라인 합격 기준)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="정지 물체를 알려진 거리에서 촬영")
    camlib.add_profile_arg(c)
    c.add_argument("--label", required=True, choices=["can", "pet_bottle", "paper_cup"])
    c.add_argument("--distance", type=float, required=True, help="카메라~물체 실측 거리(m)")
    c.add_argument("--frames", type=int, default=100)
    c.set_defaults(func=cmd_capture)

    m = sub.add_parser("measure", help="학습 모델로 bbox 폭 분산 측정")
    m.add_argument("--weights", required=True)
    m.add_argument("--imgsz", type=int, default=640)
    m.add_argument("--conf", type=float, default=0.25)
    m.add_argument("--f-px", type=float, default=792.0,
                   help="캘리브레이션에서 얻은 f_px (기본값은 GS+2.8mm 이론값)")
    m.add_argument("--object-w-m", type=float, default=0.066, help="물체 실물 폭(m)")
    m.set_defaults(func=cmd_measure)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())

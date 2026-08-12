"""σ_w(bbox 폭 분산) 측정.

⚠⚠ **더 이상 합격 기준이 아니다 (2026-08-10).** ⚠⚠

깊이 추정이 `z = f·W_real / w_px`(bbox 크기비율)에서 중력 기반 포물선 최소제곱
(pi5/trajectory.py)으로 바뀌었다. 이제 궤적 추정에 쓰는 건 bbox **중심(u,v)** 뿐이고
폭은 아무 데도 안 쓴다. 따라서 σ_w가 커도 시스템은 성립한다.

바뀐 이유: 물체마다 실제 크기를 등록해야 했고, 물체를 500ml 페트병 하나로 고정해도
공중에서 회전하면 보이는 폭이 3배까지 흔들려서(세로 65mm ↔ 가로 210mm) 크기비율
방식으로는 거리를 안정적으로 못 뽑았다.

**대신 봐야 할 지표:**

  ① **재투영 잔차 (residual_px)** — pi5/trajectory.py의 `Fit.residual_px`.
     추정한 궤적을 다시 화면에 투영해서 실제 관측과 몇 px 어긋나는지. 이게 곧
     "검출 노이즈 + 렌즈 왜곡 + 모델 오차"의 총합이고, config.MAX_RESIDUAL_PX(6px)를
     넘으면 그 트랙은 버려진다. 2px 근처면 좋고, 4px 이상이면 MIN_TIME_SPAN_S를
     늘려야 한다.
  ② **착지 예측 오차** — 실제로 던져서 예측 착지점과 실제 착지점의 거리를 잰다.
     이게 최종 성능이다.

이 스크립트는 참고용으로 남겨둔다 — bbox 폭 산포가 크면 검출 자체가 흔들린다는
신호이므로 중심 좌표 품질의 간접 지표로는 여전히 쓸 만하다. 다만 **2px 합격선은
이제 의미가 없다.**

    python measure_sigma_w.py capture --label trash --distance 2.0
    python measure_sigma_w.py measure --weights runs/detect/catcher/weights/best.pt
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
                              auto_lock=args.auto_lock_exposure,
                              max_exposure_us=args.max_exposure_us, max_gain=args.max_gain)
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
    print(f"\n최대 σ_w = {worst:.2f} px")
    print("  ⚠ 이 숫자에 합격선은 더 이상 없다 — 깊이 추정이 bbox 폭을 안 쓴다.")
    print("    (2026-08-10 중력 기반 궤적 최소제곱으로 전환. 파일 상단 주석 참고)")
    if worst > 4.0:
        print(f"  다만 {worst:.1f}px는 검출 자체가 흔들린다는 신호다. bbox 중심도 같이")
        print("    떨릴 가능성이 높고, 그건 궤적 피팅의 유일한 입력이므로 확인할 것:")
        print("     · 모션 블러 (노출을 줄이고 게인을 올려라)")
        print("     · 라벨 일관성 (review_labels.py)")
        print("     · 먼 거리(작은 물체) 학습 샘플 부족")
    print("\n진짜 봐야 할 지표는 궤적 피팅의 재투영 잔차(residual_px)와")
    print("실제 투척 시 착지 예측 오차다.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="σ_w 측정 (파이프라인 합격 기준)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="정지 물체를 알려진 거리에서 촬영")
    camlib.add_profile_arg(c)
    c.add_argument("--label", required=True, choices=["trash"])
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

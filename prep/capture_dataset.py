"""MOG2 배경차분으로 낙하 물체를 자동 검출 + 자동 라벨링해 데이터셋을 모은다. (라파이에서 실행)

    python capture_dataset.py --label can --camera csi
    python capture_dataset.py --label pet_bottle --session-note "복도 형광등"

동작: 카메라를 고정하고 천장을 향하게 둔 뒤, 물체를 떨어뜨린다. 배경(천장)은 정지해 있고
움직이는 건 물체뿐이므로 MOG2가 물체만 골라낸다. 클래스는 --label로 미리 알려주므로
사람이 라벨을 찍을 필요가 없다.

┌─ MOG2가 이 상황에 맞는 이유 ─────────────────────────────────────────────┐
│ · 카메라가 고정이다           — 배경 모델이 성립하는 전제                  │
│ · 배경(천장)이 정적이다        — 전경 = 물체                              │
│ · 한 번에 하나만 떨어뜨린다    — 가장 큰 덩어리 = 물체                     │
│ · 클래스를 미리 안다           — 라벨 자동 부여                           │
└──────────────────────────────────────────────────────────────────────────┘

┌─ 그래도 조심할 것 (필터로 처리했다) ──────────────────────────────────────┐
│ ① 던지는 손이 같이 잡힌다      — 덩어리가 2개 이상인 프레임은 버린다        │
│ ② 프레임 경계에 걸친 물체      — bbox 폭이 잘려 z가 틀어진다. 버린다        │
│ ③ 모션 블러가 bbox를 키운다    — 계통 오차라 OBJECT_SIZE_M 보정으로 흡수    │
│ ④ 형광등 깜빡임/노이즈         — 워밍업 + 면적 하한으로 거른다             │
│ ⑤ 배경과 색이 비슷하면 실패    — 흰 종이컵 + 흰 천장 조합은 피할 것         │
└──────────────────────────────────────────────────────────────────────────┘

┌─ 화면이 어두울 때 ────────────────────────────────────────────────────────┐
│ 카메라가 천장(=조명)을 보고 있어서 자동노출이 물체를 어둡게 만든다.        │
│ 시작할 때 진단이 뜨고, 실행 중에도 아래 키로 바로 고칠 수 있다:            │
│   ] / [  EV 올림/내림 (자동노출일 때 가장 먼저 쓸 것)                      │
│   + / -  수동 노출 늘림/줄임    A  자동노출 토글                          │
│   B      화면만 밝게 (저장 이미지는 그대로 — 확인용)                       │
│ 자세한 원인 구분은 camera.py 의 diagnose() 주석 참고.                      │
└──────────────────────────────────────────────────────────────────────────┘

**자동 라벨은 초안이다.** bbox 폭이 곧 거리 정확도이므로 review_labels.py로 반드시 검수한다
(../docs/vision-pipeline.md 4장).

┌─ 카메라 교체 시 ──────────────────────────────────────────────────────────┐
│ 로직은 그대로다. --camera 프로파일만 바꾸면 된다.                          │
│ 다만 **수집한 데이터는 재사용할 수 없다** — 화각·왜곡·셔터가 달라 물체가    │
│ 다른 크기/형태로 찍힌다. GS 카메라가 오면 처음부터 다시 모을 것.           │
│ (지금 CSI로 모으는 건 파이프라인 점검과 학습 절차 연습이 목적이다)          │
└──────────────────────────────────────────────────────────────────────────┘
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib

import numpy as np

import camera as camlib

CLASSES = ["can", "pet_bottle", "paper_cup"]   # catcher/config.py TARGET_CLASSES와 순서 일치
ROOT = pathlib.Path("dataset_raw")


class BlobFinder:
    """MOG2 전경 마스크에서 '물체 하나'를 찾아낸다."""

    def __init__(self, cv2, args, frame_area: int) -> None:
        self.cv2 = cv2
        self.a = args
        self.min_area = max(args.min_area, int(frame_area * 1e-5))
        self.max_area = int(frame_area * args.max_area_frac)
        # detectShadows=False: 그림자를 회색(127)으로 표시하는 기능. 천장 배경에는 불필요하고
        # 마스크만 지저분해진다.
        self.mog = cv2.createBackgroundSubtractorMOG2(
            history=args.history, varThreshold=args.var_threshold, detectShadows=False)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def __call__(self, frame, learning_rate: float):
        cv2 = self.cv2
        mask = self.mog.apply(frame, learningRate=learning_rate)
        # 열림 -> 점 노이즈 제거, 닫힘 -> 물체 내부 구멍 메우기
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = [c for c in contours if self.min_area <= cv2.contourArea(c) <= self.max_area]
        if not big:
            return mask, None, "none"
        if len(big) > 1:
            # ① 손 + 물체처럼 덩어리가 여럿이면 어느 쪽이 물체인지 확신할 수 없다
            return mask, None, f"multi({len(big)})"

        x, y, w, h = cv2.boundingRect(big[0])
        H, W = mask.shape[:2]
        m = self.a.edge_margin
        if x <= m or y <= m or x + w >= W - m or y + h >= H - m:
            return mask, None, "edge"          # ② 잘린 물체는 bbox 폭이 의미 없다
        ar = w / h if h else 0
        if not (self.a.min_aspect <= ar <= self.a.max_aspect):
            return mask, None, f"aspect({ar:.2f})"
        if cv2.contourArea(big[0]) / (w * h) < self.a.min_fill:
            return mask, None, "sparse"        # 길쭉한 노이즈 제거
        return mask, (x, y, w, h), "ok"


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description="MOG2 자동 라벨링 데이터 수집")
    camlib.add_profile_arg(ap)
    ap.add_argument("--label", required=True, choices=CLASSES, help="이번 세션에서 떨어뜨릴 물체")
    ap.add_argument("--session-note", default="", help="배경/조명 메모 (다양성 추적용)")
    ap.add_argument("--warmup", type=int, default=60, help="배경 학습 프레임 수")
    ap.add_argument("--arm-frames", type=int, default=2,
                    help="연속 이 프레임 이상 유효해야 저장 시작 (손 구간 회피)")
    ap.add_argument("--min-area", type=int, default=80)
    ap.add_argument("--max-area-frac", type=float, default=0.25,
                    help="프레임 대비 최대 면적. 넘으면 손/사람으로 본다")
    ap.add_argument("--min-aspect", type=float, default=0.2)
    ap.add_argument("--max-aspect", type=float, default=5.0)
    ap.add_argument("--min-fill", type=float, default=0.3, help="bbox 대비 윤곽 채움 비율")
    ap.add_argument("--edge-margin", type=int, default=4)
    ap.add_argument("--history", type=int, default=300)
    ap.add_argument("--var-threshold", type=float, default=25.0)
    ap.add_argument("--max-frames", type=int, default=0, help="0이면 무제한")
    ap.add_argument("--preview-boost", action="store_true",
                    help="화면만 밝게 본다 (저장 이미지는 그대로). 실행 중 B 키로도 전환")
    args = ap.parse_args()

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"{args.label}_{args.camera}_{stamp}"
    out_img = ROOT / "images" / session
    out_lbl = ROOT / "labels" / session
    out_img.mkdir(parents=True, exist_ok=True)
    out_lbl.mkdir(parents=True, exist_ok=True)

    cam = camlib.open_from_args(args)
    frame, _ = cam.read()
    H, W = frame.shape[:2]
    finder = BlobFinder(cv2, args, W * H)
    cls_id = CLASSES.index(args.label)

    # 어두우면 여기서 원인을 짚어준다. 어두운 채로 수집하면 MOG2도 YOLO도 같이 나빠진다.
    ok, msg = camlib.diagnose(cam, frame)
    print(f"\n[노출] {msg}")
    if not ok:
        print("[노출] 고친 뒤 수집을 시작하는 게 좋다 — 어두운 데이터는 다시 못 살린다.")

    print(f"\n세션 {session}   클래스 {args.label}(id={cls_id})")
    print("[조작]  SPACE 일시정지/재개   R 배경 재학습   Q 종료")
    print("[노출]  ] / [ EV   + / - 수동노출   A 자동노출 토글   B 화면만 밝게")
    print(f"[순서]  ① 카메라 고정 ② 워밍업 {args.warmup}프레임 동안 화면 비우기 "
          f"③ 물체 투척 반복\n")

    saved = 0
    streak = 0
    paused = False
    boost = args.preview_boost
    st = cam.stats()
    cam_final = st
    n = 0
    dark_frames = 0
    reasons: dict[str, int] = {}
    try:
        for i in range(args.warmup):
            f, _ = cam.read()
            finder(f, learning_rate=-1)        # -1: 자동 학습률로 배경 습득
            if i % 20 == 0:
                print(f"  워밍업 {i}/{args.warmup}")
        print("  워밍업 완료 — 이제 던져도 된다\n")

        while args.max_frames == 0 or n < args.max_frames:
            frame, _ = cam.read()
            n += 1
            if paused:
                cv2.imshow("capture (paused)",
                           camlib.preview_boost(frame) if boost else frame)
                if (cv2.waitKey(30) & 0xFF) == ord(" "):
                    paused = False
                continue

            # 학습률 0: 물체가 배경으로 흡수되지 않게 고정한다.
            # ⚠ MOG2에는 **원본**을 준다. preview_boost한 프레임을 주면 어두운 영역의
            #   노이즈까지 같이 증폭돼 헛 덩어리가 늘어난다. 밝기 보정은 눈으로 볼 때만.
            mask, box, why = finder(frame, learning_rate=0.0)
            reasons[why] = reasons.get(why, 0) + 1
            streak = streak + 1 if box else 0

            view = camlib.preview_boost(frame) if boost else frame.copy()
            if box:
                x, y, w, h = box
                good = streak >= args.arm_frames
                color = (0, 255, 0) if good else (0, 200, 255)
                cv2.rectangle(view, (x, y), (x + w, y + h), color, 2)
                if good:
                    name = f"{session}_{saved:05d}"
                    cv2.imwrite(str(out_img / f"{name}.jpg"), frame,
                                [cv2.IMWRITE_JPEG_QUALITY, 95])
                    # YOLO 포맷: class cx cy w h  (0~1 정규화)
                    (out_lbl / f"{name}.txt").write_text(
                        f"{cls_id} {(x + w / 2) / W:.6f} {(y + h / 2) / H:.6f} "
                        f"{w / W:.6f} {h / H:.6f}\n", encoding="utf-8")
                    saved += 1
            else:
                cv2.putText(view, why, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(view, f"{args.label}  saved={saved}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # 노출 상태를 항상 띄운다 — 어두울 때 무엇을 돌려야 하는지 보이게.
            if n % 15 == 1:
                st = cam.stats()
            exp_ms = (st.get("exposure_us") or 0) / 1000.0
            mean = float(frame.mean())
            if mean < 60:
                dark_frames += 1
            cv2.putText(view,
                        f"exp {exp_ms:.1f}ms  gain {st.get('gain') or 0:.1f}x  "
                        f"EV{cam.ev:+.1f}  mean {mean:.0f}" + ("  [boost]" if boost else ""),
                        (10, H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255) if mean < 60 else (200, 200, 200), 2)

            cv2.imshow("capture", view)
            cv2.imshow("mask", cv2.resize(mask, (W // 2, H // 2)))

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                paused = True
            if key in (ord("]"), ord("[")):
                cam.set_ev(cam.ev + (0.5 if key == ord("]") else -0.5))
                print(f"  EV {cam.ev:+.1f}")
            if key in (ord("+"), ord("="), ord("-"), ord("_")):
                base = st.get("exposure_us") or 5000
                factor = 1.5 if key in (ord("+"), ord("=")) else 1 / 1.5
                new = int(min(cam.max_exposure_us, max(100, base * factor)))
                cam.set_exposure(new)
                print(f"  수동 노출 {new/1000:.1f}ms (상한 {cam.max_exposure_us/1000:.1f}ms)")
            if key == ord("a"):
                cam.set_exposure(None if cam.exposure_us is not None
                                 else st.get("exposure_us"))
                print(f"  {'자동' if cam.exposure_us is None else '수동'} 노출")
            if key == ord("b"):
                boost = not boost
                print(f"  화면 밝기 보정 {'켬 — 저장 이미지는 그대로다' if boost else '끔'}")
            if key == ord("r"):
                finder = BlobFinder(cv2, args, W * H)
                print("  배경 재학습 — 화면을 비워라")
                for _ in range(args.warmup):
                    f, _ = cam.read()
                    finder(f, learning_rate=-1)
                print("  완료")
    finally:
        cam_final = cam.stats()          # 닫기 전에 확정된 노출값을 기록해둔다
        cam.close()
        cv2.destroyAllWindows()

    manifest = ROOT / "sessions.json"
    data = json.loads(manifest.read_text(encoding="utf-8")) if manifest.exists() else {}
    data[session] = {"label": args.label, "class_id": cls_id, "camera": args.camera,
                     "frames": saved, "note": args.session_note, "created": stamp,
                     "image_size": [W, H],
                     # 노출을 남겨둔다 — 나중에 "이 세션은 왜 어두웠나"를 추적할 수 있다
                     "exposure": {k: v for k, v in cam_final.items() if v is not None},
                     "dark_frac": round(dark_frames / max(1, n), 3)}
    manifest.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"\n{saved}장 저장 -> {out_img}")
    print(f"기각 사유: {dict(sorted(reasons.items(), key=lambda kv: -kv[1]))}")
    if reasons.get("multi", 0) > saved:
        print("⚠ 'multi'가 많다 — 손이 오래 잡히고 있다. 더 빨리 손을 빼거나 위에서 놓아라.")
    if reasons.get("edge", 0) > saved:
        print("⚠ 'edge'가 많다 — 물체가 화면 가장자리로 지나간다. 카메라 정렬을 확인하라.")
    if saved and dark_frames > saved * 0.5:
        print(f"⚠ 프레임의 {dark_frames*100//max(1,n)}%가 어두웠다(평균<60). "
              f"학습 품질이 떨어진다 — 노출을 고치고 이 세션은 다시 찍는 걸 권한다.")
        print("   최종 설정: " + json.dumps(
            {k: v for k, v in cam_final.items() if v is not None}, ensure_ascii=False))
    print("\n다음: python review_labels.py --session", session)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

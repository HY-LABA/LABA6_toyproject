"""MOG2 배경차분으로 낙하 물체를 자동 검출 + 자동 라벨링해 데이터셋을 모은다. (라파이에서 실행)

    python capture_dataset.py --label can --camera csi
    python capture_dataset.py --label pet_bottle --session-note "복도 형광등"

    # SSH/VNC 로 접속했다면 반드시 이렇게. 창을 원격으로 보내는 비용이
    # 프레임률을 10분의 1로 떨어뜨린다 (TROUBLESHOOTING.md 3번)
    python capture_dataset.py --label pet_bottle --no-display

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
import statistics
import sys
import time

import numpy as np

import camera as camlib

CLASSES = ["can", "pet_bottle", "paper_cup"]   # catcher/config.py TARGET_CLASSES와 순서 일치
ROOT = pathlib.Path("dataset_raw")


def beep() -> None:
    """저장될 때마다 소리로 알린다. --no-display 로 돌리면 이게 유일한 즉시 피드백이다
    (그리고 어차피 던지는 사람은 화면 앞이 아니라 방 건너편에 있다)."""
    sys.stdout.write("\a")
    sys.stdout.flush()


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
    ap.add_argument("--no-display", dest="display", action="store_false",
                    help="창을 띄우지 않는다. SSH/VNC 로 접속했다면 반드시 붙일 것 — "
                         "X11 로 프레임을 보내는 비용이 30fps를 3fps로 떨어뜨린다")
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
    args = ap.parse_args()

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"{args.label}_{args.camera}_{stamp}"
    out_img = ROOT / "images" / session
    out_lbl = ROOT / "labels" / session
    out_img.mkdir(parents=True, exist_ok=True)
    out_lbl.mkdir(parents=True, exist_ok=True)

    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                              auto_lock=args.auto_lock_exposure)
    frame, _ = cam.read()
    H, W = frame.shape[:2]
    finder = BlobFinder(cv2, args, W * H)
    cls_id = CLASSES.index(args.label)

    print(f"\n세션 {session}   클래스 {args.label}(id={cls_id})")
    if args.display:
        print("[조작]  SPACE 일시정지/재개   R 배경 재학습   Q 종료")
    else:
        print("[조작]  Ctrl+C 종료   (창 없음 — 저장될 때마다 비프음이 난다)")
    print(f"[순서]  ① 카메라 고정 ② 워밍업 {args.warmup}프레임 동안 화면 비우기 "
          f"③ 물체 투척 반복\n")

    saved = 0
    streak = 0
    paused = False
    reasons: dict[str, int] = {}
    fps_log: list[float] = []
    try:
        for i in range(args.warmup):
            f, _ = cam.read()
            finder(f, learning_rate=-1)        # -1: 자동 학습률로 배경 습득
            if i % 20 == 0:
                print(f"  워밍업 {i}/{args.warmup}")
        print("  워밍업 완료 — 이제 던져도 된다\n")

        n = 0
        fps_t0 = time.monotonic()
        while args.max_frames == 0 or n < args.max_frames:
            frame, _ = cam.read()
            n += 1
            if paused:
                cv2.imshow("capture (paused)", frame)
                if (cv2.waitKey(30) & 0xFF) == ord(" "):
                    paused = False
                continue

            # 학습률 0: 물체가 배경으로 흡수되지 않게 고정한다.
            mask, box, why = finder(frame, learning_rate=0.0)
            reasons[why] = reasons.get(why, 0) + 1
            streak = streak + 1 if box else 0

            good = bool(box) and streak >= args.arm_frames
            if good:
                x, y, w, h = box
                name = f"{session}_{saved:05d}"
                cv2.imwrite(str(out_img / f"{name}.jpg"), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                # YOLO 포맷: class cx cy w h  (0~1 정규화)
                (out_lbl / f"{name}.txt").write_text(
                    f"{cls_id} {(x + w / 2) / W:.6f} {(y + h / 2) / H:.6f} "
                    f"{w / W:.6f} {h / H:.6f}\n", encoding="utf-8")
                saved += 1
                beep()

            # 그리기와 창 전송은 --display 일 때만 한다. 원격 접속에서는 이 블록
            # 하나가 나머지 전부를 합친 것보다 비싸다 — 프레임당 6MB 넘게 나간다.
            if args.display:
                view = frame.copy()
                if box:
                    x, y, w, h = box
                    color = (0, 255, 0) if good else (0, 200, 255)
                    cv2.rectangle(view, (x, y), (x + w, y + h), color, 2)
                else:
                    cv2.putText(view, why, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 0, 255), 2)
                cv2.putText(view, f"{args.label}  saved={saved}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("capture", view)
                cv2.imshow("mask", cv2.resize(mask, (W // 2, H // 2)))

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    paused = True
                if key == ord("r"):
                    finder = BlobFinder(cv2, args, W * H)
                    print("  배경 재학습 — 화면을 비워라")
                    for _ in range(args.warmup):
                        f, _ = cam.read()
                        finder(f, learning_rate=-1)
                    print("  완료")

            # 실효 프레임률. 창이 없으면 비프음 말고는 이게 유일한 피드백이고,
            # 창이 있어도 전송이 프레임을 잡아먹는지는 이 숫자로만 알 수 있다.
            # 놓친 프레임은 그대로 투척당 수집 장수의 손실이다.
            if n % 30 == 0:
                now = time.monotonic()
                fps = 30.0 / max(1e-6, now - fps_t0)
                fps_t0 = now
                fps_log.append(fps)
                print(f"  {fps:5.1f} fps   saved={saved}   최근={why}")
    except KeyboardInterrupt:
        # Ctrl+C 로 끊어도 통계와 sessions.json 은 남겨야 한다. 사진 자체는
        # 프레임마다 즉시 쓰이므로 이미 디스크에 있다.
        print("\n  Ctrl+C — 중단한다. 저장된 사진은 그대로 남아 있다.")
    finally:
        cam.close()
        if args.display:
            cv2.destroyAllWindows()

    manifest = ROOT / "sessions.json"
    data = json.loads(manifest.read_text(encoding="utf-8")) if manifest.exists() else {}
    data[session] = {"label": args.label, "class_id": cls_id, "camera": args.camera,
                     "frames": saved, "note": args.session_note, "created": stamp,
                     "image_size": [W, H]}
    manifest.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

    # 절대경로로 찍는다. ROOT 가 상대경로라 실행 디렉토리에 따라 위치가 바뀌고,
    # 그 때문에 "폴더는 생겼는데 사진을 못 찾겠다"가 실제로 한 번 발생했다.
    print(f"\n{saved}장 저장 -> {out_img.resolve()}")
    print(f"기각 사유: {dict(sorted(reasons.items(), key=lambda kv: -kv[1]))}")

    if fps_log:
        avg = statistics.mean(fps_log)
        print(f"실효 프레임률: 평균 {avg:.1f} fps  (카메라 설정 {cam.spec.fps} fps)")
        if avg < cam.spec.fps * 0.7:
            print("⚠ 프레임을 놓치고 있다 — 투척당 잡히는 장수가 그만큼 줄어든다.")
            if args.display:
                print("   원격 접속(SSH/VNC) 중이라면 --no-display 를 붙여라.")
            else:
                print("   창이 없는데도 느리다면 MOG2/모폴로지가 풀해상도라 무겁다는 뜻이다.")
                print("   camera.py 에서 더 낮은 해상도 프로파일을 쓰는 것을 검토할 것.")

    multi_count = sum(v for k, v in reasons.items() if k.startswith("multi"))
    if multi_count > saved:
        print("⚠ 'multi'가 많다 — 손이 오래 잡히고 있다. 더 빨리 손을 빼거나 위에서 놓아라.")
    if reasons.get("edge", 0) > saved:
        print("⚠ 'edge'가 많다 — 물체가 화면 가장자리로 지나간다. 카메라 정렬을 확인하라.")
    print("\n다음: python review_labels.py --session", session)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""녹화된 영상에 .pt(ultralytics) 가중치를 직접 돌려서 bbox 그린 프레임을 저장한다.

.hef로 변환한 뒤 실기 성능이 떨어졌을 때, **변환 문제인지 pt 자체 학습 문제인지**
갈라보기 위한 개인 디버깅용 스크립트. .hef를 거치지 않고 ultralytics로 직접
추론하므로, 여기서도 성능이 나쁘면 pt(=학습) 문제, 여기선 괜찮은데 파이에서만
나쁘면 .hef 변환/HailoRT 쪽 문제로 좁혀진다.

    python debug_pt_on_video.py 영상.mp4 --weights best.pt

conf 임계값 이상인 검출은 전부 그린다 — 최고 신뢰도(초록, live_predict가 실제로
쓰는 것과 같은 기준)와 나머지 후보(노랑)를 구분해서, "여러 개 애매하게 잡히는지"
같은 것도 같이 보인다. 검출 없는 프레임은 저장 안 한다(용량 절약, live_predict
--save-debug와 동일한 방침).

출력 폴더(.jpg)는 .gitignore에 이미 걸려 있어서 커밋 안 된다.
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import cv2

# 윈도우 기본 콘솔은 cp949라 유니코드 문자에 UnicodeEncodeError로 죽는다.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def _draw(frame, boxes_xyxy, confs, best_idx: int):
    img = frame.copy()
    for i, ((x1, y1, x2, y2), conf) in enumerate(zip(boxes_xyxy, confs)):
        color = (0, 255, 0) if i == best_idx else (0, 220, 255)  # 초록=최고, 노랑=나머지
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img, p1, p2, color, 2)
        cv2.putText(img, f"{conf:.2f}", (p1[0], max(0, p1[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return img


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", help="capture_video.py로 찍은 영상 (mp4)")
    ap.add_argument("--weights", required=True, help="테스트할 .pt 가중치")
    ap.add_argument("--conf", type=float, default=0.25,
                    help="이 값 이상인 검출을 전부 그린다 (live_predict 실전 기준인 "
                         "config.YOLO_CONF_THRESHOLD=0.5보다 낮게 잡아서, 문턱값 "
                         "근처에서 애매하게 걸리는 것도 보이게 함)")
    ap.add_argument("--imgsz", type=int, default=640,
                    help="추론 입력 크기 — train_yolo.py에 쓴 값과 맞출 것")
    ap.add_argument("--out", default="pt_debug_frames", help="bbox 그린 프레임 저장 폴더")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="앞에서부터 이 프레임 수만 처리 (영상이 길 때 빠른 확인용)")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics 가 없다:  pip install ultralytics")
        return 1

    model = YOLO(args.weights)
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"영상을 못 열었다: {args.video}")
        return 1

    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    fps = cap.get(cv2.CAP_PROP_FPS) or 60.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[video] {args.video}  fps={fps:.1f}  frames={total or '?'}  weights={args.weights}")
    print(f"[out]   {out_dir.resolve()}\n")

    frame_idx = 0
    n_detected = 0
    n_saved = 0
    confs_all: list[float] = []

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if args.max_frames and frame_idx >= args.max_frames:
            break

        res = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
        n_hits = len(res.boxes)

        if n_hits > 0:
            n_detected += 1
            boxes_xyxy = res.boxes.xyxy.tolist()
            confs = res.boxes.conf.tolist()
            best_idx = int(res.boxes.conf.argmax())
            confs_all.append(confs[best_idx])

            img = _draw(frame, boxes_xyxy, confs, best_idx)
            t = frame_idx / fps
            cv2.imwrite(str(out_dir / f"{frame_idx:05d}_t{t:.3f}_n{n_hits}.jpg"), img)
            n_saved += 1

        frame_idx += 1
        if total and frame_idx % 50 == 0:
            print(f"  {frame_idx}/{total} 처리, 검출 {n_detected}, 저장 {n_saved}")

    cap.release()

    print(f"\n총 {frame_idx}프레임 처리, 검출 {n_detected}프레임 "
          f"({n_detected / frame_idx * 100:.1f}%), 사진 {n_saved}장 저장")
    if confs_all:
        print(f"최고 신뢰도 검출 기준: 평균 {sum(confs_all)/len(confs_all):.3f}  "
              f"최소 {min(confs_all):.3f}  최대 {max(confs_all):.3f}")
    else:
        print("⚠ 한 프레임도 검출 안 됐다 — --conf를 더 낮춰서 재시도해볼 것")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""녹화된 영상에 YOLO를 입혀서 프레임별 검출을 CSV로 뽑는다. (Colab에서 실행)

무거운 YOLO 추론만 여기서 하고 끝낸다. 궤적 계산은 안 한다 — 이 CSV를
predict_trajectory.py에 넘기면 그쪽에서 로컬 PC로 가볍게 돌릴 수 있다.

    python yolo_to_csv.py 영상.mp4 --weights best.pt --csv out.csv

Colab에서 쓸 때:
    !pip install ultralytics
    !python yolo_to_csv.py /content/drive/MyDrive/영상.mp4 --weights best.pt --csv out.csv
그 다음 out.csv를 드라이브/로컬로 받아서 predict_trajectory.py에 넘길 것.
"""

from __future__ import annotations

import argparse
import csv as csv_module
import sys

import cv2

# 윈도우 기본 콘솔은 cp949라 유니코드 문자에 UnicodeEncodeError로 죽는다.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", help="capture_video.py로 찍은 클립 (던지기 1회짜리 권장)")
    ap.add_argument("--weights", required=True, help="학습된 YOLO 가중치 (.pt)")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold")
    ap.add_argument("--fps", type=float, default=None,
                    help="영상 fps를 강제로 지정 (생략하면 파일 메타데이터에서 읽음)")
    ap.add_argument("--csv", required=True, help="검출 결과를 저장할 CSV 경로")
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
    fps = args.fps or cap.get(cv2.CAP_PROP_FPS) or 60.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[video] {args.video}  fps={fps:.1f}  frames={total or '?'}")

    csv_file = open(args.csv, "w", newline="", encoding="utf-8")
    writer = csv_module.writer(csv_file)
    writer.writerow(["frame", "t", "u", "v", "conf"])

    frame_idx = 0
    n_detected = 0
    next_pct = 10
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = frame_idx / fps

        res = model.predict(frame, conf=args.conf, verbose=False)[0]
        if len(res.boxes) == 0:
            writer.writerow([frame_idx, f"{t:.4f}", "", "", ""])
        else:
            i = int(res.boxes.conf.argmax())
            x1, y1, x2, y2 = res.boxes.xyxy[i].tolist()
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            conf = float(res.boxes.conf[i])
            writer.writerow([frame_idx, f"{t:.4f}", f"{u:.2f}", f"{v:.2f}", f"{conf:.3f}"])
            n_detected += 1

        frame_idx += 1
        if total and frame_idx * 100 // total >= next_pct:
            print(f"  {next_pct}% ({frame_idx}/{total})")
            next_pct += 10

    cap.release()
    csv_file.close()
    print(f"\n총 {frame_idx}프레임, 검출 {n_detected}프레임 -> {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

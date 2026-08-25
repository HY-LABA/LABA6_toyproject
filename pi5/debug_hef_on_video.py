"""녹화된 영상에 실제 .hef(Hailo)를 그대로 돌려서 bbox 그린 영상으로 뽑는다.

live_predict.py는 카메라 실시간 캡처 전용이라 예전에 찍어둔 영상은 재현할 수
없다. 이 스크립트는 카메라 대신 저장된 mp4를 프레임 소스로 써서, 실제
파이프라인이 쓰는 `vision._HailoYolo`를 그대로 재사용해 hef 추론 결과를 확인한다.

`debug_pt_on_video.py`(.pt를 ultralytics로 직접 돌림)로 같은 영상을 돌려서 비교하면:
  · 여기서도 pt와 비슷하게 잘 잡으면 → hef 변환/HailoRT 문제 아님
  · 여기서 pt보다 확실히 나쁘면 → .hef 변환 쪽 문제로 좁혀짐

⚠ HailoRT(hailo_platform)가 설치된 **파이에서만** 돈다 (Picamera2는 필요 없음 —
영상 파일만 읽으므로 카메라 연결 안 해도 됨).

    python debug_hef_on_video.py 영상.mp4
    python debug_hef_on_video.py 영상.mp4 --hef /path/to/other.hef --out result.mp4
"""

from __future__ import annotations

import argparse
import sys

import cv2

# 윈도우 기본 콘솔은 cp949라 유니코드 문자에 UnicodeEncodeError로 죽는다.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def _draw(frame, hits: list[tuple[tuple[float, float, float, float], float]], best_idx: int):
    img = frame.copy()
    for i, (bbox, conf) in enumerate(hits):
        cx, cy, w, h = bbox
        x1, y1 = int(cx - w / 2), int(cy - h / 2)
        x2, y2 = int(cx + w / 2), int(cy + h / 2)
        color = (0, 255, 0) if i == best_idx else (0, 220, 255)  # 초록=최고, 노랑=나머지
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, f"{conf:.2f}", (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return img


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", help="capture_video.py로 찍은 영상 (mp4)")
    ap.add_argument("--hef", default=None,
                     help="테스트할 .hef 경로. 생략하면 config.YOLO_MODEL_PATH를 쓴다")
    ap.add_argument("--conf", type=float, default=0.25,
                     help="이 값 이상인 검출을 전부 그린다 (실전 기준 "
                          "config.YOLO_CONF_THRESHOLD=0.5보다 낮게 잡아서 문턱값 "
                          "근처도 보이게 함)")
    ap.add_argument("--out", default="hef_debug_out.mp4", help="bbox 그린 결과 영상 경로")
    ap.add_argument("--max-frames", type=int, default=None,
                     help="앞에서부터 이 프레임 수만 처리 (영상이 길 때 빠른 확인용)")
    args = ap.parse_args()

    import config
    import vision  # HailoRT 필요 — 파이에서만 import 성공

    hef_path = args.hef or config.YOLO_MODEL_PATH
    if hef_path is None:
        print("hef 경로가 없다 — --hef로 직접 주거나 config.YOLO_MODEL_PATH를 설정할 것")
        return 1

    yolo = vision._HailoYolo(hef_path)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"영상을 못 열었다: {args.video}")
        return 1

    fps = cap.get(cv2.CAP_PROP_FPS) or 60.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[video] {args.video}  fps={fps:.1f}  {w}x{h}  frames={total or '?'}  hef={hef_path}")

    writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    if not writer.isOpened():
        print(f"출력 영상을 못 열었다: {args.out}")
        cap.release()
        return 1
    print(f"[out]   {args.out}\n")

    frame_idx = 0
    n_detected = 0
    confs_all: list[float] = []

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if args.max_frames and frame_idx >= args.max_frames:
            break

        hits = [(bbox, conf) for bbox, conf in yolo.infer(frame) if conf >= args.conf]

        if hits:
            n_detected += 1
            best_idx = max(range(len(hits)), key=lambda i: hits[i][1])
            confs_all.append(hits[best_idx][1])
            out_frame = _draw(frame, hits, best_idx)
        else:
            out_frame = frame

        writer.write(out_frame)

        frame_idx += 1
        if total and frame_idx % 50 == 0:
            print(f"  {frame_idx}/{total} 처리, 검출 {n_detected}")

    cap.release()
    writer.release()

    print(f"\n총 {frame_idx}프레임 처리, 검출 {n_detected}프레임 "
          f"({n_detected / frame_idx * 100:.1f}%) -> {args.out}")
    if confs_all:
        print(f"최고 신뢰도 검출 기준: 평균 {sum(confs_all)/len(confs_all):.3f}  "
              f"최소 {min(confs_all):.3f}  최대 {max(confs_all):.3f}")
    else:
        print("⚠ 한 프레임도 검출 안 됐다 — --conf를 더 낮춰서 재시도해볼 것")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

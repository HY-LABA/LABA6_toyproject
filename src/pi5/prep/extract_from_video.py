"""녹화된 영상에서 물체 프레임만 뽑아 YOLO 라벨을 만든다 (오프라인, capture_video.py 짝).

`capture_video.py`로 찍은 클립들을 읽어서, 거기 있는 **MOG2 로직**(`BlobFinder`
재사용)으로 물체 트랙을 찾고 프레임+라벨을 저장한다. 캡처와
라벨링을 분리했으므로 이 단계는 실시간일 필요가 없다 — 라파이가 아니어도 되고,
느려도 상관없다. **이 스크립트는 Picamera2가 필요 없다** — 녹화된 .mp4 파일만
있으면 되므로, GUI가 불안정한 라파이 대신 PC로 클립을 옮겨서 돌리는 걸 추천한다.

    python extract_from_video.py capture_sessions/session_20260812_190000

여기선 후보를 전부 자동 저장한다(빠름) — 사람 검수는 여기서 안 하고,
`review_labels.py --zoom`으로 촬영이 다 끝난 뒤 한 번에 쭉 넘겨보면서 한다.
(추출 도중 프레임마다 물어보면 처리가 계속 끊겨서, "다 찍고 나서 한꺼번에
검수"하는 흐름에 안 맞는다고 판단해 분리했다.)

세션 하나에 여러 클립(여러 번 던진 것)이 있으면 전부 처리한다. 클립마다 앞부분
`--warmup` 프레임(기본 30 = 0.5초@60fps)을 배경 학습에 쓴다 — 녹화 시작 직후는
보통 빈 화면이라는 전제다 (`capture_video.py`가 카운트다운 후 녹화를 시작하므로
성립한다).

출력: dataset_raw/images|labels/<세션이름>/ — prepare_dataset.py를 그대로 이어서
쓸 수 있는 위치.
"""

from __future__ import annotations

import argparse
import json
import pathlib

import cv2
import numpy as np

from capture_video import BlobFinder, CLASSES

OUT_ROOT = pathlib.Path("dataset_raw")


def process_clip(path: pathlib.Path, cls_id: int, args,
                  out_img: pathlib.Path, out_lbl: pathlib.Path) -> tuple[int, dict]:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        print(f"  ! 못 엶: {path}")
        return 0, {}

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    finder = BlobFinder(cv2, args, w * h)

    frame_idx = 0
    next_pct = 10

    def _progress() -> None:
        # 10%씩만 찍는다 — 오래 걸리는(1~3분) 작업인데 진행 상황이 전혀 안 보이면
        # 멈춘 건지 도는 건지 알 수가 없다. 그렇다고 매 프레임 찍으면 그 자체가
        # 콘솔 출력 병목이 되니 10% 단위로만.
        nonlocal next_pct
        if total <= 0:
            return
        pct = frame_idx * 100 // total
        while pct >= next_pct and next_pct <= 100:
            print(f"  {next_pct}%...")
            next_pct += 10

    # 클립 앞부분은 카운트다운 직후라 보통 빈 화면이다 — 그걸로 배경을 학습한다.
    for _ in range(args.warmup):
        ok, frame = cap.read()
        frame_idx += 1
        if not ok:
            cap.release()
            return 0, {}
        finder(frame, learning_rate=-1)
        _progress()

    saved = 0
    streak = 0
    tracking = False
    miss = 0
    arm_miss = 0
    pending: list[tuple[np.ndarray, tuple[int, int, int, int]]] = []
    reasons: dict[str, int] = {}

    def _save(save_frame: np.ndarray, save_box: tuple[int, int, int, int]) -> None:
        nonlocal saved
        x, y, bw, bh = save_box
        name = f"{path.stem}_{saved:05d}"
        cv2.imwrite(str(out_img / f"{name}.jpg"), save_frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        (out_lbl / f"{name}.txt").write_text(
            f"{cls_id} {(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} "
            f"{bw / w:.6f} {bh / h:.6f}\n", encoding="utf-8")
        saved += 1

    # 확정(arm)/추적 상태기계. 오프라인 배치 처리라 실시간 캡처 루프(파일 vs
    # 라이브 카메라)와 제어 흐름이 미묘하게 다르다.
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_idx += 1
        _progress()

        mask, box, why = finder(frame, learning_rate=0.0)
        reasons[why] = reasons.get(why, 0) + 1

        if not tracking:
            if box:
                streak += 1
                arm_miss = 0
                pending.append((frame.copy(), box))
                if streak >= args.arm_frames:
                    for pframe, pbox in pending:
                        _save(pframe, pbox)
                    pending.clear()
                    tracking = True
                    miss = 0
            elif pending:
                arm_miss += 1
                if arm_miss > args.arm_grace:
                    streak = 0
                    pending.clear()
                    arm_miss = 0
        else:
            if box:
                miss = 0
                _save(frame, box)
            else:
                miss += 1
                if miss > args.track_grace:
                    tracking = False
                    streak = 0
                    miss = 0

    cap.release()
    return saved, reasons


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="capture_video.py가 만든 세션 폴더")
    ap.add_argument("--warmup", type=int, default=30, help="클립 앞부분 배경학습 프레임 수")
    ap.add_argument("--arm-frames", type=int, default=2)
    ap.add_argument("--arm-grace", type=int, default=2)
    ap.add_argument("--track-grace", type=int, default=6)
    ap.add_argument("--min-area", type=int, default=80)
    ap.add_argument("--max-area-frac", type=float, default=0.25)
    ap.add_argument("--min-aspect", type=float, default=0.2)
    ap.add_argument("--max-aspect", type=float, default=5.0)
    ap.add_argument("--min-fill", type=float, default=0.3)
    ap.add_argument("--edge-margin", type=int, default=4)
    ap.add_argument("--history", type=int, default=300)
    ap.add_argument("--var-threshold", type=float, default=25.0)
    args = ap.parse_args()

    session_dir = pathlib.Path(args.session)
    meta_path = session_dir / "session.json"
    if not meta_path.exists():
        print(f"session.json 없음: {meta_path} (capture_video.py로 만든 폴더가 맞는지 확인)")
        return 1
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    if not meta["clips"]:
        print("이 세션엔 클립이 없다.")
        return 1

    out_img = OUT_ROOT / "images" / session_dir.name
    out_lbl = OUT_ROOT / "labels" / session_dir.name
    out_img.mkdir(parents=True, exist_ok=True)
    out_lbl.mkdir(parents=True, exist_ok=True)

    total_saved = 0
    total_reasons: dict[str, int] = {}
    for clip in meta["clips"]:
        label = clip["label"]
        cls_id = CLASSES.index(label) if label in CLASSES else 0
        path = session_dir / "clips" / clip["file"]
        print(f"{clip['file']} 처리 중...")
        saved, reasons = process_clip(path, cls_id, args, out_img, out_lbl)
        total_saved += saved
        for k, v in reasons.items():
            total_reasons[k] = total_reasons.get(k, 0) + v
        print(f"  {saved}장 저장  (기각: {dict(sorted(reasons.items(), key=lambda kv: -kv[1]))})")

    print(f"\n총 {total_saved}장 저장 -> {out_img.resolve()}")
    print(f"전체 기각 사유: {dict(sorted(total_reasons.items(), key=lambda kv: -kv[1]))}")
    print(f"\n다음: python review_labels.py --session {session_dir.name} --zoom")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

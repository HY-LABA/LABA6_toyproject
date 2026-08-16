"""녹화된 영상에서 물체 프레임만 뽑아 YOLO 라벨을 만든다 (오프라인, capture_video.py 짝).

`capture_video.py`로 찍은 클립들을 읽어서, `capture_dataset.py`와 **같은 MOG2
로직**(`BlobFinder` 재사용)으로 물체 트랙을 찾고 프레임+라벨을 저장한다. 캡처와
라벨링을 분리했으므로 이 단계는 실시간일 필요가 없다 — 라파이가 아니어도 되고,
느려도 상관없다.

    python extract_from_video.py capture_sessions/session_20260812_190000

세션 하나에 여러 클립(여러 번 던진 것)이 있으면 전부 처리한다. 클립마다 앞부분
`--warmup` 프레임(기본 30 = 0.5초@60fps)을 배경 학습에 쓴다 — 녹화 시작 직후는
보통 빈 화면이라는 전제다 (`capture_video.py`가 카운트다운 후 녹화를 시작하므로
성립한다).

출력: dataset_raw/images|labels/<세션이름>/ — capture_dataset.py와 같은 위치라
prepare_dataset.py를 그대로 이어서 쓸 수 있다.
"""

from __future__ import annotations

import argparse
import json
import pathlib

import cv2
import numpy as np

from capture_dataset import BlobFinder, CLASSES

OUT_ROOT = pathlib.Path("dataset_raw")


def process_clip(path: pathlib.Path, cls_id: int, args,
                  out_img: pathlib.Path, out_lbl: pathlib.Path) -> tuple[int, dict]:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        print(f"  ! 못 엶: {path}")
        return 0, {}

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    finder = BlobFinder(cv2, args, w * h)

    # 클립 앞부분은 카운트다운 직후라 보통 빈 화면이다 — 그걸로 배경을 학습한다.
    for _ in range(args.warmup):
        ok, frame = cap.read()
        if not ok:
            cap.release()
            return 0, {}
        finder(frame, learning_rate=-1)

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

    # capture_dataset.py의 확정(arm)/추적 상태기계와 동일하다 — 로직을 두 군데서
    # 따로 관리하지 않으려면 원래 공용 함수로 빼는 게 맞지만, 지금은 오프라인
    # 배치 처리라 실시간 루프와 제어 흐름이 미묘하게 달라(파일 vs 라이브 카메라)
    # 당장은 복제해 둔다.
    while True:
        ok, frame = cap.read()
        if not ok:
            break

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
    print(f"\n다음: python review_labels.py --session {session_dir.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

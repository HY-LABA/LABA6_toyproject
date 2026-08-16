"""녹화된 영상에서 물체 프레임만 뽑아 YOLO 라벨을 만든다 (오프라인, capture_video.py 짝).

`capture_video.py`로 찍은 클립들을 읽어서, `capture_dataset.py`와 **같은 MOG2
로직**(`BlobFinder` 재사용)으로 물체 트랙을 찾고 프레임+라벨을 저장한다. 캡처와
라벨링을 분리했으므로 이 단계는 실시간일 필요가 없다 — 라파이가 아니어도 되고,
느려도 상관없다. **이 스크립트는 Picamera2가 필요 없다** — 녹화된 .mp4 파일만
있으면 되므로, GUI가 불안정한 라파이 대신 PC로 클립을 옮겨서 돌리는 걸 추천한다.

    python extract_from_video.py capture_sessions/session_20260812_190000
    python extract_from_video.py capture_sessions/session_20260812_190000 --review

`--review` 없이 돌리면 MOG2가 찾은 후보를 전부 저장한다(빠름, 나중에
review_labels.py로 사후 검수). `--review`를 붙이면 **저장하기 전에** 프레임마다
bbox를 확대해서 보여주고 승인/거부를 직접 고른다 — 잘못된 라벨이 애초에
dataset_raw/에 들어가지 않는다.

[--review 조작]
    Y / SPACE   저장
    N           버림 (저장 안 함)
    Q           중단 (지금까지 승인된 것까지만 저장하고 전체 종료)

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
REVIEW_WIN = "review (Y/SPACE=저장  N=버림  Q=중단)"


def _zoomed_view(frame: np.ndarray, box: tuple[int, int, int, int],
                  pad_frac: float = 0.6, min_size: int = 480) -> np.ndarray:
    """bbox 주변만 잘라서 확대한다 — 화면 전체를 놓고 작은 물체를 보는 것보다
    실제로 박스가 물체를 제대로 감쌌는지 판단하기 쉽다."""
    h, w = frame.shape[:2]
    x, y, bw, bh = box
    pad = int(max(bw, bh) * pad_frac) + 20
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(w, x + bw + pad), min(h, y + bh + pad)
    crop = frame[y0:y1, x0:x1].copy()
    cv2.rectangle(crop, (x - x0, y - y0), (x - x0 + bw, y - y0 + bh), (0, 255, 0), 2)
    ch, cw = crop.shape[:2]
    scale = max(1, min_size // max(1, min(ch, cw)))
    if scale > 1:
        crop = cv2.resize(crop, (cw * scale, ch * scale), interpolation=cv2.INTER_NEAREST)
    return crop


def _confirm(frame: np.ndarray, box: tuple[int, int, int, int], info: str) -> str:
    """확대한 후보를 보여주고 사람이 저장 여부를 정한다. 반환: 'keep'|'drop'|'quit'."""
    view = _zoomed_view(frame, box)
    cv2.putText(view, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow(REVIEW_WIN, view)
    key_map = {ord("y"): "keep", ord(" "): "keep", ord("n"): "drop", ord("q"): "quit"}
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key in key_map:
            return key_map[key]


def process_clip(path: pathlib.Path, cls_id: int, args,
                  out_img: pathlib.Path, out_lbl: pathlib.Path) -> tuple[int, dict, bool]:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        print(f"  ! 못 엶: {path}")
        return 0, {}, False

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    finder = BlobFinder(cv2, args, w * h)

    # 클립 앞부분은 카운트다운 직후라 보통 빈 화면이다 — 그걸로 배경을 학습한다.
    for _ in range(args.warmup):
        ok, frame = cap.read()
        if not ok:
            cap.release()
            return 0, {}, False
        finder(frame, learning_rate=-1)

    saved = 0
    dropped = 0
    streak = 0
    tracking = False
    miss = 0
    arm_miss = 0
    pending: list[tuple[np.ndarray, tuple[int, int, int, int]]] = []
    reasons: dict[str, int] = {}
    aborted = False

    def _save(save_frame: np.ndarray, save_box: tuple[int, int, int, int]) -> None:
        nonlocal saved
        x, y, bw, bh = save_box
        name = f"{path.stem}_{saved:05d}"
        cv2.imwrite(str(out_img / f"{name}.jpg"), save_frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        (out_lbl / f"{name}.txt").write_text(
            f"{cls_id} {(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} "
            f"{bw / w:.6f} {bh / h:.6f}\n", encoding="utf-8")
        saved += 1

    def _maybe_save(cand_frame: np.ndarray, cand_box: tuple[int, int, int, int]) -> bool:
        """--review면 저장 전에 사람 확인을 받는다. True를 반환하면 중단해야 한다."""
        nonlocal dropped, aborted
        if not args.review:
            _save(cand_frame, cand_box)
            return False
        decision = _confirm(cand_frame, cand_box,
                             f"{path.name}  저장 {saved} / 버림 {dropped}")
        if decision == "keep":
            _save(cand_frame, cand_box)
        elif decision == "drop":
            dropped += 1
        else:  # quit
            aborted = True
            return True
        return False

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
                        if _maybe_save(pframe, pbox):
                            break
                    pending.clear()
                    tracking = True
                    miss = 0
                    if aborted:
                        break
            elif pending:
                arm_miss += 1
                if arm_miss > args.arm_grace:
                    streak = 0
                    pending.clear()
                    arm_miss = 0
        else:
            if box:
                miss = 0
                if _maybe_save(frame, box):
                    break
            else:
                miss += 1
                if miss > args.track_grace:
                    tracking = False
                    streak = 0
                    miss = 0

    cap.release()
    if args.review and dropped:
        print(f"    (검수: 저장 {saved} / 버림 {dropped})")
    return saved, reasons, aborted


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="capture_video.py가 만든 세션 폴더")
    ap.add_argument("--review", action="store_true",
                    help="저장 전에 프레임마다 bbox를 확대해서 보여주고 사람이 승인/거부")
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

    if args.review:
        cv2.namedWindow(REVIEW_WIN)
        print("[검수 모드] Y/SPACE=저장  N=버림  Q=중단(지금까지 저장분 유지하고 전체 종료)\n")

    total_saved = 0
    total_reasons: dict[str, int] = {}
    try:
        for clip in meta["clips"]:
            label = clip["label"]
            cls_id = CLASSES.index(label) if label in CLASSES else 0
            path = session_dir / "clips" / clip["file"]
            print(f"{clip['file']} 처리 중...")
            saved, reasons, aborted = process_clip(path, cls_id, args, out_img, out_lbl)
            total_saved += saved
            for k, v in reasons.items():
                total_reasons[k] = total_reasons.get(k, 0) + v
            print(f"  {saved}장 저장  (기각: {dict(sorted(reasons.items(), key=lambda kv: -kv[1]))})")
            if aborted:
                print("  검수 중단 — 남은 클립은 처리하지 않는다.")
                break
    finally:
        if args.review:
            cv2.destroyAllWindows()

    print(f"\n총 {total_saved}장 저장 -> {out_img.resolve()}")
    print(f"전체 기각 사유: {dict(sorted(total_reasons.items(), key=lambda kv: -kv[1]))}")
    print(f"\n다음: python review_labels.py --session {session_dir.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

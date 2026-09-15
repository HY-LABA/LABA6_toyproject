"""capture_roam.py(또는 capture_video.py)로 찍은 영상을 사람이 직접 훑으며 라벨링한다.
(PC에서 실행 권장 — extract_from_video.py와 같은 이유: 이 스크립트는 Picamera2가
전혀 필요 없고 cv2 GUI만 쓰는데, 라파이의 GUI가 불안정하다. 영상 파일을
capture_sessions 폴더째로 PC에 옮겨서 실행할 것.)

    python label_throws.py capture_sessions/session_20260908_010024
    python label_throws.py capture_sessions/session_20260908_010024 --clip roam.mp4

★ 왜 필요한가 — collect_throws.py 물리 게이트의 대안
------------------------------------------------------
`collect_throws.py`는 `TrackerPool`의 궤적 피팅(`trajectory.Fit.ok`)이 성공해야만
저장한다. 화면에 멀쩡히 다 찍힌 투척도 피팅 조건(관측 수·잔차·궤적 범위 등) 중
하나만 못 채우면 통째로 버려져서, 실제로 "컷당하는" 진짜 투척이 많으면 수집이
느려진다. 이 도구는 그 물리 게이트를 아예 거치지 않는다 — `capture_roam.py`가
찍어둔 영상에서 **사람이 직접** 구간을 지정하고 프레임마다 바운딩 박스를 그린다.
자동 판정이 없으니 "찍히긴 했는데 버려지는" 일 자체가 없다. 대신 라벨링이
자동이 아니라 수동이다 — 두 도구는 서로 대체가 아니라 상호 보완 관계다
(`capture_roam.py`의 모듈 docstring 참고).

[조작]
  a / d          이전 / 다음 프레임 (한 장씩)
  A / D          이전 / 다음 10프레임 (빨리 훑기)
  마우스 드래그   현재 프레임에 바운딩 박스 그리기 (놓으면 확정 전 '임시' 상태)
  space / enter  임시 박스를 이 프레임 라벨로 확정하고 다음 프레임으로 이동
                 (박스를 새로 안 그렸으면 직전 프레임 박스를 그대로 이어받는다 —
                  물체가 거의 안 움직인 연속 프레임에서 매번 다시 그릴 필요 없다)
  r              이 프레임의 임시 박스를 지운다 (확정 전 상태만 취소)
  k              이 프레임은 저장하지 않고 건너뛴다 (구간은 계속 유지 — 물체가
                 잠깐 화면 밖으로 나갔다 다시 보이는 경우. tracker.py가 짧은 결측을
                 허용하는 것과 같은 이유)
  i              구간 시작(진입점) — 지금 프레임부터 저장 대상으로 잡는다
  o              구간 끝(종료점) — 지금까지 확정한 프레임들을 하나의 트랙으로 저장한다
                 (지금 프레임 자체는 "물체가 이미 사라진" 프레임이라는 뜻이라 저장 안 됨)
  x              진행 중인 구간을 취소한다 (지금까지 확정한 프레임 전부 버림)
  u              구간 안에서 마지막으로 확정한 프레임을 무른다 (박스를 잘못 그렸을 때)
  n              다음 클립으로 (세션에 클립이 여러 개일 때)
  q / ESC        종료

출력: dataset_raw/images/<세션이름>/*.jpg + dataset_raw/labels/<세션이름>/<stem>.txt
     (collect_throws.py 등과 완전히 같은 규약 — 별도 images/labels 트리, 파일 stem으로
     매칭). 트랙 메타는 dataset_raw/meta/<세션이름>.jsonl 에 `"kind":"track"` 레코드로
     쌓여서 review_labels.py --by-track 로 그대로 훑어볼 수 있다.

⚠ 이 도구는 물리 피팅을 전혀 안 하므로 meta 레코드에 "fit"이 없다 — review_labels.py
  --by-track 화면에 z/v0/resid 는 '?'/nan 으로 뜬다(정상. 그 스크립트가 기본값으로
  처리하도록 이미 만들어져 있다). "labeled_by": "manual" 로 자동 라벨과 구분해둔다.

⚠ 화면 밖 프레임을 배경(빈 라벨)으로 저장하지 않는다 — `k`(건너뛰기)는 그 프레임을
  **아예 저장하지 않는다.** collect_live.py가 경고하는 것과 같은 이유: 사실은 물체가
  있는(잠깐 안 보일 뿐인) 프레임을 "물체 없음"으로 학습시키면 안 된다.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

ROOT = pathlib.Path("dataset_raw")   # 다른 prep 도구와 같은 cwd 기준 상대경로


# ── 순수 로직 (GUI 없이 테스트 가능) ─────────────────────────────────────────

def bbox_to_yolo(x0: float, y0: float, x1: float, y1: float,
                 w: int, h: int) -> tuple[float, float, float, float]:
    """픽셀 좌표(순서 무관) -> YOLO 정규화 (cx, cy, bw, bh)."""
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    cx = (x0 + x1) / 2.0 / w
    cy = (y0 + y1) / 2.0 / h
    bw = (x1 - x0) / w
    bh = (y1 - y0) / h
    return cx, cy, bw, bh


@dataclass
class ChunkFrame:
    idx: int                                   # 영상 전체 기준 프레임 인덱스
    frame: "object"                             # np.ndarray (BGR), accept 시점의 복사본
    bbox_px: tuple[float, float, float, float]  # x0,y0,x1,y1 (원본 해상도 픽셀)


@dataclass
class ChunkRecorder:
    """구간(트랙) 하나의 진행 상태. 확정 프레임은 `o`(finish) 전까지 메모리에만 있다 —
    그래야 `x`(취소)/`u`(undo)가 디스크에 아무 흔적도 안 남기고 깔끔하다."""

    active: bool = False
    frames: list = field(default_factory=list)

    def start(self) -> bool:
        if self.active:
            return False
        self.active = True
        self.frames = []
        return True

    def accept(self, idx: int, frame, bbox_px) -> bool:
        if not self.active:
            return False
        self.frames.append(ChunkFrame(idx, frame, bbox_px))
        return True

    def undo(self) -> bool:
        if not self.frames:
            return False
        self.frames.pop()
        return True

    def cancel(self) -> list:
        dropped = self.frames
        self.active = False
        self.frames = []
        return dropped

    def finish(self) -> list:
        frames = self.frames
        self.active = False
        self.frames = []
        return frames


def save_chunk(frames: list, *, clip_stem: str, track_id: int, fps: float,
              img_dir: pathlib.Path, lbl_dir: pathlib.Path, quality: int = 95) -> dict | None:
    """확정된 프레임들을 이미지+라벨로 쓰고, review_labels.py --by-track 이 읽을 수 있는
    트랙 메타 레코드를 돌려준다. 프레임이 하나도 없으면 아무것도 안 쓰고 None."""
    if not frames:
        return None
    import cv2

    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    files: list[str] = []
    centers: list[list[float]] = []
    for k, cf in enumerate(frames):
        h, w = cf.frame.shape[:2]
        cx, cy, bw, bh = bbox_to_yolo(*cf.bbox_px, w, h)
        name = f"{clip_stem}_t{track_id:03d}_{k:03d}"
        cv2.imwrite(str(img_dir / f"{name}.jpg"), cf.frame,
                    [cv2.IMWRITE_JPEG_QUALITY, quality])
        (lbl_dir / f"{name}.txt").write_text(
            f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n", encoding="utf-8")
        files.append(f"{name}.jpg")
        centers.append([cx, cy])

    idx0, idx1 = frames[0].idx, frames[-1].idx
    return {
        "kind": "track",
        "track_id": track_id,
        "source_clip": clip_stem,
        "frame_range": [idx0, idx1],
        "files": files,
        "centers": centers,
        "span_s": (idx1 - idx0) / fps if fps > 0 else 0.0,
        "n_saved": len(files),
        "n_observed": idx1 - idx0 + 1,
        "labeled_by": "manual",
    }


def next_track_id(meta_path: pathlib.Path) -> int:
    """meta.jsonl에 이미 있는 트랙들 다음 번호부터 이어간다 — 같은 세션을 나중에
    다시 열어 라벨링을 이어갈 때 track_id가 겹치지 않게."""
    if not meta_path.exists():
        return 1
    best = 0
    for line in meta_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        if rec.get("kind") == "track":
            best = max(best, int(rec.get("track_id", 0)))
    return best + 1


class VideoSource:
    """`cv2.VideoCapture` 래퍼 — 임의 인덱스로 탐색하되 최근 프레임을 캐시한다.

    ⚠ capture_video.py/capture_roam.py의 기본 코덱이 mjpeg인 게 여기서도 중요하다 —
    프레임마다 독립적으로 압축돼 있어서(모든 프레임이 키프레임) `CAP_PROP_POS_FRAMES`
    탐색이 다른 코덱처럼 가까운 키프레임까지 되감았다가 다시 디코드하는 일 없이
    정확하고 빠르다. h264로 찍었다면 탐색이 느리거나 부정확할 수 있다.
    """

    def __init__(self, path: pathlib.Path, cache_size: int = 64) -> None:
        import cv2
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise RuntimeError(f"영상을 열 수 없다: {path}")
        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self._cache: dict[int, object] = {}
        self._order: deque = deque()
        self._cache_size = cache_size

    def get(self, idx: int):
        if self.frame_count > 0:
            idx = max(0, min(idx, self.frame_count - 1))
        else:
            idx = max(0, idx)
        cached = self._cache.get(idx)
        if cached is not None:
            return idx, cached
        self.cap.set(__import__("cv2").CAP_PROP_POS_FRAMES, idx)
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return idx, None
        self._cache[idx] = frame
        self._order.append(idx)
        if len(self._order) > self._cache_size:
            old = self._order.popleft()
            self._cache.pop(old, None)
        return idx, frame

    def close(self) -> None:
        self.cap.release()


# ── GUI (사람이 직접 훑는 부분 — 하드웨어/디스플레이 필요, 단위 테스트 대상 아님) ──

def _run_clip(clip_path: pathlib.Path, *, img_dir: pathlib.Path, lbl_dir: pathlib.Path,
             meta_path: pathlib.Path, quality: int, scale: float) -> str:
    """클립 하나를 훑는다. 반환값: 'next'(다음 클립으로) 또는 'quit'(프로그램 종료)."""
    import cv2

    src = VideoSource(clip_path)
    clip_stem = clip_path.stem
    print(f"\n[{clip_path.name}] {src.frame_count}프레임 @ {src.fps:.1f}fps")

    idx = 0
    staged_bbox: tuple[float, float, float, float] | None = None
    drag_start: tuple[int, int] | None = None
    drag_cur: tuple[int, int] | None = None
    chunk = ChunkRecorder()
    track_id = next_track_id(meta_path)
    saved_this_clip = 0

    win = "label_throws"
    cv2.namedWindow(win)

    def on_mouse(event, x, y, flags, userdata):
        nonlocal drag_start, drag_cur, staged_bbox
        if event == cv2.EVENT_LBUTTONDOWN:
            drag_start = (x, y)
            drag_cur = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and drag_start is not None:
            drag_cur = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and drag_start is not None:
            x0, y0 = drag_start
            x1, y1 = x, y
            drag_start = None
            drag_cur = None
            if abs(x1 - x0) >= 3 and abs(y1 - y0) >= 3:
                staged_bbox = (x0 / scale, y0 / scale, x1 / scale, y1 / scale)

    cv2.setMouseCallback(win, on_mouse)

    result = "next"
    while True:
        idx, frame = src.get(idx)
        if frame is None:
            print("  (더 이상 프레임이 없다)")
            break
        view = frame.copy()
        if scale != 1.0:
            view = cv2.resize(view, (int(view.shape[1] * scale), int(view.shape[0] * scale)))

        box_to_draw = None
        if drag_start is not None and drag_cur is not None:
            box_to_draw = (*drag_start, *drag_cur)
            box_color = (0, 255, 255)
        elif staged_bbox is not None:
            x0, y0, x1, y1 = staged_bbox
            box_to_draw = (int(x0 * scale), int(y0 * scale), int(x1 * scale), int(y1 * scale))
            box_color = (0, 200, 0)
        if box_to_draw is not None:
            cv2.rectangle(view, box_to_draw[:2], box_to_draw[2:], box_color, 2)

        status = f"구간 진행중 (트랙 {track_id}, {len(chunk.frames)}장 확정)" if chunk.active else "구간 없음 (i=시작)"
        header = f"[{clip_stem}] {idx+1}/{src.frame_count}  {status}  이번클립저장 {saved_this_clip}"
        cv2.putText(view, header, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow(win, view)

        key = cv2.waitKey(0) & 0xFF
        if key in (ord("q"), 27):
            if chunk.active and chunk.frames:
                print("  ⚠ 저장 안 된 구간이 있다 — o(저장) 또는 x(취소) 후 종료할 것.")
                continue
            result = "quit"
            break
        elif key == ord("n"):
            if chunk.active and chunk.frames:
                print("  ⚠ 저장 안 된 구간이 있다 — o(저장) 또는 x(취소) 후 다음 클립으로 갈 것.")
                continue
            result = "next"
            break
        elif key == ord("a"):
            idx = max(0, idx - 1)
            staged_bbox = None
        elif key == ord("d"):
            idx = idx + 1
            staged_bbox = None
        elif key == ord("A"):
            idx = max(0, idx - 10)
            staged_bbox = None
        elif key == ord("D"):
            idx = idx + 10
            staged_bbox = None
        elif key == ord("r"):
            staged_bbox = None
        elif key == ord("i"):
            if chunk.start():
                track_id = next_track_id(meta_path)
                print(f"  구간 시작 (트랙 {track_id}, 프레임 {idx})")
            else:
                print("  이미 구간이 진행 중이다.")
        elif key == ord("k"):
            if chunk.active:
                idx += 1
                staged_bbox = None
            else:
                print("  구간이 없다 — i로 먼저 시작할 것.")
        elif key == ord("u"):
            if chunk.undo():
                print(f"  마지막 확정 프레임을 무름 ({len(chunk.frames)}장 남음)")
            else:
                print("  무를 확정 프레임이 없다.")
        elif key == ord("x"):
            dropped = chunk.cancel()
            print(f"  구간 취소 ({len(dropped)}장 버림, 디스크엔 안 씀)")
            staged_bbox = None
        elif key == ord("o"):
            frames = chunk.finish()
            rec = save_chunk(frames, clip_stem=clip_stem, track_id=track_id, fps=src.fps,
                             img_dir=img_dir, lbl_dir=lbl_dir, quality=quality)
            if rec is None:
                print("  확정된 프레임이 없어 저장할 게 없다.")
            else:
                with meta_path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                saved_this_clip += rec["n_saved"]
                print(f"  트랙 {track_id} 저장: {rec['n_saved']}장, {rec['span_s']:.2f}s"
                      f"  -> {img_dir.name}/")
            staged_bbox = None
        elif key in (32, 13):  # space / enter
            if not chunk.active:
                print("  구간이 없다 — i로 먼저 시작할 것.")
            elif staged_bbox is None:
                print("  이 프레임에 그려진(또는 이어받은) 박스가 없다 — 드래그로 그릴 것.")
            else:
                chunk.accept(idx, frame, staged_bbox)
                idx += 1
                # staged_bbox는 그대로 이어받는다 — 다음 프레임에서 물체가 거의 같은
                # 자리에 있으면 곧장 space로 확정할 수 있게.

    cv2.destroyWindow(win)
    src.close()
    return result


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", type=pathlib.Path,
                    help="capture_roam.py/capture_video.py가 만든 session_YYYYMMDD_HHMMSS 폴더")
    ap.add_argument("--clip", default=None,
                    help="이 클립 파일 하나만 (기본: clips/ 안의 모든 클립을 순서대로)")
    ap.add_argument("--session-name", default=None,
                    help="dataset_raw 아래 쓰일 세션 이름 (기본: session_dir 폴더 이름)")
    ap.add_argument("--out", type=pathlib.Path, default=ROOT, help="dataset_raw 루트")
    ap.add_argument("--quality", type=int, default=95, help="JPEG 품질")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="화면 표시 배율 (예: 0.7 — 해상도가 화면보다 크면 줄여서 보기). "
                         "저장되는 이미지는 항상 원본 해상도다")
    args = ap.parse_args()

    if not args.session_dir.exists():
        ap.error(f"세션 폴더가 없다: {args.session_dir}")
    clipdir = args.session_dir / "clips"
    if args.clip:
        clips = [clipdir / args.clip]
        if not clips[0].exists():
            ap.error(f"클립이 없다: {clips[0]}")
    else:
        clips = sorted(clipdir.glob("*.mp4"))
        if not clips:
            ap.error(f"clips/ 에 mp4가 없다: {clipdir}")

    session_name = args.session_name or args.session_dir.name
    img_dir = args.out / "images" / session_name
    lbl_dir = args.out / "labels" / session_name
    meta_path = args.out / "meta" / f"{session_name}.jsonl"
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    if not meta_path.exists():
        meta_path.write_text(json.dumps({
            "header": True, "session": session_name,
            "created": datetime.now().isoformat(timespec="seconds"),
            "kind": "track_header",
            "note": "label_throws.py로 수동 라벨링 — fit 없음 (labeled_by=manual)",
        }, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"세션: {session_name}  ({len(clips)}개 클립)")
    print(f"저장 위치: {img_dir}  /  {lbl_dir}  /  {meta_path}")

    for clip_path in clips:
        outcome = _run_clip(clip_path, img_dir=img_dir, lbl_dir=lbl_dir, meta_path=meta_path,
                            quality=args.quality, scale=args.scale)
        if outcome == "quit":
            break

    print(f"\n다음: python review_labels.py --session {session_name} --by-track")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

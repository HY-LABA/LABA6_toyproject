"""자동 라벨 검수 도구 — MOG2/궤적 피팅이 만든 bbox를 눈으로 확인하고 걸러낸다.

    python review_labels.py                      # 전체 세션, 프레임 단위
    python review_labels.py --session can_csi_20260802_101500
    python review_labels.py --zoom                # bbox 주변만 잘라서 확대
    python review_labels.py --by-track            # 트랙 단위 (collect_throws.py 세션용)
    python review_labels.py --by-track --session throw_lab_a_20260910_140000

**왜 검수가 필요한가:** bbox 폭이 곧 거리 추정값이다. 자동 라벨은 초안일 뿐이고,
모션 블러 꼬리나 잘린 물체가 섞이면 σ_w가 커져 시스템 정확도가 그대로 떨어진다
(../docs/vision-pipeline.md 4장).

★ `--by-track` — 트랙 단위 검수 (`collect_throws.py` 전용)
------------------------------------------------------------
`collect_throws.py`가 만든 세션은 `dataset_raw/meta/<세션>.jsonl`에 트랙별 요약
(`"kind": "track"` 레코드 — track_id, 관측된 궤적점, 잔차·깊이·|v0|)을 같이 남긴다.

자동 라벨의 오류는 **프레임 단위로 독립적이지 않다.** 물리 게이트를 통과한 트랙이면
그 안의 프레임은 대체로 다 맞고, 잘못 통과한 트랙이면(정적 오탐이 우연히 게이트를
넘은 경우 등) 그 안의 프레임은 전부 틀렸을 가능성이 크다 — 오류가 트랙 단위로
상관돼 있으므로 판단도 트랙 단위로 하는 게 맞다. 화면엔 트랙의 첫 프레임 위에
**궤적점을 전부 겹쳐 그려서** 매끈한 포물선(진짜)과 뭉치거나 지그재그인 것(오탐)을
한눈에 가른다. 프레임 15장을 하나씩 넘기는 대신 결정 한 번으로 끝나 훨씬 빠르다.

의심스러운 트랙은 `F`로 프레임 단위 검수에 들어가 개별 프레임을 볼 수 있다 — 결정은
같은 `dropped.json`에 쌓이므로 두 모드를 오가도 어긋나지 않는다.

`--by-track`은 meta 파일이 있는 세션만 다룬다(`collect_live.py`/`extract_from_video.py`
세션처럼 meta가 없으면 트랙 개념이 없으므로 건너뛴다 — 프레임 단위 모드를 쓸 것).

[조작 — 프레임 단위 (기본)]
    → / ←      다음 / 이전 (안 먹히면 `.`/`,` 또는 `l`/`j`)
    +  / -      (--zoom일 때) 물체 중심으로 확대 / 축소
    F          (--zoom일 때) 전체화면 즉시 토글 (다시 F 누르면 확대 상태로 복귀)
    D          이 프레임 버림
    K          유지 (기본값)
    A          bbox 수동 조정 모드 (드래그로 다시 그림)
    S          지금까지 결정 저장
    Q          저장하고 종료

[조작 — 트랙 단위 (--by-track)]
    → / ←      다음 / 이전 트랙 (`.`/`,` 또는 `l`/`j`도 동작)
    D          이 트랙 전체 버림 (포함된 프레임 전부)
    K          이 트랙 전체 유지
    F          이 트랙만 프레임 단위로 들어가서 개별 확인 (Q로 트랙 목록에 복귀)
    S          지금까지 결정 저장
    Q          저장하고 종료
"""

from __future__ import annotations

import argparse
import json
import pathlib

ROOT = pathlib.Path("dataset_raw")
CLASSES = ["trash"]   # capture_video.py / collect_throws.py 와 같아야 한다


def load_label(p: pathlib.Path):
    txt = p.read_text(encoding="utf-8").strip()
    if not txt:
        return None
    parts = txt.split()[:5]
    return int(parts[0]), *(float(v) for v in parts[1:5])


def save_label(p: pathlib.Path, cls: int, cx, cy, w, h) -> None:
    p.write_text(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n", encoding="utf-8")


def _zoom_region(lab, W: int, H: int, pad_frac: float = 0.6,
                  min_size: int = 480) -> tuple[int, int, int, int, int]:
    """bbox 기준 크롭 영역(x0,y0,x1,y1)과 정수 확대 배율을 계산한다.

    라벨이 없으면(lab=None) 확대할 기준이 없으니 전체 프레임을 그대로 쓴다.
    """
    if lab is None:
        return 0, 0, W, H, 1
    _cls, cx, cy, w, h = lab
    bx0, by0 = (cx - w / 2) * W, (cy - h / 2) * H
    bx1, by1 = (cx + w / 2) * W, (cy + h / 2) * H
    pad = max(bx1 - bx0, by1 - by0) * pad_frac + 20
    x0 = max(0, int(bx0 - pad))
    y0 = max(0, int(by0 - pad))
    x1 = min(W, int(bx1 + pad))
    y1 = min(H, int(by1 + pad))
    scale = max(1, min_size // max(1, min(x1 - x0, y1 - y0)))
    return x0, y0, x1, y1, scale


def _save_dropped(drop_file: pathlib.Path, dropped: set) -> None:
    drop_file.write_text(json.dumps(sorted(dropped), indent=2), encoding="utf-8")


# ── 프레임 단위 검수 ─────────────────────────────────────────────────────

def review_frame_items(items: list, drop_file: pathlib.Path, dropped: set, args,
                       title_suffix: str = "") -> None:
    """`items` = [(img_path, lbl_path), ...] 를 한 장씩 보여주고 keep/drop을 받는다.

    전체 세션 스캔(기본 모드)과 `--by-track`의 `F`(트랙 드릴다운) 양쪽이 이 함수를
    공유한다 — 같은 `dropped` 집합에 쓰므로 두 모드를 오가도 결정이 안 어긋난다.
    """
    import cv2

    i = 0
    drag = {"on": False, "p0": None, "p1": None}
    zoom_state = {"pad_mult": 1.0, "full": False}

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            drag.update(on=True, p0=(x, y), p1=(x, y))
        elif event == cv2.EVENT_MOUSEMOVE and drag["on"]:
            drag["p1"] = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            drag["on"] = False

    cv2.namedWindow("review")
    cv2.setMouseCallback("review", on_mouse)

    while 0 <= i < len(items):
        img_p, lbl_p = items[i]
        img = cv2.imread(str(img_p))
        if img is None:
            print(f"  ! 이미지를 못 읽었다: {img_p}")
            i += 1
            continue
        H, W = img.shape[:2]
        lab = load_label(lbl_p)

        if args.zoom and not zoom_state["full"]:
            zx0, zy0, zx1, zy1, zscale = _zoom_region(lab, W, H, pad_frac=0.6 * zoom_state["pad_mult"])
        else:
            zx0, zy0, zx1, zy1, zscale = 0, 0, W, H, 1
        view = img[zy0:zy1, zx0:zx1].copy()
        if zscale > 1:
            view = cv2.resize(view, ((zx1 - zx0) * zscale, (zy1 - zy0) * zscale),
                              interpolation=cv2.INTER_NEAREST)

        def to_view(px: float, py: float) -> tuple[int, int]:
            return int((px - zx0) * zscale), int((py - zy0) * zscale)

        def to_orig(vx: float, vy: float) -> tuple[float, float]:
            return zx0 + vx / zscale, zy0 + vy / zscale

        if lab:
            cls, cx, cy, w, h = lab
            x0, y0 = int((cx - w / 2) * W), int((cy - h / 2) * H)
            x1, y1 = int((cx + w / 2) * W), int((cy + h / 2) * H)
            is_dropped = str(img_p) in dropped
            color = (0, 0, 255) if is_dropped else (0, 255, 0)
            cv2.rectangle(view, to_view(x0, y0), to_view(x1, y1), color, 2)
            cv2.putText(view, f"{CLASSES[cls]}  w={x1-x0}px", to_view(x0, max(20, y0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        if drag["p0"] and drag["p1"]:
            cv2.rectangle(view, drag["p0"], drag["p1"], (255, 200, 0), 2)

        status = "DROP" if str(img_p) in dropped else "keep"
        if not args.zoom:
            zoom_tag = ""
        elif zoom_state["full"]:
            zoom_tag = " [FULL — F로 확대복귀]"
        else:
            zoom_tag = f" [ZOOM x{zoom_state['pad_mult']:.1f} — F로 전체화면]"
        cv2.putText(view, f"[{i+1}/{len(items)}] {status}  {img_p.parent.name}"
                    f"{zoom_tag}{title_suffix}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("review", view)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break
        elif key in (ord("d"),):
            dropped.add(str(img_p)); i += 1
        elif key in (ord("k"),):
            dropped.discard(str(img_p)); i += 1
        elif key == ord("a") and drag["p0"] and drag["p1"]:
            (ax, ay) = to_orig(*drag["p0"])
            (bx, by) = to_orig(*drag["p1"])
            x0, x1 = sorted((ax, bx)); y0, y1 = sorted((ay, by))
            if x1 - x0 > 3 and y1 - y0 > 3:
                save_label(lbl_p, lab[0] if lab else 0,
                           (x0 + x1) / 2 / W, (y0 + y1) / 2 / H, (x1 - x0) / W, (y1 - y0) / H)
                print(f"  수정: {img_p.name}  폭 {x1-x0:.0f}px")
            drag.update(p0=None, p1=None)
        elif key in (83, ord("."), ord("l")):
            i += 1
        elif key in (81, ord(","), ord("j")):
            i = max(0, i - 1)
        elif key in (ord("+"), ord("=")):
            zoom_state["pad_mult"] = max(0.2, zoom_state["pad_mult"] / 1.4)
        elif key == ord("-"):
            zoom_state["pad_mult"] = min(15.0, zoom_state["pad_mult"] * 1.4)
        elif key == ord("f"):
            zoom_state["full"] = not zoom_state["full"]
        elif key == ord("s"):
            _save_dropped(drop_file, dropped)
            print(f"  저장: 버림 {len(dropped)}건")

    cv2.destroyWindow("review")


# ── 트랙 단위 검수 ───────────────────────────────────────────────────────

def _load_tracks(sessions: list) -> list:
    """세션들의 `dataset_raw/meta/<세션>.jsonl`에서 `"kind": "track"` 레코드를 모은다.

    meta 파일이 없는 세션(`collect_live.py`/`extract_from_video.py` 산출물처럼 트랙
    개념이 없는 것)은 조용히 건너뛴다.
    """
    tracks = []
    for s in sessions:
        meta_p = ROOT / "meta" / f"{s.name}.jsonl"
        if not meta_p.exists():
            continue
        for line in meta_p.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get("kind") == "track":
                rec["_session"] = s.name
                tracks.append(rec)
    return tracks


def _track_items(rec: dict) -> list:
    """트랙 레코드 -> [(img_path, lbl_path), ...] (드릴다운·일괄 drop/keep용)."""
    img_dir = ROOT / "images" / rec["_session"]
    lbl_dir = ROOT / "labels" / rec["_session"]
    out = []
    for fname in rec.get("files", []):
        stem = pathlib.Path(fname).stem
        img_p = img_dir / fname
        lbl_p = lbl_dir / f"{stem}.txt"
        if img_p.exists() and lbl_p.exists():
            out.append((img_p, lbl_p))
    return out


def review_by_track(sessions: list, drop_file: pathlib.Path, dropped: set, args) -> None:
    import cv2

    tracks = _load_tracks(sessions)
    if not tracks:
        print("트랙 메타(dataset_raw/meta/*.jsonl)를 가진 세션이 없다 — "
              "collect_throws.py로 만든 세션인지 확인할 것. 프레임 단위(기본 모드)로는 볼 수 있다.")
        return
    print(f"{len(tracks)}개 트랙 검수 시작\n" + __doc__.split("[조작 — 트랙 단위")[1])

    cv2.namedWindow("review")
    i = 0
    while 0 <= i < len(tracks):
        rec = tracks[i]
        items = _track_items(rec)
        if not items:
            print(f"  ! 트랙 {rec.get('track_id')}: 이미지를 하나도 못 찾았다 — 건너뜀")
            i += 1
            continue

        img = cv2.imread(str(items[0][0]))
        if img is None:
            i += 1
            continue
        H, W = img.shape[:2]
        view = img.copy()

        centers = rec.get("centers") or []
        pts = [(int(cx * W), int(cy * H)) for cx, cy in centers]
        for k in range(1, len(pts)):
            cv2.line(view, pts[k - 1], pts[k], (0, 255, 255), 2)
        for p in pts:
            cv2.circle(view, p, 4, (0, 140, 255), -1)

        all_dropped = all(str(p) in dropped for p, _ in items)
        any_dropped = any(str(p) in dropped for p, _ in items)
        color = (0, 0, 255) if all_dropped else ((0, 165, 255) if any_dropped else (0, 255, 0))
        status = "DROP" if all_dropped else ("부분" if any_dropped else "keep")

        fit = rec.get("fit") or {}
        header1 = (f"[{i+1}/{len(tracks)}] {status}  {rec['_session']}  "
                   f"트랙 {rec.get('track_id')}")
        header2 = (f"n={fit.get('n', '?')}  span={rec.get('span_s', 0):.2f}s  "
                   f"z={fit.get('z_m', float('nan')):.2f}m  "
                   f"|v0|={fit.get('v0_mps', float('nan')):.2f}m/s  "
                   f"resid={fit.get('residual_px', float('nan')):.2f}px  "
                   f"저장 {rec.get('n_saved', len(items))}/{rec.get('n_observed', '?')}")
        cv2.putText(view, header1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(view, header2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1)
        cv2.imshow("review", view)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("d"):
            for p, _ in items:
                dropped.add(str(p))
            i += 1
        elif key == ord("k"):
            for p, _ in items:
                dropped.discard(str(p))
            i += 1
        elif key == ord("f"):
            cv2.destroyWindow("review")
            print(f"  트랙 {rec.get('track_id')} 프레임 단위로 진입 ({len(items)}장). "
                  f"Q로 트랙 목록에 복귀.")
            review_frame_items(items, drop_file, dropped, args,
                               title_suffix=f"  [트랙 {rec.get('track_id')} 드릴다운]")
            cv2.namedWindow("review")
        elif key in (83, ord("."), ord("l")):
            i += 1
        elif key in (81, ord(","), ord("j")):
            i = max(0, i - 1)
        elif key == ord("s"):
            _save_dropped(drop_file, dropped)
            print(f"  저장: 버림 {len(dropped)}건")

    cv2.destroyAllWindows()
    n_items = sum(len(_track_items(r)) for r in tracks)
    n_dropped = sum(1 for r in tracks for p, _ in _track_items(r) if str(p) in dropped)
    print(f"\n트랙 검수 종료. 프레임 기준 유지 {n_items - n_dropped}장 / 버림 {n_dropped}장")


# ── 진입점 ───────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="자동 라벨 검수")
    ap.add_argument("--session", default=None, help="특정 세션만. 생략하면 전체")
    ap.add_argument("--apply", action="store_true",
                    help="버림 표시된 파일을 실제로 삭제 (기본은 표시만)")
    ap.add_argument("--zoom", action="store_true",
                    help="bbox 주변만 잘라서 확대해서 보여준다 (작은 물체 판단용)")
    ap.add_argument("--by-track", action="store_true",
                    help="트랙 단위로 검수한다 (collect_throws.py 세션 전용, meta 필요)")
    args = ap.parse_args()

    img_root = ROOT / "images"
    if not img_root.is_dir():
        print(f"{img_root} 가 없다 — 먼저 수집할 것.")
        return 1
    sessions = [img_root / args.session] if args.session else sorted(
        d for d in img_root.iterdir() if d.is_dir())
    sessions = [s for s in sessions if s.is_dir()]
    if not sessions:
        print("검수할 세션이 없다.")
        return 1

    drop_file = ROOT / "dropped.json"
    dropped = set(json.loads(drop_file.read_text(encoding="utf-8"))) if drop_file.exists() else set()

    if args.by_track:
        review_by_track(sessions, drop_file, dropped, args)
    else:
        items = []
        for s in sessions:
            for img in sorted(s.glob("*.jpg")):
                lbl = ROOT / "labels" / s.name / f"{img.stem}.txt"
                if lbl.exists():
                    items.append((img, lbl))
        if not items:
            print("검수할 항목이 없다.")
            return 1
        print(f"{len(items)}장 검수 시작\n" + __doc__.split("[조작 — 프레임 단위")[1].split("[조작 — 트랙")[0])
        review_frame_items(items, drop_file, dropped, args)

    _save_dropped(drop_file, dropped)
    print(f"\n검수 종료. 전체 버림 {len(dropped)}건")

    if args.apply and dropped:
        for s in dropped:
            p = pathlib.Path(s)
            lbl = ROOT / "labels" / p.parent.name / f"{p.stem}.txt"
            p.unlink(missing_ok=True)
            lbl.unlink(missing_ok=True)
        drop_file.unlink(missing_ok=True)
        print(f"{len(dropped)}건 실제 삭제 완료")
    elif dropped:
        print("실제로 지우려면 --apply 를 붙여 다시 실행하라.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

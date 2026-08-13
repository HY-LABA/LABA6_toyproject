"""자동 라벨 검수 — **투척 단위로, 크롭만 모아서 본다.**

    python review_labels.py                  # 전체 세션
    python review_labels.py --session trash_gs_20260813_120000
    python review_labels.py --apply          # 버림 표시된 파일을 실제로 삭제

무엇을 판단하는가
-----------------
이 데이터셋의 목적은 **"쓰레기라는 물체를 인식하는 YOLO 학습"** 이다. 그래서 검수 질문은
하나다 — **"검출기가 잡은 게 물체가 맞나, 아니면 손·그림자·반사광·노이즈인가."**

왜 투척 단위인가
----------------
한 번의 투척에서 검출기는 물체를 잡았거나 못 잡았거나 **둘 중 하나**다. 같은 투척의
30프레임을 한 장씩 판정하는 건 같은 결정을 30번 반복하는 것이다. 투척 단위로 보면
1500장 훑기가 투척 50~100개 훑기로 줄어든다.

왜 크롭인가
-----------
전체 프레임에서 작은 박스를 눈으로 좇는 것보다, **bbox 안쪽만 잘라 나란히 놓고 보는 게
판별이 훨씬 쉽다.** 배경 잡동사니가 안 보이고, 한 화면에 투척 전체가 들어온다.
물체를 제대로 쫓았으면 크롭들이 서로 닮았고, 반사광이나 그림자를 쫓았으면 눈에 띄게 튄다.

[조작]
    → / ←      다음 / 이전 투척
    D          이 투척 전체 버림      K  이 투척 전체 유지
    마우스 클릭  그 프레임 하나만 토글 (손이 같이 잡힌 앞쪽 몇 장만 버릴 때)
    S          지금까지 결정 저장     Q  저장하고 종료
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib

ROOT = pathlib.Path("dataset_raw")


def load_label(p: pathlib.Path):
    """YOLO 라벨 -> (cls, cx, cy, w, h). 전부 0~1 정규화값."""
    txt = p.read_text(encoding="utf-8").strip()
    if not txt:
        return None
    parts = txt.split()[:5]
    return int(parts[0]), *(float(v) for v in parts[1:5])


def crop_bbox(img, label, pad: float):
    """라벨 주변을 pad 배로 넓혀 잘라낸다. 여백이 있어야 뭔지 알아보기 쉽다."""
    H, W = img.shape[:2]
    _, cx, cy, w, h = label
    half = max(w * W, h * H) * pad / 2          # 정사각으로 잘라야 리사이즈에 안 찌그러진다
    x0 = max(0, int(cx * W - half)); x1 = min(W, int(cx * W + half))
    y0 = max(0, int(cy * H - half)); y1 = min(H, int(cy * H + half))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    return img[y0:y1, x0:x1]


def group_by_throw(session_dir: pathlib.Path) -> list[list[tuple]]:
    """세션 하나를 투척별 (이미지, 라벨) 목록으로 나눈다.

    meta/{세션}.json 이 없으면(갱신 전에 모은 데이터) 세션 전체를 투척 하나로 본다.
    """
    pairs = []
    for img in sorted(session_dir.glob("*.jpg")):
        lbl = ROOT / "labels" / session_dir.name / f"{img.stem}.txt"
        if lbl.exists():
            pairs.append((img, lbl))
    if not pairs:
        return []

    meta_p = ROOT / "meta" / f"{session_dir.name}.json"
    if not meta_p.exists():
        return [pairs]
    throw_of = {m["name"]: m.get("throw", 0)
                for m in json.loads(meta_p.read_text(encoding="utf-8"))}

    groups: dict[int, list] = {}
    for img, lbl in pairs:
        groups.setdefault(throw_of.get(img.name, 0), []).append((img, lbl))
    return [groups[k] for k in sorted(groups)]


def build_montage(cv2, items, dropped: set[str], tile: int, cols: int, pad: float):
    """투척 하나의 크롭들을 격자로 붙인다. -> (몽타주 이미지, 타일별 파일명)."""
    import numpy as np

    tiles, names = [], []
    for img_p, lbl_p in items:
        img = cv2.imread(str(img_p))
        label = load_label(lbl_p)
        crop = crop_bbox(img, label, pad) if (img is not None and label) else None
        if crop is None:
            cell = np.zeros((tile, tile, 3), np.uint8)
            cv2.putText(cell, "??", (8, tile // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
        else:
            cell = cv2.resize(crop, (tile, tile), interpolation=cv2.INTER_NEAREST)
        if img_p.name in dropped:
            # 버림 표시: 붉게 덮고 테두리
            cell = cv2.addWeighted(cell, 0.45, np.full_like(cell, (0, 0, 200)), 0.55, 0)
            cv2.rectangle(cell, (0, 0), (tile - 1, tile - 1), (0, 0, 255), 3)
        cv2.putText(cell, str(len(tiles)), (3, 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 255, 255), 1)
        tiles.append(cell)
        names.append(img_p.name)

    rows = math.ceil(len(tiles) / cols)
    sheet = np.zeros((rows * tile, cols * tile, 3), np.uint8)
    for i, cell in enumerate(tiles):
        r, c = divmod(i, cols)
        sheet[r * tile:(r + 1) * tile, c * tile:(c + 1) * tile] = cell
    return sheet, names


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description="자동 라벨 검수 (투척 단위 크롭 검수)")
    ap.add_argument("--session", default=None, help="특정 세션만. 생략하면 전체")
    ap.add_argument("--tile", type=int, default=110, help="크롭 한 칸 크기(px)")
    ap.add_argument("--cols", type=int, default=10, help="한 줄에 몇 칸")
    ap.add_argument("--pad", type=float, default=2.0,
                    help="bbox 주변을 몇 배로 넓혀 자를지. 작으면 물체만, 크면 맥락까지")
    ap.add_argument("--apply", action="store_true",
                    help="버림 표시된 파일을 실제로 삭제 (기본은 표시만)")
    args = ap.parse_args()

    img_root = ROOT / "images"
    if not img_root.exists():
        print(f"{img_root} 가 없다.")
        return 1
    session_dirs = ([img_root / args.session] if args.session
                    else sorted(d for d in img_root.iterdir() if d.is_dir()))

    # (세션명, 투척 인덱스, 항목들)
    throws: list[tuple[str, int, list]] = []
    for sdir in session_dirs:
        for i, g in enumerate(group_by_throw(sdir)):
            throws.append((sdir.name, i, g))
    if not throws:
        print("검수할 항목이 없다.")
        return 1

    total_frames = sum(len(g) for _, _, g in throws)
    print(f"투척 {len(throws)}개 / 프레임 {total_frames}장")
    print(__doc__.split("[조작]")[1])

    drop_file = ROOT / "dropped.json"
    # 파일명만 저장한다 — 라파이(/)와 PC(\)의 경로 표기가 달라 경로로 적으면
    # prepare_dataset.py 가 하나도 못 걸러낸다.
    dropped = set(json.loads(drop_file.read_text(encoding="utf-8"))) if drop_file.exists() else set()
    dropped = {pathlib.PurePosixPath(s.replace("\\", "/")).name for s in dropped}

    def save_decisions() -> None:
        drop_file.write_text(json.dumps(sorted(dropped), indent=2), encoding="utf-8")

    click = {"idx": None}

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            click["idx"] = (y // args.tile) * args.cols + (x // args.tile)

    cv2.namedWindow("review")
    cv2.setMouseCallback("review", on_mouse)

    cache: dict[int, list[str]] = {}
    i = 0
    while 0 <= i < len(throws):
        session, tno, items = throws[i]
        sheet, names = build_montage(cv2, items, dropped, args.tile, args.cols, args.pad)
        cache[i] = names

        n_drop = sum(1 for nm in names if nm in dropped)
        bar = sheet.shape[1]
        header = f"[{i+1}/{len(throws)}] {session}  throw {tno}  " \
                 f"{len(names)}장 (버림 {n_drop})"
        pad_img = sheet.copy()
        cv2.rectangle(pad_img, (0, 0), (bar, 24), (0, 0, 0), -1)
        cv2.putText(pad_img, header, (6, 17), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        cv2.imshow("review", pad_img)

        key = cv2.waitKey(30) & 0xFF
        if click["idx"] is not None:
            j = click["idx"]
            click["idx"] = None
            if 0 <= j < len(names):
                nm = names[j]
                dropped.discard(nm) if nm in dropped else dropped.add(nm)
            continue                                  # 다시 그려서 결과를 바로 보여준다

        if key == ord("q"):
            break
        elif key == ord("d"):
            dropped.update(names); i += 1
        elif key == ord("k"):
            dropped.difference_update(names); i += 1
        elif key in (83, ord("."), ord("l")):         # →
            i += 1
        elif key in (81, ord(","), ord("j")):         # ←
            i = max(0, i - 1)
        elif key == ord("s"):
            save_decisions()
            print(f"  저장: 버림 {len(dropped)}장")

    cv2.destroyAllWindows()
    save_decisions()
    print(f"\n검수 종료. 버림 {len(dropped)}장 / 전체 {total_frames}장")

    if args.apply and dropped:
        n = 0
        for sdir in session_dirs:
            for img in list(sdir.glob("*.jpg")):
                if img.name in dropped:
                    (ROOT / "labels" / sdir.name / f"{img.stem}.txt").unlink(missing_ok=True)
                    img.unlink(missing_ok=True)
                    n += 1
        drop_file.unlink(missing_ok=True)
        print(f"{n}건 실제 삭제 완료 (이미지 + 라벨)")
    elif dropped:
        print("prepare_dataset.py 가 dropped.json 을 읽어 자동으로 제외한다.")
        print("파일까지 지우려면 --apply 를 붙여 다시 실행할 것.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""자동 라벨 검수 도구 — MOG2가 만든 bbox를 눈으로 확인하고 걸러낸다.

    python review_labels.py                      # 전체 세션
    python review_labels.py --session can_csi_20260802_101500
    python review_labels.py --zoom                # bbox 주변만 잘라서 확대

**왜 검수가 필요한가:** bbox 폭이 곧 거리 추정값이다. 자동 라벨은 초안일 뿐이고,
모션 블러 꼬리나 잘린 물체가 섞이면 σ_w가 커져 시스템 정확도가 그대로 떨어진다
(../docs/vision-pipeline.md 4장).

`--zoom`은 물체가 화면에서 작게 찍혀 전체 프레임으로는 bbox가 제대로 쳐졌는지
판단하기 어려울 때 쓴다 — bbox 주변만 잘라 확대해서 보여준다. `A`로 다시 그리는
좌표는 자동으로 원본 프레임 기준으로 환산되므로 확대 상태에서도 그대로 쓸 수 있다.

투명 물병처럼 라벨(스티커)만 작게 잡힌 경우, 자동 크롭 범위가 라벨 크기 기준이라
물병 전체가 화면 밖으로 나가서 크게 다시 그리기 힘들 수 있다 — 이때 `+`/`-`로
물체를 중심으로 잡은 채 확대/축소 배율을 조절할 것 (전체화면으로 벗어나는 게
아니라 같은 중심을 유지한 채 보이는 범위만 넓어진다/좁아진다).

⚠ `--apply`로 재검수할 때도 창이 너무 크면(=`--zoom` 없이 원본 해상도 그대로면)
일부 원격 데스크톱 환경에서 화살표 키가 창 밖(창 관리자)으로 새서 안 먹힐 수
있다 — 그러면 `--zoom`을 같이 붙이거나, 화살표 대신 `.`/`,`(또는 `l`/`j`)를 쓸 것.

[조작]
    → / ←      다음 / 이전 (안 먹히면 `.`/`,` 또는 `l`/`j`)
    +  / -      (--zoom일 때) 물체 중심으로 확대 / 축소
    F          (--zoom일 때) 전체화면 즉시 토글 (다시 F 누르면 확대 상태로 복귀)
    D          이 프레임 버림 (라벨+이미지 삭제 표시)
    K          유지 (기본값)
    A          bbox 수동 조정 모드 (드래그로 다시 그림)
    S          지금까지 결정 저장
    Q          저장하고 종료
"""

from __future__ import annotations

import argparse
import json
import pathlib

ROOT = pathlib.Path("dataset_raw")
CLASSES = ["trash"]   # capture_video.py와 동일해야 한다 (2026-08-10 단일 클래스로 전환)


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


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description="자동 라벨 검수")
    ap.add_argument("--session", default=None, help="특정 세션만. 생략하면 전체")
    ap.add_argument("--apply", action="store_true",
                    help="버림 표시된 파일을 실제로 삭제 (기본은 표시만)")
    ap.add_argument("--zoom", action="store_true",
                    help="bbox 주변만 잘라서 확대해서 보여준다 (작은 물체 판단용)")
    args = ap.parse_args()

    img_root = ROOT / "images"
    sessions = [img_root / args.session] if args.session else sorted(
        d for d in img_root.iterdir() if d.is_dir())
    items = []
    for s in sessions:
        for img in sorted(s.glob("*.jpg")):
            lbl = ROOT / "labels" / s.name / f"{img.stem}.txt"
            if lbl.exists():
                items.append((img, lbl))
    if not items:
        print("검수할 항목이 없다.")
        return 1
    print(f"{len(items)}장 검수 시작\n" + __doc__.split("[조작]")[1])

    drop_file = ROOT / "dropped.json"
    dropped = set(json.loads(drop_file.read_text(encoding="utf-8"))) if drop_file.exists() else set()

    i = 0
    drag = {"on": False, "p0": None, "p1": None}
    zoom_state = {"pad_mult": 1.0, "full": False}   # +/- 로 세밀 조절, F로 전체화면 토글

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
            """원본 프레임 좌표 -> 지금 보여주는(크롭+확대) 좌표."""
            return int((px - zx0) * zscale), int((py - zy0) * zscale)

        def to_orig(vx: float, vy: float) -> tuple[float, float]:
            """지금 보여주는 좌표 -> 원본 프레임 좌표 (드래그 결과 저장용)."""
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
        cv2.putText(view, f"[{i+1}/{len(items)}] {status}  {img_p.parent.name}{zoom_tag}",
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
            # 드래그 좌표는 화면(크롭+확대) 기준이므로 원본 프레임 좌표로 환산한다.
            (ax, ay) = to_orig(*drag["p0"])
            (bx, by) = to_orig(*drag["p1"])
            x0, x1 = sorted((ax, bx)); y0, y1 = sorted((ay, by))
            if x1 - x0 > 3 and y1 - y0 > 3:
                save_label(lbl_p, lab[0] if lab else 0,
                           (x0 + x1) / 2 / W, (y0 + y1) / 2 / H, (x1 - x0) / W, (y1 - y0) / H)
                print(f"  수정: {img_p.name}  폭 {x1-x0:.0f}px")
            drag.update(p0=None, p1=None)
        elif key in (83, ord("."), ord("l")):      # →
            i += 1
        elif key in (81, ord(","), ord("j")):      # ←
            i = max(0, i - 1)
        elif key in (ord("+"), ord("=")):
            # 확대 — 물체 중심은 그대로 두고 보이는 범위(패딩)만 줄인다
            zoom_state["pad_mult"] = max(0.2, zoom_state["pad_mult"] / 1.4)
        elif key == ord("-"):
            # 축소 — 물체가 화면 밖으로 나갈 만큼 큰 경우(예: 라벨만 잡힌 투명 물병)
            # 전체화면으로 점프하는 대신 물체 중심을 유지한 채 범위만 넓힌다
            zoom_state["pad_mult"] = min(15.0, zoom_state["pad_mult"] * 1.4)
        elif key == ord("f"):
            # 전체화면 즉시 토글 — +/-는 점진적 조절이라 화면 전체를 한번에 보고
            # 싶을 때(예: 물체 위치 자체를 놓친 것 같을 때)는 이게 더 빠르다.
            zoom_state["full"] = not zoom_state["full"]
        elif key == ord("s"):
            drop_file.write_text(json.dumps(sorted(dropped), indent=2), encoding="utf-8")
            print(f"  저장: 버림 {len(dropped)}건")

    cv2.destroyAllWindows()
    drop_file.write_text(json.dumps(sorted(dropped), indent=2), encoding="utf-8")
    print(f"\n검수 종료. 유지 {len(items)-len(dropped)}장 / 버림 {len(dropped)}장")

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

"""자동 라벨 검수 도구 — MOG2가 만든 bbox를 눈으로 확인하고 걸러낸다.

    python review_labels.py                      # 전체 세션
    python review_labels.py --session can_csi_20260802_101500

**왜 검수가 필요한가:** bbox 폭이 곧 거리 추정값이다. 자동 라벨은 초안일 뿐이고,
모션 블러 꼬리나 잘린 물체가 섞이면 σ_w가 커져 시스템 정확도가 그대로 떨어진다
(../docs/vision-pipeline.md 4장).

[조작]
    → / ←      다음 / 이전
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
CLASSES = ["trash"]   # capture_dataset.py와 동일해야 한다 (2026-08-10 단일 클래스로 전환)


def load_label(p: pathlib.Path):
    txt = p.read_text(encoding="utf-8").strip()
    if not txt:
        return None
    parts = txt.split()[:5]
    return int(parts[0]), *(float(v) for v in parts[1:5])


def save_label(p: pathlib.Path, cls: int, cx, cy, w, h) -> None:
    p.write_text(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n", encoding="utf-8")


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description="자동 라벨 검수")
    ap.add_argument("--session", default=None, help="특정 세션만. 생략하면 전체")
    ap.add_argument("--apply", action="store_true",
                    help="버림 표시된 파일을 실제로 삭제 (기본은 표시만)")
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
        view = img.copy()

        if lab:
            cls, cx, cy, w, h = lab
            x0, y0 = int((cx - w / 2) * W), int((cy - h / 2) * H)
            x1, y1 = int((cx + w / 2) * W), int((cy + h / 2) * H)
            is_dropped = str(img_p) in dropped
            color = (0, 0, 255) if is_dropped else (0, 255, 0)
            cv2.rectangle(view, (x0, y0), (x1, y1), color, 2)
            cv2.putText(view, f"{CLASSES[cls]}  w={x1-x0}px", (x0, max(20, y0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        if drag["p0"] and drag["p1"]:
            cv2.rectangle(view, drag["p0"], drag["p1"], (255, 200, 0), 2)

        status = "DROP" if str(img_p) in dropped else "keep"
        cv2.putText(view, f"[{i+1}/{len(items)}] {status}  {img_p.parent.name}",
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
            (ax, ay), (bx, by) = drag["p0"], drag["p1"]
            x0, x1 = sorted((ax, bx)); y0, y1 = sorted((ay, by))
            if x1 - x0 > 3 and y1 - y0 > 3:
                save_label(lbl_p, lab[0] if lab else 0,
                           (x0 + x1) / 2 / W, (y0 + y1) / 2 / H, (x1 - x0) / W, (y1 - y0) / H)
                print(f"  수정: {img_p.name}  폭 {x1-x0}px")
            drag.update(p0=None, p1=None)
        elif key in (83, ord("."), ord("l")):      # →
            i += 1
        elif key in (81, ord(","), ord("j")):      # ←
            i = max(0, i - 1)
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

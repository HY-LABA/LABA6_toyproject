"""자동 라벨 검수 — **궤적에서 벗어난 프레임을 먼저 찾아준다.**

    python review_labels.py                  # 의심 프레임만 (기본)
    python review_labels.py --all            # 전부 훑기
    python review_labels.py --screen-only    # 눈으로 안 보고 자동 선별만
    python review_labels.py --apply          # 버림 표시된 파일을 실제로 삭제

**무엇을 보는가가 바뀌었다.** 깊이가 중력에서 나오게 되면서 bbox **폭은 안 쓴다.**
궤적 피팅의 유일한 입력은 bbox **중심(u,v)** 이다. 그래서 검수 기준은
"박스가 물체를 잘 감쌌나"가 아니라 **"십자가 물체 한가운데 있나"** 다.

왜 그래도 검수가 필요한가:
  · MOG2가 손·그림자·반사를 잡으면 중심이 물체와 무관한 곳에 찍힌다
  · 모션 블러 꼬리를 한쪽만 포함하면 중심이 한 방향으로 밀린다
  · **이 오차는 재투영 잔차로 못 잡는다.** 편향이 계통적이면 궤적이 통째로
    평행이동해서 관측끼리는 여전히 일관되고, 착지점만 틀린다
    (어안 보정을 빠뜨렸을 때와 같은 실패 모드다)

자동 선별의 원리:
  한 번의 투척에서 물체는 화면에서도 **매끄러운 곡선**을 그린다. 손·그림자·반사는
  그 곡선을 따르지 않으므로 튄다. 투척마다 u(t), v(t)에 **로버스트 다항 피팅**을
  하고 잔차가 큰 프레임을 골라낸다. 사람은 그것만 보면 된다.

  차수는 프레임 수에 맞춰 자동으로 정한다. 어안 투영이라 궤적이 꽤 휘어서 2차로는
  부족하다 — 합성 데이터에서 2차는 정상/불량 분리가 2.8배였는데 5차는 36배였다.
  대신 프레임이 적은 투척에 높은 차수를 쓰면 노이즈까지 맞춰버려 불량이 묻힌다.

  (전체 프레임을 눈으로 훑는 건 1000장 넘어가면 현실적이지 않고, 지치면 검수 품질이
   떨어져 안 하느니만 못해진다.)

[조작]
    → / ←      다음 / 이전
    D          이 프레임 버림      K  유지
    A          드래그한 사각형으로 라벨 다시 그리기
    S          지금까지 결정 저장   Q  저장하고 종료
"""

from __future__ import annotations

import argparse
import json
import pathlib

import numpy as np

ROOT = pathlib.Path("dataset_raw")
CLASS_NAME = "trash"          # capture_dataset.py 와 동일해야 한다


def load_label(p: pathlib.Path):
    txt = p.read_text(encoding="utf-8").strip()
    if not txt:
        return None
    parts = txt.split()[:5]
    return int(parts[0]), *(float(v) for v in parts[1:5])


def save_label(p: pathlib.Path, cls: int, cx, cy, w, h) -> None:
    p.write_text(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n", encoding="utf-8")


# ── 자동 선별 ──────────────────────────────────────────────────────────

def fit_degree(n: int) -> int:
    """프레임 수에 맞는 다항 차수. 적으면 낮춰야 과적합으로 불량이 묻히지 않는다."""
    return min(5, max(2, n // 6))


def robust_poly_residuals(t: np.ndarray, y: np.ndarray, deg: int = 2,
                          rounds: int = 3) -> np.ndarray:
    """다항 피팅 잔차. 이상치가 피팅 자체를 끌어당기지 않게 반복적으로 잘라낸다.

    한 번만 피팅하면 튄 점이 곡선을 자기 쪽으로 당겨서 정작 자기 잔차는 작아진다.
    그래서 큰 잔차를 뺀 채 다시 피팅하기를 몇 번 반복한다.
    """
    n = len(t)
    if n < deg + 3:
        return np.zeros(n)                 # 점이 너무 적으면 판단하지 않는다
    keep = np.ones(n, dtype=bool)
    resid = np.zeros(n)
    for _ in range(rounds):
        if keep.sum() < deg + 2:
            break
        c = np.polyfit(t[keep], y[keep], deg)
        resid = y - np.polyval(c, t)
        # MAD 기반 산포 — 표준편차와 달리 이상치에 끌려가지 않는다
        s = 1.4826 * np.median(np.abs(resid[keep] - np.median(resid[keep])))
        if s < 1e-9:
            break
        keep = np.abs(resid) < 3.0 * s
    return resid


def session_wh(session: str, sample: pathlib.Path, cv2) -> tuple[int, int]:
    """세션의 이미지 크기. sessions.json 에 있으면 그걸 쓴다 — 크기 하나 알자고
    이미지를 디코드할 이유가 없고, 첫 장이 깨져 있어도 안 죽는다."""
    man = ROOT / "sessions.json"
    if man.exists():
        try:
            wh = json.loads(man.read_text(encoding="utf-8"))[session].get("image_size")
            if wh:
                return int(wh[0]), int(wh[1])
        except (KeyError, ValueError, TypeError):
            pass
    img = cv2.imread(str(sample))
    if img is None:
        raise SystemExit(f"이미지를 읽을 수 없다: {sample}\n"
                         f"  sessions.json 에 image_size 가 있으면 그걸 쓴다 — "
                         f"라파이에서 dataset_raw 를 통째로 가져왔는지 확인하라.")
    return img.shape[1], img.shape[0]


def screen_session(session: str, items: list[tuple[pathlib.Path, pathlib.Path]],
                   img_wh: tuple[int, int]) -> dict[str, float]:
    """세션 안의 프레임별 '궤적 이탈 거리(px)'. meta 가 없으면 빈 dict."""
    meta_p = ROOT / "meta" / f"{session}.json"
    if not meta_p.exists():
        return {}
    meta = {m["name"]: m for m in json.loads(meta_p.read_text(encoding="utf-8"))}
    W, H = img_wh

    # 투척(throw)별로 묶는다 — 한 세션에 여러 번 던지므로 통째로 피팅하면 안 된다
    throws: dict[int, list[tuple[float, float, float, str]]] = {}
    for img_p, lbl_p in items:
        m = meta.get(img_p.name)
        lab = load_label(lbl_p)
        if m is None or lab is None:
            continue
        _, cx, cy, _, _ = lab
        throws.setdefault(m["throw"], []).append((m["t"], cx * W, cy * H, img_p.name))

    out: dict[str, float] = {}
    for pts in throws.values():
        pts.sort()
        t = np.array([p[0] for p in pts])
        t = t - t[0]
        deg = fit_degree(len(pts))
        ru = robust_poly_residuals(t, np.array([p[1] for p in pts]), deg=deg)
        rv = robust_poly_residuals(t, np.array([p[2] for p in pts]), deg=deg)
        for i, p in enumerate(pts):
            out[p[3]] = float(np.hypot(ru[i], rv[i]))
    return out


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description="자동 라벨 검수 (중심 기준)")
    ap.add_argument("--session", default=None, help="특정 세션만. 생략하면 전체")
    ap.add_argument("--all", action="store_true",
                    help="자동 선별을 무시하고 전 프레임을 훑는다")
    ap.add_argument("--screen-only", action="store_true",
                    help="창을 띄우지 않고 자동 선별 결과만 출력")
    ap.add_argument("--thresh-px", type=float, default=8.0,
                    help="궤적 이탈이 이 값을 넘으면 의심 프레임 (기본 8px)")
    ap.add_argument("--apply", action="store_true",
                    help="버림 표시된 파일을 실제로 삭제 (기본은 표시만)")
    args = ap.parse_args()

    img_root = ROOT / "images"
    if not img_root.exists():
        print(f"{img_root} 가 없다.")
        return 1
    sessions = [img_root / args.session] if args.session else sorted(
        d for d in img_root.iterdir() if d.is_dir())

    # ── 수집 + 자동 선별 ────────────────────────────────────────────────
    items: list[tuple[pathlib.Path, pathlib.Path]] = []
    devi: dict[str, float] = {}
    no_meta: list[str] = []
    for s in sessions:
        pairs = []
        for img in sorted(s.glob("*.jpg")):
            lbl = ROOT / "labels" / s.name / f"{img.stem}.txt"
            if lbl.exists():
                pairs.append((img, lbl))
        if not pairs:
            continue
        d = screen_session(s.name, pairs, session_wh(s.name, pairs[0][0], cv2))
        if not d:
            no_meta.append(s.name)
        devi.update(d)
        items += pairs

    if not items:
        print("검수할 항목이 없다.")
        return 1

    if no_meta:
        print(f"⚠ meta 없는 세션 {len(no_meta)}개 — 자동 선별 불가, 전부 훑어야 한다:")
        print(f"    {', '.join(no_meta[:4])}{' ...' if len(no_meta) > 4 else ''}")
        print("    (capture_dataset.py 갱신 전에 모은 데이터다)")

    # 판정선은 고정값과 **실제 산포**의 큰 쪽으로 잡는다. 검출 노이즈가 크면
    # 고정 8px로는 정상 프레임까지 다 걸리고, 아주 깨끗하면 8px가 너무 느슨하다.
    cut = args.thresh_px
    vals = np.array(list(devi.values())) if devi else np.zeros(0)
    if devi:
        mad = 1.4826 * np.median(np.abs(vals - np.median(vals)))
        cut = max(args.thresh_px, 5.0 * mad)
    flagged = sorted((n for n, v in devi.items() if v > cut), key=lambda n: -devi[n])
    if devi:
        print(f"\n자동 선별: {len(devi)}장 중 **{len(flagged)}장** 의심 "
              f"({len(flagged)/len(devi)*100:.1f}%)   판정선 {cut:.1f}px"
              + (f" (실측 산포 기준 — --thresh-px {args.thresh_px:.0f} 보다 크다)"
                 if cut > args.thresh_px + 1e-9 else ""))
        print(f"  궤적 이탈: 중앙값 {np.median(vals):.1f}px  "
              f"90퍼센타일 {np.percentile(vals, 90):.1f}px  최대 {vals.max():.1f}px")
        for n in flagged[:10]:
            print(f"    {devi[n]:7.1f}px  {n}")
        if len(flagged) > 10:
            print(f"    ... 외 {len(flagged)-10}장")

    if args.screen_only:
        print("\n(--screen-only 라 창을 띄우지 않는다)")
        return 0

    # 의심 프레임만 보되, --all 이면 전부. 자동 선별이 안 된 세션은 항상 포함한다.
    if not args.all and devi:
        flag = set(flagged)
        items = [it for it in items
                 if it[0].name in flag or it[0].parent.name in set(no_meta)]
        if not items:
            print("\n✅ 의심 프레임이 없다. 전부 보려면 --all.")
            return 0
    print(f"\n{len(items)}장 검수 시작\n" + __doc__.split("[조작]")[1])

    drop_file = ROOT / "dropped.json"
    dropped = set(json.loads(drop_file.read_text(encoding="utf-8"))) if drop_file.exists() else set()
    dropped = {pathlib.PurePosixPath(s.replace("\\", "/")).name for s in dropped}

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
        is_dropped = img_p.name in dropped

        if lab:
            _, cx, cy, w, h = lab
            px, py = int(cx * W), int(cy * H)
            color = (0, 0, 255) if is_dropped else (0, 255, 0)
            # 박스는 참고용으로 얇게. **판단 대상은 중심 십자다.**
            cv2.rectangle(view, (int((cx - w/2)*W), int((cy - h/2)*H)),
                          (int((cx + w/2)*W), int((cy + h/2)*H)), color, 1)
            cv2.drawMarker(view, (px, py), (0, 255, 255), cv2.MARKER_CROSS, 28, 2)
            cv2.circle(view, (px, py), 3, (0, 255, 255), -1)

        dev = devi.get(img_p.name)
        tag = f"  이탈 {dev:.1f}px" if dev is not None else "  (meta 없음)"
        status = "DROP" if is_dropped else "keep"
        cv2.putText(view, f"[{i+1}/{len(items)}] {status}{tag}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(view, f"{img_p.parent.name} / {img_p.name}",
                    (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        if drag["p0"] and drag["p1"]:
            cv2.rectangle(view, drag["p0"], drag["p1"], (255, 200, 0), 2)
        cv2.imshow("review", view)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("d"):
            dropped.add(img_p.name); i += 1
        elif key == ord("k"):
            dropped.discard(img_p.name); i += 1
        elif key == ord("a") and drag["p0"] and drag["p1"]:
            (ax, ay), (bx, by) = drag["p0"], drag["p1"]
            x0, x1 = sorted((ax, bx)); y0, y1 = sorted((ay, by))
            if x1 - x0 > 3 and y1 - y0 > 3:
                save_label(lbl_p, lab[0] if lab else 0,
                           (x0 + x1) / 2 / W, (y0 + y1) / 2 / H, (x1 - x0) / W, (y1 - y0) / H)
                print(f"  수정: {img_p.name}  중심 ({(x0+x1)//2}, {(y0+y1)//2})")
            drag.update(p0=None, p1=None)
        elif key in (83, ord("."), ord("l")):      # →
            i += 1
        elif key in (81, ord(","), ord("j")):      # ←
            i = max(0, i - 1)
        elif key == ord("s"):
            drop_file.write_text(json.dumps(sorted(dropped), indent=2), encoding="utf-8")
            print(f"  저장: 버림 {len(dropped)}건")

    cv2.destroyAllWindows()
    # 파일명만 저장한다 — 라파이(/)와 PC(\)의 경로 표기가 달라 경로로 적으면
    # prepare_dataset.py 가 하나도 못 걸러낸다.
    drop_file.write_text(json.dumps(sorted(dropped), indent=2), encoding="utf-8")
    print(f"\n검수 종료. 버림 {len(dropped)}장")

    if args.apply and dropped:
        n = 0
        for s in sessions:
            for img in list(s.glob("*.jpg")):
                if img.name in dropped:
                    (ROOT / "labels" / s.name / f"{img.stem}.txt").unlink(missing_ok=True)
                    img.unlink(missing_ok=True)
                    n += 1
        drop_file.unlink(missing_ok=True)
        print(f"{n}건 실제 삭제 완료")
    elif dropped:
        print("실제로 지우려면 --apply 를 붙여 다시 실행하라.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""검출 파라미터를 **정답 박스에 맞춰 탐색**한다. (PC에서 실행)

    # 정답을 label_gt.py 로 그린 경우
    python tune_params.py --session gs_20260814_101500

    # 정답을 Roboflow / CVAT / LabelImg 로 그린 경우 (YOLO 포맷으로 내보내기)
    python tune_params.py --session gs_20260814_101500 --gt-dir ./exported_labels

    python tune_params.py --session gs_20260814_101500 --detector mog2 --min-precision 0.98

정답 포맷 두 가지를 다 읽는다
----------------------------
    gt.json     label_gt.py 가 만드는 형식
    --gt-dir    **YOLO txt 폴더** — Roboflow/CVAT/LabelImg 가 내보내는 표준 형식.
                프레임명과 같은 이름의 .txt 를 찾는다.
                  내용 있음 -> 물체 있음 (첫 줄의 박스를 정답으로 쓴다)
                  빈 파일   -> **물체 없음(negative)**
                  파일 없음 -> 라벨 안 함 -> 채점에서 제외

    ⚠ 외부 도구를 쓸 때 **"물체 없는 프레임"을 반드시 포함해서 내보낼 것.**
      Roboflow 는 "null/background" 이미지로 표시해야 빈 txt 가 나온다. 이게 없으면
      오검출률을 못 재고, "아무거나 다 잡는" 파라미터가 만점을 받는다.

찾은 파라미터를 어떻게 쓰나
--------------------------
마지막에 출력되는 옵션을 **--replay 와 함께** 쓰면 녹화본 전체에 적용된다:

    python capture_dataset.py --replay raw/<세션> --detector ref --min-area 80 ...

라벨 안 그린 프레임까지 전부 자동 라벨링돼서 dataset_raw/ 에 쌓인다.
그다음은 기존 흐름과 같다 — review_labels.py -> prepare_dataset.py -> train_yolo.py

코랩에서 돌리려면
----------------
raw/<세션>/ 폴더와 함께 **camera.py, frame_saver.py, capture_dataset.py** 를 올린다
(tune_params 가 capture_dataset 을 import 하고, 그게 나머지를 끌어온다).
picamera2 는 함수 안에서만 import 하므로 코랩에서도 문제없다.

무엇을 하는가
-------------
`capture_dataset.py` 의 `min_area`, `diff_threshold` 같은 값들은 지금 전부 **추측**이다.
이 스크립트는 녹화본을 재생하면서 파라미터 조합을 하나씩 대입해보고, `label_gt.py` 로
그려둔 정답과 얼마나 맞는지 채점한다. **검출기가 뭘 배우는 게 아니라, 고정된 숫자
조합을 전부 대입해보고 제일 나은 걸 고르는 것**이다(그리드 서치).

채점은 `capture_dataset.BlobFinder` 를 **그대로 import 해서** 한다. 여기서 좋게 나온
값이 실제 수집에서도 똑같이 동작한다는 걸 보장하기 위해서다.

왜 정밀도 우선인가
------------------
두 오류의 비용이 다르다.

  · 놓친 프레임(FN)  -> 데이터가 조금 줄어든다
  · 잘못 잡은 프레임(FP) -> **YOLO에게 틀린 걸 가르친다.** 반사광을 쓰레기로 배우면
    실제 구동에서 오검출이 나고, 그 오검출이 궤적 피팅에 그대로 들어간다

그래서 F1 최대화가 아니라 **"정밀도 X% 이상인 조합 중 재현율이 가장 높은 것"** 을 찾는다.

과적합 방지
-----------
투척 몇 개를 **홀드아웃**으로 빼고 나머지로만 탐색한 뒤, 홀드아웃에서 성능이 유지되는지
확인한다. 유지가 안 되면 그 파라미터는 "그 방, 그 조명"에만 맞춘 값이다.
"""

from __future__ import annotations

import argparse
import itertools
import json
import pathlib
import types

ROOT = pathlib.Path("raw")

# 탐색 범위. 조합 수 = 곱. 기본 5*4*3*2*3 = 360
GRID = {
    "diff_threshold": [15, 20, 25, 30, 40],     # ref 전용 (밝기 차이 0~255)
    "var_threshold": [16, 25, 36, 50],          # mog2 전용 (분산 기준 거리)
    "min_area": [40, 80, 150, 300],
    "close_iters": [0, 1, 2],
    "max_area_frac": [0.10, 0.25],
    "min_fill": [0.20, 0.30, 0.45],
}
FIXED = {"min_aspect": 0.2, "max_aspect": 5.0, "edge_margin": 4,
         "history": 300, "normalize": True}


def load_gt(sdir: pathlib.Path, gt_dir: str | None) -> dict:
    """정답을 {프레임명: [cx,cy,w,h] 또는 None} 으로 읽는다.

    None 은 "물체 없음(negative)"이고, **키 자체가 없으면 "라벨 안 함"** 이다.
    이 구분이 중요하다 — negative 가 있어야 오검출률을 잴 수 있다.
    """
    if gt_dir is None:
        p = sdir / "gt.json"
        return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}

    d = pathlib.Path(gt_dir)
    if not d.is_dir():
        raise SystemExit(f"--gt-dir 가 폴더가 아니다: {d}")
    gt: dict = {}
    for txt in sorted(d.glob("*.txt")):
        body = txt.read_text(encoding="utf-8").strip()
        name = f"{txt.stem}.jpg"
        if not body:
            gt[name] = None                       # 빈 파일 = 물체 없음
            continue
        parts = body.splitlines()[0].split()      # "cls cx cy w h"
        if len(parts) >= 5:
            gt[name] = [float(v) for v in parts[1:5]]
    n_pos = sum(1 for v in gt.values() if v is not None)
    print(f"YOLO txt 정답 {len(gt)}건 읽음 (물체 {n_pos} / 없음 {len(gt)-n_pos})")
    if len(gt) - n_pos == 0:
        print("⚠ '물체 없음' 라벨이 하나도 없다 — 오검출률을 못 잰다. "
              "빈 프레임을 background 로 표시해 함께 내보낼 것.")
    return gt


def iou(a, b) -> float:
    """두 박스(cx, cy, w, h 정규화)의 겹침 비율."""
    ax0, ay0, ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx0, by0, bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    iy = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = ix * iy
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 1e-12 else 0.0


def evaluate(cv2, cd, params, frames, gt, sdir, W, H, iou_thresh):
    """파라미터 하나로 전체를 재생하고 (TP, FP, FN, IoU합) 을 센다."""
    args = types.SimpleNamespace(**{**FIXED, **params})
    finder = cd.BlobFinder(cv2, args, W * H)

    # 워밍업으로 배경을 만든다. mog2 는 이후에도 순서대로 재생해야 하므로 건너뛸 수 없다.
    for f in frames:
        if not f["warmup"]:
            break
        img = cv2.imread(str(sdir / "frames" / f["name"]))
        if img is not None:
            finder.learn(img)
    finder.finish_learning()

    sequential = params["detector"] == "mog2"     # ref 는 상태가 없어 건너뛰어도 된다
    tp = fp = fn = 0
    iou_sum = 0.0
    for f in frames:
        if f["warmup"]:
            continue
        name = f["name"]
        labeled = name in gt
        if not labeled and not sequential:
            continue                              # 채점 안 할 프레임은 읽지도 않는다
        img = cv2.imread(str(sdir / "frames" / name))
        if img is None:
            continue
        _, box, _ = finder(img)
        if not labeled:
            continue

        truth = gt[name]
        if box is None:
            fn += 1 if truth is not None else 0
            continue
        x, y, w, h = box
        pred = [(x + w / 2) / W, (y + h / 2) / H, w / W, h / H]
        if truth is None:
            fp += 1                               # 아무것도 없는데 잡았다
        else:
            v = iou(pred, truth)
            if v >= iou_thresh:
                tp += 1
                iou_sum += v
            else:
                fp += 1                           # 잡긴 했는데 엉뚱한 데를 잡았다
                fn += 1
    return tp, fp, fn, iou_sum


def score(tp, fp, fn):
    prec = tp / (tp + fp) if tp + fp else 0.0
    rec = tp / (tp + fn) if tp + fn else 0.0
    return prec, rec


def main() -> int:
    import cv2
    import capture_dataset as cd

    ap = argparse.ArgumentParser(description="검출 파라미터 그리드 서치")
    ap.add_argument("--session", required=True)
    ap.add_argument("--gt-dir", default=None,
                    help="YOLO txt 정답 폴더 (Roboflow/CVAT/LabelImg 내보내기). "
                         "생략하면 label_gt.py 가 만든 gt.json 을 쓴다")
    ap.add_argument("--detector", choices=["ref", "mog2"], default="ref")
    ap.add_argument("--iou", type=float, default=0.3,
                    help="이 이상 겹쳐야 '맞췄다'로 본다. 물체가 작아 0.5는 가혹하다")
    ap.add_argument("--min-precision", type=float, default=0.95,
                    help="이 정밀도 미만인 조합은 후보에서 제외한다")
    ap.add_argument("--holdout", type=float, default=0.3,
                    help="정답 중 이 비율을 검증용으로 빼둔다 (과적합 확인)")
    ap.add_argument("--top", type=int, default=10, help="상위 몇 개를 출력할지")
    args = ap.parse_args()

    sdir = ROOT / args.session
    meta_p = sdir / "meta.json"
    if not meta_p.exists():
        print(f"{meta_p} 가 없다. record_raw.py 를 먼저 돌릴 것.")
        return 1
    meta = json.loads(meta_p.read_text(encoding="utf-8"))
    gt_all = load_gt(sdir, args.gt_dir)
    if not gt_all:
        print("정답이 없다. label_gt.py 를 돌리거나 --gt-dir 로 YOLO txt 폴더를 줄 것.")
        return 1
    W, H = meta["image_size"]
    frames = meta["frames"]

    # 홀드아웃: 시간순 뒤쪽을 통째로 뺀다. 무작위로 빼면 같은 투척이 양쪽에 들어가
    # 검증이 무의미해진다 (prepare_dataset.py 가 세션 단위로 나누는 것과 같은 이유).
    names = [f["name"] for f in frames if f["name"] in gt_all]
    cut = int(len(names) * (1 - args.holdout))
    tune_gt = {n: gt_all[n] for n in names[:cut]}
    hold_gt = {n: gt_all[n] for n in names[cut:]}
    n_pos = sum(1 for v in tune_gt.values() if v is not None)
    print(f"정답 {len(gt_all)}건 -> 탐색 {len(tune_gt)}건(물체 {n_pos}) / "
          f"홀드아웃 {len(hold_gt)}건")
    if n_pos < 20:
        print("⚠ 탐색용 '물체 있음'이 20건 미만이다. 결과를 신뢰하기 어렵다.")

    # 검출기에 따라 안 쓰는 축은 값 하나로 고정해 조합 수를 줄인다
    grid = dict(GRID)
    grid.pop("var_threshold" if args.detector == "ref" else "diff_threshold")
    keys = list(grid)
    combos = list(itertools.product(*(grid[k] for k in keys)))
    print(f"조합 {len(combos)}개 x 프레임 {len(tune_gt)}장 재생 시작...\n")

    results = []
    for i, values in enumerate(combos, 1):
        params = dict(zip(keys, values))
        params["detector"] = args.detector
        params.setdefault("diff_threshold", 25.0)
        params.setdefault("var_threshold", 25.0)
        tp, fp, fn, iou_sum = evaluate(cv2, cd, params, frames, tune_gt, sdir, W, H, args.iou)
        prec, rec = score(tp, fp, fn)
        results.append((prec, rec, iou_sum / tp if tp else 0.0, params, (tp, fp, fn)))
        if i % 20 == 0 or i == len(combos):
            print(f"  {i}/{len(combos)}")

    ok = [r for r in results if r[0] >= args.min_precision]
    if not ok:
        best_prec = max(r[0] for r in results)
        print(f"\n⚠ 정밀도 {args.min_precision:.0%} 를 넘는 조합이 없다 "
              f"(최고 {best_prec:.1%}). --min-precision 을 낮추거나, 조명·검출기부터 "
              f"손봐야 한다.")
        ok = results
    ok.sort(key=lambda r: (-r[1], -r[0]))          # 재현율 우선, 동률이면 정밀도

    print(f"\n{'정밀도':>7}{'재현율':>8}{'평균IoU':>9}   파라미터")
    print("-" * 78)
    for prec, rec, miou, params, (tp, fp, fn) in ok[:args.top]:
        show = {k: params[k] for k in keys}
        print(f"{prec:7.1%}{rec:8.1%}{miou:9.2f}   {show}  (TP{tp} FP{fp} FN{fn})")

    # 홀드아웃 검증
    best = ok[0]
    print(f"\n=== 홀드아웃 검증 (탐색에 안 쓴 {len(hold_gt)}건) ===")
    if hold_gt:
        tp, fp, fn, iou_sum = evaluate(cv2, cd, best[3], frames, hold_gt, sdir, W, H, args.iou)
        hp, hr = score(tp, fp, fn)
        print(f"  탐색셋   정밀도 {best[0]:.1%}  재현율 {best[1]:.1%}")
        print(f"  홀드아웃 정밀도 {hp:.1%}  재현율 {hr:.1%}   (TP{tp} FP{fp} FN{fn})")
        if best[1] - hr > 0.15 or best[0] - hp > 0.15:
            print("  ⚠ 홀드아웃에서 크게 떨어진다 — 이 방·이 조명에만 맞춘 값일 수 있다.")
        else:
            print("  ✅ 유지된다.")
    else:
        print("  (홀드아웃이 비었다 — 정답을 더 그리거나 --holdout 을 올릴 것)")

    print("\n=== capture_dataset.py 실행 옵션 ===")
    p = best[3]
    opts = [f"--detector {p['detector']}"]
    for k in keys:
        opts.append(f"--{k.replace('_', '-')} {p[k]}")
    print("  python capture_dataset.py --label trash " + " ".join(opts))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

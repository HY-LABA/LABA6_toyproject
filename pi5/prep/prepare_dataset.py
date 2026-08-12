"""수집한 데이터를 YOLO 학습 폴더 구조로 정리한다. (PC에서 실행)

    python prepare_dataset.py --out dataset

⚠ **분할은 프레임이 아니라 세션(투척 회차) 단위로 한다.**
   같은 세션의 프레임은 서로 거의 동일하다. 프레임 단위로 무작위 분할하면 train과 val에
   사실상 같은 이미지가 들어가 **검증 점수가 부풀려지고 과적합을 못 잡는다.**
   (../docs/vision-pipeline.md 3장)

라파이에서 PC로 옮길 때:
    rsync -av pi@raspberrypi:~/LABA6_toyproject/prep/dataset_raw/ ./dataset_raw/
"""

from __future__ import annotations

import argparse
import collections
import json
import pathlib
import random
import shutil

RAW = pathlib.Path("dataset_raw")
# 단일 클래스 (2026-08-10). capture_dataset.py의 CLASSES와 반드시 같아야 한다.
CLASSES = ["trash"]


def class_of(session_name: str) -> str:
    # 세션명 = f"{label}_{camera}_{stamp}". label에 "_"가 들어갈 수 있으므로
    # 앞 토큰 하나만 자르지 않고 CLASSES와 매칭한다.
    for c in CLASSES:
        if session_name.startswith(c + "_"):
            return c
    return session_name.split("_")[0]


def main() -> int:
    ap = argparse.ArgumentParser(description="YOLO 데이터셋 준비")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--test-frac", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--copy", action="store_true", help="심볼릭 링크 대신 파일 복사")
    args = ap.parse_args()

    img_root = RAW / "images"
    if not img_root.exists():
        print(f"{img_root} 가 없다. 라파이에서 dataset_raw 를 가져왔는지 확인하라.")
        return 1

    dropped = set()
    dp = RAW / "dropped.json"
    if dp.exists():
        dropped = set(json.loads(dp.read_text(encoding="utf-8")))
        print(f"검수에서 버린 {len(dropped)}장 제외")

    # 세션별 파일 수집
    sessions: dict[str, list[pathlib.Path]] = {}
    for sdir in sorted(d for d in img_root.iterdir() if d.is_dir()):
        files = [p for p in sorted(sdir.glob("*.jpg")) if str(p) not in dropped]
        if files:
            sessions[sdir.name] = files
    if not sessions:
        print("사용할 이미지가 없다.")
        return 1

    # 클래스별로 세션을 나눠 각 split에 고르게 들어가게 한다
    by_class: dict[str, list[str]] = collections.defaultdict(list)
    for name in sessions:
        by_class[class_of(name)].append(name)

    rng = random.Random(args.seed)
    split_of: dict[str, str] = {}
    for cls, names in by_class.items():
        names = names[:]
        rng.shuffle(names)
        n = len(names)
        n_val = max(1, round(n * args.val_frac)) if n > 2 else (1 if n > 1 else 0)
        n_test = max(1, round(n * args.test_frac)) if n > 3 else 0
        for i, nm in enumerate(names):
            split_of[nm] = "val" if i < n_val else ("test" if i < n_val + n_test else "train")

    out = pathlib.Path(args.out)
    for sp in ("train", "val", "test"):
        (out / "images" / sp).mkdir(parents=True, exist_ok=True)
        (out / "labels" / sp).mkdir(parents=True, exist_ok=True)

    counts: dict[tuple[str, str], int] = collections.Counter()
    for name, files in sessions.items():
        sp = split_of[name]
        cls = class_of(name)
        for img in files:
            lbl = RAW / "labels" / name / f"{img.stem}.txt"
            if not lbl.exists():
                continue
            for src, sub in ((img, "images"), (lbl, "labels")):
                dst = out / sub / sp / src.name
                if dst.exists():
                    dst.unlink()
                if args.copy:
                    shutil.copy2(src, dst)
                else:
                    try:
                        dst.symlink_to(src.resolve())
                    except OSError:          # 윈도우 권한 문제 등
                        shutil.copy2(src, dst)
            counts[(sp, cls)] += 1

    yaml_path = out / "dataset.yaml"
    yaml_path.write_text(
        f"path: {out.resolve().as_posix()}\n"
        "train: images/train\nval: images/val\ntest: images/test\n\nnames:\n"
        + "".join(f"  {i}: {c}\n" for i, c in enumerate(CLASSES)),
        encoding="utf-8")

    print(f"\n세션 {len(sessions)}개 -> {out}")
    print(f"{'':10s} " + "".join(f"{c:>12s}" for c in CLASSES) + f"{'합계':>8s}")
    for sp in ("train", "val", "test"):
        row = [counts[(sp, c)] for c in CLASSES]
        print(f"{sp:10s} " + "".join(f"{v:12d}" for v in row) + f"{sum(row):8d}")
    print("\n세션 배정:")
    for sp in ("train", "val", "test"):
        names = [n for n, s in split_of.items() if s == sp]
        print(f"  {sp:5s} {len(names):2d}개  {', '.join(names[:3])}{' ...' if len(names) > 3 else ''}")

    total = sum(counts.values())
    if total < 800:
        print(f"\n⚠ 총 {total}장. 목표는 1300~1800장이다 (../docs/vision-pipeline.md 3장)")
    for cls in CLASSES:
        if counts[("val", cls)] == 0:
            print(f"⚠ val에 '{cls}' 세션이 없다 — 세션을 더 모아라")
    print(f"\n다음: python train_yolo.py --data {yaml_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

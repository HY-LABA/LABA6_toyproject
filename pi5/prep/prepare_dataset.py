"""수집한 데이터를 YOLO 학습 폴더 구조로 정리한다. (PC에서 실행)

    python prepare_dataset.py --out dataset

클래스가 `trash` 하나뿐이라 이 스크립트가 할 일은 **세션을 train/val/test로 가르고
파일을 배치하는 것**뿐이다. 한 세션에 캔·페트병·종이컵이 섞여 들어와도 상관없다.

⚠ **분할은 프레임이 아니라 세션(투척 회차) 단위로 한다. 이게 이 파일의 존재 이유다.**
   같은 세션의 프레임은 서로 거의 동일하다. 프레임 단위로 무작위 분할하면 train과 val에
   사실상 같은 이미지가 들어가 **검증 점수가 부풀려지고 과적합을 못 잡는다.**
   (../../docs/vision-pipeline.md 3장)

라파이에서 PC로 옮길 때:
    rsync -av pi@raspberrypi:~/LABA6_toyproject/pi5/prep/dataset_raw/ ./dataset_raw/
"""

from __future__ import annotations

import argparse
import json
import pathlib
import random
import shutil

RAW = pathlib.Path("dataset_raw")
CLASS_NAME = "trash"       # capture_dataset.py의 CLASSES와 같아야 한다 (라벨 id는 항상 0)


def load_dropped() -> set[str]:
    """review_labels.py가 버린 이미지의 **파일명** 집합.

    dropped.json에는 경로 문자열이 들어 있는데, 검수는 라파이(`/`)에서 하고 정리는
    PC(`\\`)에서 하므로 경로를 그대로 비교하면 하나도 안 걸린다.
    파일명이 `{세션}_{번호}.jpg`로 전역 유일하므로 **파일명으로 맞춘다.**
    """
    dp = RAW / "dropped.json"
    if not dp.exists():
        return set()
    names = {pathlib.PurePosixPath(s.replace("\\", "/")).name
             for s in json.loads(dp.read_text(encoding="utf-8"))}
    print(f"검수에서 버린 {len(names)}장 제외")
    return names


def split_sessions(names: list[str], val_frac: float, test_frac: float,
                   seed: int) -> dict[str, str]:
    """세션 목록 -> {세션명: 'train'|'val'|'test'}."""
    names = sorted(names)                 # 입력 순서에 흔들리지 않게
    random.Random(seed).shuffle(names)
    n = len(names)

    n_val = max(1, round(n * val_frac)) if n >= 2 else 0
    n_test = max(1, round(n * test_frac)) if n >= 4 else 0
    # train이 비면 학습 자체가 안 된다. val/test를 줄여서라도 train을 남긴다.
    n_val = min(n_val, n - 1) if n >= 2 else 0
    n_test = min(n_test, n - 1 - n_val)

    out = {}
    for i, nm in enumerate(names):
        out[nm] = "val" if i < n_val else ("test" if i < n_val + n_test else "train")
    return out


def place(src: pathlib.Path, dst: pathlib.Path, copy: bool) -> None:
    if dst.exists():
        dst.unlink()
    if copy:
        shutil.copy2(src, dst)
        return
    try:
        dst.symlink_to(src.resolve())
    except OSError:                       # 윈도우는 개발자 모드가 아니면 심볼릭 링크 불가
        shutil.copy2(src, dst)


def main() -> int:
    ap = argparse.ArgumentParser(description="YOLO 데이터셋 준비 (단일 클래스)")
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

    dropped = load_dropped()

    # 세션별 (이미지, 라벨) 쌍 수집. 라벨이 없는 이미지는 학습에 못 쓰므로 버린다.
    sessions: dict[str, list[tuple[pathlib.Path, pathlib.Path]]] = {}
    missing_label = 0
    for sdir in sorted(d for d in img_root.iterdir() if d.is_dir()):
        pairs = []
        for img in sorted(sdir.glob("*.jpg")):
            if img.name in dropped:
                continue
            lbl = RAW / "labels" / sdir.name / f"{img.stem}.txt"
            if lbl.exists():
                pairs.append((img, lbl))
            else:
                missing_label += 1
        if pairs:
            sessions[sdir.name] = pairs

    if not sessions:
        print("사용할 이미지가 없다.")
        return 1
    if missing_label:
        print(f"⚠ 라벨 없는 이미지 {missing_label}장 건너뜀")

    split_of = split_sessions(list(sessions), args.val_frac, args.test_frac, args.seed)

    out = pathlib.Path(args.out)
    for sp in ("train", "val", "test"):
        (out / "images" / sp).mkdir(parents=True, exist_ok=True)
        (out / "labels" / sp).mkdir(parents=True, exist_ok=True)

    counts = {"train": 0, "val": 0, "test": 0}
    for name, pairs in sessions.items():
        sp = split_of[name]
        for img, lbl in pairs:
            place(img, out / "images" / sp / img.name, args.copy)
            place(lbl, out / "labels" / sp / lbl.name, args.copy)
        counts[sp] += len(pairs)

    (out / "dataset.yaml").write_text(
        f"path: {out.resolve().as_posix()}\n"
        "train: images/train\nval: images/val\ntest: images/test\n\n"
        f"names:\n  0: {CLASS_NAME}\n",
        encoding="utf-8")

    # ── 결과 ────────────────────────────────────────────────────────────
    total = sum(counts.values())
    print(f"\n세션 {len(sessions)}개, 이미지 {total}장 -> {out}")
    print(f"\n{'분할':<7} {'세션':>5} {'이미지':>7} {'비율':>7}")
    for sp in ("train", "val", "test"):
        ses = [n for n, s in split_of.items() if s == sp]
        print(f"{sp:<7} {len(ses):5d} {counts[sp]:7d} {counts[sp]/total*100:6.1f}%")
        if ses:
            print(f"        {', '.join(sorted(ses)[:3])}"
                  f"{' ...' if len(ses) > 3 else ''}")

    # 세션 수로 나누므로 세션마다 장수가 다르면 실제 비율이 목표에서 벗어난다.
    want = args.val_frac * 100
    got = counts["val"] / total * 100
    if counts["val"] and abs(got - want) > 10:
        print(f"\n⚠ val 실제 비율 {got:.0f}% (목표 {want:.0f}%) — 세션별 장수 편차가 크다."
              f" --seed 를 바꿔 다시 나눠보라")
    if counts["val"] == 0:
        print("\n⚠ val이 비었다 — 세션이 2개 이상 있어야 검증이 가능하다")
    if total < 800:
        print(f"\n⚠ 총 {total}장. 목표는 1300~1800장이다 (../../docs/vision-pipeline.md 3장)")

    # 카메라가 다르면 화각·왜곡이 달라 같이 학습하면 안 된다
    cams = {n.split("_")[1] for n in sessions if len(n.split("_")) >= 3}
    if len(cams) > 1:
        print(f"\n⚠ 카메라가 섞였다: {sorted(cams)}"
              f"\n  화각·왜곡·셔터가 다르면 물체가 다르게 찍힌다. 한 카메라만 쓸 것.")

    print(f"\n다음: python train_yolo.py --data {out / 'dataset.yaml'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

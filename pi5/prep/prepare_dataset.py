"""수집한 데이터를 YOLO 학습 폴더 구조로 정리한다.

    python prepare_dataset.py                        # 여기서 바로 학습할 때
    python prepare_dataset.py --portable             # 다른 데(코랩 등)로 옮길 때

⚠ **분할은 프레임이 아니라 세션(투척 회차) 단위로 한다.**
   같은 세션의 프레임은 서로 거의 동일하다. 프레임 단위로 무작위 분할하면 train과 val에
   사실상 같은 이미지가 들어가 **검증 점수가 부풀려지고 과적합을 못 잡는다.**
   (../../docs/vision-pipeline.md 3장)

┌─ 다른 환경으로 옮길 거면 --portable ──────────────────────────────────────┐
│ 기본 동작은 **심볼릭 링크**라 dataset/ 만 압축해 옮기면 링크가 깨진다.     │
│ (tar 는 링크를 링크째로 담는다. zip 은 따라가서 내용을 담는다 — 도구마다   │
│  달라서 믿을 게 못 된다.)                                                 │
│                                                                          │
│ 그리고 dataset.yaml 의 `path:` 가 **이 컴퓨터의 절대경로**로 박힌다.       │
│ 코랩에 올리면 그 경로가 없어서 학습이 시작도 안 된다.                      │
│                                                                          │
│ `--portable` 이 둘 다 처리한다 = 실제 복사 + path 줄 생략.                 │
└──────────────────────────────────────────────────────────────────────────┘

라파이 -> 코랩 전체 흐름:

    # 라파이에서
    python prepare_dataset.py --portable
    tar czf dataset.tar.gz dataset

    # 코랩에서 dataset.tar.gz 업로드 후
    !tar xzf dataset.tar.gz
    !yolo train model=yolov8n.pt data=dataset/dataset.yaml imgsz=640 epochs=100

라파이에서 PC로 원본을 가져올 때:
    rsync -av pi@raspberrypi:~/LABA6_toyproject/pi5/prep/dataset_raw/ ./dataset_raw/
"""

from __future__ import annotations

import argparse
import collections
import json
import pathlib
import random
import shutil
import sys

# 윈도우 기본 콘솔은 cp949라 "⚠" 한 글자에 UnicodeEncodeError로 죽는다.
# 이 스크립트는 PC에서 도는 쪽이라 실제로 터진다 (라파이는 UTF-8이라 안 터짐).
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

RAW = pathlib.Path("dataset_raw")
# 단일 클래스 (2026-08-10). capture_dataset.py / extract_from_video.py 와 같아야 한다.
CLASS_NAME = "trash"

# 세션 이름에서 클래스를 알아내던 코드가 있었는데 지웠다. 이유 두 가지:
#   · 클래스가 하나뿐이라 알아낼 게 없다
#   · capture_video.py 가 세션을 "session_YYYYMMDD_HHMMSS" 로 만들면서
#     "trash_" 로 시작하지 않게 됐다 — 그래서 통계표가 전부 0으로 찍혔다


def main() -> int:
    ap = argparse.ArgumentParser(description="YOLO 데이터셋 준비")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--test-frac", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--copy", action="store_true", help="심볼릭 링크 대신 파일 복사")
    ap.add_argument("--portable", action="store_true",
                    help="다른 환경(코랩 등)으로 옮길 데이터셋을 만든다. "
                         "--copy 를 켜고 dataset.yaml 의 path 를 상대경로로 쓴다")
    ap.add_argument("--path", default=None,
                    help="dataset.yaml 에 적을 path 값을 직접 지정 "
                         "(예: /content/dataset). 생략하면 이 컴퓨터의 절대경로")
    args = ap.parse_args()
    if args.portable:
        args.copy = True

    img_root = RAW / "images"
    if not img_root.exists():
        print(f"{img_root} 가 없다. 라파이에서 dataset_raw 를 가져왔는지 확인하라.")
        return 1

    # 검수는 라파이(경로 구분자 /), 정리는 PC(\) 에서 하므로 경로 문자열을 그대로
    # 비교하면 **하나도 안 걸러진다.** 파일명이 세션 안에서 유일하므로 파일명으로 맞춘다.
    dropped = set()
    dp = RAW / "dropped.json"
    if dp.exists():
        dropped = {pathlib.PurePosixPath(s.replace("\\", "/")).name
                   for s in json.loads(dp.read_text(encoding="utf-8"))}
        print(f"검수에서 버린 {len(dropped)}장 제외")

    # 세션별 파일 수집
    sessions: dict[str, list[pathlib.Path]] = {}
    for sdir in sorted(d for d in img_root.iterdir() if d.is_dir()):
        files = [p for p in sorted(sdir.glob("*.jpg")) if p.name not in dropped]
        if files:
            sessions[sdir.name] = files
    if not sessions:
        print("사용할 이미지가 없다.")
        return 1

    names = sorted(sessions)
    random.Random(args.seed).shuffle(names)
    n = len(names)
    n_val = max(1, round(n * args.val_frac)) if n >= 2 else 0
    n_test = max(1, round(n * args.test_frac)) if n >= 4 else 0
    n_val = min(n_val, n - 1) if n >= 2 else 0      # train 이 비면 학습이 안 된다
    n_test = min(n_test, n - 1 - n_val)
    split_of = {nm: ("val" if i < n_val else ("test" if i < n_val + n_test else "train"))
                for i, nm in enumerate(names)}

    out = pathlib.Path(args.out)
    for sp in ("train", "val", "test"):
        (out / "images" / sp).mkdir(parents=True, exist_ok=True)
        (out / "labels" / sp).mkdir(parents=True, exist_ok=True)

    counts: dict[str, int] = collections.Counter()
    n_link = 0
    for name, files in sessions.items():
        sp = split_of[name]
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
                        n_link += 1
                    except OSError:          # 윈도우 권한 문제 등
                        shutil.copy2(src, dst)
            counts[sp] += 1

    # path 는 옮길 때 가장 잘 깨지는 값이다. ultralytics 의 해석 규칙(실측 확인):
    #   · path 가 절대경로   -> 그대로 쓴다. 다른 컴퓨터로 옮기면 그 경로가 없어 실패
    #   · path 가 상대경로   -> **실행 디렉토리(cwd) 기준**. yaml 위치 기준이 아니다.
    #                          그래서 "path: ." 는 dataset 폴더 밖에서 돌리면 틀린다
    #   · **path 키가 없으면 -> yaml 파일이 있는 폴더** <- 옮겨도 항상 맞는 유일한 형태
    # 그래서 --portable 은 path 줄을 아예 쓰지 않는다.
    if args.path is not None:
        path_line = f"path: {args.path}\n"
    elif args.portable:
        path_line = ""                       # 생략 = dataset.yaml 이 있는 폴더
    else:
        path_line = f"path: {out.resolve().as_posix()}\n"

    yaml_path = out / "dataset.yaml"
    yaml_path.write_text(
        path_line
        + "train: images/train\nval: images/val\ntest: images/test\n\nnames:\n"
        + f"  0: {CLASS_NAME}\n",
        encoding="utf-8")

    total = sum(counts.values())
    print(f"\n세션 {len(sessions)}개, 이미지 {total}장 -> {out}")
    print(f"\n{'분할':<7} {'세션':>5} {'이미지':>7} {'비율':>7}")
    for sp in ("train", "val", "test"):
        ses = sorted(n for n, s in split_of.items() if s == sp)
        print(f"{sp:<7} {len(ses):5d} {counts[sp]:7d} {counts[sp]/total*100:6.1f}%")
        if ses:
            print(f"        {', '.join(ses[:3])}{' ...' if len(ses) > 3 else ''}")

    # 세션 개수로 나누므로 세션마다 장수가 다르면 실제 비율이 목표에서 벗어난다
    want = args.val_frac * 100
    got = counts["val"] / total * 100 if total else 0
    if counts["val"] and abs(got - want) > 10:
        print(f"\n[!] val 실제 비율 {got:.0f}% (목표 {want:.0f}%) — 세션별 장수 편차가 크다."
              f" --seed 를 바꿔 다시 나눠보라")
    if counts["val"] == 0:
        print("\n[!] val 이 비었다 — 세션이 2개 이상 있어야 검증이 가능하다")
    if total < 800:
        print(f"\n[!] 총 {total}장. 목표는 1300~1800장이다 (../../docs/vision-pipeline.md 3장)")

    if n_link:
        print(f"\n[!] {n_link}개가 **심볼릭 링크**다 (실제 파일이 아니다)."
              f"\n    이 폴더를 다른 컴퓨터로 옮길 거면 링크가 깨진다 —"
              f" --portable 로 다시 만들 것.")
    if args.portable:
        print("\n옮길 준비 완료 — 실제 복사 + path 줄 생략 (yaml 폴더 기준)")
        print(f"    tar czf {out}.tar.gz {out}")
        print(f"    # 코랩에서:  !tar xzf {out.name}.tar.gz"
              f"  ->  data={out.name}/dataset.yaml")
    else:
        print(f"\n다음: python train_yolo.py --data {yaml_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

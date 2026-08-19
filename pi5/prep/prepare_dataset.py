r"""수집한 데이터를 YOLO 학습 폴더 구조로 정리한다.

    python prepare_dataset.py                        # 여기서 바로 학습할 때
    python prepare_dataset.py --portable             # 다른 데(코랩 등)로 옮길 때
    python prepare_dataset.py --list-sessions        # 세션 목록·장수만 보기

⚠ **분할은 프레임이 아니라 세션 단위로 한다.**
   같은 세션의 프레임은 서로 거의 동일하다. 프레임 단위로 무작위 분할하면 train과 val에
   사실상 같은 이미지가 들어가 **검증 점수가 부풀려지고 과적합을 못 잡는다.**
   (../../docs/vision-pipeline.md 3장)

┌─ 어느 세션을 어디에 넣을지 직접 정할 수 있다 ─────────────────────────────┐
│ 세션 하나 = 배경(천장) 하나다. 한 세션 안에서 캔·종이컵·물병을 여러 번     │
│ 던지므로 물체 종류는 세션마다 고루 섞여 있고, **세션 간 차이는 배경뿐이다.** │
│ 그래서 "어느 배경으로 검증할지"는 무작위에 맡길 게 아니라 사람이 고른다.   │
│                                                                          │
│   python prepare_dataset.py --list-sessions        # 먼저 목록 확인       │
│   python prepare_dataset.py --portable \                                 │
│       --val-sessions  session_20260812_101233 \                          │
│       --test-sessions session_20260814_160512                            │
│                                                                          │
│ · 이름은 **부분만 써도 된다** — 한 세션에만 걸리면 "0812" 로도 충분하다   │
│ · --train-sessions 를 생략하면 나머지 전부 train (보통 이걸로 충분)       │
│ · --train-sessions 를 적으면 어디에도 안 적은 세션은 제외된다             │
│ · 아무것도 안 주면 예전처럼 무작위로 나눈다                               │
└──────────────────────────────────────────────────────────────────────────┘

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


def resolve_sessions(spec: str, available: list[str], flag: str) -> list[str]:
    """쉼표로 구분한 세션 지정을 실제 세션 이름으로 바꾼다.

    세션 이름이 "session_20260810_143022" 처럼 길어서 다 치기 번거롭다.
    그래서 **부분 일치**를 허용한다 — "0810" 처럼 한 세션에만 걸리는 조각이면 된다.
    여러 세션에 걸리면 어느 쪽인지 알 수 없으므로 에러를 낸다. 조용히 하나를
    고르면 의도와 다른 배경이 val 로 들어가도 눈치채지 못한다.
    """
    out: list[str] = []
    for tok in (s.strip() for s in spec.split(",")):
        if not tok:
            continue
        if tok in available:
            out.append(tok)
            continue
        hits = [s for s in available if tok in s]
        if len(hits) == 1:
            out.append(hits[0])
        elif not hits:
            raise ValueError(f"{flag}: '{tok}' 에 맞는 세션이 없다.\n"
                             f"    있는 세션: {', '.join(available)}")
        else:
            raise ValueError(f"{flag}: '{tok}' 이 세션 여러 개에 걸린다 "
                             f"({', '.join(hits)}). 더 길게 쓸 것")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="YOLO 데이터셋 준비")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--test-frac", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--list-sessions", action="store_true",
                    help="세션 목록과 장수만 보여주고 끝낸다. 어느 세션을 어디에 "
                         "넣을지 정할 때 먼저 이걸로 확인할 것")
    # 세션마다 배경(천장)이 다르므로 어느 배경을 검증/시험에 쓸지는 사람이 정하는 게
    # 맞다. 무작위로 맡기면 비슷한 배경끼리 train/val 로 갈려 점수가 부풀려지거나,
    # 반대로 유독 어려운 배경 하나가 val 로 빠져 조기 종료가 엉뚱하게 걸린다.
    for sp in ("train", "val", "test"):
        ap.add_argument(f"--{sp}-sessions", default=None, metavar="A,B",
                        help=f"{sp} 에 넣을 세션 (쉼표 구분, 부분 이름 가능)")
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

    all_names = sorted(sessions)

    if args.list_sessions:
        print(f"\n{'세션':<32} {'이미지':>7}")
        for nm in all_names:
            print(f"{nm:<32} {len(sessions[nm]):7d}")
        print(f"{'합계':<32} {sum(len(v) for v in sessions.values()):7d}")
        print("\n예:  python prepare_dataset.py --portable \\\n"
              f"       --val-sessions {all_names[0]} \\\n"
              f"       --test-sessions {all_names[-1]}")
        return 0

    manual = {sp: getattr(args, f"{sp}_sessions") for sp in ("train", "val", "test")}
    manual_used = any(v is not None for v in manual.values())
    if manual_used:
        split_of: dict[str, str] = {}
        try:
            for sp in ("val", "test", "train"):
                if manual[sp] is None:
                    continue
                for nm in resolve_sessions(manual[sp], all_names, f"--{sp}-sessions"):
                    if nm in split_of:
                        print(f"\n[!] {nm} 이 {split_of[nm]} 과 {sp} 양쪽에 지정됐다.")
                        return 1
                    split_of[nm] = sp
        except ValueError as e:
            print(f"\n[!] {e}")
            return 1

        rest = [nm for nm in all_names if nm not in split_of]
        if manual["train"] is None:
            # train 을 안 적었으면 나머지 전부 train. 세션이 늘어도 손댈 게 없다.
            split_of.update({nm: "train" for nm in rest})
        elif rest:
            print(f"\n[!] 어디에도 지정하지 않은 세션 {len(rest)}개는 **제외된다**: "
                  f"{', '.join(rest)}")
    else:
        names = list(all_names)
        random.Random(args.seed).shuffle(names)
        n = len(names)
        n_val = max(1, round(n * args.val_frac)) if n >= 2 else 0
        n_test = max(1, round(n * args.test_frac)) if n >= 4 else 0
        n_val = min(n_val, n - 1) if n >= 2 else 0  # train 이 비면 학습이 안 된다
        n_test = min(n_test, n - 1 - n_val)
        split_of = {nm: ("val" if i < n_val else ("test" if i < n_val + n_test else "train"))
                    for i, nm in enumerate(names)}

    sessions = {nm: f for nm, f in sessions.items() if nm in split_of}

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
                # 대상 폴더는 평평한데(images/train 에 전부), capture_video.py 는
                # 클립 번호를 **세션마다 1번부터 다시 센다** (f"{label}_{n:04d}.mp4").
                # 그래서 같은 --label 로 찍은 다른 세션의 프레임이 이름까지 똑같아지고,
                # src.name 을 그대로 쓰면 뒤에 온 게 앞엣것을 덮어써서 이미지가
                # 조용히 사라진다 (통계표에는 멀쩡한 수가 찍힌 채로). 세션 이름을 붙인다.
                dst = out / sub / sp / f"{name}__{src.name}"
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
        for nm in ses:
            print(f"        {nm}  ({len(sessions[nm])}장)")

    if not manual_used:
        # 세션 개수로 나누므로 세션마다 장수가 다르면 실제 비율이 목표에서 벗어난다
        want = args.val_frac * 100
        got = counts["val"] / total * 100 if total else 0
        if counts["val"] and abs(got - want) > 10:
            print(f"\n[!] val 실제 비율 {got:.0f}% (목표 {want:.0f}%) — 세션별 장수 편차가"
                  f" 크다. --seed 를 바꾸거나 --val-sessions 로 직접 지정할 것")
    if counts["val"] == 0:
        print("\n[!] val 이 비었다 — ultralytics 는 val 없이 학습을 시작하지 않는다"
              " (\"'val:' key missing\"). --val-sessions 로 하나는 지정할 것")
    # 세션 수가 적고 장수 편차가 크면 train 이 절반도 안 되는 일이 실제로 생긴다
    # (세션 개수로 나누기 때문이다). 학습 데이터가 그만큼 날아간 것이니 경고한다.
    if total and counts["train"] / total < 0.5:
        print(f"\n[!] train 이 {counts['train']/total*100:.0f}% 뿐이다 — 학습에 쓸 데이터가"
              f" 너무 적다.\n    --train-sessions/--val-sessions 로 직접 지정하는 게 낫다"
              f" (--list-sessions 로 장수 확인)")
    if counts["test"] and counts["test"] < 150:
        print(f"\n[!] test 가 {counts['test']}장뿐이다 — mAP 오차범위가 커서 숫자를"
              f" 믿기 어렵다. 더 큰 세션을 test 로 돌리는 게 낫다")
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

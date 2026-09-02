"""천장만 찍은 영상에서 **배경 이미지(negative sample)** 를 뽑는다. (PC에서 실행)

    python extract_background.py --video ceiling_a.mp4 --count 120 --tag ceilingA
    python extract_background.py --frames-dir raw/gs_2026.../frames --count 120 --tag ceilingA

무엇을 만드는가
---------------
`dataset_raw/images/bg_<tag>_<시각>/` 에 프레임을 저장하고,
`dataset_raw/labels/bg_<tag>_<시각>/` 에 **빈 .txt** 를 같은 이름으로 만든다.

빈 라벨 = "이 사진에는 물체가 없다" 는 뜻이고, YOLO 는 이걸로 **오검출(false positive)
을 줄이는 법**을 배운다. 조명·환기구·반사광에 박스를 칠 필요가 전혀 없다 —
클래스가 `trash` 하나뿐이라 "이건 쓰레기가 아니다" 라는 박스는 존재할 수 없다.

⚠ **빈 파일이어야 하고, 파일이 없으면 안 된다.**
   prepare_dataset.py 가 라벨 파일이 없는 이미지를 학습에서 제외하기 때문에,
   라벨을 아예 안 만들면 배경 이미지가 조용히 사라진다.

몇 장을 뽑아야 하나
------------------
**전체 데이터셋의 0~10%.** (Ultralytics 권장. 참고로 COCO 는 1%)
던진 데이터가 4000장이면 배경은 최대 400장이다. 그 이상 넣으면 물체를 배우는
신호가 묽어진다.

영상 전체에 **고르게** 뽑는다 — 형광등 맥동의 여러 위상, 움직이는 반사광, 사람이
지나가는 그림자 같은 게 골고루 들어가야 한다. 연속된 프레임을 몰아서 뽑으면
같은 사진 여러 장이라 효과가 없다.

천장마다 따로 뽑을 것
--------------------
`--tag` 를 천장별로 다르게 준다. prepare_dataset.py 가 **세션 단위로** train/val/test 를
나누므로, 배경 세션이 하나뿐이면 그 전부가 한쪽 split 에만 들어간다.
천장별로 나눠 두면 각 split 에 배경 이미지가 골고루 배분된다.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib

ROOT = pathlib.Path("dataset_raw")


def frames_from_video(cv2, path: pathlib.Path, count: int):
    """영상에서 count 장을 **고르게** 뽑는다. -> [(순번, 프레임), ...]"""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"영상을 열 수 없다: {path}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    print(f"  {path.name}: {total}프레임, {fps:.1f} fps "
          f"({total / fps:.1f}초)" if fps else f"  {path.name}: {total}프레임")

    if total <= 0:                       # 일부 코덱은 프레임 수를 안 알려준다
        out, i = [], 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            out.append((i, frame))
            i += 1
        cap.release()
        step = max(1, len(out) // count)
        return out[::step][:count]

    step = max(1, total // count)
    picked = []
    for idx in range(0, total, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if ok:
            picked.append((idx, frame))
        if len(picked) >= count:
            break
    cap.release()
    return picked


def frames_from_dir(cv2, path: pathlib.Path, count: int):
    """이미 프레임으로 쪼개진 폴더에서 고르게 뽑는다."""
    files = sorted(path.glob("*.jpg")) + sorted(path.glob("*.png"))
    if not files:
        raise SystemExit(f"이미지가 없다: {path}")
    print(f"  {path.name}: {len(files)}장")
    step = max(1, len(files) // count)
    picked = []
    for i, f in enumerate(files[::step][:count]):
        img = cv2.imread(str(f))
        if img is not None:
            picked.append((i * step, img))
    return picked


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description="배경 이미지(negative sample) 추출")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--video", help="천장만 찍은 영상 파일")
    src.add_argument("--frames-dir", help="이미 프레임으로 쪼개진 폴더")
    ap.add_argument("--count", type=int, default=100,
                    help="뽑을 장수. 전체 데이터셋의 0~10%% 가 되게 잡을 것")
    ap.add_argument("--tag", required=True,
                    help="천장 구분용 이름 (예: ceilingA). 천장마다 다르게 줄 것")
    ap.add_argument("--quality", type=int, default=85, help="JPEG 품질")
    args = ap.parse_args()

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"bg_{args.tag}_{stamp}"
    out_img = ROOT / "images" / session
    out_lbl = ROOT / "labels" / session
    out_img.mkdir(parents=True, exist_ok=True)
    out_lbl.mkdir(parents=True, exist_ok=True)

    if args.video:
        picked = frames_from_video(cv2, pathlib.Path(args.video), args.count)
    else:
        picked = frames_from_dir(cv2, pathlib.Path(args.frames_dir), args.count)

    H = W = 0
    for n, (idx, frame) in enumerate(picked):
        H, W = frame.shape[:2]
        name = f"{session}_{n:05d}"
        cv2.imwrite(str(out_img / f"{name}.jpg"), frame,
                    [cv2.IMWRITE_JPEG_QUALITY, args.quality])
        # ★ 빈 라벨 = "물체 없음". 파일 자체가 없으면 prepare_dataset.py 가 건너뛴다.
        (out_lbl / f"{name}.txt").write_text("", encoding="utf-8")

    manifest = ROOT / "sessions.json"
    data = json.loads(manifest.read_text(encoding="utf-8")) if manifest.exists() else {}
    data[session] = {"label": "background", "class_id": None, "camera": args.tag,
                     "frames": len(picked), "throws": 0,
                     "note": f"배경 이미지(negative). 출처={args.video or args.frames_dir}",
                     "created": stamp, "image_size": [W, H]}
    manifest.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"\n{len(picked)}장 저장 -> {out_img.resolve()}")
    print(f"빈 라벨 {len(picked)}개 -> {out_lbl.resolve()}")

    # 비율 점검 — 배경이 너무 많으면 물체를 배우는 신호가 묽어진다
    n_obj = 0
    img_root = ROOT / "images"
    if img_root.exists():
        for d in img_root.iterdir():
            if d.is_dir() and not d.name.startswith("bg_"):
                n_obj += len(list(d.glob("*.jpg")))
    n_bg = sum(len(list(d.glob("*.jpg")))
               for d in img_root.iterdir() if d.is_dir() and d.name.startswith("bg_"))
    total = n_obj + n_bg
    if total:
        frac = n_bg / total * 100
        print(f"\n현재 배경 비율: {n_bg} / {total} = {frac:.1f}%  (권장 0~10%)")
        if frac > 12:
            print("⚠ 배경이 너무 많다. --count 를 줄이거나 던진 데이터를 더 모을 것.")
        elif frac < 1 and n_obj > 500:
            print("  더 넣어도 된다.")

    print(f"\n다음: python prepare_dataset.py   (배경 세션도 자동으로 포함된다)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

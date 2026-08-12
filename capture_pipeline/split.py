#!/usr/bin/env python3
"""
split.py -- split the dataset into train/val BY CLIP, not by frame.

Why this matters: frames from a single throw are near-identical. Splitting
randomly at the frame level puts near-copies of val images into train, so
validation accuracy looks excellent and the model still fails on a new throw.
Splitting whole clips keeps the val set genuinely unseen.

Usage:
  python3 split.py dataset
  python3 split.py dataset --val-frac 0.25 --move
"""

import argparse
import os
import random
import re
import shutil
from collections import defaultdict


def clip_of(filename):
    """'can_0007_f00042.jpg' -> 'can_0007'"""
    return re.sub(r"_f\d+$", "", os.path.splitext(filename)[0])


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("dataset", help="dataset directory from extract.py")
    p.add_argument("--val-frac", type=float, default=0.2)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--move", action="store_true", help="move instead of copy")
    args = p.parse_args()

    img_dir = os.path.join(args.dataset, "images")
    lbl_dir = os.path.join(args.dataset, "labels")
    images = sorted(f for f in os.listdir(img_dir) if f.lower().endswith(".jpg"))
    if not images:
        raise SystemExit(f"no images in {img_dir}")

    by_clip = defaultdict(list)
    for f in images:
        by_clip[clip_of(f)].append(f)

    clips = sorted(by_clip)
    random.Random(args.seed).shuffle(clips)
    n_val = max(1, round(len(clips) * args.val_frac))
    val_clips = set(clips[:n_val])

    for split in ("train", "val"):
        for sub in ("images", "labels"):
            os.makedirs(os.path.join(args.dataset, split, sub), exist_ok=True)

    op = shutil.move if args.move else shutil.copy2
    counts = {"train": 0, "val": 0}
    for clip, files in by_clip.items():
        split = "val" if clip in val_clips else "train"
        for f in files:
            stem = os.path.splitext(f)[0]
            op(os.path.join(img_dir, f),
               os.path.join(args.dataset, split, "images", f))
            lbl = os.path.join(lbl_dir, stem + ".txt")
            if os.path.exists(lbl):
                op(lbl, os.path.join(args.dataset, split, "labels", stem + ".txt"))
            counts[split] += 1

    classes_path = os.path.join(args.dataset, "classes.txt")
    names = ["object"]
    if os.path.exists(classes_path):
        with open(classes_path) as fh:
            names = [l.strip() for l in fh if l.strip()]

    yaml_path = os.path.join(args.dataset, "data.yaml")
    with open(yaml_path, "w") as fh:
        fh.write(f"path: {os.path.abspath(args.dataset)}\n")
        fh.write("train: train/images\n")
        fh.write("val: val/images\n")
        fh.write(f"nc: {len(names)}\n")
        fh.write("names:\n")
        for i, n in enumerate(names):
            fh.write(f"  {i}: {n}\n")

    print(f"{len(clips)} clips -> {len(clips) - n_val} train / {n_val} val")
    print(f"{counts['train']} train frames, {counts['val']} val frames")
    print(f"wrote {yaml_path}")
    print(f"\nTrain with:\n  yolo detect train data={yaml_path} model=yolo11n.pt "
          f"imgsz=1024 epochs=100")


if __name__ == "__main__":
    main()

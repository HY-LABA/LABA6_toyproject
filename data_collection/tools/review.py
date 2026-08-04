#!/usr/bin/env python3
"""Inspect what has been collected so far.

    python3 tools/review.py                      # summary
    python3 tools/review.py --montage sheet.png  # contact sheet of crops

The per-track detection count is the number to watch during a session. It is
how many frames caught each throw, so it sets both how many crops you get and
how well the parabola is constrained. Below 6 a track is rejected outright,
so a median hovering near 7 means throws are being lost to timing rather than
to anything you did wrong.
"""

from __future__ import annotations

import argparse
import os
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from brain.vision.dataset import (  # noqa: E402
    find_manifests, load_manifests, objects_by_label, summarize,
)


def percentile(values, q):
    if not values:
        return 0
    s = sorted(values)
    i = int(round((len(s) - 1) * q))
    return s[i]


def print_summary(samples, verbose=False):
    s = summarize(samples)
    print(f"crops   {s['crops']}")
    print(f"tracks  {s['tracks']}   (throws -- this is your effective sample count)")
    print(f"objects {s['objects']}")
    print(f"labels  {s['by_label']}")

    per_track = Counter(x.track_key for x in samples)
    n = list(per_track.values())
    print(f"\ndetections per throw (n):")
    print(f"  min {min(n)}   p25 {percentile(n, .25)}   median {percentile(n, .5)}"
          f"   p75 {percentile(n, .75)}   max {max(n)}")
    weak = sum(1 for v in n if v < 12)
    if weak:
        print(f"  ! {weak}/{len(n)} throws under 12 detections -- raise --scale "
              "so detection keeps up with the frame rate")
    else:
        print("  healthy: throws are being captured across most of the flight")

    res = [x.fit_residual for x in samples if x.fit_residual == x.fit_residual]
    if res:
        print(f"\nfit residual (px): median {percentile(res, .5):.2f}  "
              f"max {max(res):.2f}")

    print("\nper object:")
    by_obj = defaultdict(lambda: [0, set(), ""])
    for x in samples:
        by_obj[x.object_id][0] += 1
        by_obj[x.object_id][1].add(x.track_key)
        by_obj[x.object_id][2] = x.label
    for obj in sorted(by_obj):
        crops, tracks, label = by_obj[obj]
        flag = "" if len(tracks) >= 8 else "   <- fewer than 8 throws"
        print(f"  {obj:24s} {label:12s} {len(tracks):3d} throws  "
              f"{crops:5d} crops{flag}")

    if verbose:
        print("\nobjects by label:")
        for label, objs in objects_by_label(samples).items():
            print(f"  {label}: {', '.join(objs)}")


def build_montage(samples, out_path, label=None, limit=144, cell=64, cols=12):
    """Tile crops into one image so a whole session can be eyeballed at once."""
    import cv2
    import numpy as np

    chosen = [x for x in samples if label is None or x.label == label]
    if not chosen:
        print(f"no crops with label={label}")
        return
    # Spread across tracks rather than taking the first N, which would all be
    # consecutive frames of a single throw and show nothing useful.
    by_track = defaultdict(list)
    for x in chosen:
        by_track[x.track_key].append(x)
    picked = []
    round_ = 0
    while len(picked) < limit:
        added = False
        for key in sorted(by_track):
            rows = by_track[key]
            if round_ < len(rows):
                picked.append(rows[round_])
                added = True
                if len(picked) >= limit:
                    break
        if not added:
            break
        round_ += 1

    rows = (len(picked) + cols - 1) // cols
    sheet = np.full((rows * cell, cols * cell, 3), 30, np.uint8)
    missing = 0
    for i, s in enumerate(picked):
        img = cv2.imread(s.crop_path)
        if img is None:
            missing += 1
            continue
        img = cv2.resize(img, (cell, cell))
        r, c = divmod(i, cols)
        sheet[r * cell:(r + 1) * cell, c * cell:(c + 1) * cell] = img

    cv2.imwrite(out_path, sheet)
    print(f"wrote {out_path}  ({len(picked)} crops, {rows}x{cols})"
          + (f", {missing} unreadable" if missing else ""))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--data", default="data")
    p.add_argument("--montage", help="write a contact sheet here")
    p.add_argument("--label", help="restrict montage to one label")
    p.add_argument("--limit", type=int, default=144)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    manifests = find_manifests(args.data)
    if not manifests:
        print(f"no manifest.jsonl under {args.data}/ -- nothing collected yet")
        return 1
    samples = load_manifests(manifests)
    print(f"{len(manifests)} manifest(s) under {args.data}/\n")
    print_summary(samples, args.verbose)

    if args.montage:
        print()
        build_montage(samples, args.montage, args.label, args.limit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

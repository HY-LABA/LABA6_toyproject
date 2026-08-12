#!/usr/bin/env python3
"""
extract.py -- turn throw clips into a labelled YOLO dataset.

Runs on any machine with OpenCV (a laptop is faster than the Pi).

Method
------
The camera is static, so anything that differs from the background is a moving
object. Per clip:

  1. Background = median of frames sampled across the clip. Rebuilt every
     --bg-window seconds so slow lighting drift does not accumulate.
  2. Each frame is brightness-normalised to the background before differencing.
     This absorbs residual mains flicker and gradual light changes.
  3. Difference -> blur -> adaptive threshold. Adaptive, not global: a global
     value silently drops objects in the darker vignetted corners of a 140 deg
     lens, and those are exactly the hard examples worth keeping.
  4. Blobs are linked into tracks across frames by nearest centroid.
  5. Tracks are filtered. This is what separates a thrown object from a hand,
     a lid, or someone walking past -- see the track filter below.
  6. Surviving detections are written as JPEG + YOLO .txt label.

Track filter rationale (continuous footage sees a lot besides throws):
  - too long        -> a hand reaching in, or a lid; a throw crosses in <15 frames
  - too slow        -> a person in the background, or a shadow
  - too little total displacement -> something that appeared and sat still
  - too large       -> a torso or an arm filling the frame

Usage:
  python3 extract.py session_20260812_101500
  python3 extract.py session_*/ --review            # draw boxes to eyeball
  python3 extract.py session_A session_B -o dataset
  python3 extract.py session_A --multi-class        # per-clip labels from session.json
"""

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np


# ---------------------------------------------------------------- background

def sample_frames(cap, start, count, step):
    """Read `count` frames starting at index `start`, every `step` frames."""
    frames = []
    for i in range(count):
        cap.set(cv2.CAP_PROP_POS_FRAMES, start + i * step)
        ok, f = cap.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
    return frames


def build_background(cap, start, end, n=40):
    """Median of up to n frames sampled evenly over [start, end)."""
    span = max(1, end - start)
    step = max(1, span // n)
    frames = sample_frames(cap, start, min(n, span), step)
    if not frames:
        return None
    return np.median(np.stack(frames), axis=0).astype(np.uint8)


def normalise(frame_gray, bg_gray):
    """
    Scale the frame so its median matches the background's. Cancels an overall
    brightness shift (flicker residue, cloud, someone switching a light) that
    would otherwise light up the entire frame as 'motion'.
    """
    fm = float(np.median(frame_gray))
    bm = float(np.median(bg_gray))
    if fm < 1.0:
        return frame_gray
    return np.clip(frame_gray.astype(np.float32) * (bm / fm), 0, 255).astype(np.uint8)


# ---------------------------------------------------------------- detection

def detect_blobs(frame_bgr, bg_gray, args):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    gray = normalise(gray, bg_gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    diff = cv2.absdiff(gray, cv2.GaussianBlur(bg_gray, (5, 5), 0))

    # Adaptive threshold over the difference image: local, so corner falloff
    # does not suppress detections there.
    mask = cv2.adaptiveThreshold(
        diff, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, args.block_size, -args.adaptive_c)

    # Floor: adaptive thresholding amplifies sensor noise in flat regions.
    mask[diff < args.min_delta] = 0

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    h, w = mask.shape
    frame_area = h * w
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < args.min_area or area > args.max_area_frac * frame_area:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if bw < 8 or bh < 8:
            continue
        out.append({"box": (x, y, bw, bh), "area": area,
                    "cx": x + bw / 2.0, "cy": y + bh / 2.0})
    return out


# ------------------------------------------------------------------ tracking

def link_tracks(per_frame, max_dist):
    """Greedy nearest-centroid linking. Adequate for one object at a time."""
    tracks, active = [], []
    for idx, dets in enumerate(per_frame):
        unmatched = list(dets)
        still = []
        for tr in active:
            last = tr["dets"][-1]
            best, best_d = None, max_dist
            for d in unmatched:
                dist = np.hypot(d["cx"] - last["cx"], d["cy"] - last["cy"])
                if dist < best_d:
                    best, best_d = d, dist
            if best is not None:
                unmatched.remove(best)
                tr["dets"].append(best)
                tr["frames"].append(idx)
                still.append(tr)
            else:
                tracks.append(tr)
        for d in unmatched:
            still.append({"dets": [d], "frames": [idx]})
        active = still
    tracks.extend(active)
    return tracks


def track_stats(tr):
    pts = [(d["cx"], d["cy"]) for d in tr["dets"]]
    if len(pts) < 2:
        return 0.0, 0.0
    steps = [np.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
             for i in range(len(pts) - 1)]
    displacement = np.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
    return float(np.mean(steps)), float(displacement)


def keep_track(tr, args):
    n = len(tr["dets"])
    if n < args.min_track or n > args.max_track:
        return False, ("short" if n < args.min_track else "long")
    speed, disp = track_stats(tr)
    if speed < args.min_speed:
        return False, "slow"
    if disp < args.min_displacement:
        return False, "static"
    return True, "ok"


# -------------------------------------------------------------------- output

def yolo_line(box, w, h, cls):
    x, y, bw, bh = box
    return f"{cls} {(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} {bw / w:.6f} {bh / h:.6f}\n"


def process_clip(path, label_map, args, out, stats):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"  ! cannot open {path}")
        return
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 60.0
    stem = os.path.splitext(os.path.basename(path))[0]
    cls = label_map.get(os.path.basename(path), 0)

    window = max(1, int(args.bg_window * fps))
    backgrounds = {}
    for w0 in range(0, max(total, 1), window):
        bg = build_background(cap, w0, min(w0 + window, total))
        if bg is not None:
            backgrounds[w0 // window] = bg
    if not backgrounds:
        print(f"  ! no frames in {path}")
        cap.release()
        return

    # Pass 1: detect.
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    per_frame, idx = [], 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        key = idx // window
        bg = backgrounds[key] if key in backgrounds else list(backgrounds.values())[-1]
        per_frame.append(detect_blobs(frame, bg, args))
        idx += 1

    tracks = link_tracks(per_frame, args.max_link_dist)
    kept, rejected = [], {}
    for tr in tracks:
        ok, why = keep_track(tr, args)
        if ok:
            kept.append(tr)
        else:
            rejected[why] = rejected.get(why, 0) + 1

    wanted = {}
    for tr in kept:
        last_c = None
        for fi, det in zip(tr["frames"], tr["dets"]):
            c = (det["cx"], det["cy"])
            # Drop near-duplicates: consecutive frames where the object barely
            # moved add no information and skew the dataset.
            if last_c and np.hypot(c[0] - last_c[0], c[1] - last_c[1]) < args.dedup_px:
                continue
            last_c = c
            wanted[fi] = det

    # Pass 2: write the frames we decided to keep.
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    idx, written = 0, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx in wanted:
            det = wanted[idx]
            h, w = frame.shape[:2]
            name = f"{stem}_f{idx:05d}"
            cv2.imwrite(os.path.join(out, "images", name + ".jpg"), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            with open(os.path.join(out, "labels", name + ".txt"), "w") as fh:
                fh.write(yolo_line(det["box"], w, h, cls))
            if args.review:
                vis = frame.copy()
                x, y, bw, bh = det["box"]
                cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.putText(vis, f"{name} a={int(det['area'])}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imwrite(os.path.join(out, "review", name + ".jpg"), vis)
            written += 1
        idx += 1
    cap.release()

    stats["frames"] += written
    stats["tracks"] += len(kept)
    rej = ", ".join(f"{k}:{v}" for k, v in sorted(rejected.items())) or "none"
    print(f"  {os.path.basename(path)}: {written} frames from {len(kept)} track(s) "
          f"[rejected {rej}]")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("sessions", nargs="+", help="session directories from capture.py")
    p.add_argument("-o", "--out", default="dataset", help="output dataset directory")
    p.add_argument("--review", action="store_true",
                   help="also write frames with boxes drawn, for eyeballing")
    p.add_argument("--multi-class", action="store_true",
                   help="use per-clip labels from session.json instead of one class")

    g = p.add_argument_group("detection")
    g.add_argument("--min-area", type=float, default=300,
                   help="min blob area px^2 (default 300)")
    g.add_argument("--max-area-frac", type=float, default=0.25,
                   help="reject blobs above this fraction of the frame (arm/torso)")
    g.add_argument("--min-delta", type=int, default=18,
                   help="min pixel difference from background (noise floor)")
    g.add_argument("--block-size", type=int, default=31,
                   help="adaptive threshold neighbourhood, must be odd")
    g.add_argument("--adaptive-c", type=float, default=6.0)

    t = p.add_argument_group("track filter")
    t.add_argument("--min-track", type=int, default=2)
    t.add_argument("--max-track", type=int, default=15,
                   help="longer than this is a hand or a lid, not a throw")
    t.add_argument("--min-speed", type=float, default=12.0,
                   help="min mean px moved per frame")
    t.add_argument("--min-displacement", type=float, default=40.0,
                   help="min total px travelled start to end")
    t.add_argument("--max-link-dist", type=float, default=400.0,
                   help="max px between frames to call it the same object")
    t.add_argument("--dedup-px", type=float, default=6.0)
    t.add_argument("--bg-window", type=float, default=30.0,
                   help="rebuild background every N seconds of footage")
    args = p.parse_args()

    if args.block_size % 2 == 0:
        args.block_size += 1

    for sub in ("images", "labels"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    if args.review:
        os.makedirs(os.path.join(args.out, "review"), exist_ok=True)

    label_map, names = {}, {}
    stats = {"frames": 0, "tracks": 0}

    for sess in args.sessions:
        meta_path = os.path.join(sess, "session.json")
        clips = sorted(glob.glob(os.path.join(sess, "clips", "*.mp4")))
        if not clips:
            clips = sorted(glob.glob(os.path.join(sess, "*.mp4")))
        if not clips:
            print(f"{sess}: no clips found, skipping")
            continue
        print(f"{sess}: {len(clips)} clip(s)")

        if args.multi_class and os.path.exists(meta_path):
            with open(meta_path) as fh:
                meta = json.load(fh)
            for entry in meta.get("clips", []):
                lbl = entry.get("label", "object")
                if lbl not in names:
                    names[lbl] = len(names)
                label_map[entry["file"]] = names[lbl]
        for clip in clips:
            process_clip(clip, label_map, args, args.out, stats)

    if not names:
        names = {"object": 0}
    with open(os.path.join(args.out, "classes.txt"), "w") as fh:
        for n, _ in sorted(names.items(), key=lambda kv: kv[1]):
            fh.write(n + "\n")

    print(f"\n{stats['frames']} labelled frames from {stats['tracks']} tracks -> {args.out}")
    print("Classes: " + ", ".join(sorted(names, key=names.get)))
    if args.review:
        print(f"Check {os.path.join(args.out, 'review')} before training. "
              "Expect to fix roughly 1 in 10.")
    print(f"Next:  python3 split.py {args.out}")


if __name__ == "__main__":
    sys.exit(main())

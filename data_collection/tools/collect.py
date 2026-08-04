#!/usr/bin/env python3
"""Capture thrown objects and auto-label them by block.

You assert the class once, on the command line. The detector finds what is
moving, the ballistic gate decides which tracks are real throws, and every
crop from a surviving track inherits the block's label. Nothing is labelled
by hand.

    # check the setup before throwing 400 times
    python3 tools/collect.py --check

    # collect one object, ten throws
    python3 tools/collect.py --label trash --object-id can_coke_01 \
        --session s01 --block b03 --target 10

Tracks that fail the ballistic gate are saved as `distractor` rather than
discarded: an arm at release, a shadow, someone walking behind. Those are the
negatives the classifier actually needs, and they are free.

Run one invocation per object. Tedious, but `object_id` is what makes an
honest train/test split possible, and mixing objects inside one invocation
would quietly destroy it.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402

from brain.planning.camera import Camera  # noqa: E402
from brain.planning.trajectory import MIN_OBSERVATIONS  # noqa: E402
from brain.vision.detector import MotionDetector  # noqa: E402
from brain.vision.tracker import PROJECTILE, TrackManager  # noqa: E402

# Camera Module v2.1 binned to 1640x1232. Replace with the IMX296 + 4mm
# values (fx 1159) when the camera changes; nothing else moves.
DEFAULT_FX = 1357.0
DEFAULT_SIZE = (1640, 1232)


def build_camera(args) -> Camera:
    w, h = args.width, args.height
    return Camera.looking_down(
        fx=args.fx, fy=args.fx, cx=w / 2.0, cy=h / 2.0,
        altitude=args.cam_height, tilt_deg=args.tilt, width=w, height=h,
    )


def build_source(args):
    if args.source == "synthetic":
        from brain.capture.source import SyntheticThrowSource
        cam = build_camera(args)
        return SyntheticThrowSource(
            cam, p0=(3.2, 0.25, 1.60), v0=(-3.6, -0.12, 2.7),
            radius_m=0.05, fps=args.fps, noise=3.0,
            stop_height=0.8, seed=int(time.time()) % 1000,
            idle_frames=args.warmup + 10,
        )
    from brain.capture.source import PiCamera2Source
    return PiCamera2Source(size=(args.width, args.height), fps=args.fps,
                           exposure_us=args.exposure, gain=args.gain)


def beep() -> None:
    """Audible confirmation -- you will be across the room, not at the screen."""
    sys.stdout.write("\a")
    sys.stdout.flush()


def throw_summary(track, frames, banked: bool, width: int = 960,
                  cell: int = 96, max_crops: int = 10):
    """One image per throw: the trail it traced, and the crops it produced.

    Built only when a track closes, so it costs a few milliseconds per throw
    rather than anything per frame. That matters -- detection is already the
    bottleneck on the Pi, and a per-frame preview would make the very problem
    it is meant to diagnose worse.
    """
    idx = [i for i in track.frames if i in frames]
    if not idx:
        return None

    base = frames[idx[len(idx) // 2]].copy()
    pts = [(int(d.u), int(d.v)) for d in track.detections]
    for i, (u, v) in enumerate(pts):
        # Fade from dark to bright along the flight so direction is readable.
        s = int(60 + 195 * i / max(1, len(pts) - 1))
        cv2.circle(base, (u, v), 6, (0, s, s), -1)
    for a, b in zip(pts, pts[1:]):
        cv2.line(base, a, b, (0, 200, 200), 2)
    x, y, w, h = track.detections[-1].bbox
    cv2.rectangle(base, (x, y), (x + w, y + h), (0, 255, 0), 2)

    scale = width / base.shape[1]
    top = cv2.resize(base, (width, int(base.shape[0] * scale)))

    picks = [track.detections[i] for i in
             np.linspace(0, len(track.detections) - 1,
                         min(max_crops, len(track.detections))).astype(int)]
    pick_idx = [track.frames[i] for i in
                np.linspace(0, len(track.frames) - 1,
                            min(max_crops, len(track.frames))).astype(int)]
    strip = np.full((cell, width, 3), 25, np.uint8)
    for i, (det, fi) in enumerate(zip(picks, pick_idx)):
        if fi not in frames or (i + 1) * cell > width:
            continue
        strip[:, i * cell:(i + 1) * cell] = cv2.resize(
            det.crop(frames[fi], out_size=cell), (cell, cell))

    banner = np.full((34, width, 3), 25, np.uint8)
    colour = (80, 220, 80) if banked else (80, 80, 230)
    text = (f"BANKED   n={track.n}  residual={track.residual_px:.2f}px"
            if banked else
            f"REJECTED  n={track.n}  residual={track.residual_px:.2f}px")
    cv2.putText(banner, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                colour, 2, cv2.LINE_AA)
    return np.vstack([banner, top, strip])


def show_throw(image, out_dir: str, window: bool) -> bool:
    """Write the summary, and show it if a display is available.

    Returns whether the window is still usable -- a headless run falls back to
    the file without complaining twice.
    """
    if image is None:
        return window
    cv2.imwrite(os.path.join(out_dir, "last_throw.png"), image)
    if not window:
        return False
    try:
        cv2.imshow("last throw", image)
        cv2.waitKey(1)
        return True
    except Exception:  # noqa: BLE001 - no display; keep writing the file
        print("  (no display -- writing last_throw.png instead)")
        return False


def warm_up(stream, detector, n: int) -> None:
    """Let the background model learn the empty scene before anyone throws.

    Takes the live iterator rather than the source: calling source.frames()
    twice would build two independent generators, restarting frame indices at
    zero after warm-up and desynchronising them from the frame cache.
    """
    print(f"warming up background model ({n} frames) -- keep the scene still...")
    seen = 0
    for frame in stream:
        detector.detect(frame.image)
        seen += 1
        if seen >= n:
            return
    print("  warning: source ended during warm-up")


def run_check(args) -> int:
    """Report what the detector sees, saving nothing.

    Run this before every collection session. It is the difference between
    discovering a bad exposure now and discovering it after 400 throws.
    """
    source = build_source(args)
    detector = MotionDetector(min_area=args.min_area, max_area=args.max_area,
                              warmup=args.warmup, scale=args.scale)
    tracker = TrackManager(build_camera(args), max_link_px=args.link_px)

    print("=== CHECK MODE -- nothing is saved ===")
    print("Throw a few objects. Watch that:")
    print("  * frames with a throw report 1 detection, not 0 and not 5")
    print("  * areas sit inside [%d, %d]" % (args.min_area, args.max_area))
    print("  * idle frames report 0 detections (if not, you have flicker)\n")

    stream = source.frames()
    warm_up(stream, detector, args.warmup + 5)

    idle_hits = idle_frames = 0
    areas, ms, projectiles, distractors = [], [], 0, 0
    try:
        for frame in stream:
            dets = detector.detect(frame.image)
            ms.append(detector.last_ms)
            if dets:
                areas += [d.area for d in dets]
                print(f"  frame {frame.index:5d}  {len(dets)} det  "
                      f"areas={[d.area for d in dets]}")
            else:
                idle_hits += 0
            idle_frames += 1 if not dets else 0
            for tr in tracker.update(frame.index, frame.t, dets):
                if tr.verdict == PROJECTILE:
                    projectiles += 1
                    print(f"  -> PROJECTILE  n={tr.n}  residual={tr.residual_px:.2f}px")
                    beep()
                elif tr.n >= 4:
                    distractors += 1
                    print(f"  -> distractor  n={tr.n}  residual={tr.residual_px:.2f}px")
            if args.max_frames and frame.index >= args.max_frames:
                break
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        for tr in tracker.close_all():
            if tr.verdict == PROJECTILE:
                projectiles += 1
        source.close()

    print("\n--- summary ---")
    print(f"  detect: median {np.median(ms) if ms else 0:.1f} ms/frame")
    if areas:
        print(f"  areas:  min {min(areas)}  median {int(np.median(areas))}  max {max(areas)}")
        if min(areas) < args.min_area * 1.3:
            print("  ! smallest blobs are near min_area -- lower it or move closer")
        if max(areas) > args.max_area * 0.8:
            print("  ! largest blobs are near max_area -- raise it or you will")
            print("    silently drop the closest frames, which are your best crops")
    else:
        print("  no detections at all -- check exposure, and that AE/AWB are locked")
    print(f"  tracks: {projectiles} projectile, {distractors} distractor")
    print(f"  idle frames with no detection: {idle_frames}")
    if projectiles == 0:
        print("  ! no throw passed the ballistic gate. Check fx/tilt, or that the")
        print("    whole arc stays in frame -- the fit needs the curvature.")
    return 0


def save_track(track, frames, out_dir, args, session, block, throw_idx,
               run_id) -> int:
    """Write crops, full frames, and manifest rows for one track.

    `run_id` is essential, not decorative. Track ids restart at zero every
    invocation, and the intended workflow is one invocation per object -- so
    without it every object would overwrite the previous object's crops while
    still appending manifest rows, leaving rows that point at another object's
    images. Silent, and fatal to the dataset.
    """
    projectile = track.verdict == PROJECTILE
    label = args.label if projectile else "distractor"
    # Distractors are not the object you were throwing -- they are usually the
    # thrower walking back into frame. Giving them the thrown object's id would
    # drag them along when that object is held out for test, swamping the test
    # set with unrelated crops.
    object_id = args.object_id if projectile else f"distractor_{run_id}"
    crops_dir = os.path.join(out_dir, "crops", label)
    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(crops_dir, exist_ok=True)
    os.makedirs(frames_dir, exist_ok=True)
    manifest = os.path.join(out_dir, "manifest.jsonl")

    written = 0
    with open(manifest, "a") as fh:
        for frame_idx, t, det in zip(track.frames, track.times, track.detections):
            image = frames.get(frame_idx)
            if image is None:
                continue
            stem = (f"{session}_{block}_{args.object_id}_{run_id}"
                    f"_t{track.id:04d}_f{frame_idx:05d}")
            crop_path = os.path.join(crops_dir, stem + ".png")
            if os.path.exists(crop_path):
                raise RuntimeError(
                    f"refusing to overwrite {crop_path} -- filename collision "
                    "means the manifest would point at the wrong image")
            cv2.imwrite(crop_path, det.crop(image, out_size=args.crop, pad=args.pad))

            frame_path = os.path.join(frames_dir, stem + ".jpg")
            if args.save_frames:
                # Keep the full frame: when the detector or crop size changes
                # you want to re-crop, not re-throw.
                cv2.imwrite(frame_path, image,
                            [int(cv2.IMWRITE_JPEG_QUALITY), 92])

            fh.write(json.dumps({
                "session_id": session,
                "block_id": block,
                "object_id": object_id,
                "label": label,
                "verdict": track.verdict,
                "track_id": track.id,
                # Track ids restart every invocation; run_id is what makes
                # a track identifiable across runs.
                "run_id": run_id,
                "throw_idx": throw_idx,
                "frame_idx": frame_idx,
                "t": round(float(t), 6),
                "bbox": list(det.bbox),
                "area": int(det.area),
                "fit_residual": round(float(track.residual_px), 3),
                "crop": os.path.relpath(crop_path, out_dir),
                "frame": os.path.relpath(frame_path, out_dir) if args.save_frames else None,
                "fx": args.fx,
                "camera": args.camera_name,
            }) + "\n")
            written += 1
    return written


def run_collect(args) -> int:
    if not args.label or not args.object_id:
        print("error: --label and --object-id are required to collect")
        return 2

    out_dir = os.path.join(args.out, args.session)
    os.makedirs(out_dir, exist_ok=True)

    source = build_source(args)
    detector = MotionDetector(min_area=args.min_area, max_area=args.max_area,
                              warmup=args.warmup, scale=args.scale)
    tracker = TrackManager(build_camera(args), max_link_px=args.link_px)

    print(f"=== COLLECT  label={args.label}  object={args.object_id} "
          f"session={args.session} block={args.block} target={args.target} ===")
    stream = source.frames()
    warm_up(stream, detector, args.warmup + 5)
    print("ready -- start throwing\n")

    run_id = time.strftime("%H%M%S")
    frames_cache = {}
    kept = distractors = crops = 0
    window = bool(args.preview)
    if args.preview:
        print(f"  preview on -- also written to {out_dir}/last_throw.png\n")
    try:
        for frame in stream:
            frames_cache[frame.index] = frame.image
            # Only the recent past can still belong to an open track. Each
            # frame is ~6 MB at 1640x1232, so this window is real memory: 120
            # frames was ~730 MB and enough to cause pressure on a 4 GB Pi.
            # A flight is ~32 frames, so 45 covers any track that can still
            # be open.
            for old in [k for k in frames_cache if k < frame.index - 45]:
                del frames_cache[old]

            dets = detector.detect(frame.image)
            for tr in tracker.update(frame.index, frame.t, dets):
                if tr.verdict == PROJECTILE:
                    kept += 1
                    crops += save_track(tr, frames_cache, out_dir, args,
                                        args.session, args.block, kept, run_id)
                    print(f"  [{kept}/{args.target}] throw banked -- "
                          f"n={tr.n} residual={tr.residual_px:.2f}px")
                    beep()
                    if args.preview:
                        window = show_throw(
                            throw_summary(tr, frames_cache, True),
                            out_dir, window)
                else:
                    # Say why. Silence during collection is useless: you are
                    # across the room and cannot tell a missed throw from one
                    # the gate rejected.
                    if tr.n >= MIN_OBSERVATIONS:
                        if tr.residual_px == float("inf"):
                            why = "not a projectile (fit degenerate)"
                        else:
                            why = f"not ballistic (residual {tr.residual_px:.1f}px)"
                        print(f"  rejected -- n={tr.n}, {why}")
                        if args.preview:
                            window = show_throw(
                                throw_summary(tr, frames_cache, False),
                                out_dir, window)
                    if tr.n >= args.min_distractor:
                        distractors += 1
                        crops += save_track(tr, frames_cache, out_dir, args,
                                            args.session, args.block,
                                            -distractors, run_id)
            if kept >= args.target:
                break
    except KeyboardInterrupt:
        print("\ninterrupted -- what was banked is already on disk")
    finally:
        # A track still open when the run ends is closed here. It must be
        # reported exactly like one that closed mid-loop -- otherwise the last
        # throw of every session banks silently, with no beep and no preview.
        for tr in tracker.close_all():
            if tr.verdict == PROJECTILE and kept < args.target:
                kept += 1
                crops += save_track(tr, frames_cache, out_dir, args,
                                    args.session, args.block, kept, run_id)
                print(f"  [{kept}/{args.target}] throw banked -- "
                      f"n={tr.n} residual={tr.residual_px:.2f}px")
                beep()
                if args.preview:
                    window = show_throw(throw_summary(tr, frames_cache, True),
                                        out_dir, window)
        if args.preview:
            try:
                cv2.destroyAllWindows()
            except Exception:  # noqa: BLE001 - headless
                pass
        source.close()

    print(f"\n{kept} throws, {distractors} distractors, {crops} crops -> {out_dir}")
    if kept < args.target:
        print(f"  ! only {kept} of {args.target}. Re-run to top up; nothing is lost.")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--check", action="store_true",
                   help="report what the detector sees; save nothing")
    p.add_argument("--label", choices=["trash", "not-trash"],
                   help="the class for this block -- you assert it once")
    p.add_argument("--object-id", help="e.g. can_coke_01; required, and it is what")
    p.add_argument("--session", default="s01")
    p.add_argument("--block", default="b01")
    p.add_argument("--target", type=int, default=10)
    p.add_argument("--out", default="data")

    p.add_argument("--source", choices=["picamera2", "synthetic"],
                   default="picamera2")
    p.add_argument("--width", type=int, default=DEFAULT_SIZE[0])
    p.add_argument("--height", type=int, default=DEFAULT_SIZE[1])
    p.add_argument("--fps", type=float, default=40.0)
    p.add_argument("--exposure", type=int, default=1000, help="microseconds")
    p.add_argument("--gain", type=float, default=4.0)
    p.add_argument("--camera-name", default="imx219_v21_binned")

    p.add_argument("--fx", type=float, default=DEFAULT_FX)
    p.add_argument("--cam-height", type=float, default=0.80)
    p.add_argument("--tilt", type=float, default=-16.0)

    p.add_argument("--min-area", type=int, default=300)
    p.add_argument("--max-area", type=int, default=60000)
    # Default tuned for the Pi, not a laptop. Background subtraction over
    # 1640x1232 costs ~22ms on a Pi 5 at scale=2, which eats most of a 25ms
    # frame budget and starves each throw of detections.
    p.add_argument("--scale", type=int, default=3)
    p.add_argument("--warmup", type=int, default=30)
    p.add_argument("--link-px", type=float, default=120.0)
    # Retrieving the thrown object generates a lot of motion, and at 5 that
    # produced ~180 saved distractor tracks per throw cycle. Only substantial
    # tracks are worth keeping as hard negatives.
    p.add_argument("--min-distractor", type=int, default=15)

    p.add_argument("--preview", action="store_true",
                   help="after each throw, show the trail and the crops it "
                        "produced (also written to last_throw.png)")
    p.add_argument("--crop", type=int, default=64)
    p.add_argument("--pad", type=float, default=1.4)
    p.add_argument("--save-frames", action="store_true", default=True)
    p.add_argument("--no-save-frames", dest="save_frames", action="store_false")
    p.add_argument("--max-frames", type=int, default=0)

    args = p.parse_args()
    return run_check(args) if args.check else run_collect(args)


if __name__ == "__main__":
    raise SystemExit(main())

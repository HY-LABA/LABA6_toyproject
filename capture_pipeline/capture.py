#!/usr/bin/env python3
"""
capture.py -- throw-data capture for Raspberry Pi 5 + IMX296 global shutter.

Setup assumed:
  - Camera rim-mounted on a trashcan, looking across the opening.
  - M12 lens, ~140 deg FOV, fixed aperture, focus locked.
  - Indoor lighting (the deployment environment).

Two modes:
  clip mode (default)  : press Enter -> countdown -> record one short clip per throw.
  continuous mode      : record one long clip; throw whenever you like.

All auto algorithms are DISABLED. Auto-exposure or auto-white-balance hunting
mid-throw makes every frame a different brightness, which breaks the background
differencing in extract.py.

Usage:
  python3 capture.py --label can
  python3 capture.py --label paper --mains 60
  python3 capture.py --continuous 300 --label mixed
  python3 capture.py --label can --exposure 2000 --gain 8.0   # brighter room

Output layout:
  session_YYYYmmdd_HHMMSS/
    session.json          camera settings + lens notes (self-documenting dataset)
    background.jpg        median of empty-scene frames
    clips/can_0001.mp4
    clips/can_0002.mp4
    ...
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import numpy as np

try:
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder
    from picamera2.outputs import FfmpegOutput
except ImportError:
    sys.exit(
        "picamera2 not found.\n"
        "On Pi OS Bookworm:  sudo apt install -y python3-picamera2\n"
        "Run this script with the system python3, not inside a venv that\n"
        "lacks --system-site-packages."
    )

# IMX296 native full-frame readout.
WIDTH, HEIGHT = 1456, 1088


def check_overlay():
    """Warn early if the sensor overlay is missing, rather than failing cryptically."""
    cfg = "/boot/firmware/config.txt"
    if not os.path.exists(cfg):
        cfg = "/boot/config.txt"
    try:
        with open(cfg) as fh:
            text = fh.read()
    except OSError:
        return
    if "imx296" not in text:
        print(
            f"WARNING: no 'dtoverlay=imx296' found in {cfg}.\n"
            "         If the camera is not detected, add it and reboot.\n"
            "         On Pi 5 specify the port if not default, e.g.\n"
            "           dtoverlay=imx296,cam0\n",
            file=sys.stderr,
        )


def flicker_safe_exposure(mains_hz):
    """
    Indoor LED/fluorescent light pulses at 2x mains frequency. An exposure that
    is a whole multiple of the half-cycle integrates the same amount of light
    every frame, so brightness stays constant. Anything else gives randomly
    bright and dark frames.

      50 Hz mains -> 10000 us (1/100 s)
      60 Hz mains ->  8333 us (1/120 s)
    """
    return 10000 if mains_hz == 50 else 8333


def build_camera(args):
    picam2 = Picamera2()

    # 60 fps == 16666 us per frame. Exposure must fit inside that budget.
    frame_us = int(1_000_000 / args.fps)
    config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameDurationLimits": (frame_us, frame_us)},
        buffer_count=6,
    )
    picam2.configure(config)

    controls = {
        "ExposureTime": args.exposure,
        "AnalogueGain": args.gain,
        "AeEnable": False,          # locked: no mid-throw brightness hunting
        "AwbEnable": False,         # locked: IMX296 is a Bayer colour sensor
        "ColourGains": (args.red_gain, args.blue_gain),
    }
    picam2.start(config)
    picam2.set_controls(controls)
    time.sleep(1.5)  # let the sensor settle on the locked values
    return picam2, controls


def grab_background(picam2, path, n=15):
    """
    Median of n empty-scene frames. Median (not mean) so a person walking
    through during setup does not smear into the reference image.
    """
    print(f"Capturing background from {n} frames -- keep the scene EMPTY.")
    for i in range(3, 0, -1):
        print(f"  {i}...", flush=True)
        time.sleep(1)
    frames = []
    for _ in range(n):
        frames.append(picam2.capture_array("main"))
        time.sleep(0.05)
    median = np.median(np.stack(frames), axis=0).astype(np.uint8)
    try:
        import cv2
        cv2.imwrite(path, median)
    except ImportError:
        from PIL import Image
        Image.fromarray(median[:, :, ::-1]).save(path)
    print(f"Background saved -> {path}\n")


def record(picam2, encoder, out_path, seconds, countdown=True):
    if countdown:
        for i in (3, 2, 1):
            print(f"  {i}...", flush=True)
            time.sleep(0.7)
        print("  THROW NOW", flush=True)
    picam2.start_recording(encoder, FfmpegOutput(out_path))
    time.sleep(seconds)
    picam2.stop_recording()
    print(f"  saved {os.path.basename(out_path)}\n")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--label", default="object",
                   help="object type in these clips (can, paper, bottle...). "
                        "Stored per-clip so you can split into multiple classes "
                        "LATER without recapturing.")
    p.add_argument("--outdir", default=None, help="session directory")
    p.add_argument("--seconds", type=float, default=2.0,
                   help="clip length per throw (default 2.0)")
    p.add_argument("--continuous", type=float, default=None,
                   metavar="SECS", help="record one long clip instead of per-throw clips")
    p.add_argument("--fps", type=int, default=60, help="frame rate (IMX296 max ~60 at full frame)")
    p.add_argument("--mains", type=int, choices=(50, 60), default=50,
                   help="mains frequency, for flicker-safe exposure (default 50)")
    p.add_argument("--exposure", type=int, default=None,
                   help="exposure in microseconds. Default is flicker-safe for --mains. "
                        "Only override if the room is bright enough to go faster.")
    p.add_argument("--gain", type=float, default=4.0, help="analogue gain (default 4.0)")
    p.add_argument("--red-gain", type=float, default=1.8, help="fixed AWB red gain")
    p.add_argument("--blue-gain", type=float, default=1.8, help="fixed AWB blue gain")
    p.add_argument("--bitrate", type=int, default=20_000_000,
                   help="H.264 bitrate. High by default: compression artifacts on "
                        "fast motion look like noise to the background differencer.")
    args = p.parse_args()

    if args.exposure is None:
        args.exposure = flicker_safe_exposure(args.mains)
        print(f"Using flicker-safe exposure {args.exposure} us for {args.mains} Hz mains.")

    frame_us = int(1_000_000 / args.fps)
    if args.exposure > frame_us:
        sys.exit(f"Exposure {args.exposure} us exceeds the {frame_us} us frame budget "
                 f"at {args.fps} fps. Lower --fps or --exposure.")

    check_overlay()

    outdir = args.outdir or datetime.now().strftime("session_%Y%m%d_%H%M%S")
    clipdir = os.path.join(outdir, "clips")
    os.makedirs(clipdir, exist_ok=True)

    picam2, controls = build_camera(args)
    encoder = H264Encoder(bitrate=args.bitrate)

    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "sensor": "IMX296 colour global shutter",
        "resolution": [WIDTH, HEIGHT],
        "fps": args.fps,
        "mains_hz": args.mains,
        "lens": "M12, ~140 deg FOV, fixed aperture, focus locked",
        "mount": "rim-mounted, looking across the can opening",
        "environment": "indoor (deployment environment)",
        "controls": controls,
        "clips": [],
    }

    try:
        grab_background(picam2, os.path.join(outdir, "background.jpg"))

        if args.continuous:
            name = f"{args.label}_continuous.mp4"
            print(f"Continuous mode: recording {args.continuous:.0f}s. Throw freely.")
            record(picam2, encoder, os.path.join(clipdir, name),
                   args.continuous, countdown=False)
            meta["clips"].append({"file": name, "label": args.label,
                                  "mode": "continuous", "seconds": args.continuous})
        else:
            print("Clip mode. Press Enter to record a throw, or 'q' then Enter to stop.")
            print("Vary entry angle, spin and speed between throws -- that variety\n"
                  "matters more than the total number of frames.\n")
            n = 0
            while True:
                cmd = input(f"[{n} clips] Enter to record > ").strip().lower()
                if cmd in ("q", "quit", "exit"):
                    break
                n += 1
                name = f"{args.label}_{n:04d}.mp4"
                record(picam2, encoder, os.path.join(clipdir, name), args.seconds)
                meta["clips"].append({"file": name, "label": args.label,
                                      "mode": "clip", "seconds": args.seconds})
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        with open(os.path.join(outdir, "session.json"), "w") as fh:
            json.dump(meta, fh, indent=2)
        picam2.stop()
        picam2.close()
        print(f"\n{len(meta['clips'])} clip(s) -> {outdir}")
        print(f"Next:  python3 extract.py {outdir}")


if __name__ == "__main__":
    main()

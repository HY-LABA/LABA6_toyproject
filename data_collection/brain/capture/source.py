"""Frame sources behind one interface.

The point of the interface is the Aug 15 sensor swap: `PiCamera2Source` is
replaced by a differently-configured instance and nothing downstream changes.
`SyntheticThrowSource` renders a physically correct throw through the same
`Camera` model the estimator uses, so detection and tracking can be developed
and regression-tested with no camera attached at all.

Timestamps are seconds and only differences matter. The synthetic source
emits an exact clock; the real one must carry the sensor timestamp, not the
wall clock at the moment Python got the buffer -- the latter includes
scheduling jitter that the trajectory fit would read as motion.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterator, Optional, Tuple

import numpy as np

from ..planning.camera import G, Camera


class Frame:
    __slots__ = ("index", "t", "image")

    def __init__(self, index: int, t: float, image: np.ndarray):
        self.index = index
        self.t = t
        self.image = image

    @property
    def shape(self) -> Tuple[int, int]:
        return self.image.shape[0], self.image.shape[1]

    def __repr__(self):
        h, w = self.shape
        return f"Frame(#{self.index}, t={self.t:.4f}, {w}x{h})"


class CaptureSource(ABC):
    """Yields timestamped frames."""

    @abstractmethod
    def frames(self) -> Iterator[Frame]:
        ...

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class SyntheticThrowSource(CaptureSource):
    """Renders one thrown object on a static textured background.

    Exists so the detector can be held to a known answer: the true pixel
    position of the object is available for every frame, which no real
    footage provides.
    """

    def __init__(self, cam: Camera, p0, v0, radius_m: float = 0.05,
                 fps: float = 40.0, size: Optional[Tuple[int, int]] = None,
                 noise: float = 3.0, blur_px: float = 0.0,
                 stop_height: float = 0.0, seed: int = 0,
                 clutter: bool = False, idle_frames: int = 0):
        self.cam = cam
        self.p0 = np.asarray(p0, dtype=float)
        self.v0 = np.asarray(v0, dtype=float)
        self.radius_m = radius_m
        self.fps = fps
        self.noise = noise
        self.blur_px = blur_px
        self.stop_height = stop_height
        self.clutter = clutter
        # Empty-scene frames emitted before the throw, so consumers that must
        # warm up a background model can be exercised realistically.
        self.idle_frames = int(idle_frames)
        self.rng = np.random.default_rng(seed)

        w = size[0] if size else (cam.width or 1640)
        h = size[1] if size else (cam.height or 1232)
        self.w, self.h = int(w), int(h)
        self._background = self._make_background()
        # Ground truth, filled in as frames are produced.
        self.truth: list = []

    def _make_background(self) -> np.ndarray:
        """Low-frequency mottle plus a few hard edges.

        A flat grey background would make any detector look good; real rooms
        have texture and structure, so the background model has something to
        actually learn.
        """
        small = self.rng.integers(60, 140, size=(12, 16, 3), dtype=np.uint8)
        import cv2
        bg = cv2.resize(small, (self.w, self.h), interpolation=cv2.INTER_CUBIC)
        # Static "furniture": a couple of rectangles and a bright patch that
        # stand in for lights and hard edges.
        cv2.rectangle(bg, (0, int(self.h * 0.78)), (self.w, self.h),
                      (95, 95, 100), -1)
        cv2.rectangle(bg, (int(self.w * 0.05), int(self.h * 0.15)),
                      (int(self.w * 0.22), int(self.h * 0.55)), (150, 145, 130), -1)
        cv2.circle(bg, (int(self.w * 0.8), int(self.h * 0.12)),
                   int(self.h * 0.06), (230, 230, 220), -1)
        return bg

    def _flight_time(self) -> float:
        z0, vz = self.p0[2], self.v0[2]
        disc = vz * vz + 2.0 * G * (z0 - self.stop_height)
        if disc < 0:
            return 0.5
        return float((vz + np.sqrt(disc)) / G)

    def position(self, t: float) -> np.ndarray:
        return self.p0 + self.v0 * t + 0.5 * np.array([0.0, 0.0, -G]) * t * t

    def frames(self) -> Iterator[Frame]:
        import cv2

        t_end = self._flight_time()
        n = max(1, int(t_end * self.fps))
        prev_uv = None

        for i in range(self.idle_frames):
            img = self._background.copy()
            if self.noise:
                img = np.clip(
                    img.astype(np.int16)
                    + self.rng.normal(0, self.noise, img.shape).astype(np.int16),
                    0, 255,
                ).astype(np.uint8)
            self.truth.append((i - self.idle_frames, i / self.fps,
                               np.nan, np.nan, 0))
            yield Frame(i - self.idle_frames, i / self.fps, img)

        for i in range(n):
            t = i / self.fps
            p = self.position(t)
            uv = self.cam.project(p)
            depth = self.cam.world_to_cam(p)[2]

            img = self._background.copy()
            if self.noise:
                img = np.clip(
                    img.astype(np.int16)
                    + self.rng.normal(0, self.noise, img.shape).astype(np.int16),
                    0, 255,
                ).astype(np.uint8)

            visible = bool(np.all(np.isfinite(uv))) and depth > 0.1
            if visible:
                # Apparent radius from real size and depth -- so the object
                # grows as it approaches, exactly as it will in the field.
                r_px = max(2, int(self.cam.fx * self.radius_m / depth))
                u, v = int(round(uv[0])), int(round(uv[1]))
                if -r_px < u < self.w + r_px and -r_px < v < self.h + r_px:
                    overlay = img.copy()
                    cv2.circle(overlay, (u, v), r_px, (40, 190, 220), -1)
                    cv2.circle(overlay, (u, v), r_px, (20, 120, 160), max(1, r_px // 5))
                    if self.blur_px > 0 and prev_uv is not None:
                        # Smear along the direction of travel, as exposure does.
                        k = max(3, int(self.blur_px) | 1)
                        ker = np.zeros((k, k), np.float32)
                        d = np.array(uv) - np.array(prev_uv)
                        ang = np.arctan2(d[1], d[0])
                        cx = ck = k // 2
                        for s in range(k):
                            x = int(round(cx + (s - ck) * np.cos(ang)))
                            y = int(round(ck + (s - ck) * np.sin(ang)))
                            if 0 <= x < k and 0 <= y < k:
                                ker[y, x] = 1
                        ker /= max(1.0, ker.sum())
                        overlay = cv2.filter2D(overlay, -1, ker)
                    img = overlay
                    self.truth.append((i, t, float(uv[0]), float(uv[1]), r_px))
                else:
                    visible = False
            if not visible:
                self.truth.append((i, t, np.nan, np.nan, 0))

            if self.clutter and i % 3 == 0:
                # A slow distractor: violates the ballistic model, so the
                # gate downstream should reject it.
                cy = int(self.h * 0.6 + 40 * np.sin(i * 0.2))
                cx2 = int(self.w * 0.15 + i * 2)
                cv2.circle(img, (cx2, cy), 14, (200, 120, 200), -1)

            prev_uv = uv if visible else None
            yield Frame(i, t, img)

    def truth_uv(self, index: int):
        for i, _t, u, v, _r in self.truth:
            if i == index:
                return (u, v)
        return (np.nan, np.nan)


class PiCamera2Source(CaptureSource):
    """Real camera. Import is deferred so this module loads anywhere."""

    def __init__(self, size=(1640, 1232), fps=40.0, exposure_us=1000,
                 gain=4.0, max_frames: Optional[int] = None):
        from picamera2 import Picamera2  # noqa: PLC0415 - Pi-only dependency

        self.max_frames = max_frames
        self.picam2 = Picamera2()
        dur = int(1_000_000 / fps)
        cfg = self.picam2.create_video_configuration(
            main={"size": size, "format": "RGB888"},
            controls={
                "ExposureTime": int(exposure_us),
                "AnalogueGain": float(gain),
                # Auto-exposure and auto-white-balance hunt between frames,
                # which a background subtractor reads as global motion. Both
                # must be locked for differencing to mean anything.
                "AeEnable": False,
                "AwbEnable": False,
                "FrameDurationLimits": (dur, dur),
            },
        )
        self.picam2.configure(cfg)
        self.picam2.start()

    def frames(self) -> Iterator[Frame]:
        i = 0
        while self.max_frames is None or i < self.max_frames:
            request = self.picam2.capture_request()
            try:
                img = request.make_array("main")
                meta = request.get_metadata()
                # Sensor timestamp, not wall clock: the latter carries Python
                # scheduling jitter that the trajectory fit would read as motion.
                ts = meta.get("SensorTimestamp")
                t = ts / 1e9 if ts else i / 40.0
            finally:
                request.release()
            yield Frame(i, t, img)
            i += 1

    def close(self) -> None:
        try:
            self.picam2.stop()
        finally:
            self.picam2.close()

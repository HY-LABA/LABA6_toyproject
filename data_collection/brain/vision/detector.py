"""Classical motion detection.

No neural network, and none is wanted: a thrown object is the only fast-moving
thing in the frame, so background subtraction finds it in a couple of
milliseconds with zero training data. That is what makes the whole
auto-labelling scheme possible -- the detector has to work *before* any
labelled data exists, so it cannot itself be learned.

The detector deliberately does not know what trash is. It answers "which
pixels are moving", and nothing more.
"""

from __future__ import annotations

import time
from typing import List, Optional, Tuple

import cv2
import numpy as np


class Detection:
    __slots__ = ("u", "v", "bbox", "area")

    def __init__(self, u: float, v: float, bbox: Tuple[int, int, int, int], area: int):
        self.u = u
        self.v = v
        self.bbox = bbox  # x, y, w, h
        self.area = area

    @property
    def uv(self) -> Tuple[float, float]:
        return (self.u, self.v)

    def crop(self, image: np.ndarray, out_size: int = 64,
             pad: float = 1.4) -> np.ndarray:
        """Square crop centred on the blob, padded and resized.

        Square and centred on purpose: the classifier should key on the object,
        not on how the detector happened to frame it. `pad` leaves a margin of
        context, which helps far more than a tight crop at these pixel sizes.
        """
        x, y, w, h = self.bbox
        side = int(max(w, h) * pad)
        side = max(side, 8)
        cx, cy = int(round(self.u)), int(round(self.v))
        x0, y0 = cx - side // 2, cy - side // 2
        x1, y1 = x0 + side, y0 + side

        H, W = image.shape[:2]
        # Pad rather than clamp: clamping would shift the object off-centre
        # for anything near the frame edge, which is exactly where fast
        # objects are.
        px0, py0 = max(0, -x0), max(0, -y0)
        px1, py1 = max(0, x1 - W), max(0, y1 - H)
        sub = image[max(0, y0):min(H, y1), max(0, x0):min(W, x1)]
        if px0 or py0 or px1 or py1:
            sub = cv2.copyMakeBorder(sub, py0, py1, px0, px1,
                                     cv2.BORDER_REPLICATE)
        if sub.size == 0:
            return np.zeros((out_size, out_size, image.shape[2]), image.dtype)
        return cv2.resize(sub, (out_size, out_size), interpolation=cv2.INTER_AREA)

    def __repr__(self):
        return f"Detection(u={self.u:.1f}, v={self.v:.1f}, area={self.area})"


class MotionDetector:
    def __init__(self, min_area: int = 300, max_area: int = 60000,
                 history: int = 300, var_threshold: float = 32.0,
                 warmup: int = 10, max_detections: int = 4, scale: int = 2):
        """Areas are given in full-resolution pixels regardless of `scale`.

        The area window must span the object's *whole* approach, not its
        typical size. A 10cm object at 1640x1232 grows from ~900 px^2 at 4m to
        ~38000 px^2 at 0.6m; a ceiling set for the far end silently drops
        every close-range frame, which are the most detailed crops available
        and the ones worth training on. 60000 still rejects person-sized
        blobs, which run into the hundreds of thousands of pixels here.

        `scale` downsamples before background subtraction. MOG2 cost is linear
        in pixel count, so scale=2 is ~4x cheaper -- and detection loses
        nothing, because all it needs is a centroid and a bounding box. The
        classifier crop is still taken from the full-resolution frame, so
        detail is only discarded where it was never used. Measured at
        1640x1232: ~8.9ms at scale=1 versus ~2.5ms at scale=2.
        """
        self.min_area = min_area
        self.max_area = max_area
        self.warmup = warmup
        self.max_detections = max_detections
        self.scale = max(1, int(scale))
        self._n = 0
        self.last_ms = 0.0
        # Diagnostics. A frame that is too dark, or a mask with foreground
        # pixels that never survive the area filter, both look identical from
        # the outside -- "no detections" -- but need opposite fixes.
        self.last_mean = 0.0
        self.last_fg_px = 0
        self.last_blobs = 0

        self._bg = cv2.createBackgroundSubtractorMOG2(
            history=history, varThreshold=var_threshold,
            # Shadow classification costs time and produces a third label we
            # would only throw away.
            detectShadows=False,
        )
        self._k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    @property
    def ready(self) -> bool:
        return self._n >= self.warmup

    def detect(self, image: np.ndarray) -> List[Detection]:
        t0 = time.perf_counter()
        s = self.scale
        work = image if s == 1 else cv2.resize(
            image, (image.shape[1] // s, image.shape[0] // s),
            interpolation=cv2.INTER_AREA,
        )
        mask = self._bg.apply(work)
        self._n += 1

        # Open kills speckle; close re-joins a blob split by the object's own
        # highlight or by motion blur.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._k_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._k_close)

        self.last_mean = float(work.mean())
        self.last_fg_px = int(cv2.countNonZero(mask))

        out: List[Detection] = []
        if self.ready:
            n, _lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
            self.last_blobs = n - 1  # excluding background
            # Thresholds are quoted full-resolution, so convert once here
            # rather than making every caller think about `scale`.
            lo = self.min_area / (s * s)
            hi = self.max_area / (s * s)
            for i in range(1, n):  # 0 is background
                area = int(stats[i, cv2.CC_STAT_AREA])
                if not (lo <= area <= hi):
                    continue
                x = int(stats[i, cv2.CC_STAT_LEFT]) * s
                y = int(stats[i, cv2.CC_STAT_TOP]) * s
                w = int(stats[i, cv2.CC_STAT_WIDTH]) * s
                h = int(stats[i, cv2.CC_STAT_HEIGHT]) * s
                out.append(Detection(float(cents[i][0]) * s, float(cents[i][1]) * s,
                                     (x, y, w, h), area * s * s))
            out.sort(key=lambda d: d.area, reverse=True)
            out = out[: self.max_detections]

        self.last_ms = (time.perf_counter() - t0) * 1000.0
        return out

    def reset(self) -> None:
        self._n = 0

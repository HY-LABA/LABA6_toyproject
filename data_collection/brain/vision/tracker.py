"""Link detections into tracks, then let physics decide which are real.

Association is deliberately simple -- greedy nearest-neighbour inside a radius.
Sophisticated data association solves a problem this scene does not have: there
is normally one fast object, and the expensive discrimination happens
afterwards, in the ballistic gate.

That gate is the important part. A thrown object follows a parabola under
gravity; an arm, a shadow, a swaying curtain and a person walking do not. So
the trajectory fit doubles as a classifier of *tracks*, which is what lets
crops be auto-labelled: only tracks that fly get the block's label.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..planning.camera import Camera
from ..planning.trajectory import MIN_OBSERVATIONS, Trajectory, fit_projectile
from .detector import Detection

PROJECTILE = "projectile"
DISTRACTOR = "distractor"
UNDECIDED = "undecided"


class Track:
    __slots__ = ("id", "frames", "times", "detections", "last_frame", "verdict",
                 "trajectory", "residual_px")

    def __init__(self, track_id: int):
        self.id = track_id
        self.frames: List[int] = []
        self.times: List[float] = []
        self.detections: List[Detection] = []
        self.last_frame = -1
        self.verdict = UNDECIDED
        self.trajectory: Optional[Trajectory] = None
        self.residual_px = float("inf")

    def add(self, frame_idx: int, t: float, det: Detection) -> None:
        self.frames.append(frame_idx)
        self.times.append(t)
        self.detections.append(det)
        self.last_frame = frame_idx

    @property
    def n(self) -> int:
        return len(self.detections)

    @property
    def last_uv(self) -> Tuple[float, float]:
        return self.detections[-1].uv

    @property
    def uvs(self) -> np.ndarray:
        return np.array([d.uv for d in self.detections], dtype=float)

    def __repr__(self):
        return f"Track(#{self.id}, n={self.n}, {self.verdict}, r={self.residual_px:.1f}px)"


class TrackManager:
    def __init__(self, cam: Camera, max_link_px: float = 80.0,
                 max_missing: int = 3, min_detections: int = 6,
                 max_residual_px: float = 6.0):
        self.cam = cam
        self.max_link_px = max_link_px
        self.max_missing = max_missing
        self.min_detections = min_detections
        self.max_residual_px = max_residual_px
        self._active: List[Track] = []
        self._next_id = 0

    @property
    def active(self) -> List[Track]:
        return list(self._active)

    def update(self, frame_idx: int, t: float,
               detections: Sequence[Detection]) -> List[Track]:
        """Feed one frame. Returns tracks that closed on this frame."""
        unmatched = list(detections)

        # Greedy nearest-neighbour: for each track, take the closest detection
        # within the link radius.
        for track in self._active:
            if not unmatched:
                break
            lu, lv = track.last_uv
            best_i, best_d = -1, self.max_link_px
            for i, det in enumerate(unmatched):
                d = float(np.hypot(det.u - lu, det.v - lv))
                if d < best_d:
                    best_i, best_d = i, d
            if best_i >= 0:
                track.add(frame_idx, t, unmatched.pop(best_i))

        for det in unmatched:
            tr = Track(self._next_id)
            self._next_id += 1
            tr.add(frame_idx, t, det)
            self._active.append(tr)

        closed = [tr for tr in self._active
                  if frame_idx - tr.last_frame > self.max_missing]
        self._active = [tr for tr in self._active if tr not in closed]
        for tr in closed:
            self._judge(tr)
        return closed

    def close_all(self) -> List[Track]:
        closed = self._active
        self._active = []
        for tr in closed:
            self._judge(tr)
        return closed

    def _judge(self, track: Track) -> None:
        """Decide projectile vs distractor by how well a parabola explains it."""
        if track.n < max(self.min_detections, MIN_OBSERVATIONS):
            track.verdict = DISTRACTOR
            return
        fit = fit_projectile(self.cam, track.times, track.uvs)
        track.residual_px = fit.residual_px
        if fit.ok and fit.residual_px <= self.max_residual_px:
            track.verdict = PROJECTILE
            track.trajectory = Trajectory(self.cam, fit)
        else:
            track.verdict = DISTRACTOR

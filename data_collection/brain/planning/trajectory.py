"""Monocular projectile estimation, scaled by gravity.

A single camera cannot recover absolute scale from image motion alone: a
small ball nearby and a large one far away trace identical pixels. Gravity
breaks the tie. The object obeys

    p(t) = p0 + v0*t + 0.5*g*t^2

with |g| known in real units, so the vertical curvature of the observed track
fixes the depth. Six unknowns (p0, v0), two equations per observation, so
three detections already determine the full 3D trajectory.

Why this is a *linear* solve
----------------------------
Writing the pinhole projection as a cross-product residual instead of a
division removes the nonlinearity. With a = u - cx and b = v - cy:

    a * Z(t) - fx * X(t) = 0
    b * Z(t) - fy * Y(t) = 0

X, Y and Z are affine in the unknowns, so both equations are linear in
[X0, Vx, Y0, Vy, Z0, Vz]; the known gravity terms move to the right-hand
side and become the inhomogeneous part -- which is precisely why the system
has a unique scale rather than a one-parameter family of solutions.

That means no iteration, no initial guess, and no convergence failure in the
hot loop: one least-squares solve of a 2N x 6 system, microseconds at the
sizes involved. It minimises algebraic rather than reprojection error, which
is a real but small approximation at these noise levels.

NOTE: this supersedes the ground-plane-homography approach sketched in the
build plan. A homography maps image points to the floor *assuming the point
lies on the floor*; applied to an airborne object it returns the point where
the camera ray meets z=0, which sits well beyond the true landing spot and
gets worse the higher the object flies.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from .camera import G, Camera

# A fit whose observations span too little time, or which sits near the
# degenerate configuration, produces a wild depth. These are the guards.
MIN_OBSERVATIONS = 4
MIN_TIME_SPAN_S = 0.04
MAX_CONDITION = 1e9


class ProjectileFit:
    """Result of fitting a parabola to image observations.

    p0 and v0 are in *camera* coordinates at the reference time t0.
    """

    __slots__ = ("p0", "v0", "t0", "residual_px", "condition", "n")

    def __init__(self, p0, v0, t0, residual_px, condition, n):
        self.p0 = p0
        self.v0 = v0
        self.t0 = t0
        self.residual_px = residual_px
        self.condition = condition
        self.n = n

    @property
    def ok(self) -> bool:
        """Depth must be positive -- a fit that puts the object behind the
        camera is not a near-miss, it is a failed solve."""
        return (
            np.all(np.isfinite(self.p0))
            and np.all(np.isfinite(self.v0))
            and self.p0[2] > 0
            and self.condition < MAX_CONDITION
        )

    def position_at(self, dt: float) -> np.ndarray:
        """Camera-frame position `dt` seconds after t0."""
        raise NotImplementedError  # needs gravity; see Trajectory.position_at

    def __repr__(self):
        return (f"ProjectileFit(depth={self.p0[2]:.2f}m, "
                f"resid={self.residual_px:.2f}px, n={self.n})")


def fit_projectile(cam: Camera, times: Sequence[float],
                   uvs: Sequence[Sequence[float]]) -> ProjectileFit:
    """Fit p0, v0 in camera coordinates from timestamped pixel detections.

    `times` are seconds; only differences matter. They are re-referenced to
    the first sample internally -- feeding raw CLOCK_MONOTONIC values would
    square to ~1e12 and wreck the conditioning of the normal equations.
    """
    t = np.asarray(times, dtype=float)
    uv = np.asarray(uvs, dtype=float)
    if t.ndim != 1 or uv.shape != (t.size, 2):
        raise ValueError("times must be (N,) and uvs (N, 2)")

    t0 = float(t[0])
    tr = t - t0

    g = cam.gravity_in_cam()
    a = uv[:, 0] - cam.cx
    b = uv[:, 1] - cam.cy
    n = t.size

    # Unknowns: [X0, Vx, Y0, Vy, Z0, Vz]
    A = np.zeros((2 * n, 6))
    rhs = np.zeros(2 * n)
    half_t2 = 0.5 * tr * tr

    # a*Z - fx*X = 0
    A[0::2, 0] = -cam.fx
    A[0::2, 1] = -cam.fx * tr
    A[0::2, 4] = a
    A[0::2, 5] = a * tr
    rhs[0::2] = half_t2 * (cam.fx * g[0] - a * g[2])

    # b*Z - fy*Y = 0
    A[1::2, 2] = -cam.fy
    A[1::2, 3] = -cam.fy * tr
    A[1::2, 4] = b
    A[1::2, 5] = b * tr
    rhs[1::2] = half_t2 * (cam.fy * g[1] - b * g[2])

    sol, _res, _rank, sv = np.linalg.lstsq(A, rhs, rcond=None)
    condition = float(sv[0] / sv[-1]) if sv.size and sv[-1] > 0 else np.inf

    p0 = np.array([sol[0], sol[2], sol[4]])
    v0 = np.array([sol[1], sol[3], sol[5]])

    # Report error where it is interpretable: pixels of reprojection.
    resid = _reprojection_rms(cam, p0, v0, g, tr, uv)
    return ProjectileFit(p0, v0, t0, resid, condition, n)


def _reprojection_rms(cam, p0, v0, g, tr, uv) -> float:
    pts = p0[None, :] + v0[None, :] * tr[:, None] + 0.5 * g[None, :] * (tr * tr)[:, None]
    z = pts[:, 2]
    if np.any(z <= 1e-9):
        return float("inf")
    u = cam.fx * pts[:, 0] / z + cam.cx
    v = cam.fy * pts[:, 1] / z + cam.cy
    d = np.stack([u, v], axis=1) - uv
    return float(np.sqrt(np.mean(np.sum(d * d, axis=1))))


class Trajectory:
    """A fitted trajectory expressed in world coordinates."""

    __slots__ = ("p0", "v0", "t0", "residual_px", "n")

    def __init__(self, cam: Camera, fit: ProjectileFit):
        self.p0 = cam.cam_to_world(fit.p0)
        self.v0 = cam.R.T @ fit.v0  # a direction: rotate only, no translation
        self.t0 = fit.t0
        self.residual_px = fit.residual_px
        self.n = fit.n

    def position_at(self, t_abs: float) -> np.ndarray:
        dt = t_abs - self.t0
        return self.p0 + self.v0 * dt + 0.5 * np.array([0.0, 0.0, -G]) * dt * dt

    def crossing_time(self, height: float) -> Optional[float]:
        """Absolute time the object descends through `height`, or None.

        Solves 0.5*G*dt^2 - vz*dt - (z0 - h) = 0 and takes the later root:
        a rising object crosses the plane twice and we want the descent.
        """
        z0, vz = self.p0[2], self.v0[2]
        disc = vz * vz + 2.0 * G * (z0 - height)
        if disc < 0:
            return None  # apex is below the catch plane; never gets there
        dt = (vz + np.sqrt(disc)) / G
        return None if dt < 0 else self.t0 + dt

    def landing(self, height: float = 0.0) -> Optional[Tuple[float, float, float]]:
        """(x, y, t_abs) where the object crosses `height` on the way down."""
        t = self.crossing_time(height)
        if t is None:
            return None
        p = self.position_at(t)
        return float(p[0]), float(p[1]), float(t)

    def __repr__(self):
        return (f"Trajectory(p0={np.round(self.p0, 2)}, "
                f"v0={np.round(self.v0, 2)}, resid={self.residual_px:.2f}px)")


class ProjectileTracker:
    """Accumulates detections and maintains a running landing estimate.

    Refits from scratch each frame rather than updating incrementally. At
    these sizes the solve is microseconds, and a batch fit enforces the
    physics exactly instead of letting process noise drift the parabola --
    which matters more than the saved cycles.
    """

    def __init__(self, cam: Camera, catch_height: float = 0.0,
                 window: int = 40, max_residual_px: float = 6.0):
        self.cam = cam
        self.catch_height = catch_height
        self.window = window
        self.max_residual_px = max_residual_px
        self._t: List[float] = []
        self._uv: List[Tuple[float, float]] = []
        self.trajectory: Optional[Trajectory] = None

    def reset(self) -> None:
        self._t.clear()
        self._uv.clear()
        self.trajectory = None

    @property
    def n_observations(self) -> int:
        return len(self._t)

    def add(self, t: float, uv) -> Optional[Trajectory]:
        """Add one detection; returns the updated trajectory if usable."""
        self._t.append(float(t))
        self._uv.append((float(uv[0]), float(uv[1])))
        if len(self._t) > self.window:
            del self._t[0]
            del self._uv[0]

        self.trajectory = None
        if len(self._t) < MIN_OBSERVATIONS:
            return None
        if self._t[-1] - self._t[0] < MIN_TIME_SPAN_S:
            # Too short a baseline: curvature is unmeasurable and depth
            # explodes. Better to report nothing than a confident wrong number.
            return None

        fit = fit_projectile(self.cam, self._t, self._uv)
        if not fit.ok or fit.residual_px > self.max_residual_px:
            return None

        self.trajectory = Trajectory(self.cam, fit)
        return self.trajectory

    def landing(self) -> Optional[Tuple[float, float, float]]:
        if self.trajectory is None:
            return None
        return self.trajectory.landing(self.catch_height)

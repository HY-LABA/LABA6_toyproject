"""Pinhole camera model: intrinsics, extrinsics, and projection.

Frames follow robot-software.md section 4:

    world  -- floor plane z=0, +x forward, +y left, +z up
    camera -- OpenCV convention: +x right, +y down, +z forward (optical axis)
    image  -- (u, v) pixels, origin top-left

The world->camera transform is p_cam = R @ p_world + t.

Lens distortion is deliberately absent. The detector works on the raw frame
and we undistort the handful of detected points, not the image -- undistorting
whole frames at 60fps is a waste of the Pi's time when only a few pixels per
frame matter. Add the point-undistort here when calibration provides
coefficients; the estimator consumes normalised points either way.
"""

from __future__ import annotations

import numpy as np

# Magnitude of gravity. This is not a tuning constant -- it is what gives a
# single camera absolute scale (see trajectory.py), so it must be in real
# units and must match the units of every other length in the system: metres.
G = 9.81


class Camera:
    """Calibrated pinhole camera."""

    def __init__(self, fx, fy, cx, cy, R=None, t=None, width=None, height=None):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.R = np.eye(3) if R is None else np.asarray(R, dtype=float)
        self.t = np.zeros(3) if t is None else np.asarray(t, dtype=float)
        self.width = width
        self.height = height

    # --- construction -----------------------------------------------------

    @classmethod
    def looking_down(cls, fx, fy, cx, cy, altitude, tilt_deg, **kw):
        """Camera at (0, 0, `altitude`) in world, pitched `tilt_deg` below horizontal.

        Negative `tilt_deg` looks upward, which is what a trashcan watching an
        incoming arc actually wants.

        `altitude` is the mounting height in metres -- deliberately not named
        `height`, which on the constructor means image height in pixels.

        Convenience for synthetic tests and a sane starting guess before real
        extrinsic calibration exists.
        """
        th = np.radians(tilt_deg)
        # World axes expressed in camera axes. Camera looks along world +x with
        # +z_cam forward, +y_cam down, +x_cam right (world -y).
        R = np.array([
            [0.0, -1.0, 0.0],
            [-np.sin(th), 0.0, -np.cos(th)],
            [np.cos(th), 0.0, -np.sin(th)],
        ])
        t = -R @ np.array([0.0, 0.0, float(altitude)])
        return cls(fx, fy, cx, cy, R=R, t=t, **kw)

    # --- geometry ---------------------------------------------------------

    def world_to_cam(self, p_world) -> np.ndarray:
        p = np.asarray(p_world, dtype=float)
        return (self.R @ p.T).T + self.t if p.ndim > 1 else self.R @ p + self.t

    def cam_to_world(self, p_cam) -> np.ndarray:
        p = np.asarray(p_cam, dtype=float)
        Rt = self.R.T
        return (Rt @ (p - self.t).T).T if p.ndim > 1 else Rt @ (p - self.t)

    def project(self, p_world):
        """World point(s) -> pixel(s). Points behind the camera give NaN."""
        pc = np.atleast_2d(self.world_to_cam(np.atleast_2d(p_world)))
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.fx * pc[:, 0] / z + self.cx
            v = self.fy * pc[:, 1] / z + self.cy
        behind = z <= 1e-9
        u = np.where(behind, np.nan, u)
        v = np.where(behind, np.nan, v)
        out = np.stack([u, v], axis=-1)
        return out[0] if np.ndim(p_world) == 1 else out

    def in_view(self, uv) -> bool:
        if self.width is None or self.height is None:
            return bool(np.all(np.isfinite(uv)))
        u, v = np.asarray(uv, dtype=float)[..., 0], np.asarray(uv, dtype=float)[..., 1]
        return bool(np.all(np.isfinite([u, v])) and 0 <= u < self.width
                    and 0 <= v < self.height)

    def gravity_in_cam(self) -> np.ndarray:
        """Gravity vector expressed in camera axes.

        The estimator works in camera coordinates, so the world -Z gravity has
        to be rotated in. Getting this wrong tilts every predicted parabola.
        """
        return self.R @ np.array([0.0, 0.0, -G])

    def __repr__(self):
        return (f"Camera(fx={self.fx:.1f}, fy={self.fy:.1f}, "
                f"cx={self.cx:.1f}, cy={self.cy:.1f})")

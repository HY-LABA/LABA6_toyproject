"""궤적 예측 + 재보정. (pi/trajectory.py 물리 공식 재활용 — 재보정은 TODO)

포물선 공식으로 낙하지점/도달시간 예측. 의존성: math, numpy.
"""

from __future__ import annotations
import math
import numpy as np


def solve_landing_time(z0, vz, g, z_catch):
    """z(t)=z_catch 가 되는 미래(t>0) 시각. 없으면 None."""
    a = 0.5 * g
    b = -vz
    c = z_catch - z0
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    sqrt_d = math.sqrt(disc)
    t1 = (-b - sqrt_d) / (2 * a)
    t2 = (-b + sqrt_d) / (2 * a)
    candidates = sorted(t for t in (t1, t2) if t > 1e-6)
    if not candidates:
        return None
    return candidates[-1]


def predict_landing(position, velocity, g, z_catch):
    """현재 (위치, 속도)에서 착지 예측 -> (point[x,y,z], time_to_land, valid)."""
    position = np.asarray(position, dtype=float).reshape(3)
    velocity = np.asarray(velocity, dtype=float).reshape(3)
    x0, y0, z0 = position
    vx, vy, vz = velocity

    t_land = solve_landing_time(z0, vz, g, z_catch)
    if t_land is None:
        return np.array([x0, y0, z_catch]), 0.0, False

    return np.array([x0 + vx * t_land, y0 + vy * t_land, z_catch]), t_land, True


# TODO: 재보정 — 엔코더 이동량 + 카메라 재관측 오차 반영해 궤적 재계산 (architecture.md)

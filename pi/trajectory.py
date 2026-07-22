"""포물선 궤적 예측 — 착지 시간/좌표 공식.

받는 평면 z = z_catch 에 도달하는 시점과 (x,y)를 물리 공식으로 계산.
    z(t) = z0 + vz*t - 0.5*g*t^2 = z_catch  를 t에 대해 풀고,
    하강하며 지나는 양의 근을 착지 시간으로 택한다.

의존성: math, numpy 뿐 (독립 실행 가능).
반환은 튜플 (point(3,), time_to_land, valid) — 외부 타입 의존 없음.
"""

from __future__ import annotations

import math

import numpy as np


def solve_landing_time(z0: float, vz: float, g: float, z_catch: float) -> float | None:
    """z(t)=z_catch 가 되는 미래(t>0) 시각. 없으면 None.

    0.5*g*t^2 - vz*t + (z_catch - z0) = 0 의 근 중
    물리적으로 유효한(미래, 하강 우선) 근을 고른다.
    """
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


def predict_landing(
    position: np.ndarray,
    velocity: np.ndarray,
    g: float,
    z_catch: float,
) -> tuple[np.ndarray, float, bool]:
    """현재 (위치, 속도)에서 착지 예측.

    Returns:
        (point(3,)=[x, y, z_catch], time_to_land, valid)
    """
    position = np.asarray(position, dtype=float).reshape(3)
    velocity = np.asarray(velocity, dtype=float).reshape(3)
    x0, y0, z0 = position
    vx, vy, vz = velocity

    t_land = solve_landing_time(z0, vz, g, z_catch)
    if t_land is None:
        return np.array([x0, y0, z_catch]), 0.0, False

    land_x = x0 + vx * t_land           # 수평은 등속
    land_y = y0 + vy * t_land
    return np.array([land_x, land_y, z_catch]), t_land, True

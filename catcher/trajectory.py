"""④ 착지 예측 — 궤적 상태에서 "어디에 언제 떨어지는가"를 푼다.

순수 함수. 상태 없음.

    Z(t) = Z + VZ·t − 0.5·g·t²  =  Z_CATCH   를 t에 대해 푼다.

Z_CATCH는 착지 판정 평면이다. 카메라를 통 입구 림 높이에 두므로 Z_CATCH = 0이고,
물체가 그 평면을 지날 때 통 입구 안에 있으면 캐치 성공이다.
"""

from __future__ import annotations

import math

import numpy as np

from datatypes import Landing


def solve_landing_time(z0: float, vz: float, g: float, z_catch: float) -> float | None:
    """z(t) = z_catch 가 되는 미래(t > 0) 시각. 없으면 None.

    양의 실근이 둘이면 **큰 쪽**을 택한다 — 위로 던져 올린 경우 하강 구간에서 받아야
    하기 때문이다.
    """
    a = 0.5 * g
    b = -vz
    c = z_catch - z0

    if abs(a) < 1e-12:                      # g=0인 퇴화 케이스
        return None if abs(b) < 1e-12 else (-c / b if -c / b > 1e-6 else None)

    disc = b * b - 4 * a * c
    if disc < 0:                            # 물체가 z_catch까지 내려오지 않는다
        return None

    sqrt_d = math.sqrt(disc)
    roots = [(-b - sqrt_d) / (2 * a), (-b + sqrt_d) / (2 * a)]
    positive = sorted(t for t in roots if t > 1e-6)
    return positive[-1] if positive else None


def predict_landing(
    position: np.ndarray, velocity: np.ndarray, g: float, z_catch: float
) -> Landing:
    """world frame 상태 -> 착지점(x, y) + 남은 시간."""
    x0, y0, z0 = (float(v) for v in position)
    vx, vy, vz = (float(v) for v in velocity)

    t_land = solve_landing_time(z0, vz, g, z_catch)
    if t_land is None:
        return Landing(x=x0, y=y0, t_land=0.0, valid=False)

    return Landing(x=x0 + vx * t_land, y=y0 + vy * t_land, t_land=t_land, valid=True)

"""단안 포물선 궤적 추정 — **순수 계산만. 상태를 갖지 않는다.**

관측을 누적하고 어느 검출이 이 트랙인지 고르는 일은 `tracker.py`가 한다.
여기 함수들은 전부 "입력을 주면 답이 나오는" 형태라 하드웨어 없이 테스트된다.

무엇을 푸는가
-------------
사진 한 장은 깊이를 담지 못한다. 작은 물체가 가까이 있는 것과 큰 물체가 멀리 있는
것이 완전히 똑같은 픽셀을 만들기 때문이다.

**중력을 자로 쓴다.** 중력은 물체가 뭐든 항상 9.8 m/s²이므로, "이 궤적이 9.8의
중력으로 만들어지려면 거리가 얼마여야 하는가"를 역으로 풀면 스케일이 확정된다.
미지수 6개(t=0 시점의 위치 3 + 속도 3), 관측 하나당 식 2개 → 3관측이면 풀리고,
그 이상은 최소제곱이 노이즈를 눌러준다.

왜 *선형* 풀이인가
------------------
핀홀 투영을 나눗셈(u = fx·X/Z + cx)이 아니라 외적 잔차로 쓰면 비선형성이 사라진다.
a = u − cx, b = v − cy 로 두면

    a·Z(t) − fx·X(t) = 0
    b·Z(t) − fy·Y(t) = 0

X, Y, Z는 미지수에 대해 1차이므로 위 두 식도 [X0,Vx,Y0,Vy,Z0,Vz]에 대해 1차다.
이미 아는 중력항만 우변으로 넘어가 **비제차항**이 되는데, 바로 그것 때문에 해가
"한 배율만큼 자유로운 무한개"가 아니라 유일한 스케일을 갖는다.

반복도, 초기추정도, 수렴실패도 없다. 2N×6 최소제곱 한 번이면 끝난다.

★ 카메라가 움직이면 보정해야 한다
---------------------------------
카메라가 쓰레기통에 실려 있으므로 **로봇이 출발하는 순간 "카메라 고정" 전제가
깨진다.** 보정 없이 넣으면 관측에 물체의 운동과 카메라의 운동이 섞여서 포물선이
성립하지 않는다 — 합성 검증에서 로봇 1.2 m/s일 때 깊이가 참값의 8%로 무너지고
잔차가 16 px까지 치솟아 `Fit.ok`가 거짓이 됐다.

고치는 법은 우변에 항 하나를 더하는 것뿐이다. X_cam = X_world − camX(t) 이므로

    a·Z − fx·X_world = −fx·camX(t)

좌변 행렬은 그대로고 **미지수의 의미만 카메라 좌표에서 월드 좌표로 바뀐다.**
`cams`(각 관측 시점의 카메라 월드 위치)를 넘기면 이 보정이 적용된다.

좌표계
------
+Z는 광축(카메라가 보는 위쪽). X는 좌우, Y는 앞뒤.
`cams`를 넘기면 **월드 좌표계**(트랙 시작 시점의 로봇 중심이 원점)로 풀리고,
안 넘기면 카메라 좌표계로 풀린다(카메라가 안 움직인다는 가정).
착지 기준면은 `config.CATCH_HEIGHT_M`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import config


@dataclass
class Fit:
    """한 번의 최소제곱 결과. p0/v0는 t0 시점의 값."""

    p0: np.ndarray          # (3,) X0, Y0, Z0
    v0: np.ndarray          # (3,) Vx, Vy, Vz
    t0: float               # 기준 시각(첫 관측의 절대시각)
    residual_px: float      # 재투영 RMS 오차
    condition: float        # 행렬 조건수 — 시간 스팬이 짧으면 폭발한다
    n: int                  # 사용한 관측 수

    @property
    def ok(self) -> bool:
        """**형태가** 말이 되는 해인가. 검사 네 가지:

          ① 유한성          최소제곱이 발산해 nan/inf 가 나오지 않았나
          ② Z_RANGE_M       추정 거리가 물리적으로 말이 되나. 정지한 오탐은 "아주
                            멀리서 따라오는 물체"로 해석돼 z가 수십 m로 튀는데,
                            이 범위가 그걸 잘라낸다
          ③ MAX_CONDITION   문제가 잘 풀리는 상태인가. 시간 스팬이 짧거나 궤적이
                            거의 직선이면 폭발한다
          ④ MAX_RESIDUAL_PX 추정 궤적을 다시 화면에 투영했을 때 관측과 몇 px
                            어긋나는가. **이게 탄도 게이트다** — 정지한 점은 어떤
                            포물선으로도 설명이 안 되므로 잔차가 무한대로 튄다

        ⚠ **깊이가 정확한가는 여기서 판정하지 못한다.** 스케일이 틀린 궤적도 화면에는
          잘 맞게 투영되기 때문이다 (그건 tracker.Tracker._depth_converged 가 본다).
        """
        lo, hi = config.Z_RANGE_M
        return (
            bool(np.all(np.isfinite(self.p0)))
            and bool(np.all(np.isfinite(self.v0)))
            and lo <= self.p0[2] <= hi
            and self.condition < config.MAX_CONDITION
            and self.residual_px <= config.MAX_RESIDUAL_PX
        )

    def position_at(self, dt: float) -> np.ndarray:
        """t0로부터 dt초 뒤의 위치."""
        return self.p0 + self.v0 * dt + 0.5 * gravity_cam() * dt * dt

    def __repr__(self) -> str:
        return (f"Fit(z={self.p0[2]:.2f}m v={np.round(self.v0, 2)} "
                f"resid={self.residual_px:.2f}px n={self.n})")


def gravity_cam() -> np.ndarray:
    """카메라 좌표계에서 본 중력 벡터.

    카메라가 정확히 위를 보면 중력은 광축의 반대방향, 즉 (0,0,−g)다.
    기울여 달았다면 config.GRAVITY_CAM을 회전시킨 값으로 바꿔주면 아래 수식은
    그대로 성립한다 — 중력 방향이 상수로만 들어가기 때문이다.
    """
    return np.asarray(config.GRAVITY_CAM, dtype=float)


def fit_trajectory(times, uvs, cams=None) -> Fit:
    """타임스탬프 붙은 픽셀 관측들로 p0, v0를 푼다.

    times : (N,)   초 단위. 차이만 의미가 있다
    uvs   : (N,2)  왜곡 보정된 픽셀 좌표
    cams  : (N,2)  각 관측 시점의 **카메라 월드 위치**. None이면 0으로 본다
                   (= 카메라가 안 움직였다는 가정)

    `times`는 내부에서 첫 관측 기준으로 다시 맞춘다 — CLOCK_MONOTONIC 원값을 그대로
    제곱하면 1e12 규모가 되어 정규방정식의 조건수를 망가뜨린다.
    """
    t = np.asarray(times, dtype=float)
    uv = np.asarray(uvs, dtype=float)
    if t.ndim != 1 or uv.shape != (t.size, 2):
        raise ValueError(f"times는 (N,), uvs는 (N,2)여야 한다: {t.shape}, {uv.shape}")
    cm = np.zeros((t.size, 2)) if cams is None else np.asarray(cams, dtype=float)
    if cm.shape != (t.size, 2):
        raise ValueError(f"cams는 (N,2)여야 한다: {cm.shape}")

    t0 = float(t[0])
    tr = t - t0
    n = t.size

    fx, fy = config.CAMERA_FX, config.CAMERA_FY
    g = gravity_cam()
    a = uv[:, 0] - config.CAMERA_CX
    b = uv[:, 1] - config.CAMERA_CY

    # 미지수 순서: [X0, Vx, Y0, Vy, Z0, Vz]
    A = np.zeros((2 * n, 6))
    rhs = np.zeros(2 * n)
    half_t2 = 0.5 * tr * tr

    # a·Z − fx·X_world = −fx·camX(t)
    A[0::2, 0] = -fx
    A[0::2, 1] = -fx * tr
    A[0::2, 4] = a
    A[0::2, 5] = a * tr
    rhs[0::2] = half_t2 * (fx * g[0] - a * g[2]) - fx * cm[:, 0]

    # b·Z − fy·Y_world = −fy·camY(t)
    A[1::2, 2] = -fy
    A[1::2, 3] = -fy * tr
    A[1::2, 4] = b
    A[1::2, 5] = b * tr
    rhs[1::2] = half_t2 * (fy * g[1] - b * g[2]) - fy * cm[:, 1]

    sol, _res, _rank, sv = np.linalg.lstsq(A, rhs, rcond=None)
    condition = float(sv[0] / sv[-1]) if sv.size and sv[-1] > 0 else math.inf

    p0 = np.array([sol[0], sol[2], sol[4]])
    v0 = np.array([sol[1], sol[3], sol[5]])
    resid = reprojection_rms(p0, v0, tr, uv, cm)
    return Fit(p0=p0, v0=v0, t0=t0, residual_px=resid, condition=condition, n=n)


def reprojection_rms(p0, v0, tr, uv, cams=None) -> float:
    """추정 궤적을 다시 화면에 투영해서 실제 관측과 몇 px 어긋나는지.

    카메라가 움직였으면 그 시점의 카메라 위치를 빼고 투영해야 맞다.
    """
    g = gravity_cam()
    tr = np.asarray(tr, dtype=float)
    uv = np.asarray(uv, dtype=float)
    cm = np.zeros((tr.size, 2)) if cams is None else np.asarray(cams, dtype=float)
    pts = p0[None, :] + v0[None, :] * tr[:, None] + 0.5 * g[None, :] * (tr * tr)[:, None]
    z = pts[:, 2]
    if np.any(z <= 1e-9):
        return math.inf
    u = config.CAMERA_FX * (pts[:, 0] - cm[:, 0]) / z + config.CAMERA_CX
    v = config.CAMERA_FY * (pts[:, 1] - cm[:, 1]) / z + config.CAMERA_CY
    d = np.stack([u, v], axis=1) - uv
    return float(np.sqrt(np.mean(np.sum(d * d, axis=1))))


def project(fit: Fit, t: float, cam_xy=(0.0, 0.0)) -> tuple[float, float] | None:
    """fit이 예측하는 시각 t의 **화면 좌표**. 카메라 뒤로 가면 None.

    게이팅이 "예측 위치에 가장 가까운 검출"을 고를 때 쓴다.
    """
    p = fit.position_at(t - fit.t0)
    if p[2] <= 1e-6:
        return None
    return (config.CAMERA_FX * (p[0] - cam_xy[0]) / p[2] + config.CAMERA_CX,
            config.CAMERA_FY * (p[1] - cam_xy[1]) / p[2] + config.CAMERA_CY)


def landing_time(fit: Fit, z_catch: float = None) -> float | None:
    """물체가 z_catch 평면을 **내려오면서** 통과하는 시각(t0 기준 상대초).

    Z(t) = Z0 + Vz·t + ½·g_z·t² = z_catch 를 푼다. 올라갔다 내려오는 궤적은 이
    평면을 두 번 지나므로 **나중 근**(내려올 때)을 쓴다.
    """
    if z_catch is None:
        z_catch = config.CATCH_HEIGHT_M
    gz = gravity_cam()[2]
    a = 0.5 * gz
    b = fit.v0[2]
    c = fit.p0[2] - z_catch

    if abs(a) < 1e-12:                       # 중력이 광축과 수직인 병적 케이스
        return None if abs(b) < 1e-12 else (-c / b if -c / b > 0 else None)

    disc = b * b - 4.0 * a * c
    if disc < 0:
        return None                          # 정점이 캐치 평면보다 낮다 — 도달 못 함
    sqrt_d = math.sqrt(disc)
    roots = sorted(r for r in ((-b - sqrt_d) / (2 * a), (-b + sqrt_d) / (2 * a)) if r > 1e-6)
    return roots[-1] if roots else None


def predict_landing(fit: Fit, z_catch: float = None):
    """(착지점 x, y, t0로부터의 남은시간) 또는 None.

    `cams`를 넣어 푼 fit이면 이 x, y는 **월드 좌표**(트랙 시작 시점의 로봇 중심 기준)다.
    """
    dt = landing_time(fit, z_catch)
    if dt is None:
        return None
    p = fit.position_at(dt)
    return float(p[0]), float(p[1]), float(dt)

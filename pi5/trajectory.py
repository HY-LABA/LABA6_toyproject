"""단안 포물선 궤적 추정 — **물체 크기를 몰라도 3D를 복원한다.**

사진 한 장은 깊이를 담지 못한다. 작은 물체가 가까이 있는 것과 큰 물체가 멀리
있는 것이 완전히 똑같은 픽셀을 만들기 때문이다. 예전 방식(bbox 크기 ÷ 기준
크기)은 이 한계를 "물체의 실제 크기를 미리 안다"로 우회했는데, 물체마다 크기를
등록해야 하고 공중에서 회전하면 보이는 크기가 3배까지 흔들려서 포기했다.

대신 **중력을 자로 쓴다.** 중력은 물체가 뭐든 항상 9.8 m/s²이므로,
"이 궤적이 9.8의 중력으로 만들어지려면 거리가 얼마여야 하는가"를 역으로 풀면
스케일이 확정된다. 미지수 6개(t=0 시점의 위치 3 + 속도 3), 관측 하나당 식 2개
→ 3관측이면 풀리고, 그 이상은 최소제곱으로 노이즈를 눌러준다.

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

좌표계
------
카메라 원점, 광축(= 카메라가 보는 위쪽)이 +Z. X는 좌우, Y는 앞뒤.
카메라가 로봇에 실려 있으므로 **착지점 (X,Y)가 곧 로봇이 이동할 거리**다.
착지 기준면은 `z_catch=0`, 즉 **카메라 모듈이 있는 평면**이다 — 쓰레기통 입구
높이를 따로 알 필요가 없다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import config


@dataclass
class Fit:
    """한 번의 최소제곱 결과. p0/v0는 t0 시점의 카메라 좌표계 값."""

    p0: np.ndarray          # (3,) X0, Y0, Z0
    v0: np.ndarray          # (3,) Vx, Vy, Vz
    t0: float               # 기준 시각(첫 관측의 절대시각)
    residual_px: float      # 재투영 RMS 오차 — 이 값이 곧 신뢰도다
    condition: float        # 행렬 조건수 — 시간 스팬이 짧으면 폭발한다
    n: int                  # 사용한 관측 수

    @property
    def ok(self) -> bool:
        """형태가 말이 되는 해인가. **깊이가 정확한가는 여기서 판정하지 못한다** —
        스케일이 틀린 궤적도 잔차는 작을 수 있기 때문이다(Tracker._depth_converged 참고).
        """
        lo, hi = config.Z_RANGE_M
        return (
            bool(np.all(np.isfinite(self.p0)))
            and bool(np.all(np.isfinite(self.v0)))
            and lo <= self.p0[2] <= hi   # 카메라 뒤나 6m 밖은 해가 발산한 것이다
            and self.condition < config.MAX_CONDITION
            and self.residual_px <= config.MAX_RESIDUAL_PX
        )

    def position_at(self, dt: float) -> np.ndarray:
        """t0로부터 dt초 뒤의 카메라 좌표계 위치."""
        g = _gravity_cam()
        return self.p0 + self.v0 * dt + 0.5 * g * dt * dt

    def __repr__(self) -> str:
        return (f"Fit(z={self.p0[2]:.2f}m v={np.round(self.v0, 2)} "
                f"resid={self.residual_px:.2f}px n={self.n})")


def _gravity_cam() -> np.ndarray:
    """카메라 좌표계에서 본 중력 벡터.

    카메라가 정확히 위를 보면 중력은 광축의 반대방향, 즉 (0,0,−g)다.
    기울여 달았다면 config.GRAVITY_CAM을 회전시킨 값으로 바꿔주면 아래 수식은
    그대로 성립한다 — 중력 방향이 상수로만 들어가기 때문이다.
    """
    return np.asarray(config.GRAVITY_CAM, dtype=float)


def fit_trajectory(times, uvs) -> Fit:
    """타임스탬프 붙은 픽셀 관측들로 p0, v0를 푼다.

    `times`는 초 단위이고 차이만 의미가 있다. 내부에서 첫 관측 기준으로 다시
    맞춘다 — CLOCK_MONOTONIC 원값을 그대로 제곱하면 1e12 규모가 되어 정규방정식의
    조건수를 망가뜨린다.
    """
    t = np.asarray(times, dtype=float)
    uv = np.asarray(uvs, dtype=float)
    if t.ndim != 1 or uv.shape != (t.size, 2):
        raise ValueError(f"times는 (N,), uvs는 (N,2)여야 한다: {t.shape}, {uv.shape}")

    t0 = float(t[0])
    tr = t - t0
    n = t.size

    fx, fy = config.CAMERA_FX, config.CAMERA_FY
    g = _gravity_cam()
    a = uv[:, 0] - config.CAMERA_CX
    b = uv[:, 1] - config.CAMERA_CY

    # 미지수 순서: [X0, Vx, Y0, Vy, Z0, Vz]
    A = np.zeros((2 * n, 6))
    rhs = np.zeros(2 * n)
    half_t2 = 0.5 * tr * tr

    # a·Z − fx·X = 0
    A[0::2, 0] = -fx
    A[0::2, 1] = -fx * tr
    A[0::2, 4] = a
    A[0::2, 5] = a * tr
    rhs[0::2] = half_t2 * (fx * g[0] - a * g[2])

    # b·Z − fy·Y = 0
    A[1::2, 2] = -fy
    A[1::2, 3] = -fy * tr
    A[1::2, 4] = b
    A[1::2, 5] = b * tr
    rhs[1::2] = half_t2 * (fy * g[1] - b * g[2])

    sol, _res, _rank, sv = np.linalg.lstsq(A, rhs, rcond=None)
    condition = float(sv[0] / sv[-1]) if sv.size and sv[-1] > 0 else math.inf

    p0 = np.array([sol[0], sol[2], sol[4]])
    v0 = np.array([sol[1], sol[3], sol[5]])

    # 오차는 해석 가능한 단위로 보고한다 — 재투영 픽셀.
    resid = _reprojection_rms(p0, v0, g, tr, uv)
    return Fit(p0=p0, v0=v0, t0=t0, residual_px=resid, condition=condition, n=n)


def _reprojection_rms(p0, v0, g, tr, uv) -> float:
    """추정 궤적을 다시 화면에 투영해서 실제 관측과 몇 px 어긋나는지."""
    pts = p0[None, :] + v0[None, :] * tr[:, None] + 0.5 * g[None, :] * (tr * tr)[:, None]
    z = pts[:, 2]
    if np.any(z <= 1e-9):
        return math.inf
    u = config.CAMERA_FX * pts[:, 0] / z + config.CAMERA_CX
    v = config.CAMERA_FY * pts[:, 1] / z + config.CAMERA_CY
    d = np.stack([u, v], axis=1) - uv
    return float(np.sqrt(np.mean(np.sum(d * d, axis=1))))


def landing_time(fit: Fit, z_catch: float = None) -> float | None:
    """물체가 z_catch 평면을 **내려오면서** 통과하는 시각(t0 기준 상대초).

    Z(t) = Z0 + Vz·t + ½·g_z·t² = z_catch 를 푼다. 올라갔다 내려오는 궤적은 이
    평면을 두 번 지나므로 **나중 근**(내려올 때)을 쓴다.
    """
    if z_catch is None:
        z_catch = config.CATCH_HEIGHT_M
    gz = _gravity_cam()[2]
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

    반환하는 x, y는 카메라 원점 기준이고 카메라가 로봇에 실려 있으므로
    **그대로 로봇이 이동해야 할 거리**다. 별도 좌표 변환이 필요 없다.
    """
    dt = landing_time(fit, z_catch)
    if dt is None:
        return None
    p = fit.position_at(dt)
    return float(p[0]), float(p[1]), float(dt)


@dataclass
class Tracker:
    """관측을 누적하며 착지 예측을 계속 정밀화한다.

    매 프레임 **처음부터 다시 피팅**한다. 이 규모에선 최소제곱이 마이크로초라
    아낄 게 없고, 배치 피팅은 물리를 매 프레임 정확히 강제하므로 프로세스 노이즈가
    포물선을 서서히 밀어내는 일이 없다.

    프레임을 몇 장 모아야 하는가는 **개수가 아니라 시간 스팬**의 문제다. 궤적의
    처짐량은 ⅛·g·T²이라 T에 제곱으로 붙는 반면, 같은 시간에 프레임만 늘리면
    노이즈 평균 효과로 √N밖에 못 얻는다. 60fps 연속 3장(T=0.05s)이면 2m 거리에서
    처짐이 1.2px — 검출 노이즈에 묻힌다. 그래서 MIN_TIME_SPAN_S로 막는다.
    """

    times: list[float] = field(default_factory=list)
    uvs: list[tuple[float, float]] = field(default_factory=list)
    fit: Fit | None = None
    _first_shown: bool = field(default=False, repr=False)

    def reset(self) -> None:
        self.times.clear()
        self.uvs.clear()
        self.fit = None
        self._first_shown = False

    @property
    def n(self) -> int:
        return len(self.times)

    @property
    def time_span(self) -> float:
        return (self.times[-1] - self.times[0]) if self.times else 0.0

    def add(self, u: float, v: float, t: float) -> Fit | None:
        """관측 하나 추가하고 갱신된 fit을 돌려준다. 아직 못 믿으면 None."""
        self.times.append(float(t))
        self.uvs.append((float(u), float(v)))
        if len(self.times) > config.TRACK_WINDOW:
            del self.times[0]
            del self.uvs[0]

        self.fit = None
        if self.n < config.MIN_OBSERVATIONS:
            return None
        if self.time_span < config.MIN_TIME_SPAN_S:
            # 시간 스팬이 짧으면 곡률을 못 재고 깊이가 발산한다.
            # 자신 있게 틀린 숫자를 주느니 아무것도 안 주는 게 낫다.
            return None

        fit = fit_trajectory(self.times, self.uvs)
        if not fit.ok:
            return None

        # 첫 번째로 fit.ok를 통과한 순간만 깊이 수렴 검사를 건너뛰고 바로 내보낸다.
        # _depth_converged는 구조적으로 n≥6 이상이어야 통과 가능한데(아래 참고),
        # 초반 깊이는 항상 과소추정 방향으로만 편향된다(errors-in-variables) —
        # 즉 로봇이 목표보다 덜 가는 쪽으로만 틀리므로 오버슈트 위험이 없고, 다음
        # 프레임들이 이어서 보정한다. 그래서 "느리지만 확실한 첫 값"보다 "빠르지만
        # 거친 첫 값"이 낫다고 판단. 두 번째 프레임부터는 다시 엄격하게 검사한다.
        if not self._first_shown:
            self._first_shown = True
            self.fit = fit
            return fit

        if not self._depth_converged(fit):
            return None
        self.fit = fit
        return fit

    def _depth_converged(self, fit: Fit) -> bool:
        """앞쪽 관측만으로 다시 풀어서 깊이가 안정됐는지 본다.

        **잔차로는 이 판정을 할 수 없다.** 스케일이 틀린 궤적도 화면에는 잘 맞게
        투영되기 때문이다 — 그게 단안 카메라의 원래 모호성이고, 중력이 그 모호성을
        깨주지만 관측 시간이 짧으면 충분히 못 깬다. 합성 검증에서 스팬 0.5초일 때
        잔차 1.58px(아주 좋음)인데 깊이는 참값의 69%였다.

        관측이 짧을수록 깊이가 **작게** 나오는 계통 편향(errors-in-variables)이
        있으므로, 관측을 덜 쓴 해와 전부 쓴 해가 가까워졌다면 수렴한 것이다.
        노이즈 크기를 미리 몰라도 되는 자기 교정식 판정이다.
        """
        k = int(len(self.times) * config.DEPTH_CHECK_FRACTION)
        if k < config.MIN_OBSERVATIONS:
            return False
        sub = fit_trajectory(self.times[:k], self.uvs[:k])
        z_full, z_sub = fit.p0[2], sub.p0[2]
        if not np.isfinite(z_sub) or z_sub <= 0.0:
            return False
        ratio = z_full / z_sub
        limit = config.DEPTH_STABILITY_RATIO
        return (1.0 / limit) <= ratio <= limit

    def landing(self):
        """(x, y, 착지까지 남은 절대시각까지의 초) 또는 None."""
        if self.fit is None:
            return None
        out = predict_landing(self.fit)
        if out is None:
            return None
        x, y, dt_from_t0 = out
        # 남은 시간은 "마지막 관측 시각" 기준으로 환산해서 준다 —
        # 호출부는 지금이 몇 시인지로 판단해야 하기 때문.
        remaining = self.fit.t0 + dt_from_t0 - self.times[-1]
        return x, y, remaining

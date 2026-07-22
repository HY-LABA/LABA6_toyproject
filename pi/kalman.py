"""3D 등가속(중력) 칼만 필터 — 포물선 운동용.

상태 x = [px, py, pz, vx, vy, vz]  (위치 m, 속도 m/s)
- 운동 모델: 등속 + 중력(g)을 알려진 제어입력으로 z축 속도에 반영
- 측정: 위치 (px, py, pz)

의존성: numpy 뿐 (독립 실행 가능).
"""

from __future__ import annotations

import numpy as np


class ProjectileKalman:
    def __init__(self, gravity: float, process_std: float, measure_std: float) -> None:
        self.g = gravity
        self.q = process_std
        self.r = measure_std
        self.x: np.ndarray | None = None      # 상태 (6,)
        self.P: np.ndarray | None = None      # 공분산 (6,6)
        self._last_t: float | None = None

        self.H = np.zeros((3, 6))             # 측정 행렬 (위치 3개만 관측)
        self.H[:3, :3] = np.eye(3)
        self.R = (self.r ** 2) * np.eye(3)

    @property
    def initialized(self) -> bool:
        return self.x is not None

    def reset(self) -> None:
        self.x = None
        self.P = None
        self._last_t = None

    def _init_state(self, pos: np.ndarray, t: float) -> None:
        self.x = np.zeros(6)
        self.x[:3] = pos               # 위치=관측값, 속도=0 으로 시작
        self.P = np.eye(6)
        self.P[:3, :3] *= self.r ** 2  # 위치 불확실성 = 측정 노이즈
        self.P[3:, 3:] *= 10.0 ** 2    # 속도는 크게 불확실
        self._last_t = t

    def _transition(self, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """상태전이 F, 제어입력 Bu(중력), 프로세스 노이즈 Q."""
        F = np.eye(6)
        F[:3, 3:] = dt * np.eye(3)     # p += v*dt

        a = np.array([0.0, 0.0, -self.g])
        Bu = np.zeros(6)
        Bu[:3] = 0.5 * a * dt ** 2     # p += 0.5*a*dt^2
        Bu[3:] = a * dt                # v += a*dt

        q2 = self.q ** 2               # 백색 가속 노이즈 (축별 2x2 블록)
        blk = q2 * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                             [dt ** 3 / 2, dt ** 2]])
        Q = np.zeros((6, 6))
        for i in range(3):
            Q[i, i] = blk[0, 0]
            Q[i, i + 3] = blk[0, 1]
            Q[i + 3, i] = blk[1, 0]
            Q[i + 3, i + 3] = blk[1, 1]
        return F, Bu, Q

    def predict(self, t: float) -> None:
        if not self.initialized:
            return
        dt = t - self._last_t
        if dt <= 0:
            return
        F, Bu, Q = self._transition(dt)
        self.x = F @ self.x + Bu
        self.P = F @ self.P @ F.T + Q
        self._last_t = t

    def update(self, pos: np.ndarray, t: float) -> np.ndarray:
        """위치 관측 pos(3,)을 시각 t에 반영하고 상태 반환."""
        pos = np.asarray(pos, dtype=float).reshape(3)
        if not self.initialized:
            self._init_state(pos, t)
            return self.x.copy()

        self.predict(t)
        y = pos - self.H @ self.x                     # 잔차
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)      # 칼만 이득
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        return self.x.copy()

    @property
    def position(self) -> np.ndarray:
        assert self.x is not None
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        assert self.x is not None
        return self.x[3:].copy()

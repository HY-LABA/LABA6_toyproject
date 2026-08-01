"""③ 궤적 추정 — 포물선 운동 칼만필터.

상태: [X, Y, Z, VX, VY, VZ]  (world frame, 6차원)
모델: 등가속 (0, 0, -g_eff).  수평 등속 + 수직 등가속 = 포물선 운동.

왜 프레임별 차분이 아니라 필터인가:
  3프레임 차분으로 구한 vz의 오차는 6.5 m/s로 vz 자체보다 크다. 비행 전체를 중력 제약
  하에 피팅하면 0.183 m/s로 35배 좋아진다. (../docs/physics.md 8장)
  최종 착지 예측 오차는 0.5~0.7cm — 통 반경 15cm의 1/25이다.

이 모듈이 시스템에서 유일하게 상태를 누적하는 곳이다.
"""

from __future__ import annotations

import numpy as np

import config
import frames
from datatypes import Detection, Pose

_POS = slice(0, 3)
_VEL = slice(3, 6)


class ProjectileEstimator:
    """포물선 궤적 추정기. 관측을 누적해 [위치, 속도]를 낸다."""

    def __init__(self) -> None:
        self._x: np.ndarray | None = None   # 상태 (6,)
        self._P: np.ndarray | None = None   # 공분산 (6, 6)
        self._t: float | None = None        # 마지막 갱신 시각
        self._class_name: str | None = None

    # ── 조회 ──────────────────────────────────────────────────────────
    @property
    def initialized(self) -> bool:
        return self._x is not None

    @property
    def position(self) -> np.ndarray:
        assert self._x is not None
        return self._x[_POS].copy()

    @property
    def velocity(self) -> np.ndarray:
        assert self._x is not None
        return self._x[_VEL].copy()

    @property
    def z(self) -> float:
        assert self._x is not None
        return float(self._x[2])

    @property
    def class_name(self) -> str:
        assert self._class_name is not None
        return self._class_name

    @property
    def confident(self) -> bool:
        """속도 불확실성이 임계 이하이면 명령을 낼 수 있다. 보통 3프레임이면 통과."""
        if self._P is None:
            return False
        return float(np.trace(self._P[_VEL, _VEL])) <= config.CONFIDENCE_TRACE_MAX

    @property
    def confidence(self) -> float:
        """0~1. 초기 불확실 구간에서 명령 크기를 줄이는 데 쓴다."""
        if self._P is None:
            return 0.0
        trace = float(np.trace(self._P[_VEL, _VEL]))
        return float(np.clip(config.CONFIDENCE_TRACE_MAX / max(trace, 1e-9), 0.0, 1.0))

    def stale(self, now: float) -> bool:
        """마지막 관측 이후 너무 오래 지났는가."""
        if self._t is None:
            return False
        return (now - self._t) > config.LOST_TIMEOUT_S

    def reset(self) -> None:
        self.__init__()

    # ── 예측 ──────────────────────────────────────────────────────────
    def _transition(self, dt: float, g: float, q: float):
        F = np.eye(6)
        F[_POS, _VEL] = dt * np.eye(3)

        a = np.array([0.0, 0.0, -g])
        Bu = np.zeros(6)
        Bu[_POS] = 0.5 * a * dt**2
        Bu[_VEL] = a * dt

        # 백색 가속 노이즈. 축마다 [위치, 속도] 2x2 블록.
        q2 = q**2
        Q = np.zeros((6, 6))
        for i in range(3):
            Q[i, i] = q2 * dt**4 / 4
            Q[i, i + 3] = Q[i + 3, i] = q2 * dt**3 / 2
            Q[i + 3, i + 3] = q2 * dt**2
        return F, Bu, Q

    def predict_state(self, t: float) -> np.ndarray:
        """t 시점의 상태를 외삽한다 (필터를 갱신하지 않는다)."""
        assert self._x is not None and self._t is not None
        dt = t - self._t
        if dt <= 0:
            return self._x.copy()
        F, Bu, _ = self._transition(dt, config.gravity_for(self.class_name), 0.0)
        return F @ self._x + Bu

    def predict_image_position(self, t: float, pose: Pose) -> tuple[float, float]:
        """④ ROI 중심. 다음 프레임에서 물체가 있을 이미지 좌표를 예측한다."""
        p_world = self.predict_state(t)[_POS]
        u, v = frames.world_to_pixel(p_world, pose)
        # 크롭이 프레임을 벗어나지 않도록 안쪽으로 민다
        half = config.ROI_SIZE_PX / 2
        u = float(np.clip(u, half, config.FRAME_WIDTH - half))
        v = float(np.clip(v, half, config.FRAME_HEIGHT - half))
        return u, v

    # ── 갱신 ──────────────────────────────────────────────────────────
    def update(self, det: Detection, pose: Pose, t: float, sigma_bbox_px: float) -> None:
        """관측 하나를 반영한다.

        det   ① 인식 결과 (원본 픽셀 좌표계)
        pose  프레임 시각의 로봇 자세
        t     프레임 캡처 시각 (s)
        """
        p_body = frames.pixel_to_body(det)
        p_world = frames.body_to_world(p_body, pose)
        # 노이즈는 z가 아니라 **직선거리 R** 기준이다 — 어안 렌즈에서 축 밖으로 갈수록 갈린다
        sigma = frames.measurement_sigma(det, frames.estimate_range(det), sigma_bbox_px)
        R = np.diag(sigma**2)

        if not self.initialized:
            self._init_state(p_world, sigma, t, det.class_name)
            return

        # 클래스가 바뀌면 다른 물체다 — 다시 시작한다
        if det.class_name != self._class_name:
            self.reset()
            self._init_state(p_world, sigma, t, det.class_name)
            return

        dt = t - self._t  # type: ignore[operator]
        if dt > 0:
            F, Bu, Q = self._transition(
                dt, config.gravity_for(det.class_name), config.process_noise_for(det.class_name)
            )
            self._x = F @ self._x + Bu
            self._P = F @ self._P @ F.T + Q
            self._t = t

        H = np.zeros((3, 6))
        H[:, _POS] = np.eye(3)

        y = p_world - H @ self._x                       # 잔차
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)            # 칼만 이득
        self._x = self._x + K @ y
        self._P = (np.eye(6) - K @ H) @ self._P

    def _init_state(self, p_world, sigma, t: float, class_name: str) -> None:
        """첫 관측. 속도는 모르므로 0으로 두되 공분산을 크게 잡는다.

        낙하물의 속도는 0이 아니지만, (10 m/s)^2 불확실성을 주면 2~3프레임 안에 수렴한다.
        그때까지는 confident가 False라 명령을 내지 않는다.
        """
        self._x = np.zeros(6)
        self._x[_POS] = p_world
        self._P = np.eye(6)
        self._P[_POS, _POS] = np.diag(sigma**2)
        self._P[_VEL, _VEL] = np.diag([100.0, 100.0, 100.0])
        self._t = t
        self._class_name = class_name

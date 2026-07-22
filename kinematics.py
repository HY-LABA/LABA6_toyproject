"""3륜 옴니(kiwi) 베이스 역기구학 / 순기구학 공식.

바디 속도 [Vx, Vy, ω] <-> 각 바퀴 선속도 [v0, v1, v2] 변환.

바퀴 i (장착각 β_i)의 구동 방향은 접선(β_i + 90°):
    v_i = -sin(β_i)·Vx + cos(β_i)·Vy + L·ω
행렬로  wheel = M · [Vx, Vy, ω],  M 행 = [-sin β_i, cos β_i, L]
순기구학은 M 의 역행렬.

의존성: numpy 뿐 (독립 실행 가능). 입출력 모두 plain ndarray.
"""

from __future__ import annotations

import numpy as np


class OmniKinematics:
    def __init__(self, wheel_angles, base_radius: float) -> None:
        if len(wheel_angles) != 3:
            raise ValueError("3륜 옴니는 바퀴 각도 3개가 필요합니다.")
        self.base_radius = base_radius
        angles = np.asarray(wheel_angles, dtype=float)
        self.M = np.column_stack([          # 바디속도 -> 바퀴속도 (3x3)
            -np.sin(angles),
            np.cos(angles),
            np.full(3, base_radius),
        ])
        self.M_inv = np.linalg.inv(self.M)

    def inverse(self, body_vel) -> np.ndarray:
        """바디 속도 [Vx, Vy, ω] -> 바퀴 선속도 [v0, v1, v2]."""
        body_vel = np.asarray(body_vel, dtype=float).reshape(3)
        return self.M @ body_vel

    def forward(self, wheel_speeds) -> np.ndarray:
        """바퀴 선속도 [v0, v1, v2] -> 바디 속도 [Vx, Vy, ω]."""
        wheel_speeds = np.asarray(wheel_speeds, dtype=float).reshape(3)
        return self.M_inv @ wheel_speeds

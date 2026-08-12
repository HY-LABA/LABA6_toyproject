"""착지 예측 → 피코에 보낼 **목표 속도**.

예전에는 착지점 좌표와 구동시간을 보내고 피코가 `target_x / drive_time_s`로
속도를 계산했는데, 두 가지가 잘못돼 있었다:

  ① 피코가 자기 오도메트리를 빼지 않았다 — 매 사이클 "처음부터의 거리"를 다시
     받아서, 이미 이동한 만큼을 또 가려 했다. 오버슈트가 구조적으로 보장돼 있었다.
  ② drive_time_s가 0.1초로 잘려 있어서 0.5m 목표면 목표속도가 5 m/s가 됐다.
     로봇 최대속도가 1.8 m/s이므로 PID가 영구 포화 → PWM 100% 고정.
     감속 구간이 없어서 "밟으면 끝까지 가는" 동작이 됐다.

지금은 파이5가 계획을 다 세운다. **남은 거리 ÷ 남은 시간**으로 목표속도를 내면
목표에 가까워질수록 목표속도가 저절로 줄어들어, 별도 감속 프로파일 없이 P 제어가
감속기 역할을 한다:

    남은거리 0.50m / 남은시간 0.50s → 1.00 m/s
    남은거리 0.20m / 남은시간 0.25s → 0.80 m/s
    남은거리 0.03m / 남은시간 0.10s → 0.30 m/s

피코는 "이 속도로 이 시간까지 돌려라"만 실행한다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import config


@dataclass
class DriveCommand:
    """피코로 보내는 명령. target_*는 **body frame 속도(m/s)** 다.

    timeout_s는 구동시간이 아니라 **워치독**이다 — 이 시간 안에 새 명령이 안 오면
    피코가 알아서 멈춘다. 파이가 죽었을 때 로봇이 계속 굴러가는 걸 막는 장치이고,
    정상 동작 중에는 매 사이클 새 명령이 오므로 만료되지 않는다.
    """

    target_vx: float
    target_vy: float
    timeout_s: float

    @property
    def speed(self) -> float:
        return math.hypot(self.target_vx, self.target_vy)


STOP = DriveCommand(target_vx=0.0, target_vy=0.0, timeout_s=config.DRIVE_TIMEOUT_S)


def to_drive_command(landing_xy: tuple[float, float], time_remaining: float,
                     odometry_xy: tuple[float, float] = (0.0, 0.0)) -> DriveCommand:
    """착지점 + 남은 시간 + 지금까지 이동한 거리 → 목표 속도.

    `landing_xy`는 **카메라 원점 기준**이다. 카메라가 로봇에 실려 있으므로
    "트랙을 시작한 시점의 로봇 위치"가 원점이고, `odometry_xy`(피코가 회신한
    누적 이동량)를 빼면 남은 거리가 된다.
    """
    remain_x = landing_xy[0] - odometry_xy[0]
    remain_y = landing_xy[1] - odometry_xy[1]
    distance = math.hypot(remain_x, remain_y)

    # 도착했으면 멈춘다. 이 여유가 없으면 남은거리가 0 근처에서 부호가 뒤집히며
    # 앞뒤로 덜컹거린다.
    if distance <= config.POSITION_TOLERANCE_M:
        return STOP

    # 남은 시간이 0이거나 음수면(이미 착지 시각을 지났으면) 시간으로 나눌 수 없다.
    # 이 경우엔 최대속도로 붙는다 — 늦었으니 갈 수 있는 만큼 간다.
    if time_remaining <= 1e-3:
        speed = config.ROBOT_MAX_SPEED_MPS
    else:
        speed = distance / time_remaining

    # 물리적으로 불가능한 목표를 주면 PID가 영구 포화되고, 그 상태에서는 세 바퀴의
    # 속도 비율이 깨져서 **방향까지 틀어진다**. 크기만 깎고 방향은 보존한다.
    if speed > config.ROBOT_MAX_SPEED_MPS:
        speed = config.ROBOT_MAX_SPEED_MPS

    scale = speed / distance
    return DriveCommand(
        target_vx=remain_x * scale,
        target_vy=remain_y * scale,
        timeout_s=config.DRIVE_TIMEOUT_S,
    )

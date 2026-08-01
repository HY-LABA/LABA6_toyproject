"""④ 목표 속도 생성 — 착지점과 현재 자세에서 피코로 보낼 속도를 만든다.

순수 함수. 상태 없음.

왜 이렇게 단순해도 되는가:
  40 Hz로 매 프레임 다시 계산하므로 일종의 비례 유도(proportional guidance)가 된다.
  정교한 가감속 프로파일을 넣어도 다음 프레임에 덮어쓰이므로 이득이 없다.
"""

from __future__ import annotations

import math

import config
import frames
from datatypes import Landing, Pose, VelocityCommand


def reachable_distance(t: float, v_max: float = None, a_max: float = None) -> float:
    """t초 동안 로봇이 갈 수 있는 최대 거리(m). 2구간 모델.

        a·t ≤ v_max :  0.5·a·t²                     (가속만 하다 끝)
        그 외        :  v²/(2a) + v·(t − v/a)        (최고속도 도달 후 등속)

    a_max는 견인 한계 0.5·μ·g다 — 모터가 아니라 바닥 마찰이 병목이다.
    (../docs/physics.md 2·3장)
    """
    v_max = config.V_MAX if v_max is None else v_max
    a_max = config.A_MAX if a_max is None else a_max
    if t <= 0:
        return 0.0
    if a_max * t <= v_max:
        return 0.5 * a_max * t * t
    return v_max * v_max / (2 * a_max) + v_max * (t - v_max / a_max)


def _clamp_norm(x: float, y: float, limit: float) -> tuple[float, float]:
    """방향은 유지하고 크기만 자른다. 축별로 자르면 방향이 틀어진다."""
    norm = math.hypot(x, y)
    if norm <= limit or norm < 1e-12:
        return x, y
    scale = limit / norm
    return x * scale, y * scale


def to_velocity_command(
    landing: Landing, pose: Pose, confidence: float = 1.0
) -> VelocityCommand:
    """착지점(world) -> 목표 속도(body).

    confidence는 칼만필터 수렴도(0~1)다. 초기 불확실 구간에서 명령을 줄여
    엉뚱한 방향으로 튀어나가는 것을 막는다.
    """
    dx = landing.x - pose.x
    dy = landing.y - pose.y

    # 남은 시간. T_MIN 하한이 없으면 착지 직전에 속도가 발산한다.
    t_remaining = max(landing.t_land - config.LATENCY_S, config.T_MIN_S)

    vx_w, vy_w = dx / t_remaining, dy / t_remaining
    vx_w, vy_w = _clamp_norm(vx_w, vy_w, config.V_MAX)
    gain = max(0.0, min(1.0, confidence))
    vx_w, vy_w = vx_w * gain, vy_w * gain

    vx_b, vy_b = frames.rotate2(vx_w, vy_w, -pose.theta)
    return VelocityCommand(vx=vx_b, vy=vy_b, ttl_s=config.COMMAND_TTL_S)


def is_reachable(landing: Landing, pose: Pose) -> bool:
    """물리적으로 도달 가능한가. 불가능해도 명령은 낸다 — 최대한 가까이 가는 게 최선."""
    distance = math.hypot(landing.x - pose.x, landing.y - pose.y)
    return distance <= reachable_distance(landing.t_land)


def stop_command() -> VelocityCommand:
    return VelocityCommand(vx=0.0, vy=0.0, ttl_s=config.COMMAND_TTL_S)

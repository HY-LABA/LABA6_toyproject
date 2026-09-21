# 3륜 옴니 역/순기구학. pico/kinematics.c 의 MicroPython 포팅.
# 공식: v_i = -sin(b_i)*vx + cos(b_i)*vy + L*omega
#
# C 와 다른 점은 딱 하나 — 각도가 상수라 sin/cos 과 순기구학 역행렬을 import
# 시점에 **한 번만** 계산해 두고 루프에서는 곱셈만 한다. 값은 동일하다.

import math

import config

_N = config.NUM_MOTORS
_L = config.WHEEL_MOUNT_RADIUS_M

# 역기구학 행 [-sin(b_i), cos(b_i), L] 을 미리 펼쳐둔다.
_NEG_SIN = tuple(-math.sin(b) for b in config.WHEEL_ANGLES_RAD)
_COS = tuple(math.cos(b) for b in config.WHEEL_ANGLES_RAD)


def _build_forward_matrix():
    """wheel = M * [vx,vy,omega] 의 M 을 여인수/수반행렬로 역산 (C 와 같은 식)."""
    m = [[_NEG_SIN[i], _COS[i], _L] for i in range(3)]

    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    inv = [[0.0, 0.0, 0.0] for _ in range(3)]
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det
    return inv


_FWD = _build_forward_matrix()

# max_body_speed 내부용 임시 버퍼 — 제어 루프에서 매 틱 리스트를 새로 만들지
# 않기 위한 것(GC 지터 억제, README 3장).
_scratch = [0.0, 0.0, 0.0]


def inverse_kinematics(vx, vy, omega, out):
    """바디 속도 -> 바퀴 선속도. 결과를 `out`(길이 3 리스트)에 써넣는다."""
    for i in range(_N):
        out[i] = _NEG_SIN[i] * vx + _COS[i] * vy + _L * omega
    return out


def forward_kinematics(wheel):
    """바퀴 선속도 -> (vx, vy, omega)."""
    w0 = wheel[0]
    w1 = wheel[1]
    w2 = wheel[2]
    vx = _FWD[0][0] * w0 + _FWD[0][1] * w1 + _FWD[0][2] * w2
    vy = _FWD[1][0] * w0 + _FWD[1][1] * w1 + _FWD[1][2] * w2
    omega = _FWD[2][0] * w0 + _FWD[2][1] * w1 + _FWD[2][2] * w2
    return vx, vy, omega


def max_body_speed(vx, vy, omega):
    """**이 방향으로** 갈 때 바퀴가 포화되지 않는 최대 body 속력(m/s).

    (vx, vy)는 크기가 무시되고 방향만 쓰인다. 방향이 유리하면
    WHEEL_MAX_SPEED_MPS 보다 크게 나온다 — 3륜 옴니에서 body 1 m/s 에 필요한
    최대 바퀴 속도가 방향에 따라 0.866~1.000 으로 다르기 때문이다.

    ⚠ pi5 `control.max_body_speed()` 와 **같은 식이어야 한다.** 파이는 이 값으로
      표적 도달 가능성을 판정한다(`tracker._reach()`). 여기가 더 짜게 자르면
      파이는 못 잡을 표적을 쫓고, 더 후하게 자르면 바퀴가 포화돼 방향이 틀어진다.
    """
    n = math.sqrt(vx * vx + vy * vy)
    if n < 1e-9:
        return config.WHEEL_MAX_SPEED_MPS

    # 단위 속도로 갔을 때의 바퀴 속도를 구하고, 그 중 가장 큰 것이 바퀴 한계에
    # 닿는 지점이 곧 이 방향의 body 속력 상한이다.
    w = inverse_kinematics(vx / n, vy / n, (omega / n) if omega != 0.0 else 0.0,
                           _scratch)

    peak = 0.0
    for i in range(3):
        a = w[i] if w[i] >= 0.0 else -w[i]
        if a > peak:
            peak = a
    if peak < 1e-9:
        return config.WHEEL_MAX_SPEED_MPS
    return config.WHEEL_MAX_SPEED_MPS / peak

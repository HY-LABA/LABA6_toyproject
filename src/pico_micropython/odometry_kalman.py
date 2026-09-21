# pico/odometry_kalman.c 의 MicroPython 포팅.
#
# 축(vx,vy,omega)별 독립 1D 칼만필터. 상태=속도 자체(가속도 모델 없음),
# 엔코더 양자화 노이즈를 스무딩하는 용도.
#
# ★★ 이 필터의 출력은 **파이5로 보고하는 속도에만** 쓴다. pose 적분에는 절대
#    쓰지 않는다. 이유:
#
#    이 파라미터 조합에서 사전분산 p 는 정지 상태로 몇 초만 지나도 정상상태로
#    수렴하고, 그때 칼만 이득이 k≈0.014 로 굳는다. 시정수가 τ = dt/K ≈ 1.4초다.
#
#    캐치 기동은 0.5초짜리다. 시정수 1.4초짜리 필터를 통과한 속도를 적분하면
#    pose 가 실제 이동량을 이만큼 과소보고한다 (1.0 m/s 스텝, 정지 2초 후):
#
#        t=0.20s   실제 20cm  ->  pose  1.6cm  ( 8%)
#        t=0.50s   실제 50cm  ->  pose  8.9cm  (18%)   ← 41cm 과소보고
#        t=1.00s   실제 100cm ->  pose 30.6cm  (31%)
#
#    main 의 목표점 경로는 `남은거리 = target − pose` 이므로, 50cm 를 갔는데
#    9cm 갔다고 믿고 계속 최고속도로 달린다 — **구조적으로 보장된 오버슈트**다.
#
#    ⚠ 스무딩을 pose 에 다시 넣고 싶어지면, 먼저 위 표를 이 주기(dt=0.02)와
#      새 파라미터로 다시 계산해서 시정수가 0.1초 이하인지 확인할 것.

import math

PROCESS_NOISE = 0.01  # 정해야함: 실측하며 튜닝 (텔레메트리 스무딩에만 영향)
MEASURE_NOISE = 1.0   # 정해야함: 실측하며 튜닝 (텔레메트리 스무딩에만 영향)

# 칼만 상태: [x, p] * 3축 (vx, vy, omega)
_kf_x = [0.0, 0.0, 0.0]
_kf_p = [1.0, 1.0, 1.0]

# 누적 pose (world frame)
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0

# 보고용 스무딩 속도
smooth_vx = 0.0
smooth_vy = 0.0
smooth_omega = 0.0


def odometry_kalman_init():
    global pose_x, pose_y, pose_theta, smooth_vx, smooth_vy, smooth_omega
    for i in range(3):
        _kf_x[i] = 0.0
        _kf_p[i] = 1.0
    pose_x = 0.0
    pose_y = 0.0
    pose_theta = 0.0
    smooth_vx = 0.0
    smooth_vy = 0.0
    smooth_omega = 0.0


def _kalman1d(i, measurement, dt):
    p = _kf_p[i] + PROCESS_NOISE * dt
    k = p / (p + MEASURE_NOISE)
    x = _kf_x[i] + k * (measurement - _kf_x[i])
    _kf_x[i] = x
    _kf_p[i] = p * (1.0 - k)
    return x


def odometry_kalman_update(vx, vy, omega, dt):
    """raw body 속도를 받아 (스무딩 속도 3개)를 돌려주고 pose 를 적분한다."""
    global pose_x, pose_y, pose_theta, smooth_vx, smooth_vy, smooth_omega

    # 반환값(= 파이5로 보고할 속도)만 스무딩한다.
    smooth_vx = _kalman1d(0, vx, dt)
    smooth_vy = _kalman1d(1, vy, dt)
    smooth_omega = _kalman1d(2, omega, dt)

    # ★ pose 는 **raw 속도**로 적분한다 (위 주석 참고).
    #   body frame 속도를 world frame 으로 회전 변환 후 적분.
    cos_t = math.cos(pose_theta)
    sin_t = math.sin(pose_theta)
    pose_x += (vx * cos_t - vy * sin_t) * dt
    pose_y += (vx * sin_t + vy * cos_t) * dt
    pose_theta += omega * dt

    return smooth_vx, smooth_vy, smooth_omega

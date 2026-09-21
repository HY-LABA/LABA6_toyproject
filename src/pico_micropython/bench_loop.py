# 제어 루프 한 틱이 실제로 몇 마이크로초 걸리는지 재는 스크립트.
#
# "C 랑 파이썬이랑 속도 차이가 많이 나나?" 에 대한 답은 추정이 아니라 이걸로
# 확인하면 된다. 재는 건 **한 틱의 계산 시간**이고, 비교 대상은 제어 주기
# CONTROL_PERIOD_MS(=20ms = 20000us)다. 합계가 20000us 보다 충분히 작으면
# 파이썬이어도 제어 성능은 C 와 같다 (README 3장).
#
# ⚠ **모터 드라이버 전원을 내리거나 바퀴를 띄운 상태에서 돌릴 것.**
#   PWM 출력을 실제로 쓰는 구간이 포함돼 있다 (듀티는 0 근처지만 확실히 할 것).
#
# Thonny 에서 이 파일을 열고 Run. 결과는 Shell 에 표로 찍힌다.

import gc
import math

import utime

import communication
import config
import encoder_pio
import kinematics
import motor_control
import odometry_kalman

N = 500  # 반복 횟수


def _bench(name, fn, results):
    gc.collect()
    t0 = utime.ticks_us()
    for _ in range(N):
        fn()
    t1 = utime.ticks_us()
    us = utime.ticks_diff(t1, t0) / N
    results.append((name, us))
    return us


def main():
    encoder_pio.encoder_init_all()
    motor_control.motor_control_init()
    odometry_kalman.odometry_kalman_init()
    encoder_pio.encoder_sync()

    dt = config.CONTROL_PERIOD_S
    wheel_speed = [0.0, 0.0, 0.0]
    wheel_target = [0.0, 0.0, 0.0]
    results = []

    # --- 1. 엔코더 3개 읽기 (PIO FIFO 드레인 + int32 변환) ---------------
    _bench("encoder_read_deltas", encoder_pio.encoder_read_deltas, results)

    # --- 2. 바퀴 카운트 -> 바퀴 속도 3개 --------------------------------
    deltas = [10, -7, 3]

    def _speeds():
        for i in range(3):
            wheel_speed[i] = ((deltas[i] / config.ENCODER_COUNTS_PER_REV
                               * config.WHEEL_CIRCUMFERENCE_M) / dt
                              * config.MOTOR_SIGN[i])
    _bench("counts->wheel speed", _speeds, results)

    # --- 3. 순기구학 ----------------------------------------------------
    _bench("forward_kinematics", lambda: kinematics.forward_kinematics(wheel_speed),
           results)

    # --- 4. 칼만 3축 + pose 적분 ----------------------------------------
    _bench("odometry+kalman",
           lambda: odometry_kalman.odometry_kalman_update(0.3, 0.2, 0.0, dt), results)

    # --- 5. 목표점 경로의 삼각함수/클램프 --------------------------------
    def _target_math():
        rx = 1.0 - odometry_kalman.pose_x
        ry = 0.5 - odometry_kalman.pose_y
        dist = math.sqrt(rx * rx + ry * ry)
        th = odometry_kalman.pose_theta
        c = math.cos(th)
        s = math.sin(th)
        bx = c * rx + s * ry
        by = -s * rx + c * ry
        limit = kinematics.max_body_speed(bx, by, 0.0)
        v = dist / 0.5
        if v > limit:
            v = limit
        return (bx / dist) * v, (by / dist) * v
    _bench("target math (+max_body_speed)", _target_math, results)

    # --- 6. 안전 클램프 (max_body_speed 한 번 더) ------------------------
    _bench("safety clamp", lambda: kinematics.max_body_speed(0.8, 0.4, 0.0), results)

    # --- 7. 역기구학 ----------------------------------------------------
    _bench("inverse_kinematics",
           lambda: kinematics.inverse_kinematics(0.8, 0.4, 0.0, wheel_target), results)

    # --- 8. PID 3개 + PWM 6개 쓰기 --------------------------------------
    _bench("pid + pwm write",
           lambda: motor_control.motor_control_update(wheel_target, wheel_speed, dt),
           results)

    # --- 9. 수신 파싱 (호스트가 안 보내도 poll 비용은 그대로 든다) --------
    communication.communication_init()
    try:
        _bench("rx poll/parse (idle)", communication.communication_try_receive,
               results)
        _bench("tx odometry frame",
               lambda: communication.communication_send_odometry(
                   0.1, 0.2, 0.0, 0.3, 0.4, 0.0), results)
    finally:
        communication.communication_deinit()

    motor_control.stop_all()

    total = 0.0
    print()
    print("=== 제어 루프 한 틱 구성요소별 소요시간 (n=%d) ===" % N)
    for name, us in results:
        total += us
        print("  %-30s %8.1f us" % (name, us))
    budget = config.CONTROL_PERIOD_MS * 1000.0
    print("  %-30s %8.1f us" % ("합계", total))
    print()
    print("제어 주기 예산: %.0f us (%d Hz)" % (budget, 1000 // config.CONTROL_PERIOD_MS))
    print("사용률: %.1f%%   여유: %.0f us" % (total / budget * 100.0, budget - total))
    print()
    print("※ 수신 파싱은 파이5가 실제로 보내는 동안이 더 비싸다 — 프레임당 27바이트를")
    print("  1바이트씩 읽으므로, 60Hz 송신 기준 틱당 약 32바이트가 추가로 붙는다.")


if __name__ == "__main__":
    main()

# 3륜 옴니휠 로봇 펌웨어 — pico/main.c 의 MicroPython 포팅.
#
# Pico 2 W 의 파일시스템에 이 이름(main.py) 그대로 올려두면 전원이 들어올 때
# 자동 실행된다. 업로드 방법과 C 대비 속도 이야기는 README.md 참고.
#
# ⚠ 시작하면 BOOT_GRACE_S 초 뒤에 Ctrl-C 가 꺼진다 (communication.py 1번 주석).
#   Thonny 로 멈추고 싶으면 **그 몇 초 안에** Ctrl-C 를 눌러야 한다.

import gc
import math

import utime
import micropython

import communication
import config
import encoder_pio
import kinematics
import motor_control
import odometry_kalman

micropython.alloc_emergency_exception_buf(100)

# 구동 모드 — 목표점(TargetCommand)과 속도(teleop/정지) 두 경로 중 가장 최근에
# 받은 쪽이 활성화된다 (docs/design/protocol.md 2장).
DRIVE_NONE = 0
DRIVE_TARGET = 1
DRIVE_VELOCITY = 2

# 이 시간 동안은 Ctrl-C 가 살아 있다. 0 으로 두면 잘못된 main.py 를 올렸을 때
# Thonny 로 멈출 방법이 없어져서 MicroPython 을 다시 구워야 한다.
BOOT_GRACE_S = 3

# 몇 틱마다 GC 를 직접 돌릴지. 안 돌리면 힙이 찰 때 **아무 틱에서나** 자동 GC 가
# 걸려 그 틱만 수 ms 늦는다 — 직접 돌려서 그 비용을 예측 가능한 자리로 옮긴다.
# 모터 출력을 내보낸 **직후**에 돌리므로 엔코더->PWM 지연에는 영향이 없다.
GC_EVERY_N_TICKS = 25  # 50Hz 기준 0.5초마다


def _wheel_speed(delta_counts, dt):
    # ENCODER_COUNTS_PER_REV 는 실측으로 이미 출력축(바퀴축) 1회전 기준 값이라
    # (핸드오프 문서: "거리 = 카운트/1375 × 바퀴둘레", GEAR_RATIO 안 나눔),
    # 여기서 GEAR_RATIO 로 또 나누면 실제 이동거리를 43.8배 적게 계산하게 된다.
    return (delta_counts / config.ENCODER_COUNTS_PER_REV
            * config.WHEEL_CIRCUMFERENCE_M) / dt


def run():
    encoder_pio.encoder_init_all()
    motor_control.motor_control_init()
    odometry_kalman.odometry_kalman_init()
    communication.communication_init()   # ← 여기서 Ctrl-C 가 꺼진다

    n = config.NUM_MOTORS
    sign = config.MOTOR_SIGN
    dt = config.CONTROL_PERIOD_S
    period_ms = config.CONTROL_PERIOD_MS
    tol = config.POSITION_TOLERANCE_M

    mode = DRIVE_NONE
    target_x = 0.0
    target_y = 0.0
    time_remaining_s = 0.0
    vel_vx = 0.0
    vel_vy = 0.0
    cmd_elapsed_s = 0.0
    cmd_timeout_s = 0.0

    # 매 틱 새 리스트를 만들지 않으려고 미리 잡아둔다.
    wheel_speed = [0.0, 0.0, 0.0]
    wheel_target = [0.0, 0.0, 0.0]

    encoder_pio.encoder_sync()
    gc.collect()

    tick = 0
    next_tick = utime.ticks_ms()

    while True:
        next_tick = utime.ticks_add(next_tick, period_ms)

        deltas = encoder_pio.encoder_read_deltas()
        for i in range(n):
            # MOTOR_SIGN: 엔코더가 실제로 측정한 값을 기구학 공식 기준 부호로 맞춘다.
            wheel_speed[i] = _wheel_speed(deltas[i], dt) * sign[i]

        vx, vy, omega = kinematics.forward_kinematics(wheel_speed)
        s_vx, s_vy, s_omega = odometry_kalman.odometry_kalman_update(vx, vy, omega, dt)

        # ★ 모드가 바뀔 때만 PID 상태를 지운다. **매 명령마다** 지우면 안 된다 —
        #   파이는 TargetCommand 를 프레임마다(30~60Hz) 갱신해 보내므로, 명령마다
        #   리셋하면 Ki 가 정지마찰을 이기려고 쌓이는 걸 매번 지워버려 목표 근처에서
        #   영원히 못 간다. 여기서 그에 대응하는 경계는 **모드 전환**이다.
        kind = communication.communication_try_receive()
        if kind == communication.RX_TARGET:
            if mode != DRIVE_TARGET:
                motor_control.motor_control_reset()
            mode = DRIVE_TARGET
            target_x = communication.rx_target_x
            target_y = communication.rx_target_y
            time_remaining_s = communication.rx_time_remaining_s
            cmd_timeout_s = communication.rx_timeout_s
            cmd_elapsed_s = 0.0
        elif kind == communication.RX_VELOCITY:
            if mode != DRIVE_VELOCITY:
                motor_control.motor_control_reset()
            mode = DRIVE_VELOCITY
            vel_vx = communication.rx_target_vx
            vel_vy = communication.rx_target_vy
            cmd_timeout_s = communication.rx_timeout_s
            cmd_elapsed_s = 0.0
        elif mode != DRIVE_NONE:
            # timeout_s 는 구동시간이 아니라 워치독이다. 정상 동작 중에는 파이5가
            # 매 프레임 새 명령을 보내므로 만료되지 않는다. 만료됐다는 건 파이가
            # 죽었거나 링크가 끊겼다는 뜻이므로 세우는 게 맞다.
            cmd_elapsed_s += dt
            if cmd_elapsed_s >= cmd_timeout_s:
                mode = DRIVE_NONE
                motor_control.motor_control_reset()

        target_vx = 0.0
        target_vy = 0.0
        target_omega = 0.0

        if mode == DRIVE_TARGET:
            # docs/design/protocol.md 2장: 파이는 절대좌표만 보내고, "이미 간 만큼"을
            # 빼는 건 피코가 자기 오도메트리로 한다 — 그래야 이중으로 안 빠진다.
            rx = target_x - odometry_kalman.pose_x   # world frame
            ry = target_y - odometry_kalman.pose_y
            dist = math.sqrt(rx * rx + ry * ry)

            if dist > tol:
                # ★ world frame -> body frame 회전 R(-theta).
                #   pose 는 world 축인데 inverse_kinematics 는 **body 축**을 받는다.
                #   theta 는 어디서도 리셋되지 않고 엔코더 노이즈만으로도 부팅
                #   이후 계속 누적된다. 회전은 노름을 보존하므로 dist 는 그대로 쓴다.
                th = odometry_kalman.pose_theta
                c = math.cos(th)
                s = math.sin(th)
                bx = c * rx + s * ry
                by = -s * rx + c * ry

                # 상한은 **이 방향에서** 바퀴가 포화되지 않는 값이다. 방향에 따라
                # 1.22~1.41 m/s 로 다르고, 파이의 tracker._reach() 가 믿는 값도 이것이다.
                # ⚠ 바퀴 포화는 body 방향으로 결정되므로 회전 후 값을 넣어야 한다.
                limit = kinematics.max_body_speed(bx, by, 0.0)
                # 남은거리 ÷ 남은시간 — 목표에 가까워질수록 속도가 저절로 줄어서
                # 별도 감속 프로파일 없이 P 제어가 감속기 역할을 한다.
                # ⚠ 이 분기는 pi5 control.to_drive_command() 의 `speed=math.inf`
                #   분기와 **같은 식이어야 한다.**
                v = (dist / time_remaining_s) if time_remaining_s > 1e-3 else limit
                if v > limit:
                    v = limit
                target_vx = (bx / dist) * v
                target_vy = (by / dist) * v

            # 다음 명령이 오면 덮어쓴다 — 새 명령이 안 오는 동안만 스스로 깎는다.
            time_remaining_s -= dt

        elif mode == DRIVE_VELOCITY:
            target_vx = vel_vx
            target_vy = vel_vy

        # 안전 클램프. 파이5도 클램프하지만 여기서 한 번 더 막는다 — 통신 오류나
        # 파이 쪽 버그로 말도 안 되는 값이 들어오면 PID 가 영구 포화되고, 그
        # 상태에선 세 바퀴의 속도 비율이 깨져서 크기만이 아니라 **진행 방향까지**
        # 틀어진다. 그래서 크기만 깎고 방향은 보존한다. 상한은 방향별이다.
        speed = math.sqrt(target_vx * target_vx + target_vy * target_vy)
        if speed > 1e-9:
            limit = kinematics.max_body_speed(target_vx, target_vy, target_omega)
            if speed > limit:
                scale = limit / speed
                target_vx *= scale
                target_vy *= scale

        kinematics.inverse_kinematics(target_vx, target_vy, target_omega, wheel_target)
        motor_control.motor_control_update(wheel_target, wheel_speed, dt)
        communication.communication_send_odometry(
            odometry_kalman.pose_x, odometry_kalman.pose_y, odometry_kalman.pose_theta,
            s_vx, s_vy, s_omega)

        # --- 주기 맞추기 ------------------------------------------------
        tick += 1
        if tick % GC_EVERY_N_TICKS == 0:
            gc.collect()

        sleep_ms = utime.ticks_diff(next_tick, utime.ticks_ms())
        if sleep_ms > 0:
            utime.sleep_ms(sleep_ms)
        elif sleep_ms < -period_ms:
            # 한 주기 넘게 밀렸다 — 여기서 기준을 다시 잡지 않으면 "밀린 만큼
            # 따라잡으려고 sleep 을 건너뛰는" 상태가 계속된다. C 의 sleep_until 은
            # 이 보호가 없었는데, 인터프리터 쪽이 튈 여지가 더 크니 넣어둔다.
            # (dt 는 상수로 유지된다 — 즉 이때의 속도 계산은 그만큼 부정확해진다.
            #  이게 자주 찍히면 주기를 늘리거나 부하를 줄여야 한다는 신호다.)
            next_tick = utime.ticks_ms()


def main():
    if BOOT_GRACE_S > 0:
        print("pico firmware (MicroPython) — %d초 안에 Ctrl-C 를 누르면 중단."
              % BOOT_GRACE_S)
        utime.sleep(BOOT_GRACE_S)
    try:
        run()
    finally:
        motor_control.stop_all()
        communication.communication_deinit()
        print("정지, 종료")


if __name__ == "__main__":
    main()

# 3륜 옴니휠 로봇 - 좌표 이동 테스트 (MicroPython, Thonny용)
#
# Thonny로 Pico 2 W에 이 파일을 열고 Run 하면 됨. 실행되면 Shell에 좌표를
# 입력하라는 프롬프트가 뜨고, "x y" (미터, 공백구분)를 입력하면 그 지점까지
# 폐루프 제어로 이동한다.
#
# 좌표 컨벤션 (2026-09 확정, 도면으로 실물 대조 완료):
#   x+ = M2 쪽 (우측)      x- = M3 쪽 (좌측)
#   y+ = M1 쪽 (정면)      y- = M1 반대쪽 (후면)
#
# 매번 "현재 위치 = (0,0), 현재 heading = 0"으로 리셋하고 그 기준 상대좌표로
# 이동한다 (이전 명령들 누적 안 됨 — 명령마다 그 자리에서 상대이동). "q"를
# 입력하면 종료.
#
# pico/ 폴더의 C/PIO 펌웨어(main.c 등)와는 별개의 파일. C는 UF2를 매번 다시
# 구워야 해서 느리게 반복 튜닝하기 번거로우므로, 값 튜닝은 이 스크립트로 먼저
# 하고 확정되면 pico/config.h 쪽에 반영한다.
#
# ============================================================================
# 확인 필요 / 정해야함 (아래 값들은 실측·배선 확인 전까지 임시값)
# ============================================================================
# - PIN_* : encoder_test.py 핸드오프 문서(2026-09) 기준 테스트 리그 배선.
#           최종 로봇 조립 배선과 같은지 미확인 — 다르면 여기만 고치면 됨.
# - R_EN/L_EN : 문서상 3.3V 직결(항상 활성) 가정, GPIO 제어 안 함.
#           실제로 GPIO에 연결돼 있다면 motor_init()에 enable pin 추가 필요.
# - WHEEL_ANGLES_RAD / WHEEL_MOUNT_RADIUS_M : 이론값(120도 간격, 0.15m).
#           조립 후 실측 필요.
# - PID_KP/KI/KD : placeholder. 실제 튜닝 전까지는 Kp만으로 대략 동작 확인용.
# - 모터 회전방향과 엔코더 부호가 실제로 일치하는지 첫 실행 시 저속으로 확인할
#   것. 안 맞는 모터가 있으면 해당 enc_a/enc_b를 서로 바꾸거나 PID 부호 반전.

import sys
import math
import utime
import micropython
from machine import Pin, PWM

micropython.alloc_emergency_exception_buf(100)

NUM_MOTORS = 3

# --- 핀 배치 (encoder_test.py 핸드오프 문서 5장 기준) ---------------------
PIN_RPWM = (0, 2, 4)
PIN_LPWM = (1, 3, 5)
PIN_ENC_A = (7, 10, 12)
PIN_ENC_B = (8, 11, 13)

# --- 물리 파라미터 (pico/config.h와 동일한 값 유지) ------------------------
WHEEL_DIAMETER_M = 0.100
WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * math.pi

# 바퀴 장착각: 90도 = 정면(앞), M2=330도(우측 뒤), M3=210도(좌측 뒤) — 하늘에서
# 본 실제 물리 배치 그대로 (2026-09 확인: M1 기준 정면, M3가 왼쪽, M2가
# 오른쪽). vx 부호를 억지로 뒤집는 트릭은 제거 — 위치를 실제 그대로 넣었는데도
# 축이 안 맞으면, 그건 각도 문제가 아니라 특정 모터 하나의 배선(PWM 또는
# 엔코더)이 나머지와 다른 방향 규칙으로 연결된 것 — 아래 MOTOR_SIGN으로 그
# 모터만 개별 보정한다.
# M2/M3가 정확히 120도씩인지는 아직 각도기 실측 전이라 이론값.
WHEEL_ANGLES_RAD = (math.pi * 0.5, math.pi * 11.0 / 6.0, math.pi * 7.0 / 6.0)
WHEEL_MOUNT_RADIUS_M = 0.15

# 모터별 부호 보정. set_motor로 나가는 출력과 엔코더로 들어오는 실제 속도값
# 양쪽에 다 곱해서 명령-측정 부호를 같이 맞춘다 (그래야 PID가 자기 자신과는
# 안정적으로 맞물려 돌아가면서, 실제 이동 방향만 뒤집힌 채로 조용히 수렴하는
# 상황을 바로잡을 수 있다).
# 실기 테스트(2026-09) 결과: x, y 둘 다 커맨드와 반대 방향으로 움직임
# ("-0.5 0" -> 오른쪽[+x 쪽]으로, "0 -0.5" -> 정면[+y 쪽]으로) — 모터 3개
# 전부가 공식이 가정한 방향과 정반대로 배선된 것으로 판단, 전부 -1로 보정.
MOTOR_SIGN = (-1, -1, -1)

# 1체배(A채널 상승엣지만) 디코딩 기준, 출력축(바퀴축) 1회전 기준.
# 원래 핸드오프 문서의 실측값 1375는 2체배(A 양쪽 엣지) 기준인데, 그대로
# 썼더니 목표 1m에 실제로는 1.8m를 가버리는 문제가 생겼다 — MicroPython
# 인터럽트가 최고속도 근처(초당 5700회 이상)에서 엣지를 놓쳐 카운트가
# 실제보다 적게 잡히고, 그래서 이동거리를 과소평가한 것으로 추정된다
# (2026-09, 실기 테스트). 엣지를 상승엣지만으로 줄여 인터럽트 부하를
# 절반(초당 ~2900회)으로 낮추고, CPR도 절반(1375/2 = 687.5)으로 맞춘다.
# pico/config.h의 687.5(PIO 1체배)와 같은 값 — 값 하나로 통일됨.
# 그래도 오차가 남으면 인터럽트가 여전히 못 따라가는 것이니 MAX_BODY_SPEED_MPS를
# 낮춰서 최고 펄스레이트 자체를 줄이는 것도 고려할 것.
ENCODER_COUNTS_PER_REV = 687.5

MAX_BODY_SPEED_MPS = 1.22  # 이론값, 실측 전까지 안전 클램프용

CONTROL_PERIOD_MS = 20  # 50Hz. MicroPython 인터프리터 오버헤드 감안한 값
CONTROL_PERIOD_S = CONTROL_PERIOD_MS / 1000.0

PWM_FREQ_HZ = 10000  # BTS7960 상한 25kHz, 저속 선형성 위해 낮춤

# 속도오차(m/s) -> PWM duty(-100~100) 변환 게인. 정해야함: 실측 튜닝.
PID_KP = (60.0, 60.0, 60.0)
PID_KI = (40.0, 40.0, 40.0)  # 목표 근처 저속 구간에서 정지마찰 못 이기고 멈추는 문제 보완
PID_KD = (0.0, 0.0, 0.0)

# 목표 속도가 0이 아닌데 계산된 duty가 이보다 작으면 이 값까지 끌어올린다.
# FIT0186 기동전압(~8% duty)은 무부하 기준이라, 로봇 무게를 실은 정지마찰은
# 이보다 크다 — 목표 근처에서 duty가 작아지다 못해 바퀴가 아예 안 굴러
# 영원히 도착 못 하는 문제(타임아웃)를 막기 위한 최소 구동력 바닥.
# 정해야함: 실측 튜닝 (너무 크면 목표 근처에서 오버슈트/떨림 생김).
MIN_DRIVE_DUTY_PCT = 15.0

# 목표지점 접근 시 속도 프로파일 (P 제어 + 상한)
POS_KP = 1.5          # 거리(m) -> 목표 body speed(m/s) 게인
ARRIVE_TOLERANCE_M = 0.02
DRIVE_TIMEOUT_S = 15.0

# ============================================================================
# 엔코더 (인터럽트 기반, 2체배 XOR 방향판별 — handoff 문서 4장 검증된 로직)
# ============================================================================

_encoder_count = [0, 0, 0]
_pin_a = []
_pin_b = []


def _make_handler(index, a, b):
    def handler(_pin):
        if a.value() != b.value():
            _encoder_count[index] += 1
        else:
            _encoder_count[index] -= 1
    return handler


def encoder_init_all():
    for i in range(NUM_MOTORS):
        a = Pin(PIN_ENC_A[i], Pin.IN, Pin.PULL_UP)
        b = Pin(PIN_ENC_B[i], Pin.IN, Pin.PULL_UP)
        _pin_a.append(a)
        _pin_b.append(b)
        # 상승엣지만 카운트(1체배) — 양쪽 엣지(2체배)는 고속에서 MicroPython
        # 인터럽트가 못 따라가 카운트를 놓쳤다(위 ENCODER_COUNTS_PER_REV 주석 참고).
        a.irq(trigger=Pin.IRQ_RISING, handler=_make_handler(i, a, b))


def encoder_get_counts():
    return list(_encoder_count)


# ============================================================================
# 모터 드라이버 (BTS7960, RPWM/LPWM)
# ============================================================================

_pwm_rpwm = []
_pwm_lpwm = []


def motor_init_all():
    for i in range(NUM_MOTORS):
        r = PWM(Pin(PIN_RPWM[i]))
        l = PWM(Pin(PIN_LPWM[i]))
        r.freq(PWM_FREQ_HZ)
        l.freq(PWM_FREQ_HZ)
        r.duty_u16(0)
        l.duty_u16(0)
        _pwm_rpwm.append(r)
        _pwm_lpwm.append(l)


def set_motor(index, speed_pct):
    speed_pct = max(-100.0, min(100.0, speed_pct))
    duty = int(abs(speed_pct) / 100.0 * 65535)
    if speed_pct >= 0:
        _pwm_lpwm[index].duty_u16(0)
        _pwm_rpwm[index].duty_u16(duty)
    else:
        _pwm_rpwm[index].duty_u16(0)
        _pwm_lpwm[index].duty_u16(duty)


def stop_all_motors():
    for i in range(NUM_MOTORS):
        _pwm_rpwm[i].duty_u16(0)
        _pwm_lpwm[i].duty_u16(0)


# ============================================================================
# 기구학 (pico/kinematics.c와 동일한 공식)
# ============================================================================

def inverse_kinematics(vx, vy, omega):
    out = [0.0, 0.0, 0.0]
    for i in range(NUM_MOTORS):
        b = WHEEL_ANGLES_RAD[i]
        out[i] = -math.sin(b) * vx + math.cos(b) * vy + WHEEL_MOUNT_RADIUS_M * omega
    return out


# forward_kinematics용 역행렬은 각도가 고정 상수이므로 시작할 때 한 번만 계산
def _build_forward_matrix():
    m = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(NUM_MOTORS):
        b = WHEEL_ANGLES_RAD[i]
        m[i][0] = -math.sin(b)
        m[i][1] = math.cos(b)
        m[i][2] = WHEEL_MOUNT_RADIUS_M

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


_FWD_INV = _build_forward_matrix()


def forward_kinematics(wheel_speeds):
    vx = _FWD_INV[0][0] * wheel_speeds[0] + _FWD_INV[0][1] * wheel_speeds[1] + _FWD_INV[0][2] * wheel_speeds[2]
    vy = _FWD_INV[1][0] * wheel_speeds[0] + _FWD_INV[1][1] * wheel_speeds[1] + _FWD_INV[1][2] * wheel_speeds[2]
    omega = _FWD_INV[2][0] * wheel_speeds[0] + _FWD_INV[2][1] * wheel_speeds[1] + _FWD_INV[2][2] * wheel_speeds[2]
    return vx, vy, omega


# ============================================================================
# PID (바퀴별 속도 제어)
# ============================================================================

_pid_integral = [0.0, 0.0, 0.0]
_pid_prev_error = [0.0, 0.0, 0.0]


def pid_reset():
    for i in range(NUM_MOTORS):
        _pid_integral[i] = 0.0
        _pid_prev_error[i] = 0.0


def pid_update(wheel_target, wheel_actual, dt):
    output = [0.0, 0.0, 0.0]
    for i in range(NUM_MOTORS):
        error = wheel_target[i] - wheel_actual[i]
        _pid_integral[i] += error * dt
        # 안티와인드업: 적분항 하나만으로 duty 100%를 넘길 수 없게 클램프.
        # 없으면 오래 멈춰있다 풀릴 때 적분이 쌓일 대로 쌓여서 확 튀어나간다.
        if PID_KI[i] != 0.0:
            integral_limit = 100.0 / PID_KI[i]
            if _pid_integral[i] > integral_limit:
                _pid_integral[i] = integral_limit
            elif _pid_integral[i] < -integral_limit:
                _pid_integral[i] = -integral_limit
        derivative = (error - _pid_prev_error[i]) / dt
        _pid_prev_error[i] = error
        out = (PID_KP[i] * error
               + PID_KI[i] * _pid_integral[i]
               + PID_KD[i] * derivative)

        # 데드밴드 보상: 가야 할 방향(목표 속도)이 있는데 duty가 너무 작아서
        # 못 굴러가는 상태를 막는다. 정지마찰 못 이기고 목표 근처에서
        # 영원히 멈춰버리는 문제(타임아웃)의 원인이었다.
        if abs(wheel_target[i]) > 1e-4 and abs(out) < MIN_DRIVE_DUTY_PCT:
            out = MIN_DRIVE_DUTY_PCT if out >= 0.0 else -MIN_DRIVE_DUTY_PCT

        output[i] = out
    return output


# ============================================================================
# 오도메트리 (단순 오일러 적분, C쪽 칼만필터 없이 우선 확인용)
# ============================================================================

pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0


def odometry_update(vx_body, vy_body, omega, dt):
    global pose_x, pose_y, pose_theta
    vx_world = vx_body * math.cos(pose_theta) - vy_body * math.sin(pose_theta)
    vy_world = vx_body * math.sin(pose_theta) + vy_body * math.cos(pose_theta)
    pose_x += vx_world * dt
    pose_y += vy_world * dt
    pose_theta += omega * dt


def pose_reset():
    # 명령마다 "지금 여기, 지금 이 방향"을 원점으로 다시 잡는다 — 명령이
    # 누적되지 않고 그때그때 그 자리에서 상대이동하게 하기 위함.
    global pose_x, pose_y, pose_theta
    pose_x = 0.0
    pose_y = 0.0
    pose_theta = 0.0


# ============================================================================
# 제어 루프 한 스텝: 인코더 읽기 -> 오도메트리 갱신 -> 반환(실제 body 속도)
# ============================================================================

_prev_counts = [0, 0, 0]


def control_step(target_vx, target_vy, target_omega, dt):
    counts = encoder_get_counts()
    wheel_actual = [0.0, 0.0, 0.0]
    for i in range(NUM_MOTORS):
        delta = counts[i] - _prev_counts[i]
        _prev_counts[i] = counts[i]
        wheel_revs = delta / ENCODER_COUNTS_PER_REV
        # MOTOR_SIGN으로 엔코더 읽은 값을 기구학 공식 기준 부호로 맞춘다
        # (하드웨어 배선이 반대인 모터가 있으면 여기서 보정).
        wheel_actual[i] = (wheel_revs * WHEEL_CIRCUMFERENCE_M) / dt * MOTOR_SIGN[i]

    vx, vy, omega = forward_kinematics(wheel_actual)
    odometry_update(vx, vy, omega, dt)

    wheel_target = inverse_kinematics(target_vx, target_vy, target_omega)
    output = pid_update(wheel_target, wheel_actual, dt)
    for i in range(NUM_MOTORS):
        # MOTOR_SIGN으로 기구학 공식 기준 출력을 다시 하드웨어 부호로 되돌려서 내보낸다.
        set_motor(i, output[i] * MOTOR_SIGN[i])

    return vx, vy, omega


# ============================================================================
# 좌표 이동 테스트
# ============================================================================

def drive_to(target_x, target_y):
    pose_reset()
    pid_reset()
    for i in range(NUM_MOTORS):
        _prev_counts[i] = encoder_get_counts()[i]

    start = utime.ticks_ms()
    next_tick = start

    while True:
        elapsed_s = utime.ticks_diff(utime.ticks_ms(), start) / 1000.0
        if elapsed_s > DRIVE_TIMEOUT_S:
            print("타임아웃 (%.2fs) — 목표 도달 못 함, 정지" % elapsed_s)
            break

        dx = target_x - pose_x
        dy = target_y - pose_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < ARRIVE_TOLERANCE_M:
            print("도착: x=%.3f y=%.3f (오차 %.3fm, %.2fs)" % (pose_x, pose_y, dist, elapsed_s))
            break

        # world frame 방향벡터를 body frame으로 회전 (world -> body: R(-theta))
        c = math.cos(pose_theta)
        s = math.sin(pose_theta)
        vx_body_dir = c * dx + s * dy
        vy_body_dir = -s * dx + c * dy

        speed = min(MAX_BODY_SPEED_MPS, POS_KP * dist)
        target_vx = (vx_body_dir / dist) * speed
        target_vy = (vy_body_dir / dist) * speed

        control_step(target_vx, target_vy, 0.0, CONTROL_PERIOD_S)

        next_tick = utime.ticks_add(next_tick, CONTROL_PERIOD_MS)
        sleep_ms = utime.ticks_diff(next_tick, utime.ticks_ms())
        if sleep_ms > 0:
            utime.sleep_ms(sleep_ms)

    stop_all_motors()


def main():
    encoder_init_all()
    motor_init_all()

    print("=== 옴니휠 좌표 이동 테스트 ===")
    print("형식: x y  (미터, x=우측+/좌측-, y=전진+/후진-)")
    print("매 명령마다 현재 위치·방향을 원점으로 리셋 후 그 기준 상대이동")
    print("종료: q")

    try:
        while True:
            try:
                line = input("target x y> ").strip()
            except EOFError:
                break

            if line in ("q", "quit", "exit"):
                break
            if not line:
                continue

            parts = line.split()
            if len(parts) != 2:
                print("형식 오류. 예: 0.5 0.3")
                continue

            try:
                tx = float(parts[0])
                ty = float(parts[1])
            except ValueError:
                print("숫자로 입력하세요. 예: 0.5 0.3")
                continue

            drive_to(tx, ty)
    finally:
        stop_all_motors()
        print("정지, 종료")


if __name__ == "__main__":
    main()

# pico/config.h 의 MicroPython 포팅. **값은 한 글자도 바꾸지 않았다.**
#
# ⚠⚠ pi5/control.py 의 `verify_wheel_config()` 는 이 파일이 아니라
#    **pico/config.h 를 직접 읽어** 대조한다. 그러니 값을 고칠 때는
#    두 파일을 같이 고쳐야 한다. (C 펌웨어를 버리기로 확정하면 그때
#    control.py 의 경로를 이 파일로 바꾸면 된다 — 지금은 C 쪽이 정본.)

import math

NUM_MOTORS = 3

# 실측 확정값 (2026-09, MicroPython 벤치 테스트 리그 배선 = 최종 로봇 배선 확인됨).
# R_EN/L_EN은 GPIO가 아니라 3.3V에 직결(항상 활성)돼 있어서 en 핀 자체가 없다.
PIN_RPWM = (0, 2, 4)
PIN_LPWM = (1, 3, 5)
PIN_ENC_A = (7, 10, 12)
PIN_ENC_B = (8, 11, 13)

# 파이5 통신: USB CDC (하드웨어 UART 핀 아님)

WHEEL_DIAMETER_M = 0.100
WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * math.pi
GEAR_RATIO = 43.8        # FIT0186 (계산에는 안 쓴다 — 아래 CPR 주석 참고)
MOTOR_RATED_RPM = 251.0  # FIT0186 무부하 @12V

# 바퀴 **하나**가 낼 수 있는 최대 림 속도(m/s).
#
# ★ body 속도 상한이 아니라 **바퀴 속도 상한**이다. 모터가 제한하는 건 바퀴
#   속도다. 3륜 옴니는 진행 방향에 따라 body 1 m/s 를 내는 데 필요한 최대 바퀴
#   속도가 0.866~1.000 으로 다르다. body 속력을 상수 하나로 자르면 유리한
#   방향에서 13.4% 를 버리고, 그러면 파이의 `tracker._reach()` 가 그만큼 낙관해서
#   **못 잡을 표적을 쫓는다.** 그래서 클램프는 방향별로 계산한다
#   (`kinematics.max_body_speed()`, pi5 `control.max_body_speed()` 와 같은 식).
WHEEL_MAX_SPEED_MPS = 1.8

# 바퀴 장착각(rad). **pi5/config.py 의 WHEEL_ANGLES_RAD 와 반드시 같아야 한다.**
#
# body frame: M1 방향이 +Y(전방), 그 오른쪽 직각이 +X.
# 오른손 좌표계라 omega 는 반시계가 +.
#
#                    +Y (전방)
#                       M1  90°
#                        |
#          210° M3 ------+------ M2 330°   --> +X (우측)
#
# b_i 는 중심에서 바퀴를 본 위치각이고, 옴니 바퀴가 구르는 방향은 그 접선(b_i+90°)
# 이다. 그래서 kinematics.py 의 식이 w_i = -sin(b_i)*vx + cos(b_i)*vy + L*omega 다.
WHEEL_ANGLES_RAD = (
    math.pi * 0.5,        #  90° — M1 (전방)
    math.pi * 11.0 / 6.0, # 330° — M2 (우측 뒤)
    math.pi * 7.0 / 6.0,  # 210° — M3 (좌측 뒤)
)
WHEEL_MOUNT_RADIUS_M = 0.15  # 중심-바퀴 거리(m) — 정해야함: 실측

# 모터별 부호 보정. 엔코더로 읽은 속도와 PWM 출력 **양쪽에** 곱해서 명령-측정
# 부호를 맞춘다. 양쪽에 곱하므로 뒤집어도 PID 는 자기 자신과 그대로 맞물려 돌고
# 오도메트리 보고값도 안 바뀐다 — 바뀌는 건 **로봇이 실제로 가는 방향뿐**이다.
# 그래서 판정은 반드시 눈으로 본 방향으로 해야 한다.
# (C 펌웨어 실기 기준 +1. 옛 MicroPython 벤치는 -1 이었는데 배선이 달랐다.)
MOTOR_SIGN = (1, 1, 1)

# 실측 확정값. 데이터시트 표기(16 CPR 모터축)는 부정확한 것으로 확인됨 —
# 2체배(A 양쪽 엣지) 기준 실측 1375, **출력축(바퀴축) 1회전 기준**(감속기 이후).
# 이 포팅의 PIO(encoder_pio.py)는 C 와 똑같이 1체배(A 상승엣지만)라 카운트가
# 정확히 절반이므로 1375 / 2 = 687.5 를 쓴다.
# ⚠ 이미 바퀴축 기준이므로 GEAR_RATIO 로 다시 나누면 안 된다.
ENCODER_COUNTS_PER_REV = 687.5

# MicroPython 벤치 테스트가 50Hz(20ms)에서 튜닝됐다 — PID의 Ki/Kd는 dt에
# 비례해서 누적/변화하므로 같은 게인을 다른 주기에 쓰면 거동이 달라진다.
# MicroPython 인터프리터 오버헤드를 감안해도 20ms 는 여유가 크다(README 참고).
CONTROL_PERIOD_MS = 20
CONTROL_PERIOD_S = CONTROL_PERIOD_MS / 1000.0

# ⚠⚠ **단위가 옛 벤치(goto_xy_test.py)와 다르다 — 벤치값을 100으로 나눈 값이다.**
#   벤치의 PID 출력은 `set_motor(speed_pct)` 가 받는 duty 퍼센트(-100~+100)였고
#   게인이 {60, 40} 이었다. 여기 `set_pwm()` 은 C 와 같은 **duty 비율(0~1)** 을
#   받으므로 게인도 1/100 이어야 한다. 60 을 그대로 쓰면 속도 오차 0.017 m/s 만
#   넘어도 duty 가 포화돼 비례 제어가 아니라 **사실상 온/오프(뱅뱅) 제어**가 된다.
PID_KP = (0.6, 0.6, 0.6)
PID_KI = (0.4, 0.4, 0.4)  # 목표 근처 저속에서 정지마찰 못 이기고 멈추는 문제 보완
PID_KD = (0.0, 0.0, 0.0)

# PWM 반송 주파수. BTS7960 상한은 25 kHz 이고, 저속 구간 선형성을 위해 10 kHz.
# (C 는 슬라이스 분주를 손으로 역산했지만 MicroPython `PWM.freq()` 가 알아서
#  wrap/분주를 잡아준다 — 보드/클럭이 바뀌어도 주파수는 유지된다.)
PWM_FREQ_HZ = 10000

# 목표 속도가 0이 아닌데 계산된 duty가 이보다 작으면 이 값까지 끌어올린다
# (0~1 스케일). 정지마찰 못 이기고 목표 근처에서 바퀴가 아예 안 굴러 영원히
# 멈추는 문제를 막는 데드밴드 보상.
MIN_DRIVE_DUTY = 0.15

# 목표점(TargetCommand) 도착 판정 — 남은 거리가 이보다 작으면 정지한다
# (채터링 방지). pi5/config.py의 POSITION_TOLERANCE_M과 같은 값이어야 한다.
POSITION_TOLERANCE_M = 0.02

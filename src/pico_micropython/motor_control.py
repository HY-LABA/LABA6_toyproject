# 바퀴별 속도 PID + BTS7960(RPWM/LPWM) 출력.
# pico/motor_control.c 의 MicroPython 포팅 — 식과 상수가 모두 동일하다.
#
# duty 단위는 C 와 같은 **0~1 비율**이다 (옛 벤치의 -100~+100 퍼센트가 아니다).
# config.py 의 PID 게인 주석 참고.

from machine import Pin, PWM

import config

_N = config.NUM_MOTORS

_rpwm = []
_lpwm = []

_integral = [0.0, 0.0, 0.0]
_prev_error = [0.0, 0.0, 0.0]

# 안티와인드업 한계를 미리 계산 (적분항 하나만으로 duty 1.0 을 넘길 수 없게).
_integral_limit = tuple((1.0 / k) if k != 0.0 else 0.0 for k in config.PID_KI)


def motor_control_reset():
    """PID 적분·미분 상태를 지운다. **구동 모드가 바뀔 때마다 불러야 한다.**

    모드가 바뀌면 이전 모드에서 쌓인 적분항은 의미가 없다. 남겨두면
    ki=0.4, 한계 1/ki=2.5 -> 적분항만으로 duty 0.4*2.5 = 1.0(100%),
    즉 **정지해 있던 로봇이 다음 명령 첫 틱에 전력으로 튀어나갈 수 있다.**
    """
    for i in range(_N):
        _integral[i] = 0.0
        _prev_error[i] = 0.0


def motor_control_init():
    # R_EN/L_EN 은 3.3V 직결이라 GPIO 로 켤 필요가 없다 (config.py 참고).
    motor_control_reset()
    for i in range(_N):
        r = PWM(Pin(config.PIN_RPWM[i]))
        l = PWM(Pin(config.PIN_LPWM[i]))
        # 한 모터의 rpwm/lpwm 은 같은 PWM 슬라이스라 같은 주파수를 두 번 쓰는
        # 꼴인데, 같은 값이라 무해하다 (C 의 setup_pwm_pin 과 같은 상황).
        r.freq(config.PWM_FREQ_HZ)
        l.freq(config.PWM_FREQ_HZ)
        r.duty_u16(0)
        l.duty_u16(0)
        _rpwm.append(r)
        _lpwm.append(l)


def _set_pwm(pwm, duty_0_to_1):
    if duty_0_to_1 <= 0.0:
        pwm.duty_u16(0)
    elif duty_0_to_1 >= 1.0:
        pwm.duty_u16(65535)
    else:
        pwm.duty_u16(int(duty_0_to_1 * 65535))


def stop_all():
    # motor_control_init() 전에 불릴 수 있다 (main.py 의 finally) — 그래서 길이로 돈다.
    for i in range(len(_rpwm)):
        _rpwm[i].duty_u16(0)
        _lpwm[i].duty_u16(0)


def motor_control_update(wheel_target, wheel_actual, dt):
    for i in range(_N):
        error = wheel_target[i] - wheel_actual[i]

        integral = _integral[i] + error * dt
        # 안티와인드업. 없으면 오래 멈춰있다 풀릴 때 적분이 쌓일 대로 쌓여서
        # 확 튀어나간다.
        lim = _integral_limit[i]
        if lim != 0.0:
            if integral > lim:
                integral = lim
            elif integral < -lim:
                integral = -lim
        _integral[i] = integral

        derivative = (error - _prev_error[i]) / dt
        _prev_error[i] = error

        output = (config.PID_KP[i] * error
                  + config.PID_KI[i] * integral
                  + config.PID_KD[i] * derivative)

        # 데드밴드 보상: 목표 속도가 0 이 아닌데 duty 가 정지마찰을 못 이길 만큼
        # 작아서 바퀴가 아예 안 구르는 상태(목표 근처에서 영원히 도착 못 함)를 막는다.
        t = wheel_target[i]
        if (t if t >= 0.0 else -t) > 1e-4:
            a = output if output >= 0.0 else -output
            if a < config.MIN_DRIVE_DUTY:
                output = config.MIN_DRIVE_DUTY if output >= 0.0 else -config.MIN_DRIVE_DUTY

        # MOTOR_SIGN: 기구학 공식 기준 출력을 하드웨어가 요구하는 부호로 되돌린다.
        output *= config.MOTOR_SIGN[i]

        # ★ 반대쪽을 **먼저** 0 으로 내린다. C 는 순서가 반대였는데(구동측 먼저),
        #   방향이 바뀌는 틱에서 아주 잠깐 양쪽이 동시에 high 가 될 수 있다.
        #   BTS7960 에서 그건 하프브리지 관통전류(브레이크)다. 같은 제어에
        #   순서만 안전하게 바꾼 것이라 거동은 동일하다.
        if output >= 0.0:
            _set_pwm(_lpwm[i], 0.0)
            _set_pwm(_rpwm[i], output)
        else:
            _set_pwm(_rpwm[i], 0.0)
            _set_pwm(_lpwm[i], -output)

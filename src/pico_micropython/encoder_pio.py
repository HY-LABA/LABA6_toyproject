# 엔코더 — PIO 1체배 쿼드러처 디코딩.
# pico/encoder/encoder.pio + encoder_pio.c 를 MicroPython `rp2.asm_pio` 로 옮긴 것.
# 명령어는 1:1 대응이고 동작이 완전히 같다.
#
# ★ 왜 인터럽트(`goto_xy_test.py` 방식)가 아니라 PIO 인가
#   옛 벤치는 `Pin.irq()` 로 엣지를 세다가 최고속 근처(초당 수천 엣지)에서
#   인터럽트가 못 따라가 카운트를 놓쳤다(목표 1m 에 실제 1.8m 를 가버림).
#   PIO 는 CPU 와 독립된 스테이트머신이라 파이썬 인터프리터 속도와 **무관하게**
#   sysclk 속도로 엣지를 센다 — 즉 엔코더 정확도에서는 C 와 파이썬 차이가 없다.
#   (README 3장의 "속도 차이" 논의에서 이게 핵심이다.)

import rp2
from machine import Pin

import config


@rp2.asm_pio()
def _quadrature_1x():
    # in 핀 = A상(상대 0번), jmp 핀 = B상.
    # X 레지스터 = 누적 카운트. PIO 에는 덧셈 명령이 없어서 증가는
    # "반전 -> 감소 -> 반전" 트릭을 쓴다 (원본 .pio 와 동일).
    set(x, 0)                  # 카운터 초기화 (C 판에는 없던 한 줄 — 무해)

    wrap_target()
    wait(0, pin, 0)
    wait(1, pin, 0)            # A 상승엣지
    jmp(pin, "decrement")      # 이 순간 B 가 high 면 감소 방향

    mov(x, invert(x))          # increment
    jmp(x_dec, "inc2")
    label("inc2")
    mov(x, invert(x))
    jmp("push_count")

    label("decrement")
    jmp(x_dec, "push_count")   # x-- 후 어느 쪽이든 다음 명령으로 간다

    label("push_count")
    mov(isr, x)
    push(noblock)
    wrap()


_sms = []
_pins = []            # Pin 객체를 살려둬야 풀업 설정이 유지된다
_last = [0, 0, 0]


def encoder_init_all():
    """PIO0 의 SM 0/1/2 에 모터 3개를 하나씩 올린다 (C 와 같은 배치)."""
    for i in range(config.NUM_MOTORS):
        pin_a = Pin(config.PIN_ENC_A[i], Pin.IN, Pin.PULL_UP)
        pin_b = Pin(config.PIN_ENC_B[i], Pin.IN, Pin.PULL_UP)
        _pins.append((pin_a, pin_b))

        sm = rp2.StateMachine(i, _quadrature_1x, in_base=pin_a, jmp_pin=pin_b)
        sm.active(1)
        _sms.append(sm)
        _last[i] = 0


def encoder_get_count(i):
    """RX FIFO 를 끝까지 비우고 **가장 최근** 카운트를 돌려준다 (C 와 동일).

    PIO 는 `push noblock` 이라 FIFO 가 차면 새 값을 버린다 — 그래도 X 레지스터
    자체는 계속 정확하므로, 비운 뒤 마지막으로 읽은 값이 항상 최신이다.
    """
    sm = _sms[i]
    v = _last[i]
    while sm.rx_fifo():
        v = sm.get()
    # sm.get() 은 uint32 로 올라온다 — int32 로 되돌린다.
    if v >= 0x80000000:
        v -= 0x100000000
    _last[i] = v
    return v


_prev = [0, 0, 0]
_delta = [0, 0, 0]


def encoder_sync():
    """지금 카운트를 기준점으로 삼는다 (다음 `encoder_read_deltas()` 가 0 근처).

    제어 루프를 시작하기 직전에 한 번 부른다 — 부팅부터 루프 시작까지 손으로
    바퀴를 굴렸다면 그게 첫 틱에 통째로 속도로 잡히기 때문.
    """
    for i in range(config.NUM_MOTORS):
        _prev[i] = encoder_get_count(i)


def encoder_read_deltas():
    """지난 호출 이후의 증감 3개. int32 랩어라운드를 C 의 뺄셈과 같게 처리한다.

    파이썬 int 는 무한정밀도라 카운터가 ±2^31 을 넘는 순간 델타가 42억으로
    튄다 — C 에서는 int32 뺄셈이 알아서 감싸주던 자리다.
    """
    for i in range(config.NUM_MOTORS):
        c = encoder_get_count(i)
        d = (c - _prev[i]) & 0xFFFFFFFF
        if d >= 0x80000000:
            d -= 0x100000000
        _prev[i] = c
        _delta[i] = d
    return _delta

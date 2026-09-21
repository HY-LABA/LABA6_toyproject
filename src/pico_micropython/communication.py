# [START 0xAA][LEN][PAYLOAD][CHECKSUM], CHECKSUM=sum(PAYLOAD)%256.
# pico/communication.c 의 MicroPython 포팅. **pi5/communication.py 와 바이트 단위로
# 같아야 한다** (docs/design/protocol.md 2장).
#
#   수신 16B <ffff> target_x, target_y, time_remaining_s, timeout_s — 평상시 구동(목표점)
#   수신 12B <fff>  target_vx, target_vy, timeout_s                — 텔레옵·정지(속도)
#   송신 24B <ffffff> x, y, theta, vx, vy, omega                   — 오도메트리
#
# ============================================================================
# MicroPython 에서만 조심해야 하는 것 두 가지
# ============================================================================
# 1) **micropython.kbd_intr(-1) 없이는 절대 못 쓴다.**
#    기본 설정에서 MicroPython 은 stdin 으로 들어온 0x03 바이트를 Ctrl-C 로 보고
#    KeyboardInterrupt 를 던진다. 우리가 받는 건 float 이진 데이터라 0x03 이
#    **언젠가 반드시** 섞여 들어온다 — 그 순간 제어 루프가 죽고 모터는 마지막
#    듀티로 계속 돈다. `communication_init()` 이 이걸 끈다.
#    ⚠ 끄고 나면 Thonny 의 Ctrl-C 도 안 먹는다. main.py 가 시작 직후 몇 초를
#      비워두는 이유가 그것이다 (main.py 의 BOOT_GRACE_S).
#
# 2) **stdout 은 블로킹할 수 있다.**
#    호스트가 안 읽으면 MicroPython 의 USB CDC 쓰기는 내부 타임아웃(보통 500ms)
#    까지 기다린다 — 제어 루프 안에서 그만큼 걸리면 모터가 마지막 듀티로 계속
#    돈다. C 가 `tud_cdc_write_available()` 로 막던 자리를 여기서는 stdout 에
#    POLLOUT 폴링으로 막는다. 보낼 자리가 없으면 이번 주기는 그냥 건너뛴다 —
#    오도메트리는 한 프레임 빠져도 되지만 제어 루프는 멈추면 안 된다.

import select
import struct
import sys

import micropython

FRAME_START = 0xAA
TARGET_PAYLOAD_LEN = 16
VELOCITY_PAYLOAD_LEN = 12
SEND_PAYLOAD_LEN = 24

# communication_try_receive() 의 반환값
RX_NONE = 0
RX_TARGET = 1
RX_VELOCITY = 2

# 수신 결과 — 매 틱 객체를 새로 만들지 않으려고 모듈 전역에 둔다.
rx_target_x = 0.0
rx_target_y = 0.0
rx_time_remaining_s = 0.0
rx_target_vx = 0.0
rx_target_vy = 0.0
rx_timeout_s = 0.0

_WAIT_START = 0
_WAIT_LEN = 1
_WAIT_PAYLOAD = 2
_WAIT_CHECKSUM = 3

_state = _WAIT_START
_len = 0
_index = 0
_payload = bytearray(TARGET_PAYLOAD_LEN)

_stdin = sys.stdin.buffer
_stdout = sys.stdout.buffer
_poll_in = None
_poll_out = None

# 송신 프레임 버퍼 — 헤더/LEN 은 고정이라 한 번만 채운다.
_tx = bytearray(SEND_PAYLOAD_LEN + 3)
_tx[0] = FRAME_START
_tx[1] = SEND_PAYLOAD_LEN


def communication_init():
    global _state, _index, _poll_in, _poll_out

    # (1) 이진 데이터의 0x03 을 Ctrl-C 로 해석하지 않게 한다. 위 주석 참고.
    micropython.kbd_intr(-1)

    _poll_in = select.poll()
    _poll_in.register(_stdin, select.POLLIN)

    # stdout 의 POLLOUT 은 MicroPython 판/포트에 따라 지원 안 될 수 있다. 그때는
    # 폴링 없이 그냥 보낸다 — 호스트가 안 읽으면 최대 내부 타임아웃(약 500ms)만큼
    # 이 틱이 늦어진다. 파이5가 정상 동작 중이면 생기지 않는 상황이다.
    try:
        _poll_out = select.poll()
        _poll_out.register(_stdout, select.POLLOUT)
    except (OSError, TypeError, AttributeError):
        _poll_out = None
        print("경고: stdout POLLOUT 미지원 — 오도메트리 송신이 블로킹할 수 있다")

    _state = _WAIT_START
    _index = 0


def communication_deinit():
    """Ctrl-C 를 되살린다 (Thonny 에서 다시 만질 수 있게)."""
    micropython.kbd_intr(3)


def _readable():
    for _, ev in _poll_in.ipoll(0):
        return bool(ev & select.POLLIN)
    return False


def _writable():
    if _poll_out is None:
        return True
    for _, ev in _poll_out.ipoll(0):
        return bool(ev & select.POLLOUT)
    return False


def communication_try_receive():
    """버퍼를 끝까지 비우고 **마지막 완전 프레임**의 종류를 돌려준다.

    ★ 프레임 하나를 완성해도 여기서 돌아가지 않는다. 즉시 돌아가면 이 함수는
      제어 틱당 최대 1프레임만 소비한다 — 제어 주기가 20ms(50Hz)인데 파이는
      카메라 프레임마다(30~60Hz) 보내므로, 파이가 50Hz 를 넘는 순간 USB CDC
      버퍼에 밀리기 시작하고 **지연이 무한히 누적된다.** 낡은 목표점으로 달리는
      건 오도메트리가 낡은 것과 똑같이 위험하다. 중간 프레임들은 어차피 낡은
      정보이므로 버리고 마지막 것만 채택한다 (docs/design/protocol.md 4장).
    """
    global _state, _len, _index
    global rx_target_x, rx_target_y, rx_time_remaining_s
    global rx_target_vx, rx_target_vy, rx_timeout_s

    kind = RX_NONE

    while _readable():
        byte = _stdin.read(1)[0]

        if _state == _WAIT_START:
            if byte == FRAME_START:
                _state = _WAIT_LEN

        elif _state == _WAIT_LEN:
            _len = byte
            _index = 0
            # 12든 16이든 받는다 — 그 외 길이는 조용히 버린다 (옛 펌웨어에 새
            # 프레임을 보내도 안전하게 무시되던 성질 유지).
            if byte == TARGET_PAYLOAD_LEN or byte == VELOCITY_PAYLOAD_LEN:
                _state = _WAIT_PAYLOAD
            else:
                _state = _WAIT_START

        elif _state == _WAIT_PAYLOAD:
            _payload[_index] = byte
            _index += 1
            if _index >= _len:
                _state = _WAIT_CHECKSUM

        else:  # _WAIT_CHECKSUM
            s = 0
            for i in range(_len):
                s += _payload[i]
            if (s & 0xFF) == byte:
                if _len == TARGET_PAYLOAD_LEN:
                    (rx_target_x, rx_target_y, rx_time_remaining_s,
                     rx_timeout_s) = struct.unpack_from("<ffff", _payload, 0)
                    kind = RX_TARGET
                else:
                    (rx_target_vx, rx_target_vy,
                     rx_timeout_s) = struct.unpack_from("<fff", _payload, 0)
                    kind = RX_VELOCITY
            _state = _WAIT_START

    return kind


def communication_send_odometry(x, y, theta, vx, vy, omega):
    # 논블로킹: 보낼 자리가 없으면 이번 주기는 건너뛴다 (파일 머리 주석 2번).
    if not _writable():
        return

    struct.pack_into("<ffffff", _tx, 2, x, y, theta, vx, vy, omega)
    s = 0
    for i in range(2, SEND_PAYLOAD_LEN + 2):
        s += _tx[i]
    _tx[SEND_PAYLOAD_LEN + 2] = s & 0xFF
    _stdout.write(_tx)

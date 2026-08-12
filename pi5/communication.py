"""[START 0xAA][LEN][PAYLOAD][CHECKSUM], CHECKSUM=sum(PAYLOAD)%256.

송신 PAYLOAD=<fff> target_vx, target_vy, timeout_s   (12바이트)
수신 PAYLOAD=<ffffff> x, y, theta, vx, vy, omega     (24바이트)

pico/communication.c와 반드시 일치해야 함.

**프레임 포맷은 그대로지만 송신 payload의 의미가 바뀌었다 (2026-08-10).**
예전: (목표 x좌표, 목표 y좌표, 구동시간) — 피코가 좌표÷시간으로 속도를 계산했다.
지금: (목표 vx, 목표 vy, 워치독 타임아웃) — 파이5가 이미 속도로 계산해서 보낸다.
바이트 수가 같으므로 프레임 파싱 코드는 안 깨지지만, **양쪽을 같이 업데이트하지
않으면 조용히 엉뚱한 속도로 돌아간다.** 필드 이름을 바꿔둔 이유다.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import config
from control import DriveCommand

_START_BYTE = 0xAA

_SEND_FORMAT = "<fff"
_RECV_FORMAT = "<ffffff"


@dataclass
class Odometry:
    """피코가 엔코더로 적분한 누적 이동량 + 현재 속도. 모두 world frame."""

    x: float
    y: float
    theta: float
    vx: float
    vy: float
    omega: float

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)


def _checksum(payload: bytes) -> int:
    return sum(payload) % 256


def _encode(payload: bytes) -> bytes:
    return bytes([_START_BYTE, len(payload)]) + payload + bytes([_checksum(payload)])


def _decode_payload(frame: bytes) -> bytes:
    if len(frame) < 3 or frame[0] != _START_BYTE:
        raise ValueError(f"START 바이트 불일치: {frame!r}")
    length = frame[1]
    payload = frame[2:2 + length]
    checksum = frame[2 + length]
    if len(payload) != length:
        raise ValueError(f"길이 불일치: 예상 {length}, 실제 {len(payload)}")
    if _checksum(payload) != checksum:
        raise ValueError(f"체크섬 불일치: {frame!r}")
    return payload


class SerialLink:
    def __init__(self, port: str | None = None) -> None:
        """port를 넘기면 config.SERIAL_PORT보다 우선한다 (teleop_test.py처럼 장치
        경로를 CLI에서 받는 경우용 — config.SERIAL_PORT는 아직 확정 전이라 None)."""
        import serial

        resolved = port or config.SERIAL_PORT
        if resolved is None:
            raise ValueError("SERIAL_PORT 미설정 — config.py에 채우거나 port 인자로 넘길 것")
        self._port = serial.Serial(resolved, config.SERIAL_BAUDRATE,
                                    timeout=config.SERIAL_TIMEOUT_S)

    def send_command(self, cmd: DriveCommand) -> None:
        payload = struct.pack(_SEND_FORMAT, cmd.target_vx, cmd.target_vy, cmd.timeout_s)
        self._port.write(_encode(payload))

    def try_receive_odometry(self) -> Odometry | None:
        """읽을 게 있으면 최신 오도메트리, 없으면 None.

        블로킹하지 않는다. 제어 루프가 카메라 프레임 주기(40fps = 25ms)에 묶여
        있는데 여기서 기다리면 프레임을 놓치고, 놓친 프레임은 그대로 궤적 관측의
        손실이다. 피코는 1ms마다 보내므로 버퍼에 쌓인 것 중 **가장 최근 것**만
        쓰고 나머지는 흘린다 — 오래된 오도메트리는 아무 가치가 없다.
        """
        latest: Odometry | None = None
        frame_size = 3 + struct.calcsize(_RECV_FORMAT)
        while self._port.in_waiting >= frame_size:
            start = self._port.read(1)
            if not start or start[0] != _START_BYTE:
                continue
            length = self._port.read(1)[0]
            payload = self._port.read(length)
            checksum_byte = self._port.read(1)
            if not checksum_byte or len(payload) != length:
                break
            if _checksum(payload) != checksum_byte[0]:
                continue
            if length != struct.calcsize(_RECV_FORMAT):
                continue
            x, y, theta, vx, vy, omega = struct.unpack(_RECV_FORMAT, payload)
            latest = Odometry(x=x, y=y, theta=theta, vx=vx, vy=vy, omega=omega)
        return latest

    def close(self) -> None:
        if self._port is not None:
            self._port.close()
            self._port = None


def debug_decode(raw: bytes) -> str:
    payload = _decode_payload(raw)
    if len(payload) == struct.calcsize(_RECV_FORMAT):
        x, y, theta, vx, vy, omega = struct.unpack(_RECV_FORMAT, payload)
        return (f"[odometry] x={x:.3f} y={y:.3f} theta={theta:.3f} "
                f"vx={vx:.3f} vy={vy:.3f} omega={omega:.3f}")
    if len(payload) == struct.calcsize(_SEND_FORMAT):
        target_vx, target_vy, timeout_s = struct.unpack(_SEND_FORMAT, payload)
        return (f"[drive_command] target_vx={target_vx:.3f} target_vy={target_vy:.3f} "
                f"timeout_s={timeout_s:.3f}")
    return f"[unknown] payload={payload!r}"


if __name__ == "__main__":
    import sys

    print(debug_decode(sys.stdin.buffer.read()))

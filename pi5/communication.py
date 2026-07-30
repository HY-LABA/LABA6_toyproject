"""[START 0xAA][LEN][PAYLOAD][CHECKSUM], CHECKSUM=sum(PAYLOAD)%256.
송신 PAYLOAD=<fff> target_x,target_y,drive_time_s / 수신 PAYLOAD=<ffffff> x,y,theta,vx,vy,omega.
pico/communication.c와 반드시 일치해야 함."""

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
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    omega: float


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
    def __init__(self) -> None:
        self._port = None
        # 정해야함: pyserial 연결 (config.SERIAL_PORT)

    def send_target(self, cmd: DriveCommand) -> None:
        if self._port is None:
            raise NotImplementedError
        payload = struct.pack(_SEND_FORMAT, cmd.target_x, cmd.target_y, cmd.drive_time_s)
        self._port.write(_encode(payload))

    def receive_odometry(self) -> Odometry:
        if self._port is None:
            raise NotImplementedError
        while True:
            start = self._port.read(1)
            if not start or start[0] != _START_BYTE:
                continue
            length = self._port.read(1)[0]
            payload = self._port.read(length)
            checksum_byte = self._port.read(1)[0]
            if len(payload) == length and _checksum(payload) == checksum_byte:
                x, y, theta, vx, vy, omega = struct.unpack(_RECV_FORMAT, payload)
                return Odometry(x=x, y=y, theta=theta, vx=vx, vy=vy, omega=omega)


def debug_decode(raw: bytes) -> str:
    payload = _decode_payload(raw)
    if len(payload) == struct.calcsize(_RECV_FORMAT):
        x, y, theta, vx, vy, omega = struct.unpack(_RECV_FORMAT, payload)
        return f"[odometry] x={x:.3f} y={y:.3f} theta={theta:.3f} vx={vx:.3f} vy={vy:.3f} omega={omega:.3f}"
    if len(payload) == struct.calcsize(_SEND_FORMAT):
        target_x, target_y, drive_time_s = struct.unpack(_SEND_FORMAT, payload)
        return f"[drive_command] target_x={target_x:.3f} target_y={target_y:.3f} drive_time_s={drive_time_s:.3f}"
    return f"[unknown] payload={payload!r}"


if __name__ == "__main__":
    import sys

    print(debug_decode(sys.stdin.buffer.read()))

"""피코와의 프레임 계약. pico/communication.c와 반드시 일치해야 한다.

    [START 0xAA][LEN][PAYLOAD ...][CHECKSUM]      CHECKSUM = sum(PAYLOAD) % 256

    파이5 -> 피코 (12 B, <fff)   cmd_vx, cmd_vy, ttl_s
    피코 -> 파이5 (28 B, <Iffffff) t_ms, x, y, theta, vx, vy, omega

순수 함수 + 스트리밍 파서. 포트를 직접 만지지 않으므로 테스트 가능하다.
상세는 ../docs/protocol.md.
"""

from __future__ import annotations

import struct

from datatypes import Pose, VelocityCommand

START_BYTE = 0xAA
CMD_FORMAT = "<fff"          # 12 B
ODOM_FORMAT = "<Iffffff"     # 28 B
CMD_LEN = struct.calcsize(CMD_FORMAT)
ODOM_LEN = struct.calcsize(ODOM_FORMAT)


def checksum(payload: bytes) -> int:
    return sum(payload) % 256


def encode_command(cmd: VelocityCommand) -> bytes:
    payload = struct.pack(CMD_FORMAT, cmd.vx, cmd.vy, cmd.ttl_s)
    return bytes([START_BYTE, len(payload)]) + payload + bytes([checksum(payload)])


def decode_odometry(payload: bytes) -> Pose:
    t_ms, x, y, theta, vx, vy, omega = struct.unpack(ODOM_FORMAT, payload)
    return Pose(t_ms=t_ms, x=x, y=y, theta=theta, vx=vx, vy=vy, omega=omega)


class FrameParser:
    """바이트 스트림에서 프레임을 뽑아내는 논블로킹 상태 기계.

    부분 수신·쓰레기 바이트·체크섬 오류를 모두 견딘다. USB CDC는 프레임 경계를
    보장하지 않으므로 이 계층이 필요하다.
    """

    _WAIT_START, _WAIT_LEN, _WAIT_PAYLOAD, _WAIT_CHECKSUM = range(4)

    def __init__(self, expected_len: int = ODOM_LEN) -> None:
        self._expected_len = expected_len
        self._state = self._WAIT_START
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        """받은 바이트를 넣고, 완성된 페이로드들을 순서대로 돌려준다."""
        out: list[bytes] = []
        for byte in data:
            payload = self._feed_one(byte)
            if payload is not None:
                out.append(payload)
        return out

    def _feed_one(self, byte: int) -> bytes | None:
        if self._state == self._WAIT_START:
            if byte == START_BYTE:
                self._state = self._WAIT_LEN
        elif self._state == self._WAIT_LEN:
            if byte == self._expected_len:
                self._buf.clear()
                self._state = self._WAIT_PAYLOAD
            else:
                self._state = self._WAIT_START      # 길이 불일치 -> 재동기
        elif self._state == self._WAIT_PAYLOAD:
            self._buf.append(byte)
            if len(self._buf) >= self._expected_len:
                self._state = self._WAIT_CHECKSUM
        elif self._state == self._WAIT_CHECKSUM:
            payload = bytes(self._buf)
            self._state = self._WAIT_START
            if checksum(payload) == byte:
                return payload
        return None


def debug_decode(raw: bytes) -> str:
    """raw 바이트를 사람이 읽는 텍스트로. 페이로드 길이로 방향을 구분한다."""
    if len(raw) < 3 or raw[0] != START_BYTE:
        return f"[invalid] {raw!r}"
    length = raw[1]
    payload = raw[2:2 + length]
    if len(payload) != length or raw[2 + length] != checksum(payload):
        return f"[corrupt] {raw!r}"

    if length == ODOM_LEN:
        p = decode_odometry(payload)
        return (f"[odometry] t={p.t_ms} x={p.x:.3f} y={p.y:.3f} theta={p.theta:.3f} "
                f"vx={p.vx:.3f} vy={p.vy:.3f} omega={p.omega:.3f}")
    if length == CMD_LEN:
        vx, vy, ttl = struct.unpack(CMD_FORMAT, payload)
        return f"[command] vx={vx:.3f} vy={vy:.3f} ttl={ttl:.3f}"
    return f"[unknown len={length}] {payload!r}"

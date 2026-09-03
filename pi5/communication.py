"""[START 0xAA][LEN][PAYLOAD][CHECKSUM], CHECKSUM=sum(PAYLOAD)%256.

    송신 ① 목표점  PAYLOAD=<ffff>   target_x, target_y, time_remaining_s, timeout_s  (16B)
    송신 ② 속도    PAYLOAD=<fff>    target_vx, target_vy, timeout_s                  (12B)
    수신   오도메트리 PAYLOAD=<ffffff> x, y, theta, vx, vy, omega                     (24B)

pico/communication.c 와 반드시 일치해야 한다.

★ 명령이 두 종류인 이유 — LEN 으로 구분한다
--------------------------------------------
평소 구동은 ①이다. 파이는 **도착 지점만** 보내고 방향·속도 계산은 피코가 한다.
②는 게임패드 텔레옵(`teleop_test.py`)과 정지(`control.STOP`)용이다 — 스틱 입력은
본질적으로 속도지 좌표가 아니고, 모터 최대속도·가속시간·정지거리 실측이 이 경로에
걸려 있다. 프레임에 이미 LEN 바이트가 있으므로 **페이로드 길이가 곧 명령 종류**다.
12B 와 16B 로 갈리니 섞일 수 없다.

⚠ 예전 프로토콜의 12B 송신은 의미가 **속도**였다. 지금 ②가 그 자리를 그대로 쓰므로
  옛 펌웨어에 ②를 보내면 정상 동작한다. 반대로 ①(16B)은 옛 펌웨어가 LEN 에서
  거부한다 — 조용히 틀리지 않고 **명령이 안 먹는 형태로** 드러난다.

★ 피코가 ①을 받으면 해야 하는 일 (펌웨어 계약)
-----------------------------------------------
    rx = target_x - pose.x                 // ← 파이는 이걸 빼서 보내지 않는다
    ry = target_y - pose.y
    dist = hypot(rx, ry)
    if dist <= POSITION_TOLERANCE_M: 정지   // 채터링 방지
    v = (time_remaining_s > 0) ? dist / time_remaining_s : 무한대
    v = min(v, 이 방향에서 바퀴가 포화되지 않는 상한)
    inverse_kinematics(rx/dist*v, ry/dist*v, 0, wheel_target)
    time_remaining_s -= dt                 // 다음 명령이 오면 덮어쓴다

  ⚠ 상한은 **바퀴 속도** 기준이어야 한다. body 속력을 상수 하나로 자르면 방향별
    이득(최대 15.5%)이 사라지고, 그러면 파이의 `tracker._reach()` 가 실제보다
    낙관해서 못 잡을 표적을 쫓는다.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import config
from control import DriveCommand, TargetCommand

_START_BYTE = 0xAA

_TARGET_FORMAT = "<ffff"    # 목표점 명령 (평상시 구동)
_SEND_FORMAT = "<fff"       # 속도 명령 (텔레옵·정지)
_RECV_FORMAT = "<ffffff"    # 오도메트리


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

    def send_target(self, cmd: TargetCommand) -> None:
        """★ 평상시 구동 명령 — 월드 좌표 도착 지점."""
        payload = struct.pack(_TARGET_FORMAT, cmd.target_x, cmd.target_y,
                              cmd.time_remaining_s, cmd.timeout_s)
        self._port.write(_encode(payload))

    def send_command(self, cmd: DriveCommand) -> None:
        """속도 명령 — 텔레옵과 정지에만 쓴다. 구동은 `send_target()`."""
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
    if len(payload) == struct.calcsize(_TARGET_FORMAT):
        target_x, target_y, remaining, timeout_s = struct.unpack(_TARGET_FORMAT, payload)
        return (f"[target] x={target_x:.3f} y={target_y:.3f} "
                f"t_rem={remaining:.3f}s timeout={timeout_s:.3f}s")
    if len(payload) == struct.calcsize(_SEND_FORMAT):
        target_vx, target_vy, timeout_s = struct.unpack(_SEND_FORMAT, payload)
        return (f"[drive_command] target_vx={target_vx:.3f} target_vy={target_vy:.3f} "
                f"timeout_s={timeout_s:.3f}")
    return f"[unknown] payload={payload!r}"


if __name__ == "__main__":
    import sys

    print(debug_decode(sys.stdin.buffer.read()))

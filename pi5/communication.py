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
import time
from dataclasses import dataclass

import config
import utils
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
        손실이다. 피코는 CONTROL_PERIOD_MS(pico/config.h, 현재 20ms=50Hz)마다
        보내므로 버퍼에 쌓인 것 중 **가장 최근 것**만 쓰고 나머지는 흘린다 —
        오래된 오도메트리는 아무 가치가 없다.

        ⚠ 이 docstring에 예전엔 "피코는 1ms마다 보낸다"고 적혀 있었는데,
          `pico/main.c`의 실제 제어 루프 주기(`CONTROL_PERIOD_MS`)를 확인해보니
          20ms(50Hz)였다 — 1ms는 틀린 수치였다. `LinkStats`로 실측하면 이 50Hz에
          USB/스케줄링 지연이 섞여 실제로는 그보다 낮게 나올 수 있다.
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


class LinkStats:
    """`SerialLink`를 감싸서 실제 송수신 시각을 재는 계측 래퍼. ★ 2026-09-15 추가.

    프로토콜은 하나도 안 건드리고 옆에서 `time.monotonic()` 타임스탬프만 찍는다.
    `main.py --comm-stats`로 켜면, 카메라+YOLO 부하가 실제로 걸린 상태에서
    "오도메트리가 진짜 몇 Hz로 들어오는지"와 "명령을 몇 Hz로 내보내는지"를 잰다.

    ⚠ **이건 왕복시간(RTT)이 아니다.** 오도메트리는 명령에 대한 응답(ACK)이
    아니라 피코가 독자적으로 계속 흘리는 텔레메트리라서 — `communication.py`
    모듈 docstring에 있듯 요청-응답 구조 자체가 없다 — "보낸 명령에 이 프로토콜로
    응답이 몇 ms 만에 왔나"는 원리적으로 잴 수 없다. 대신 잴 수 있는(그리고 실제로
    유용한) 건 둘이다:
      ① 오도메트리 수신 Hz — 피코의 실제 송신 주기(현재 설계상 50Hz,
         `pico/main.c`의 CONTROL_PERIOD_MS)에 USB/드라이버/파이썬 스케줄링
         지연이 섞인, **파이가 실제로 체감하는** 피드백 빈도.
      ② 명령 송신 Hz — `main.py`의 메인 루프가 실제로 몇 바퀴 도는지. 이건
         피코 통신 속도가 아니라 **카메라 캡처+YOLO 추론을 포함한 파이 쪽
         처리율**에 매인다 — 반응이 느리다고 느껴질 때 병목이 통신인지 이쪽인지
         가르는 데 쓴다.
    """

    def __init__(self, link: "SerialLink") -> None:
        self._link = link
        self._recv_t: list[float] = []
        self._send_t: list[float] = []
        self._last_periodic = time.monotonic()

    def send_target(self, cmd: TargetCommand) -> None:
        self._send_t.append(time.monotonic())
        self._link.send_target(cmd)

    def send_command(self, cmd: DriveCommand) -> None:
        self._send_t.append(time.monotonic())
        self._link.send_command(cmd)

    def try_receive_odometry(self) -> Odometry | None:
        odom = self._link.try_receive_odometry()
        if odom is not None:
            self._recv_t.append(time.monotonic())
        return odom

    def close(self) -> None:
        self._link.close()

    @staticmethod
    def _describe(times: list[float]) -> str:
        if len(times) < 2:
            return f"표본 {len(times)}개 (부족)"
        import statistics

        intervals = [b - a for a, b in zip(times, times[1:])]
        mean_iv = statistics.mean(intervals)
        return (f"{len(times)}개, 평균 {1.0 / mean_iv:.1f}Hz "
                f"(간격 평균 {mean_iv * 1000:.1f}ms, 최대 {max(intervals) * 1000:.1f}ms, "
                f"표준편차 {statistics.pstdev(intervals) * 1000:.1f}ms)")

    def maybe_log_periodic(self, period_s: float = 5.0) -> None:
        """`period_s`마다 최근 구간 통계를 run.log에 남긴다. 메인 루프에서 매
        프레임 불러도 된다 — 내부에서 알아서 주기를 확인한다."""
        now = time.monotonic()
        if now - self._last_periodic < period_s:
            return
        self._last_periodic = now
        cutoff = now - period_s
        recent_recv = [t for t in self._recv_t if t >= cutoff]
        recent_send = [t for t in self._send_t if t >= cutoff]
        utils.log(f"[comm-stats 최근 {period_s:.0f}s] 오도메트리 수신: "
                  f"{self._describe(recent_recv)} | 명령 송신: {self._describe(recent_send)}")

    def log_final_summary(self) -> None:
        """실행 전체 구간 통계 — 종료 직전(정지 명령 보내기 전)에 부른다."""
        utils.log(f"[comm-stats 전체] 오도메트리 수신: {self._describe(self._recv_t)}")
        utils.log(f"[comm-stats 전체] 명령 송신: {self._describe(self._send_t)}")


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

"""피코와의 시리얼 링크 (USB CDC).

⚠ 하드웨어 경계다. pyserial 연동은 아직 스텁이다.
   프레임 인코딩/디코딩은 protocol.py가 담당하고, 여기는 포트 I/O만 맡는다.

drain-to-latest가 이 모듈의 핵심이다:
   피코가 100 Hz로 보내는데 파이5는 40 Hz로 읽으므로 버퍼에 프레임이 쌓인다.
   순진하게 read()하면 **가장 오래된** 프레임을 읽게 되어 지연이 무한히 누적된다.
   -> 버퍼를 전부 비우고 마지막 프레임만 쓴다. (../docs/protocol.md 4장)
"""

from __future__ import annotations

from typing import Protocol

import config
import protocol
from datatypes import Pose, VelocityCommand


class Link(Protocol):
    """피코 링크. 실제 구현은 pyserial, 테스트는 가짜 객체."""

    def send(self, cmd: VelocityCommand) -> None: ...
    def latest_odometry(self) -> Pose | None: ...


class SerialLink:
    """pyserial 기반 구현. 정해야함: 실제 포트 오픈."""

    def __init__(self, port: str | None = None, baudrate: int | None = None) -> None:
        self._port = None
        self._parser = protocol.FrameParser(protocol.ODOM_LEN)
        self._last: Pose | None = None
        raise NotImplementedError(
            "정해야함: pyserial 포트 오픈 "
            f"(port={port or config.SERIAL_PORT}, baud={baudrate or config.SERIAL_BAUDRATE})"
        )

    def send(self, cmd: VelocityCommand) -> None:
        assert self._port is not None
        self._port.write(protocol.encode_command(cmd))

    def latest_odometry(self) -> Pose | None:
        """버퍼에 쌓인 프레임을 전부 소비하고 **가장 최근** 것만 돌려준다.

        새 프레임이 없으면 직전 값을 유지한다 (None이 아니라 마지막으로 알던 자세).
        """
        assert self._port is not None
        pending = self._port.in_waiting
        if pending:
            for payload in self._parser.feed(self._port.read(pending)):
                self._last = protocol.decode_odometry(payload)
        return self._last

"""파이프라인 각 단계가 주고받는 공용 데이터 구조.

의존성 없음 — 모든 모듈이 여기를 참조한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class Detection:
    """① 객체 인식의 출력. bbox는 항상 원본 센서 픽셀 좌표계 기준."""

    class_name: str
    cx: float  # bbox 중심 x (px)
    cy: float  # bbox 중심 y (px)
    w: float   # bbox 폭 (px) — z 추정에 쓰는 축 (롤링셔터 왜곡 없음)
    h: float   # bbox 높이 (px) — 자세 판별용. z 추정에는 쓰지 않는다
    confidence: float


@dataclass(frozen=True)
class Pose:
    """피코가 보내온 오도메트리. world frame 자세 + body frame 속도."""

    t_ms: int      # 피코 부팅 후 경과 (ms) — 차분으로만 쓴다
    x: float       # world (m)
    y: float       # world (m)
    theta: float   # world 기준 로봇 방향 (rad)
    vx: float      # body (m/s)
    vy: float      # body (m/s)
    omega: float   # (rad/s)

    @staticmethod
    def zero() -> "Pose":
        return Pose(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Landing:
    """④ 착지 예측 결과. world frame."""

    x: float
    y: float
    t_land: float   # 지금부터 착지까지 남은 시간 (s)
    valid: bool


@dataclass(frozen=True)
class VelocityCommand:
    """피코로 보내는 목표 속도. body frame."""

    vx: float
    vy: float
    ttl_s: float    # 유효시간. 만료되면 피코가 스스로 정지한다


class State(Enum):
    """main 루프 상태."""

    SEARCH = "search"   # 전체 프레임 축소 추론 — 위치만 찾는다
    TRACK = "track"     # ROI 네이티브 크롭 추론 — 정밀 bbox

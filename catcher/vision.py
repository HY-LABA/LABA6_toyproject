"""① 객체 인식 — 카메라 캡처 + YOLOv8n 추론.

⚠ 하드웨어 경계다. Picamera2와 HailoRT 연동은 아직 스텁이다.
   인터페이스만 확정되어 있고, main은 이 프로토콜을 만족하는 객체면 무엇이든 받는다
   (시뮬레이터의 가짜 카메라도 그대로 꽂힌다).

2단계 추론을 쓰는 이유 — 이게 이 모듈의 핵심 설계다:
   전체 프레임(1456x1088)을 640으로 줄여 추론하면 bbox가 2.3배 작아진다.
   2m 중심부에서 26px -> 11px이 되어 프레임당 거리 오차가 2.3배로 늘고,
   YOLO 검출률 자체도 떨어진다.
   -> SEARCH는 위치만 찾고, TRACK은 네이티브 해상도 ROI로 정밀 bbox를 얻는다.
   (../docs/vision-pipeline.md 2장)
"""

from __future__ import annotations

from typing import Protocol

import numpy as np

import config
from datatypes import Detection


class Camera(Protocol):
    """프레임 공급자. 실제 구현은 Picamera2, 테스트는 가짜 객체."""

    def capture(self) -> tuple[np.ndarray, float]:
        """(frame, t_capture_seconds)를 반환한다. frame은 원본 해상도."""
        ...


class Detector(Protocol):
    """YOLO 추론기. 실제 구현은 HailoRT, 테스트는 가짜 객체."""

    def infer(self, image: np.ndarray) -> list[Detection]:
        """640x640 이미지 -> Detection 목록. bbox는 **입력 이미지 좌표계** 기준."""
        ...


# ── 실제 구현 (스텁) ────────────────────────────────────────────────────

class Picamera2Camera:
    """Picamera2 래퍼. 정해야함: 실제 연동."""

    def __init__(self) -> None:
        raise NotImplementedError(
            f"정해야함: Picamera2 초기화 — {config.FRAME_WIDTH}x{config.FRAME_HEIGHT}, "
            f"{config.CAMERA_FPS}fps, 노출 {config.EXPOSURE_MAX_S*1000:.2f}ms 이하"
        )

    def capture(self) -> tuple[np.ndarray, float]:
        raise NotImplementedError


class HailoDetector:
    """HailoRT YOLOv8n 추론기. 정해야함: 실제 연동.

    ⚠ HailoRT API는 실제 설치 버전 문서로 검증할 것.
    ⚠ 모델은 yolov8n으로 확정했다 — Model Zoo 지원이 확실하기 때문이다
      (../prep/train_yolo.py, ../docs/vision-pipeline.md 5장).
    """

    def __init__(self, hef_path: str) -> None:
        raise NotImplementedError("정해야함: HEF 로드 + VDevice 구성")

    def infer(self, image: np.ndarray) -> list[Detection]:
        raise NotImplementedError


# ── 좌표 역변환 (순수 함수, 테스트 가능) ──────────────────────────────────

def _scale_detection(det: Detection, scale: float) -> Detection:
    """추론 좌표 -> 원본 좌표 (리사이즈 역변환)."""
    return Detection(
        class_name=det.class_name,
        cx=det.cx * scale, cy=det.cy * scale,
        w=det.w * scale, h=det.h * scale,
        confidence=det.confidence,
    )


def _offset_detection(det: Detection, x0: float, y0: float) -> Detection:
    """추론 좌표 -> 원본 좌표 (ROI 오프셋 역변환). 스케일은 1이다."""
    return Detection(
        class_name=det.class_name,
        cx=det.cx + x0, cy=det.cy + y0,
        w=det.w, h=det.h,
        confidence=det.confidence,
    )


def roi_bounds(center: tuple[float, float]) -> tuple[int, int]:
    """ROI 좌상단 (x0, y0). 프레임을 벗어나지 않도록 안쪽으로 민다."""
    size = config.ROI_SIZE_PX
    x0 = int(round(center[0] - size / 2))
    y0 = int(round(center[1] - size / 2))
    x0 = max(0, min(x0, config.FRAME_WIDTH - size))
    y0 = max(0, min(y0, config.FRAME_HEIGHT - size))
    return x0, y0


def _best(dets: list[Detection]) -> Detection | None:
    """대상 클래스 중 신뢰도가 가장 높은 하나."""
    candidates = [
        d for d in dets
        if d.class_name in config.TARGET_CLASSES and d.confidence >= config.YOLO_CONF_THRESHOLD
    ]
    return max(candidates, key=lambda d: d.confidence) if candidates else None


# ── main이 쓰는 두 함수 ─────────────────────────────────────────────────

def detect_full(detector: Detector, frame: np.ndarray) -> Detection | None:
    """SEARCH 단계. 전체 프레임을 640으로 줄여 추론한다.

    bbox 정밀도는 낮지만(원본 환산 σ_w ≈ 4.6 px) 위치만 찾으면 되므로 충분하다.
    """
    size = config.YOLO_INPUT_SIZE
    small = _resize(frame, size, size)
    det = _best(detector.infer(small))
    if det is None:
        return None
    return _scale_detection(det, config.FRAME_WIDTH / size)


def detect_roi(
    detector: Detector, frame: np.ndarray, center: tuple[float, float]
) -> Detection | None:
    """TRACK 단계. center 중심 640x640을 네이티브 해상도로 잘라 추론한다.

    스케일 손실이 없어 bbox가 정밀하다 (σ_w ≈ 2 px).
    """
    size = config.ROI_SIZE_PX
    x0, y0 = roi_bounds(center)
    crop = frame[y0:y0 + size, x0:x0 + size]
    det = _best(detector.infer(crop))
    if det is None:
        return None
    return _offset_detection(det, x0, y0)


def _resize(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """정해야함: cv2.resize 또는 picamera2의 리사이즈 경로."""
    raise NotImplementedError("정해야함: 리사이즈 구현 (cv2.resize)")

"""카메라 캡처 + YOLO 인식 + z 추정 + 3프레임 이동벡터 계산 (architecture.md 참고).

confirmed_frames 만큼 같은 클래스가 연속 감지되면 물체 확정 -> frame1, (건너뛰고) frame3
두 장을 비교해 vx, vy, vz를 구해 Observation으로 반환한다.

카메라(Picamera2)/YOLO(Hailo) 실제 연동은 하드웨어 확정 전이라 NotImplementedError로 남겨둔다.
"""

from __future__ import annotations

from dataclasses import dataclass

import config


@dataclass
class Detection:
    class_name: str
    bbox: tuple[float, float, float, float]  # (center_x_px, center_y_px, width_px, height_px)
    confidence: float


@dataclass
class Observation:
    class_name: str
    position: tuple[float, float, float]  # (x, y, z) m, frame3(최신) 시점 기준
    velocity: tuple[float, float, float]  # (vx, vy, vz) m/s, frame3 시점 기준
    t: float  # frame3 캡처 시각 (time.monotonic() 기준, 재보정 시 Δt 계산용)


class Camera:
    def __init__(self, resolution: tuple[int, int], fps: int) -> None:
        self.resolution = resolution
        self.fps = fps
        # TODO: Picamera2 초기화 (실제 하드웨어 연결 후)

    def capture_sample(self) -> tuple[object, float]:
        """샘플링 프레임 1장 캡처 -> (frame, timestamp_s).

        config.FRAME_SKIP 적용(2프레임마다 1장)은 이 함수 내부에서 처리한다.
        """
        raise NotImplementedError("Picamera2 연동 전")


def _detect(_frame: object) -> list[Detection]:
    """YOLO(Hailo) 추론 -> Detection 목록. TODO: Hailo NPU 연동."""
    raise NotImplementedError("Hailo YOLO 연동 전")


def _best_target_detection(frame: object) -> Detection | None:
    """TARGET_CLASSES 중 신뢰도 임계값을 넘는 최고 confidence 검출 1개, 없으면 None."""
    candidates = [
        d for d in _detect(frame)
        if d.class_name in config.TARGET_CLASSES and d.confidence >= config.YOLO_CONF_THRESHOLD
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.confidence)


def _estimate_z(class_name: str, bbox: tuple[float, float, float, float]) -> float:
    """bbox 크기 vs REFERENCE_SIZE_AT_1M 비율로 z(m) 역산."""
    ref = config.REFERENCE_SIZE_AT_1M.get(class_name)
    if ref is None:
        raise NotImplementedError(f"'{class_name}' 기준 사이즈 미실측 (config.REFERENCE_SIZE_AT_1M)")
    _, _, width_px, height_px = bbox
    # 기준(1m) 대비 현재 크기 비율의 역수 = 거리(m). 폭/높이 평균으로 노이즈 완화.
    ratio_w = ref["width_px"] / width_px
    ratio_h = ref["height_px"] / height_px
    return (ratio_w + ratio_h) / 2.0


def _pixel_to_meters_xy(bbox_center_px: tuple[float, float], z: float) -> tuple[float, float]:
    """bbox 중심의 픽셀 오프셋 -> 로봇(카메라) 기준 실제 x,y(m). 핀홀 모델."""
    if config.FOCAL_LENGTH_PX is None:
        raise NotImplementedError("카메라 캘리브레이션 전 (config.FOCAL_LENGTH_PX 없음)")
    cx_px, cy_px = bbox_center_px
    x = cx_px * z / config.FOCAL_LENGTH_PX
    y = cy_px * z / config.FOCAL_LENGTH_PX
    return x, y


def _observe_xyz(frame: object, class_name: str) -> tuple[float, float, float]:
    """프레임 하나에서 해당 클래스의 (x, y, z)를 뽑아낸다."""
    detections = [d for d in _detect(frame) if d.class_name == class_name]
    if not detections:
        raise RuntimeError(f"재관측 프레임에서 '{class_name}' 미검출")
    det = max(detections, key=lambda d: d.confidence)
    z = _estimate_z(det.class_name, det.bbox)
    cx_px, cy_px, _, _ = det.bbox
    x, y = _pixel_to_meters_xy((cx_px, cy_px), z)
    return x, y, z


def wait_for_confirmed_object(cam: Camera) -> Observation:
    """CONFIRM_FRAMES 연속 감지되면 확정 -> frame1/frame3 비교로 위치+속도 계산."""
    streak = 0
    class_name: str | None = None
    while streak < config.CONFIRM_FRAMES:
        frame, _ = cam.capture_sample()
        hit = _best_target_detection(frame)
        if hit is None or hit.class_name != class_name:
            class_name = hit.class_name if hit else None
            streak = 1 if hit else 0
        else:
            streak += 1
    assert class_name is not None

    # frame1
    frame1, t1 = cam.capture_sample()
    x1, y1, z1 = _observe_xyz(frame1, class_name)

    # frame2: 문서상 1번째/3번째만 비교에 쓰므로 캡처만 하고 버림
    cam.capture_sample()

    # frame3
    frame3, t3 = cam.capture_sample()
    x3, y3, z3 = _observe_xyz(frame3, class_name)

    dt = t3 - t1
    vx = (x3 - x1) / dt
    vy = (y3 - y1) / dt
    # frame3(최신) 시점 기준 순간속도로 보정 (등가속도 운동, vx/vy와 달리 평균≠순간속도)
    vz = (z3 - z1) / dt - 0.5 * config.GRAVITY * dt

    return Observation(
        class_name=class_name,
        position=(x3, y3, z3),
        velocity=(vx, vy, vz),
        t=t3,
    )


def reobserve(cam: Camera, class_name: str) -> Observation:
    """재보정 루프용 재관측. 단일 프레임이라 속도는 계산하지 않는다 (position, t만 유효)."""
    frame, t = cam.capture_sample()
    x, y, z = _observe_xyz(frame, class_name)
    return Observation(class_name=class_name, position=(x, y, z), velocity=(0.0, 0.0, 0.0), t=t)

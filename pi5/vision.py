from __future__ import annotations

from dataclasses import dataclass

import config
import utils


@dataclass
class Detection:
    class_name: str
    bbox: tuple[float, float, float, float]  # center_x_px, center_y_px, width_px, height_px
    confidence: float


@dataclass
class Observation:
    class_name: str
    position: tuple[float, float, float]  # x, y, z (m)
    velocity: tuple[float, float, float]  # vx, vy, vz (m/s)
    t: float


class Camera:
    def __init__(self, resolution: tuple[int, int], fps: int) -> None:
        self.resolution = resolution
        self.fps = fps
        # 정해야함: Picamera2 초기화

    def capture_sample(self) -> tuple[object, float]:
        raise NotImplementedError  # 정해야함: Picamera2 연동


class _HailoYolo:
    """⚠ HailoRT 사용 흐름 초안. 정확한 API는 실제 버전 문서로 검증 필요."""

    def __init__(self, model_path: str) -> None:
        self.model_path = model_path
        # 정해야함: HEF 로드 + VDevice/네트워크 그룹 구성

    def infer(self, _frame: object) -> list[Detection]:
        raise NotImplementedError  # 정해야함: 학습된 .hef 필요


_yolo: _HailoYolo | None = None


def _get_yolo() -> _HailoYolo:
    global _yolo
    if _yolo is None:
        if config.YOLO_MODEL_PATH is None:
            raise NotImplementedError  # 정해야함: config.YOLO_MODEL_PATH
        _yolo = _HailoYolo(config.YOLO_MODEL_PATH)
    return _yolo


def _detect(frame: object) -> list[Detection]:
    return _get_yolo().infer(frame)


def _best_target_detection(frame: object) -> Detection | None:
    candidates = [
        d for d in _detect(frame)
        if d.class_name in config.TARGET_CLASSES and d.confidence >= config.YOLO_CONF_THRESHOLD
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.confidence)


def _estimate_z(class_name: str, bbox: tuple[float, float, float, float]) -> float:
    ref = config.REFERENCE_SIZE_AT_1M.get(class_name)
    if ref is None:
        raise NotImplementedError  # 정해야함: config.REFERENCE_SIZE_AT_1M
    _, _, width_px, height_px = bbox
    ratio_w = ref["width_px"] / width_px
    ratio_h = ref["height_px"] / height_px
    return (ratio_w + ratio_h) / 2.0


def _pixel_to_meters_xy(bbox_center_px: tuple[float, float], z: float) -> tuple[float, float]:
    if config.FOCAL_LENGTH_PX is None:
        raise NotImplementedError  # 정해야함: config.FOCAL_LENGTH_PX
    cx_px, cy_px = bbox_center_px
    x = cx_px * z / config.FOCAL_LENGTH_PX
    y = cy_px * z / config.FOCAL_LENGTH_PX
    return x, y


def _observe_xyz(frame: object, class_name: str) -> tuple[float, float, float]:
    detections = [d for d in _detect(frame) if d.class_name == class_name]
    if not detections:
        raise RuntimeError(f"'{class_name}' 미검출")
    det = max(detections, key=lambda d: d.confidence)
    utils.log_detection(det.class_name, det.confidence, det.bbox)

    z = _estimate_z(det.class_name, det.bbox)
    cx_px, cy_px, _, _ = det.bbox
    x, y = _pixel_to_meters_xy((cx_px, cy_px), z)
    utils.log_position(x, y, z)
    return x, y, z


def wait_for_confirmed_object(cam: Camera) -> Observation:
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
    utils.log(f"confirmed class={class_name} (streak={streak})")

    n = config.VELOCITY_SAMPLE_FRAMES
    assert n >= 2

    frame_first, t_first = cam.capture_sample()
    x_first, y_first, z_first = _observe_xyz(frame_first, class_name)

    for _ in range(n - 2):
        cam.capture_sample()

    frame_last, t_last = cam.capture_sample()
    x_last, y_last, z_last = _observe_xyz(frame_last, class_name)

    dt = t_last - t_first
    vx = (x_last - x_first) / dt
    vy = (y_last - y_first) / dt
    # z는 등가속도 운동이라 평균속도가 아니라 중력항으로 보정한 순간속도를 써야 함
    vz = (z_last - z_first) / dt - 0.5 * config.GRAVITY * dt
    utils.log(f"velocity vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} dt={dt:.4f} frames={n}")

    return Observation(
        class_name=class_name,
        position=(x_last, y_last, z_last),
        velocity=(vx, vy, vz),
        t=t_last,
    )


def reobserve(cam: Camera, class_name: str) -> Observation:
    utils.log(f"reobserve class={class_name}")
    frame, t = cam.capture_sample()
    x, y, z = _observe_xyz(frame, class_name)
    return Observation(class_name=class_name, position=(x, y, z), velocity=(0.0, 0.0, 0.0), t=t)

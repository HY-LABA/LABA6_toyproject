"""전역 설정값. pi5/ 하위 모든 모듈이 참조한다."""

from __future__ import annotations

CAMERA_RESOLUTION = (1456, 1088)
CAMERA_FPS = 60
FRAME_SKIP = 2
SAMPLE_INTERVAL_S = FRAME_SKIP / CAMERA_FPS

YOLO_MODEL_PATH: str | None = None  # 정해야함
YOLO_CONF_THRESHOLD = 0.5

TARGET_CLASSES = (
    "pet_bottle",
    "can",
    "paper_cup",
)

REFERENCE_SIZE_AT_1M: dict[str, dict[str, float] | None] = {
    "pet_bottle": None,  # 정해야함
    "can": None,  # 정해야함
    "paper_cup": None,  # 정해야함
}

CONFIRM_FRAMES = 1
VELOCITY_SAMPLE_FRAMES = 3

FOCAL_LENGTH_PX: float | None = None  # 정해야함

GRAVITY = 9.8
CATCH_HEIGHT_M = 0.0

SERIAL_PORT: str | None = None  # 정해야함
SERIAL_BAUDRATE = 115200

RECAL_DRIVE_TIME_S = 0.1

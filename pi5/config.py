"""전역 설정값. pi5/ 하위 모든 모듈이 이 파일을 참조한다.
"""
from __future__ import annotations

# ── 카메라 (Sony IMX296 글로벌 셔터) ──────────────────────
CAMERA_RESOLUTION = (1456, 1088)  #  속도 확인 후 다운스케일 여부 재검토
CAMERA_FPS = 60
FRAME_SKIP = 2  # 2프레임마다 1장 샘플링 → 실질 30fps
SAMPLE_INTERVAL_S = FRAME_SKIP / CAMERA_FPS

# ── YOLO / Hailo ─────────────────────────────────────────
# TODO: 커스텀 학습 완료 후 실제 .hef 경로로 교체
YOLO_MODEL_PATH: str | None = None
YOLO_CONF_THRESHOLD = 0.5

# 커스텀 학습 라벨명과 정확히 일치해야 함. 새 클래스는 아래 REFERENCE_SIZE_AT_1M에도 같이 추가.
TARGET_CLASSES = (
    "pet_bottle",  # 페트병
    "can",  # 캔
    "paper_cup",  # 일회용 커피컵
)

# 클래스별 1m 거리 기준 바운딩박스 크기(px) — vision.py가 이 값과 실측 bbox 크기 비율로 z(m) 역산.
# TODO: 아직 실측 전. 카메라 고정 후 각 클래스를 1m 거리에 두고 bbox 크기를 재서 채운다.
REFERENCE_SIZE_AT_1M: dict[str, dict[str, float] | None] = {
    "pet_bottle": None,
    "can": None,
    "paper_cup": None,
}

# 몇 프레임 연속 감지돼야 "확정"으로 볼지. 일단 1(즉시 확정)로 시작, 오탐 나오면 실측하면서 조정.
CONFIRM_FRAMES = 1

# 픽셀 오프셋 -> 실제 좌우(x,y) 거리(m) 변환용 카메라 초점거리(px 단위). 핀홀 모델:
#   실제거리(m) = 픽셀거리 × z(m) ÷ FOCAL_LENGTH_PX
# TODO: 카메라 캘리브레이션 전이라 아직 없음. 이 값 없으면 vision.py의 x,y 계산 불가.
FOCAL_LENGTH_PX: float | None = None

# ── 물리 상수 ─────────────────────────────────────────────
GRAVITY = 9.8  # m/s^2
CATCH_HEIGHT_M = 0.0  # 로봇이 물체를 받는 높이 기준면 (z=0)

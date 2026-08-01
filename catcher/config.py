"""전역 파라미터. 다른 파일에 상수를 하드코딩하지 않는다.

값의 근거는 ../docs/physics.md, 실측 대상은 ../docs/pi5-algorithm.md 10장 참고.
`정해야함` = 실측/캘리브레이션 전까지는 쓸 수 없는 값.
"""

from __future__ import annotations

# ── 카메라 (InnoMaker CAM-IMX296Color-GS) ──────────────────────────────
# Sony IMX296LQR 1/2.9", 픽셀 3.45 um, 글로벌 셔터, 최대 60.3 fps.
FRAME_WIDTH = 1456
FRAME_HEIGHT = 1088
CAMERA_FPS = 60                 # 캡처 주기
PROCESS_EVERY_N = 1             # 정해야함: 추론 지연 실측 후 1(60fps) 또는 2(30fps)
EXPOSURE_MAX_S = 0.82e-3        # 모션 블러 2px 이하 (센서는 30us까지 지원)

# 번들 M12 2.8mm 광각 (F2.2, 대각 화각 140°, TV 왜곡 -17%).
#
# ⚠ 이 렌즈는 핀홀이 아니다. f=2.8mm 핀홀이면 대각 96.5°가 나와야 하는데 사양은 140°다.
#   등입체각 어안 투영(r = 2f·sin(theta/2))에 f_eff=2.73mm를 넣으면 140°와 -17% 왜곡이
#   동시에 설명된다. -> f_px = 2.73mm / 3.45um = 792
#   실제 화각: 수평 109.4° / 수직 80.3° / 대각 140°, 2m에서 수평 반폭 2.83m
#
# 프레임당 sigma_z는 크지만(중심 15.3cm) 칼만 피팅 후 착지 오차는 0.5~0.7cm다
# (../docs/hardware.md 3.1장 시뮬레이션). 광각이 물체를 놓치지 않는 쪽으로 유리하다.
PROJECTION_MODEL = "equisolid"  # "equisolid" | "pinhole" — 정해야함: 캘리브레이션으로 확인
FOCAL_LENGTH_PX = 792.0         # 2.73mm / 3.45um — 정해야함: 체커보드 캘리브레이션으로 확정
PRINCIPAL_POINT_PX = (FRAME_WIDTH / 2, FRAME_HEIGHT / 2)  # 정해야함: 캘리브레이션
CAMERA_YAW_RAD = 0.0            # 정해야함: 조립 후 카메라 장착 회전 오차

# 어안 왜곡 계수 (cv2.fisheye 순서 k1, k2, k3, k4).
# frames.py의 등입체각 모델은 이상적인 형태이고, 실물의 잔차를 이 계수로 잡는다.
# ⚠ cv2.calibrateCamera(핀홀)가 아니라 cv2.fisheye.calibrate를 써야 한다.
DISTORTION_COEFFS = (0.0, 0.0, 0.0, 0.0)        # 정해야함: 어안 캘리브레이션

# 로봇 회전 중심 -> 카메라 광학 중심 (body frame, m).
# 카메라를 통 입구 림 가장자리에 두므로 중심에서 약 15cm 벗어난다.
CAMERA_OFFSET_M = (0.15, 0.0)   # 정해야함: 조립 후 자로 실측

# ── 검출 (YOLO11n -> Hailo) ───────────────────────────────────────────
YOLO_MODEL_PATH: str | None = None   # 정해야함: 학습 + Hailo 컴파일 후 .hef 경로
YOLO_CONF_THRESHOLD = 0.5
YOLO_INPUT_SIZE = 640                # ROI 크롭 크기와 반드시 일치시킨다
ROI_SIZE_PX = 640

TARGET_CLASSES = ("can", "pet_bottle", "paper_cup")

# 실물 치수(m). w = 짧은 축(세워둔 상태의 지름), h = 긴 축.
# z 추정에는 w(폭)만 쓴다 — 짧은 축이라 물체 회전에 덜 민감하기 때문.
OBJECT_SIZE_M: dict[str, dict[str, float]] = {
    "can": {"w": 0.066, "h": 0.122},         # 정해야함: 줄자 실측
    "pet_bottle": {"w": 0.065, "h": 0.210},  # 정해야함
    "paper_cup": {"w": 0.080, "h": 0.110},   # 정해야함
}

# ── 측정 노이즈 ────────────────────────────────────────────────────────
SIGMA_BBOX_PX = 2.0        # 정해야함: 정지 물체 촬영 통계 (vision-pipeline 7장)
SIGMA_BBOX_PX_SEARCH = 4.6  # SEARCH는 640으로 축소해 추론 (1456/640 = 2.3배)
SIGMA_ODOM_M = 0.01        # 정해야함: 오도메트리 드리프트 실측

# ── 물리 ──────────────────────────────────────────────────────────────
GRAVITY = 9.8
# 클래스별 유효 중력. 항력 때문에 g보다 작다 (100g 기준 g의 88~93%).
# 정해야함: 낙하 영상으로 피팅
GRAVITY_BY_CLASS: dict[str, float] = {
    "can": 9.0,
    "pet_bottle": 9.1,
    "paper_cup": 8.6,
}
# 프로세스 노이즈(백색 가속 표준편차, m/s^2). 항력 모델 오차를 흡수한다.
# 수평축에도 0이 아닌 값을 줘야 한다 — 항력은 수평 속도도 감속시킨다.
PROCESS_NOISE_BY_CLASS: dict[str, float] = {
    "can": 1.0,
    "pet_bottle": 1.0,
    "paper_cup": 2.0,
}

Z_CATCH_M = 0.0            # 카메라가 통 입구 림 평면에 있으므로 0

# ── 로봇 구동 (FIT0186 x3, NEXUS 14049 옴니휠, 11.1V) ──────────────────
V_MAX = 1.22               # m/s  무부하 속도 (전압 보정 후)
A_MAX = 2.45               # m/s^2  견인 한계 0.5*mu*g (mu=0.5) — 정해야함: mu 실측

# ── 제어 루프 ──────────────────────────────────────────────────────────
T_MIN_S = 0.08             # 착지 직전 속도 발산 방지
COMMAND_TTL_S = 0.10       # 피코 워치독. 루프 주기(16.7~33ms)의 3~4배
LATENCY_S = 0.05           # 명령이 실제 구동으로 이어지기까지의 추정 지연
CONFIDENCE_TRACE_MAX = 1.0  # 속도 공분산 trace 임계. 이하이면 명령을 낸다

LOST_TIMEOUT_S = 0.30      # 관측 소실 후 이 시간 지나면 사이클 종료
CYCLE_TIMEOUT_S = 3.0      # 사이클 최대 시간

# ── 시리얼 ────────────────────────────────────────────────────────────
SERIAL_PORT: str | None = None   # 정해야함: /dev/ttyACM0 등
SERIAL_BAUDRATE = 115200


def object_width_m(class_name: str) -> float:
    """z 추정에 쓰는 실물 폭(m)."""
    size = OBJECT_SIZE_M.get(class_name)
    if size is None:
        raise KeyError(f"OBJECT_SIZE_M에 '{class_name}' 없음")
    return size["w"]


def gravity_for(class_name: str) -> float:
    return GRAVITY_BY_CLASS.get(class_name, GRAVITY)


def process_noise_for(class_name: str) -> float:
    return PROCESS_NOISE_BY_CLASS.get(class_name, 1.0)

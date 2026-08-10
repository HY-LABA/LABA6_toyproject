"""카메라 캡처 + YOLO 검출 → **bbox 중심 픽셀 좌표**.

이 파일이 하는 일이 예전보다 훨씬 줄었다. 궤적 추정이 중력 기반으로 바뀌면서
필요한 게 bbox 중심 하나뿐이 됐기 때문이다. 삭제된 것:

  · `_estimate_z` — bbox 크기 ÷ 기준 크기로 z 역산. 물체 크기를 알아야 했고
    공중 회전에 3배까지 흔들려서 폐기
  · 속도 계산 — 첫/마지막 프레임 위치차 ÷ Δt 에 중력 보정까지 하던 블록.
    이제 속도는 궤적 최소제곱의 해에 같이 나온다 (trajectory.Fit.v0)
  · `CONFIRM_FRAMES` 연속 확정 로직 — 탄도 게이트(재투영 잔차)가 훨씬 강한
    판별자라 별도 확정이 불필요하다

bbox **크기**는 이제 아무 데도 안 쓴다. 회전하는 물체에서 폭이 흔들리는 문제가
통째로 사라졌다는 뜻이다. 다만 중심 좌표의 정확도는 그대로 중요하다 —
그게 궤적 피팅의 유일한 입력이다.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import config
import utils


@dataclass
class Detection:
    """한 프레임에서 찾은 물체. bbox는 로그·디버깅용으로만 남겨둔다."""

    u: float                # bbox 중심 x (px, 왜곡 보정 후)
    v: float                # bbox 중심 y (px, 왜곡 보정 후)
    t: float                # 캡처 시각 (s)
    confidence: float
    bbox: tuple[float, float, float, float]  # cx, cy, w, h (원본 픽셀, 보정 전)


class Camera:
    """Picamera2 래퍼.

    타임스탬프가 궤적 피팅의 입력이라 정확도가 중요하다. 파이썬이 버퍼를 받은
    시각(time.monotonic)에는 스케줄링 지터가 섞여 있고, 피팅은 그 지터를 물체의
    운동으로 읽는다. 그래서 **센서 타임스탬프**를 쓴다.
    """

    def __init__(self, resolution: tuple[int, int], fps: int) -> None:
        self.resolution = resolution
        self.fps = fps
        self._impl = None
        # 정해야함: Picamera2 초기화.
        #   prep/camera.py 에 노출·게인·auto_lock·노이즈리덕션 처리가 이미 다 들어
        #   있으므로 그걸 재사용하는 게 맞다. 단 prep/camera.py의 read()는
        #   time.monotonic()을 쓰므로, 여기서는 SensorTimestamp를 읽도록 고쳐야 한다.
        raise NotImplementedError("정해야함: prep/camera.py 기반으로 Picamera2 연결")

    def capture(self) -> tuple[object, float]:
        """(프레임, 센서 타임스탬프[s])."""
        raise NotImplementedError  # 정해야함

    def close(self) -> None:
        if self._impl is not None:
            self._impl.stop()
            self._impl = None


class _HailoYolo:
    """⚠ HailoRT 사용 흐름 초안. 정확한 API는 실제 버전 문서로 검증 필요."""

    def __init__(self, model_path: str) -> None:
        self.model_path = model_path
        # 정해야함: HEF 로드 + VDevice/네트워크 그룹 구성

    def infer(self, _frame: object) -> list[tuple[tuple[float, float, float, float], float]]:
        """[(bbox(cx,cy,w,h), confidence), ...]. 클래스가 하나라 클래스명은 안 돌려준다."""
        raise NotImplementedError  # 정해야함: 학습된 .hef 필요


_yolo: _HailoYolo | None = None


def _get_yolo() -> _HailoYolo:
    global _yolo
    if _yolo is None:
        if config.YOLO_MODEL_PATH is None:
            raise NotImplementedError("정해야함: config.YOLO_MODEL_PATH")
        _yolo = _HailoYolo(config.YOLO_MODEL_PATH)
    return _yolo


def _undistort(u: float, v: float) -> tuple[float, float]:
    """렌즈 왜곡을 보정해 이상적인 핀홀 좌표로 옮긴다.

    **이미지 전체를 펴지 않고 점 하나만 보정한다.** 40fps로 프레임을 통째로
    undistort하는 건 파이 입장에서 낭비다 — 프레임당 의미 있는 픽셀이 한 점뿐이다.

    LS40136은 M12 광각 렌즈다. 화각이 넓으면 왜곡이 가장자리에서 커지고, 궤적
    피팅은 화면 전체를 가로지르는 궤적을 쓰므로 보정 없이는 잔차가 계통적으로
    커진다. 캘리브레이션이 끝나기 전까지는 보정 없이 돌아가되(원본 좌표 그대로),
    그 상태의 residual_px는 렌즈 왜곡을 포함한 값임을 기억할 것.
    """
    if config.CAMERA_DISTORTION is None:
        return u, v

    import cv2

    K = np.array([[config.CAMERA_FX, 0.0, config.CAMERA_CX],
                  [0.0, config.CAMERA_FY, config.CAMERA_CY],
                  [0.0, 0.0, 1.0]])
    d = np.asarray(config.CAMERA_DISTORTION, dtype=float)
    pts = np.array([[[float(u), float(v)]]], dtype=np.float64)

    if config.CAMERA_MODEL == "fisheye":
        out = cv2.fisheye.undistortPoints(pts, K, d.reshape(4, 1), P=K)
    else:
        out = cv2.undistortPoints(pts, K, d, P=K)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def detect(frame: object, t: float) -> Detection | None:
    """한 프레임에서 가장 신뢰도 높은 검출 하나. 없으면 None.

    클래스가 하나라 "어느 클래스인가"를 고민할 필요가 없다. 여러 개가 잡히면
    가장 신뢰도 높은 것만 쓴다 — 한 번에 하나만 던진다는 전제이고, 혹시 오탐이
    섞여도 궤적 피팅의 잔차 검사에서 걸러진다.
    """
    hits = [
        (bbox, conf) for bbox, conf in _get_yolo().infer(frame)
        if conf >= config.YOLO_CONF_THRESHOLD
    ]
    if not hits:
        return None

    bbox, conf = max(hits, key=lambda h: h[1])
    cx_px, cy_px, w_px, h_px = bbox
    u, v = _undistort(cx_px, cy_px)
    utils.log_detection(conf, bbox, (u, v))
    return Detection(u=u, v=v, t=t, confidence=conf, bbox=bbox)


def observe(cam: Camera) -> Detection | None:
    """한 프레임 캡처해서 검출 결과를 돌려준다."""
    frame, t = cam.capture()
    return detect(frame, t)

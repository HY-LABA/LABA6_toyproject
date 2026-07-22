"""[🖥️ Pi · 3단계] 픽셀 좌표 -> 실제 월드 좌표(m).

역할: 탐지된 물체의 화면 위치(bbox 픽셀)를 실제 3D 좌표 [x,y,z] 로 바꾼다.
     이 출력이 칼만/궤적예측의 입력이 된다.

★ 이 단계가 프로젝트의 핵심 미결정 사항 ★
  카메라 한 대(2D)로는 깊이(거리)를 알 수 없다(단안 스케일 모호성).
  깊이를 어떻게 얻을지 먼저 정해야 이 파일을 구현할 수 있다:
    - 스테레오(카메라 2대) 삼각측량
    - 깊이 센서(ToF 등)
    - 물체 실제 크기 가정 (통제 가능한 경우만)
  → 방식 정하기 전엔 구현하지 않는다. (임의 가정으로 때우지 말 것)
"""

from __future__ import annotations


def pixel_to_world(detection, timestamp: float):
    """탐지 결과 -> 월드 좌표.

    Args:
        detection: (cls_name, confidence, bbox=(x1,y1,x2,y2))
        timestamp: 관측 시각(초)
    Returns:
        (position: numpy.ndarray shape (3,) [x,y,z], timestamp: float)
    """
    # TODO: 깊이 획득 방식 결정 후 구현
    raise NotImplementedError("깊이(거리) 획득 방식 미정 — 여기부터 같이 설계")

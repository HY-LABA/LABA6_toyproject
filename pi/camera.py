"""[🖥️ Pi · 1단계] 카메라 캡처 — 프레임 받아오기.

역할: 카메라에서 한 프레임씩 이미지를 읽어 (frame, timestamp) 로 넘긴다.
     frame 은 numpy 이미지 배열(H×W×3), timestamp 는 캡처 시각(초).

아직 미정/미구현:
  - 실제 카메라 연결 방식 (Picamera2 등) → 하드웨어 붙일 때 채운다.
  - 지금은 자리만. 개발용으로 이미지/영상 파일에서 읽는 버전을 먼저 붙일 수도 있음.
"""

from __future__ import annotations


class Camera:
    def __init__(self) -> None:
        # TODO: 카메라 초기화 (해상도, fps, 노출 등)
        raise NotImplementedError("카메라 연결 방식 미정 — 같이 정한 뒤 구현")

    def read(self):
        """다음 (frame, timestamp) 반환. 더 없으면 None.

        Returns:
            (numpy.ndarray, float) | None
        """
        # TODO: 한 프레임 캡처
        raise NotImplementedError

    def close(self) -> None:
        """카메라 리소스 정리."""
        # TODO
        pass

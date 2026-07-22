"""[🖥️ Pi · 2단계] YOLO 객체 탐지 + 쓰레기 판별.

역할: 한 프레임을 받아 관심 물체(쓰레기)만 골라 탐지 결과 리스트로 넘긴다.
     탐지 하나 = (cls_name, confidence, bbox=(x1,y1,x2,y2)).

아직 미정/미구현:
  - 어떤 클래스를 '쓰레기'로 볼지 (COCO 기본 vs 학습) → 미정.
  - 모델 로딩(ultralytics 등)과 실제 추론 → 정한 뒤 구현.
"""

from __future__ import annotations


class Detector:
    def __init__(self, interest_classes=None, conf: float = 0.4) -> None:
        # interest_classes: 쓰레기로 취급할 클래스 이름들 (미정 → 나중에 채움)
        self.interest_classes = set(interest_classes or [])
        self.conf = conf
        # TODO: YOLO 모델 로드
        raise NotImplementedError("쓰레기 클래스/모델 미정 — 같이 정한 뒤 구현")

    def detect(self, frame):
        """프레임에서 쓰레기 후보 탐지.

        Args:
            frame: numpy 이미지 배열
        Returns:
            list[ (cls_name:str, confidence:float, bbox:(x1,y1,x2,y2)) ]
            (없으면 빈 리스트)
        """
        # TODO: 추론 + interest_classes 필터
        raise NotImplementedError

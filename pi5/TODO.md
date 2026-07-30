# pi5 남은 작업

## 1. 값만 채우면 되는 것 (config.py)

- [ ] `YOLO_MODEL_PATH` — 커스텀 학습 완료 후 `.hef` 경로
- [ ] `REFERENCE_SIZE_AT_1M` — pet_bottle / can / paper_cup 각각 1m 거리 기준 bbox 크기(px) 실측
- [ ] `FOCAL_LENGTH_PX` — 카메라 캘리브레이션
- [ ] `SERIAL_PORT` — 피코 연결 후 장치 경로 확인 (`/dev/ttyACM0` 등)

## 2. 값을 채워도 별도 연동 코드가 필요한 것

- [ ] `vision.Camera` — Picamera2 실제 캡처 코드
- [ ] `vision._HailoYolo` — HailoRT 실제 추론 코드 (⚠ API는 실제 버전 문서로 검증 필요)
- [ ] `communication.SerialLink.__init__` — pyserial로 실제 포트 여는 코드

## 3. 설계가 안 된 로직

- [ ] `trajectory.recalibrate()` — 엔코더 이동량 + 카메라 재관측 오차를 어떻게 합쳐 궤적을 재계산할지

## 참고

- 1·2·3이 다 채워지면 `main.py`의 전체 흐름은 로직상 끝까지 연결됨 (2026-07-30 확인).
- (추후) 재보정 루프 최대 반복횟수/타임아웃 안전장치 — `control.py` 담당 (algorithm.md 3번, 설계 범위 밖으로 명시돼 있음)

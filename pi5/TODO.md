# pi5 남은 작업

> 2026-08-10 갱신. 깊이 추정 방식 전환으로 없어진 항목이 많다 (`algorithm.md` 참고).

## 1. 실측해서 값만 채우면 되는 것 (config.py)

- [ ] **`CAMERA_FX/FY/CX/CY`** ★ 최우선 — 체스보드 캘리브레이션.
      지금은 스펙에서 계산한 값(f_px=1739)이지 실측이 아니다. CS 마운트는 백포커스를
      나사로 돌려 맞추는 구조라 초점을 맞추는 과정에서 실효 초점거리가 달라진다.
      **f가 10% 틀리면 깊이도 10% 틀어진다.** 대각 55°라 pinhole 모델로 하면 된다
      (`cv2.calibrateCamera`, 어안 아님)
- [ ] **`CAMERA_DISTORTION`** — 위 캘리브레이션에서 같이 나온다 (k1,k2,p1,p2,k3).
      None이면 보정을 건너뛰는데, 궤적이 화면을 가로지르므로 가장자리 왜곡이 잔차에
      계통적으로 섞인다
- [ ] `YOLO_MODEL_PATH` — 학습·Hailo 변환 완료 후 `.hef` 경로
- [ ] `ROBOT_MAX_SPEED_MPS` — 모터 도착 후 PWM 100% 직진으로 실측.
      `pico/config.h`의 `MAX_BODY_SPEED_MPS`도 같은 값으로 맞출 것.
      **같이 잴 것: 가속 시간, 정지 거리(오버슈트 크기)**
- [ ] `SERIAL_PORT` — 피코 연결 후 장치 경로 (`/dev/ttyACM0` 등)

## 2. 값을 채워도 별도 연동 코드가 필요한 것

- [ ] **`vision.Camera`** — Picamera2 실제 캡처.
      `prep/camera.py`에 노출·게인·auto_lock·노이즈리덕션 처리가 이미 다 들어 있으므로
      재사용하는 게 맞다. 단 `prep/camera.py`의 `read()`는 `time.monotonic()`을 쓰는데,
      여기서는 **`SensorTimestamp`로 바꿔야 한다** — 파이썬이 버퍼를 받은 시각에는
      스케줄링 지터가 섞여 있고 궤적 피팅이 그걸 물체의 운동으로 읽는다
- [ ] `vision._HailoYolo` — HailoRT 실제 추론 (⚠ API는 실제 버전 문서로 검증 필요)
- [ ] `communication.SerialLink.__init__` — pyserial로 포트 여는 코드

## 3. 만들어야 하는 것

- [ ] **`prep/calibrate.py`** — 체스보드 캘리브레이션 스크립트.
      모델은 pinhole로 확정됐으므로 `cv2.calibrateCamera` 한 번이면 된다.
      결과를 `config.py`의 CAMERA_* 에 넣는다

## 4. 검증해야 하는 것

- [ ] **실제 투척에서 검출 노이즈 측정** ★ — 궤적 피팅의 재투영 잔차(`residual_px`)를
      본다. **1px 이하가 목표.** 2px면 예측이 착지 직전에야 나와서 로봇이 못 움직인다
      (`algorithm.md` 4장 표). 크면 노출을 줄이고 게인을 올릴 것
- [ ] 예측 착지점 vs 실제 착지점 거리 — 이게 최종 성능이다
- [ ] Hailo 변환 경로 — 1에폭만 학습해 ONNX 내보내고 컴파일이 되는지 먼저 확인
      (구조만 보므로 가중치가 쓰레기여도 판정은 같다)

## 참고

- 1·2가 채워지면 `main.py`의 전체 흐름은 로직상 끝까지 연결된다.
- 궤적 추정 자체는 합성 투척으로 검증 완료 (착지 오차 중앙값 0.1~0.4cm, `algorithm.md` 4장).
- (추후) 재보정 루프 최대 반복횟수/타임아웃 안전장치 — `control.py` 담당
- (추후) 사람 검출 클래스 추가 (안전용). 병 데이터는 그대로 재사용된다

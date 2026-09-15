# pi5 남은 작업

> 2026-09-15 갱신. **지금 남은 것만** 적는다. 왜 그렇게 됐는지는 [`../CHANGELOG.md`](../CHANGELOG.md),
> 리스크 분석은 [`../docs/open-questions.md`](../docs/open-questions.md), 동작 설명은
> [`../algorithm.md`](../algorithm.md).

## 1. 실측으로 확정된 값 (`config.py`)

- [x] 카메라 내부파라미터·어안 왜곡 — 체스보드 23장, RMS 0.411 px, 격자판 교차검증
- [x] YOLO `.hef` (단일 클래스 `trash`, imgsz 640, conf 0.5) — `best_hailo_model/best.hef`, git 밖
- [x] `CAMERA_OFFSET_M = (0.0, 0.16)` — 몸체 완성 후 자로 (2026-09-07)
- [x] `CATCH_HEIGHT_M = −0.03` — 카메라가 통 입구보다 3 cm 위 (2026-09-07)
- [x] `CATCH_OPENING_DIAMETER_M = 0.26` — 통 입구 실측 (2026-09-07)
- [x] `WHEEL_ANGLES_RAD` / `WHEEL_MOUNT_RADIUS_M` — 몸체 실물 대조 (2026-09-07)
- [~] `CAMERA_YAW_RAD = 0.0` — **실기에서 맞게 가는 값**이다. 사진 측정(`measure_camera_yaw.py`,
      2026-09-09)은 180°였다. 구동 쪽 반전과 상쇄 중으로 의심 → 아래 2번 첫 항목

## 2. 아직 안 된 실측 ★

순서대로 한다. 앞의 것이 뒤의 것 판정에 들어간다.

- [ ] **방향 기준 확정** — 카메라 빼고 구동만:
      `python pico_test.py --port /dev/ttyACM0 --dir front --dist 0.5 --speed 0.3`.
      안 도는 바퀴(M1) 반대쪽으로 가면 구동 반전 → 피코 `MOTOR_SIGN` 을 고치고
      **`CAMERA_YAW_RAD` 를 180°로 같이** 바꾼다. 하나만 바꾸면 다시 반대로 간다
- [ ] **최대속도** → `WHEEL_MAX_SPEED_MPS`·`ROBOT_MAX_SPEED_MPS`. 지금 파이·피코 모두 1.8 이지만
      시험용으로 올린 클램프다(이론값 1.22). `teleop_test.py --speed-test` 로 재고 세 파일 통일
- [ ] **가속시간·정지거리** → 도달 가능 반경 → 투척 영역 규약.
      `pico_test.py --dir front --dist 1.0 --max` 가 정지거리를 재고 통 입구 반지름(13 cm)
      초과를 경고한다. 넘으면 `DRIVE_AGGRESSION` 을 낮춘다
- [ ] **오도메트리 스케일·슬립** — `teleop_test.py --go-distance 1.0` 출력과 줄자 대조
- [ ] **실기 잔차 분포** → `MAX_RESIDUAL_PX` 결정. 로봇이 움직이면 정지 천장점이 궤적으로
      채택되는데(시뮬 0.5 m/s에서 47.5%), 잔차로 갈린다. `--max-residual-px 3.0` 으로 시험
- [ ] **카메라 기울기 3.9°** — 재장착 때 낮춰 달거나 `GRAVITY_CAM` 을 회전
- [ ] **합성 투척 성능 표 재실행** — [`algorithm.md` 6장](../algorithm.md#6-검증된-성능) 표가
      실측 렌즈(f_px 973, 어안)와 다른 조건(f_px 1739)에서 나왔다

## 3. 코드로 해야 할 것

실기에서 드러났거나 리뷰로 찾았는데 아직 안 고친 것. 위에서부터 영향이 크다.

- [ ] **루프 주기(fps) 로그** — N 프레임마다 한 줄. 0.15 s 워치독을 넘는지 볼 방법이 지금 없다
- [ ] **오도메트리 시각 맞추기** — `main.py` 가 캡처·추론 **뒤에** 읽어서 pose가 추론 시간만큼
      늦다 (1.2 m/s·30 ms면 3.6 cm 계통 오차). 캡처 전후로 읽어 보간
- [ ] **검출 로그 I/O** — `vision.detect_all()` 이 검출 있는 매 프레임 `run.log` 에 동기로 쓴다.
      천장 오탐이 잡히는 대기 상태에서 fps를 깎을 수 있다
- [ ] **프레임 단위 예외 처리** — 지금은 `KeyboardInterrupt` 만 잡아서 Hailo·카메라 일시 오류
      한 번에 루프가 끝난다(`finally` 의 STOP은 나간다). 연속 N회 실패일 때만 종료
- [ ] **"착지 시각 경과" 프레임의 목표점 전송 막기** — 사이클이 끝나는 바로 그 프레임에도 남은시간
      ≤ 0 인 목표점을 먼저 보낸다(피코는 상한 속도로 붙는다). 같은 틱에 STOP 이 따라가서
      피코가 마지막 프레임을 채택하는 덕에 **지금은 우연히 안전**하다
- [ ] **캡처 시각** — `time.monotonic()` 이라 스케줄링 지터가 섞인다. `SensorTimestamp` 로 바꾸기
      전에 두 시계 기준(CLOCK_MONOTONIC vs BOOTTIME)이 맞는지 실기 확인
- [ ] **펌웨어 상수 대조** — `verify_wheel_config()` 는 `pico/config.h` 소스만 본다. 굽지 않은
      변경이나 MicroPython 판을 못 잡는다
- [ ] (선택) **ROI 크롭 추론** — 지금은 매 프레임 640 레터박스(2 m에서 물체 11 px).
      `tracker.predict_uv()` 주변만 원본 해상도로 자르면 축소가 없어진다
- [ ] (정리) `run.log` 가 루트에서 git 추적 중이다. `.gitignore` 에는 `pi5/run.log` 만 있고,
      `utils.py` 는 실행 위치 기준 `run.log` 에 쓴다

## 4. 검증 도구

| 도구 | 카메라 | 피코 | 무엇을 보나 |
|---|---|---|---|
| `pico_test.py` | — | ✅ | 8방향 목표점 왕복, 누적 드리프트, `--max` 정지거리 |
| `teleop_test.py` | — | ✅ | 게임패드 수동 / `--go-distance` 거리·가속·정지거리 / `--speed-test` 최대속도 |
| `measure_camera_yaw.py` | ✅ | — | `CAMERA_YAW_RAD` + 기울기 (4방향 촬영) |
| `test_accuracy.py` | ✅ | — | 로봇 정지 상태로 궤적 예측 정확도, 실측 착지점 입력, `--replay` |
| `debug_pt_on_video.py` | 영상 | — | `.pt` 가중치를 영상에 직접 — 학습 문제인지 `.hef` 변환 문제인지 가르기 |
| `control.py` (직접 실행) | — | — | pi5 ↔ `pico/config.h` 바퀴 설정 대조, 방향별 최대 body 속도 표 |

`main.py` 는 피코가 안 꽂혀 있으면 시리얼을 못 열어 바로 종료된다. 검출만 볼 때는
`test_accuracy.py` 를 쓴다.

## 5. 데이터셋 (`prep/`)

수집 경로가 넷이다. **라벨을 만드는 쪽의 편향이 학습시킬 모델과 달라야** 새 정보가 들어온다.

| 경로 | 카메라 | 라벨 판정 | 한계 |
|---|---|---|---|
| `capture_video.py` → `extract_from_video.py` | 정지 | MOG2 (움직임) | 카메라가 움직이면 못 쓴다. 던지는 손·그림자도 잡혀 검수 필요. **투척당 프레임 상한이 없다** |
| `capture_roam.py` → `label_throws.py` | 주행 | 사람이 수동 | 느리다. YOLO·물리 게이트 무관이라 어려운 투척도 남는다 |
| `collect_throws.py` | 주행/정지 | 현재 YOLO + 물리 fit | **지금 모델이 검출한 것만** 라벨이 된다(자기참조). 주행 중엔 `CAMERA_YAW_RAD` 가 맞아야 한다 |
| `collect_live.py` / `extract_background.py` | 주행/정지 | 빈 라벨 (하드 네거티브) | 전체의 0~10% 만. 더 넣으면 물체를 배우는 신호가 묽어진다 |

- 장소·자리마다 `--tag`(세션)를 다르게 준다 — `prepare_dataset.py` 가 **세션 단위**로 나눈다
- 다양성은 "찍는 동안 움직였나"보다 **"어디서 찍었나"** 에서 나온다 — 정지 카메라로 자리를
  옮겨가며 찍어도 된다
- 학습은 `train_yolo.py` — `yolov8n.pt`(COCO)에서 전이학습, 회전 증강은 **일부러 끔**(bbox 중심을
  흔든다), 상하좌우 반전만

### 성능 실험을 한다면 (연구실 미니실험 — "추가 데이터가 catch를 개선하는가")

- [ ] 오프라인 테스트 세션을 **학습 전에** 고정하고 끝까지 안 쓴다 (`prepare_dataset.py --test-sessions`)
- [ ] 추가 데이터 수준별 누적 구조(D50 ⊂ D150 ⊂ D300)를 **투척 단위로** 샘플링
- [ ] `extract_from_video.py` 에 투척당 프레임 상한 (같은 투척의 연속 프레임으로 수를 채우면 안 된다)
- [ ] 학습 seed·ultralytics 버전 기록, **`.hef` 양자화 캘리브레이션 세트 고정**
- [ ] 오프라인 성능은 `.pt` 만이 아니라 **실제로 쓰는 `.hef`** 로도 잰다
- [ ] 시행별 기록(검출·트래킹 유실·반응시간·필요속도는 `run.log` 에서 자동, 성공·실패 원인만 수동)
- [ ] 모델 교체는 `TRASH_HEF=경로 python main.py` 로 — 수준을 번갈아 돌려 배터리 방전 효과를 섞지 않는다

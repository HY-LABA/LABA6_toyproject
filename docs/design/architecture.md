# 소프트웨어 아키텍처

> 2026-09-15 개정. 몸체가 완성돼 **`main.py` ↔ 피코 왕복으로 실물 로봇이 주행한다** (2026-09-09~).
> 물체를 보고 방향을 잡아 끝까지 달리는 데까지는 됐고, 남은 건 방향 기준 확정과 모터 한계
> 실측이다(5장). 계약은 [`docs/design/protocol.md`](protocol.md), 알고리즘·제어 루프는
> [`algorithm.md`](algorithm.md), 경위는 [`CHANGELOG.md`](../../CHANGELOG.md).

## 1. 파일 구조

```
LABA6_toyproject/
├── README.md / README.ko.md       # 프로젝트 소개(영문 대문 / 한국어판)
├── CHANGELOG.md                   # 설계 변경 이력 (여기 말고 다른 문서엔 이력을 안 적는다)
├── LICENSE / CITATION.cff         # MIT(코드) + CC BY 4.0(문서) / 인용 정보
│
├── docs/                          # 문서 전부
│   ├── guide/                     # 처음 온 사람용 (영문) — bom, getting-started, results
│   ├── design/                    # 설계 근거 (한국어) — 이 문서, algorithm, physics, hardware,
│   │                              #   pico-control, protocol, vision-pipeline, open-questions
│   ├── images/                    # 배선도, 데모 GIF
│   └── archive/                   # 폐기 문서 (system_flow.md — 2026-08 구조)
│
└── src/                           # 코드 전부
    ├── pi5/                          # 라즈베리파이5 (Python) — 실시간 캐치
    │   ├── main.py                   # 프레임 루프: 캡처 → 검출후보 → 가설갱신·채택 → 목표점 전송 (+조기출발·coast)
    │   ├── config.py                 # 카메라 내부파라미터, 궤적/제어 파라미터 전체
    │   ├── vision.py                 # 카메라 캡처 + Hailo YOLO → 프레임의 **검출 후보 전체** + 왜곡보정
    │   ├── fisheye.py                # 어안 → 핀홀 등가 좌표 변환
    │   ├── trajectory.py             # 순수 계산만: 중력 기반 궤적 최소제곱 + 착지 예측 (상태 없음)
    │   ├── tracker.py                # ★ 상태를 갖는 부분 전부 — 가설(Tracker) 추적, 채택(TrackerPool),
    │   │                             #   카메라축↔로봇축 회전(cam_to_robot), 조기출발 방향(early_bearing)
    │   ├── control.py                # 착지점 → 월드 좌표 목표점(TargetCommand) + 방향별 속도 상한 + 설정 대조
    │   ├── communication.py          # USB 시리얼 프레임 — 목표점(16B)/속도(12B)/오도메트리(24B)
    │   ├── utils.py                  # 로깅 + 타이밍
    │   ├── pico_test.py              # 카메라 없이 피코 구동만 — 8방향 목표점 왕복, --max 정지거리
    │   ├── teleop_test.py            # 게임패드 수동 / --go-distance 거리 실측 / --speed-test 최대속도
    │   ├── measure_camera_yaw.py     # CAMERA_YAW_RAD + 카메라 기울기 실측 (4방향 촬영)
    │   ├── test_accuracy.py          # 로봇 정지 상태로 궤적 예측 정확도만 측정 (로그 저장 + 리플레이)
    │   ├── debug_pt_on_video.py      # .pt 가중치를 영상에 직접 — 학습 문제 vs .hef 변환 문제 가르기
    │   ├── TODO.md                   # 남은 실측·코드 항목
    │   └── prep/                     # 학습 데이터 준비 (로봇 구동과 무관, 오프라인)
    │       ├── camera.py             # 카메라 추상화 — 카메라 교체 시 여기만 손댄다
    │       ├── check_setup.py        # 실행 전 환경 점검
    │       ├── tune_camera.py        # 노출/게인 실시간 튜닝 (키보드)
    │       ├── calibrate.py          # 체스보드 캘리브레이션(capture+solve) — config.py 실측값의 출처
    │       ├── capture_video.py      # 투척 녹화만 (정지 카메라, 하드웨어 인코더)
    │       ├── extract_from_video.py # 녹화 영상 → MOG2 로 물체 프레임+라벨 (PC)
    │       ├── capture_roam.py       # 로봇이 배회하며 녹화만
    │       ├── label_throws.py       # 녹화 영상을 사람이 훑으며 수동 라벨 (PC)
    │       ├── collect_throws.py     # 배회하며 YOLO+물리 fit 통과 투척을 자동 라벨
    │       ├── collect_live.py       # 배회하며 배경(빈 라벨, 하드 네거티브) 자동 수집
    │       ├── extract_background.py # 천장만 찍은 영상 → 배경 샘플 (PC)
    │       ├── review_labels.py      # 라벨 수동 검수 (PC)
    │       ├── prepare_dataset.py    # 세션 단위 train/val/test 분할 + dataset.yaml
    │       ├── train_yolo.py         # YOLOv8n 전이학습 (PC/Colab)
    │       └── TROUBLESHOOTING.md    # 실기 작업 기록 (2026-08, 옛 파일명 포함)
    │
    ├── pico/                         # 라즈베리파이 피코 (C, Pico SDK) — 기본 펌웨어, UF2
    │   ├── main.c                    # 20ms(50Hz) 실시간 제어 루프 — 목표점/속도 두 명령 경로
    │   ├── config.h                  # 핀 배치, 물리 상수, MOTOR_SIGN, PID 게인, 최대속도
    │   ├── encoder/
    │   │   ├── encoder_pio.c/.h      # PIO 기반 A/B상 엔코더 카운팅
    │   │   └── encoder.pio           # PIO 프로그램
    │   ├── kinematics.c/.h           # 3륜 옴니 순/역기구학 + 방향별 바퀴기준 속도 상한
    │   ├── odometry_kalman.c/.h      # pose 는 raw 적분, 칼만은 보고용 속도 스무딩만
    │   ├── motor_control.c/.h        # PID(안티와인드업+데드밴드 보상) + BTS7960 PWM 출력
    │   ├── communication.c/.h        # 파이5와 USB CDC 통신 — 수신은 최신 프레임만, 송신은 논블로킹
    │   ├── test_controller           # (PC) 게임패드 → UDP 수동 조종 스크립트
    │   ├── CMakeLists.txt / pico_sdk_import.cmake / .vscode/
    │   └── TODO.md                   # 남은 실측 항목
    │
    └── pico_micropython/             # 피코 펌웨어 MicroPython 판 — src/pico/ 와 같은 프로토콜·제어식
        ├── main.py, config.py, communication.py, kinematics.py,
        │   motor_control.py, odometry_kalman.py, encoder_pio.py   # C 판 파일과 1:1 대응
        ├── goto_xy_test.py           # 좌표 입력 → 폐루프 이동 벤치. src/pico/config.h 확정값의 원래 출처
        ├── bench_loop.py             # 벤치 루프
        └── README.md                 # 올리는 법, C 판과의 차이
```

---

## 2. 파일별 책임

### src/pi5/ (Python) — 실시간 캐치

| 파일 | 담당 기능 |
|---|---|
| `main.py` | 프레임 단위 루프 하나로 전체를 돌린다: 캡처→**검출 후보 전체**→`TrackerPool` 갱신·채택→**월드 좌표 목표점 전송**. 채택 fit 이 없으면 ⓑ 마지막 목표를 계속 보내는 **coast**, 그것도 없으면 ⓒ 화면 이동방향으로 먼저 가속하는 **조기 출발**. 사이클 끝(착지/유실/타임아웃) 로그를 남기고 리셋한다. 어떤 경로로 빠져나가도 `finally`에서 STOP. `--once`/`--hold`/`--no-gates`/게이트 값 덮어쓰기 옵션 |
| `config.py` | 카메라 내부파라미터·왜곡, 카메라 장착(`CAMERA_YAW_RAD`, `CAMERA_OFFSET_M`, `CATCH_HEIGHT_M`), YOLO 경로·임계값, 물리 게이트, 제어(`DRIVE_AGGRESSION`, `EARLY_START_*`, `MIN_TARGET_DIST_M`, `COAST_EXTRA_S`), 바퀴 배치·최대속도·워치독. **실측으로 정한 값에는 근거가 주석으로 붙어 있다** |
| `vision.py` | ① Picamera2 캡처 ② Hailo NPU로 YOLO 추론(네트워크 활성화는 시작 시 한 번) ③ **신뢰도 임계값을 넘는 후보 전부**의 bbox 중심 ④ 후보마다 그 점만 왜곡 보정(`fisheye.py`). 모델은 `TRASH_HEF` 환경변수로 바꿀 수 있다 |
| `trajectory.py` | **순수 계산만, 상태 없음.** `fit_trajectory`(2N×6 선형 최소제곱, 카메라 이동 보정 포함), `landing_time`/`predict_landing`, `project`(연관용 화면 좌표 역산) |
| `tracker.py` | ★ 상태를 갖는 부분 전부. `Tracker`(가설 하나 — 관측·카메라 위치 누적, 재피팅, 깊이 수렴 판정, `FIT_HOLD_S` 동안 직전 해 유지, `landing(now)`), `TrackerPool`(물리 통과 가설만 채택, 도달 가능성 우선 선택, 사이클 관리 — 채택 전에도 타임아웃). 카메라축↔로봇축 회전은 `cam_to_robot`/`robot_to_cam` 두 곳에서만 일어난다 |
| `control.py` | 착지점(트랙 원점 기준) + 트랙 시작 시점 오도메트리 = **월드 좌표 목표점**. 남은시간을 `DRIVE_AGGRESSION` 으로 나눠 보내고, `MIN_TARGET_DIST_M` 하한을 적용한다. 남은 거리·속도 계산은 피코가 한다. `max_body_speed` 는 피코와 같은 식. `python control.py` 로 `src/pico/config.h` 와 설정 대조 |
| `communication.py` | `[START 0xAA][LEN][PAYLOAD][CHECKSUM]` 프레임. 수신은 **논블로킹**이고 버퍼에 쌓인 것 중 최신만 쓴다. `python communication.py < frame.bin` 으로 디코드 |
| `utils.py` | `run.log` append + 콘솔 출력. `log_cycle` 이 한 사이클의 판단 근거(관측수, 잔차, 깊이, 착지점, 남은시간, 오도메트리, 명령)를 한 줄로 남긴다 |
| `pico_test.py` | 카메라·YOLO 없이 **구동 경로만** 실기 검증 — `main.py` 와 같은 16B 목표점으로 8방향 왕복, 누적 드리프트 보고, `--max` 로 정지거리. `MOTOR_SIGN` 은 로그로 판정할 수 없어 눈으로 본 방향과 대조하게 한다 |
| `teleop_test.py` | 게임패드 왼쪽 스틱으로 body 속도(12B) 직접 전송. `--go-distance` 는 지정 거리 직진 후 가속·정지거리 보고, `--speed-test` 는 고정시간 속도 명령으로 최고속도 측정 |
| `measure_camera_yaw.py` | 하늘을 보는 카메라에선 높이 있는 물체가 방위각 그대로 찍힌다는 성질로 `CAMERA_YAW_RAD` 를 잰다. 4방향이면 기울기까지 같이 푼다. `live`/`capture`/`solve`. ⚠ 카메라 방향만 보고 모터 방향은 못 본다 |
| `test_accuracy.py` | 로봇을 세워둔 채 `main.py` 와 같은 함수로 **예측 정확도만** 측정. 관측을 저장해 `--replay` 로 파라미터를 바꿔 재실행. 실측 착지점 입력으로 부호 뒤집힘 진단 |
| `debug_pt_on_video.py` | `.pt` 를 녹화 영상에 직접 돌려 bbox 를 그린다 |

### src/pi5/prep/ (Python) — 학습 데이터 준비 (오프라인)

| 파일 | 담당 기능 |
|---|---|
| `camera.py` | **카메라를 바꿀 때 손대는 유일한 파일.** 프로파일(해상도/fps/노출/게인)을 `SPECS`에 두고 Picamera2↔OpenCV를 같은 인터페이스로 감싼다. 자동 노출 측정 후 고정, 노이즈 리덕션 차단 |
| `check_setup.py` / `tune_camera.py` | 패키지·카메라·디스크 점검 / 라이브 프리뷰 보며 노출·게인 조절(키보드 — 이 라파이의 OpenCV Qt 빌드는 슬라이더가 죽는다) |
| `calibrate.py` | 체스보드 촬영 + 풀이 → 내부파라미터·어안 왜곡계수 |
| `capture_video.py` → `extract_from_video.py` | 정지 카메라로 녹화만(프레임 손실 없음) → PC 에서 MOG2 로 물체 트랙을 찾아 프레임+라벨 저장. YOLO 와 무관한 라벨 경로 |
| `capture_roam.py` → `label_throws.py` | 로봇이 배회하며 녹화만 → 사람이 영상을 훑으며 라벨. YOLO·물리 게이트와 무관 |
| `collect_throws.py` | 배회 + 현재 YOLO + 물리 fit 이 통과한 투척만 자동 라벨. 빠르지만 **지금 모델이 검출한 것만** 남는다 |
| `collect_live.py` / `extract_background.py` | 빈 라벨(배경) 수집 — 주행 중 / 천장 영상에서. 오검출 억제용, 전체의 0~10% |
| `review_labels.py` | 라벨 검수·수정·기각 (PC) |
| `prepare_dataset.py` | **세션 단위**로 train/val/test 분할 + `dataset.yaml`. `--test-sessions` 로 테스트 세션 고정 |
| `train_yolo.py` | YOLOv8n COCO 사전학습에서 전이학습. 회전 증강은 끄고(bbox 중심을 흔든다) 상하좌우 반전만. Colab `--project`·`--resume` |
| `TROUBLESHOOTING.md` | 2026-08 실기 작업 기록. 형광등 맥동, X11 프레임 드롭, 노이즈 리덕션 잔상 |

### src/pico/ (C, Pico SDK) — 실시간 모터 제어

`src/pico_micropython/` 이 같은 구조·같은 식의 MicroPython 판이다. 설계 설명은 [`docs/design/pico-control.md`](pico-control.md).

| 파일 | 담당 기능 |
|---|---|
| `main.c` | 20ms(50Hz) 고정 주기 루프: 엔코더 → `MOTOR_SIGN` → 바퀴속도 → 순기구학 → pose raw 적분 → 파이 명령 확인(최근 받은 쪽이 활성, 모드 바뀌면 PID 리셋) → **목표점이면 R(−θ) 회전 후 남은거리÷남은시간**, 속도면 그대로 → 방향별 속도 상한 → 역기구학 → PID → PWM → 오도메트리 회신 |
| `config.h` | 핀, `MOTOR_SIGN`, 휠 지름·장착각·반경, 엔코더 CPR 687.5(바퀴축), 제어주기, PID 게인, `WHEEL_MAX_SPEED_MPS`, `POSITION_TOLERANCE_M` |
| `encoder/` | PIO로 A상 상승엣지만 세고 B상 레벨로 방향 판단(1x). CPU 개입 없이 누적 |
| `kinematics.c/.h` | 역/순기구학, `max_body_speed`(pi5 `control.py` 와 같은 식) |
| `odometry_kalman.c/.h` | pose 는 raw 속도로 적분. 축별 1D 칼만은 파이로 보고하는 속도 스무딩에만(시정수 약 1.4 s라 pose 에 쓰면 이동량을 과소보고한다) |
| `motor_control.c/.h` | 바퀴별 PID(안티와인드업 + 데드밴드 보상) → BTS7960 RPWM/LPWM PWM 10 kHz |
| `communication.c/.h` | 논블로킹 수신 상태머신 — 버퍼를 끝까지 비우고 마지막 완전 프레임만 채택. 송신은 `tud_cdc_write_available()` 여유 없으면 스킵 |

---

## 3. 파이5 ↔ 피코 역할 분담

| | 파이5 | 피코 |
|---|---|---|
| 궤적 추정 · 착지 예측 | ✅ | |
| 카메라축 → 로봇축 회전, 카메라 오프셋 | ✅ | |
| 조기 출발 방향, 목표 거리 하한, coast | ✅ | |
| 남은 거리 계산 (오도메트리 차감) | | ✅ |
| 목표 속도 산출 (남은거리÷남은시간) | (남은시간을 `DRIVE_AGGRESSION` 으로 나눠 보냄) | ✅ |
| 방향별 바퀴기준 속도 상한 | ✅ (도달판정·로그용) | ✅ (실제 적용) |
| 오도메트리 적분 · 역기구학 · PID · PWM | | ✅ |

**파이5가 착지점(월드 좌표)과 남은시간만 넘기고, 얼마나 빨리 가야 하는지는 피코가 자기
오도메트리로 계산한다.** 피코는 궤적도 착지점도 모른다 — 받는 건 좌표 하나와 시간 하나뿐이다.

---

## 4. 파이프라인 흐름과 구현 상태

### 4-0. 부팅 시점

- 피코는 켜지자마자 20 ms 루프 시작. 명령이 없으면 정지 유지하며 오도메트리만 갱신 ✅
- 파이5 `main.py` 는 시작 시 `verify_wheel_config()` 대조 + 미실측 값 경고, 카메라·Hailo·시리얼
  초기화. 피코가 안 꽂혀 있으면 시리얼을 못 열어 종료된다(검출만 볼 때는 `test_accuracy.py`)

### 4-1. 프레임 루프 — 파이5 `main.py`

```
   ┌────────────────────────────────────────────────────────────────────────┐
   │ ① 캡처 → ② 검출후보 → ③ 가설갱신·채택 → ④ 목표점 전송                     │
   │                          (a) 채택 fit 있음 → 착지점                        │
   │                          (b) 없고 직전 목표 있음 → coast (같은 목표)         │
   │                          (c) 둘 다 없음 → 조기 출발 (화면 이동방향으로 1 m)   │
   └────────────────────────────┬───────────────────────────────────────────┘
                                └──── 다음 프레임에서 반복 (실기 60 fps)
```

| 단계 | 파일 / 함수 | 상태 |
|---|---|---|
| ① 프레임 캡처 | `vision.Camera.capture` | ✅ 실기 |
| ② Hailo YOLO → 임계값 통과 후보 전체 | `vision.detect_all` | ✅ 실기 60 fps |
| ②' bbox 중심 왜곡 보정 | `vision._undistort` / `fisheye.py` | ✅ 실측 어안 계수 |
| ③ 가설 갱신 + 물리 통과분만 채택 | `tracker.TrackerPool` | ✅ 실기. ⚠ 주행 중 정지 오탐 채택은 합성으로만 측정 |
| ④ 착지점 → 로봇축 → 월드 목표점 | `Tracker.landing(now)` → `control.to_target_command` | ✅ 실기. ⚠ 방향 기준 확정 전(5장) |
| ④' 오도메트리 수신 (논블로킹) | `communication.try_receive_odometry` | ✅ 실기 왕복 |
| ④" 프레임 전송 | `communication.send_target` | ✅ 실기 왕복 |

### 4-2. USB 전송

프레임: `[START 0xAA][LEN][PAYLOAD][CHECKSUM]`, `CHECKSUM = sum(PAYLOAD) % 256`

| 방향 | PAYLOAD | 크기 |
|---|---|---|
| 파이5 → 피코 ① | `<ffff>` target_x, target_y, time_remaining_s, timeout_s (월드 좌표) | 16 B |
| 파이5 → 피코 ② | `<fff>` target_vx, target_vy, timeout_s (텔레옵·정지) | 12 B |
| 피코 → 파이5 | `<ffffff>` x, y, theta, vx, vy, omega | 24 B |

상세는 [`docs/design/protocol.md`](protocol.md).

### 4-3. 피코 실시간 루프 — `main.c` (20 ms)

| 단계 | 파일 | 상태 |
|---|---|---|
| 엔코더 카운트 (PIO, 1x) | `encoder/encoder_pio.c` | ✅ 핀·CPR 실측 |
| 카운트 → 바퀴속도 → 순기구학 → pose 적분 | `main.c`, `kinematics.c`, `odometry_kalman.c` | ✅ 실기. ⏳ 스케일·슬립 줄자 대조 전 |
| 명령 확인 (목표점/속도) | `communication.c` | ✅ 실기 |
| 남은거리÷남은시간 + 방향별 상한 | `main.c`, `kinematics.c` | ✅ 실기. ⚠ 상한 1.8 m/s 는 실측값 아님 |
| 역기구학 → PID → PWM | `kinematics.c`, `motor_control.c` | ✅ 실기. ⚠ `MOTOR_SIGN` 확정 전 |
| 오도메트리 회신 | `communication.c` | ✅ 실기 |

속도 상한은 **크기만 깎고 방향은 보존**한다. 포화 상태에서는 세 바퀴의 속도 비율이 깨져
진행 방향까지 틀어지기 때문이다.

---

## 5. 지금 막혀 있는 지점

**로봇은 물체를 보고 달린다.** 남은 건 "맞게, 멈출 수 있게" 달리는지의 확인이다. 우선순위 순:

| 순위 | 항목 | 없으면 어떻게 되나 |
|---|---|---|
| 1 | **방향 기준 확정** — `pico_test.py --dir front` 로 구동 단독 방향 확인 ⚠ | 사진 측정(yaw 180°)과 실기에서 맞는 값(0°)이 반대다. 구동 쪽 반전과 상쇄 중이면 한쪽만 고치는 순간 다시 반대로 간다 |
| 2 | **최대속도 실측** (`teleop_test.py --speed-test`) ⏳ | 상한 1.8 m/s 는 시험용 클램프(이론 1.22). 실제보다 높으면 바퀴가 제각각 포화돼 방향이 틀어지고 도달 판정이 낙관적이 된다 |
| 3 | **정지거리·가속시간 실측** (`pico_test.py --max`) ⏳ | 설계가 "전속 돌진"(`DRIVE_AGGRESSION 8`, 목표 하한 0.6 m)이라 오버슈트가 통 반지름 13 cm 를 넘는지 모른다 |
| 4 | **투척 영역 규약** ⏳ | 09-09 실기 투척 둘이 3.8·2.5 m/s 를 요구했다 — 인식이 아니라 물리적으로 못 잡는 투척이 섞이면 성공률로 아무것도 판단할 수 없다 |
| 5 | **주행 중 오탐·잔차 실측** (`MAX_RESIDUAL_PX`) ⏳ | 합성에서 로봇 0.5 m/s 면 정지 천장점이 궤적으로 채택되는 비율이 크다 |

미해결 리스크 전체는 [`docs/design/open-questions.md`](open-questions.md), 할 일은
[`src/pi5/TODO.md`](../../src/pi5/TODO.md) · [`src/pico/TODO.md`](../../src/pico/TODO.md).

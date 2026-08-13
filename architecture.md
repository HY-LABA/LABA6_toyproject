# 소프트웨어 아키텍처

파일 구조, 파일별 책임, 파이5↔피코 역할 분담, 그리고 각 단계의 구현 상태.
알고리즘 자체는 [`algorithm.md`](algorithm.md) 참고.

> 상태 표기: ✅ 로직 완성 · ⏳ 구조는 있으나 실측/연동 필요 · ⛔ 아직 없음

---

## 1. 파일 구조

```
LABA6_toyproject/
├── README.md                     # 프로젝트 소개, 하드웨어 구성, 문서 지도
├── CHANGELOG.md                  # 설계 변경 이력 (여기 말고 다른 문서엔 이력을 안 적는다)
├── architecture.md               # 이 문서 — 파일 구조, 책임, 파이프라인 흐름, 구현 상태
├── algorithm.md                  # 궤적 추정 수식, 제어 루프, 검증된 성능
├── docs/                         # 설계 근거 — "왜 그 값인가"
│
├── pi5/                          # 라즈베리파이5 (Python) — 실시간 캐치
│   ├── main.py                   # 프레임 루프: 관측 → 재피팅 → 예측 → 속도 명령
│   ├── config.py                 # 카메라 내부파라미터, 궤적/제어 파라미터 전체
│   ├── vision.py                 # 카메라 캡처 + YOLO → bbox 중심 + 왜곡보정
│   ├── trajectory.py             # ★ 중력 기반 궤적 최소제곱 + 착지 예측 + Tracker
│   ├── fisheye.py                # 어안 → 핀홀 등가 좌표 변환
│   ├── control.py                # 남은거리÷남은시간 → 목표 속도 (+ 최대속도 클램프)
│   ├── communication.py          # USB 시리얼 프레임 인코딩/디코딩
│   ├── utils.py                  # 로깅 + 타이밍
│   ├── TODO.md                   # 실측·연동 필요 항목
│   └── prep/                     # 학습 데이터 준비 (로봇 구동과 무관, 오프라인)
│       ├── camera.py             # 카메라 추상화 — 카메라 교체 시 여기만 손댄다
│       ├── check_setup.py        # 실행 전 환경 점검
│       ├── calibrate.py          # 체스보드 캘리브레이션 (pinhole/fisheye 자동 선택)
│       ├── capture_dataset.py    # MOG2 자동 검출·라벨링으로 YOLO 데이터셋 수집
│       ├── review_labels.py      # 자동 라벨 수동 검수 (PC에서)
│       ├── prepare_dataset.py    # 세션 단위 train/val/test 분할 + dataset.yaml
│       ├── train_yolo.py         # YOLOv8n 전이학습 (PC/Colab)
│       ├── measure_sigma_w.py    # 참고용 — 합격 기준으로는 쓰지 않는다
│       └── TROUBLESHOOTING.md    # 실기 작업 기록 — 겪은 문제와 해결
│
├── pico/                         # 라즈베리파이 피코 (C, Pico SDK) — 실시간 모터 제어
│   ├── main.c                    # 1 ms 실시간 제어 루프
│   ├── config.h                  # 핀 배치, 물리 상수, PID 게인, 최대속도
│   ├── encoder/
│   │   ├── encoder_pio.c/.h      # PIO 기반 A/B상 엔코더 카운팅
│   │   └── encoder.pio           # PIO 프로그램 (별도 빌드 대상이라 분리)
│   ├── kinematics.c/.h           # 3륜 옴니 순/역기구학
│   ├── odometry_kalman.c/.h      # 속도 노이즈 스무딩 + 오도메트리 적분
│   ├── motor_control.c/.h        # PID + BTS7960 PWM 출력
│   ├── communication.c/.h        # 파이5와 왕복 USB CDC 통신
│   ├── CMakeLists.txt            # 빌드 설정
│   ├── pico_sdk_import.cmake     # Pico SDK 부트스트랩
│   └── TODO.md                   # 실측·연동 필요 항목
│
└── data_collection/              # 독립 실험 도구 — 현재 구동 파이프라인에서 쓰지 않음
    ├── brain/                    # 팀원이 별도로 작성한 수집 도구
    └── tools/                    # 크롭 기반 이진분류 데이터셋 (YOLO 라벨 아님)
```

> **`data_collection/`은 `pi5/`와 완전히 독립**이며 import 관계가 없다. 같은 목적(투척 물체
> 자동 라벨링)을 다르게 구현한 것으로, `brain/planning/trajectory.py`는 `pi5/trajectory.py`와
> **같은 수식**이다. 이쪽에만 있는 것: 축소 해상도 MOG2(더 빠름), 추적+포물선 탄도 게이트,
> 물체 단위 누수 검증 분할, **카메라 없이 돌리는 합성 투척 생성기**(`SyntheticThrowSource`).

---

## 2. 파일별 책임

### pi5/ (Python) — 실시간 캐치

| 파일 | 담당 기능 |
|---|---|
| `main.py` | 프레임 단위 루프 하나로 전체를 돌린다: 캡처→검출→관측 누적→재피팅→착지 예측→오도메트리 수신→목표 속도 전송. 물체를 `TRACK_MAX_GAP_S` 이상 놓치면 트랙을 리셋하고 로봇을 세운다. 어떤 경로로 빠져나가도 `finally`에서 정지 명령을 보낸다 |
| `config.py` | 카메라 내부파라미터(`CAMERA_FX/FY/CX/CY`, 왜곡, 모델), 해상도·fps, YOLO 경로·임계값·단일 클래스, 궤적 파라미터(`MIN_OBSERVATIONS`, `MIN_TIME_SPAN_S`, `MAX_RESIDUAL_PX`, `DEPTH_STABILITY_RATIO`, `Z_RANGE_M`), 로봇 최대속도·워치독·도착 허용오차, 시리얼 설정. **각 값에 근거가 주석으로 붙어 있다** |
| `vision.py` | ① Picamera2 캡처 (**센서 타임스탬프** 사용 — 파이썬 수신 시각은 스케줄링 지터가 섞여 궤적 피팅이 그걸 물체의 운동으로 읽는다) ② Hailo NPU로 YOLO 추론 ③ 신뢰도 최고 검출 하나의 **bbox 중심**만 취함 ④ 그 **점 하나만** 왜곡 보정 (프레임 전체를 펴는 건 낭비다) |
| `trajectory.py` | ★ 핵심. `fit_trajectory`(2N×6 선형 최소제곱으로 위치·속도 6개 동시 추정), `landing_time`/`predict_landing`(캐치 평면 통과 시각·좌표), `Tracker`(관측 누적 + 매 프레임 재피팅 + **깊이 수렴 판정**) |
| `fisheye.py` | 등입체각 어안 좌표를 핀홀 등가로 옮긴다. 캘리브레이션 계수가 없는 동안 `vision._undistort`가 이걸 쓴다 |
| `control.py` | 착지점 − 오도메트리 = 남은 거리, 나누기 남은 시간 = 목표 속도. 최대속도 초과 시 **크기만 깎고 방향은 보존**(포화 상태에서는 세 바퀴 속도 비율이 깨져 진행 방향까지 틀어진다). 도착 허용오차 안이면 정지 |
| `communication.py` | `[START 0xAA][LEN][PAYLOAD][CHECKSUM]` 프레임. 수신은 **논블로킹**이고 버퍼에 쌓인 것 중 최신만 쓴다 — 여기서 기다리면 카메라 프레임을 놓치고 그건 곧 관측 손실이다 |
| `utils.py` | `run.log` append + 콘솔 동시 출력. `log_cycle`이 한 사이클의 판단 근거(관측수, 잔차, 깊이, 착지점, 남은시간, 오도메트리, 명령)를 한 줄로 남긴다 — 예측이 이상할 때 "궤적이 안 맞나(잔차↑) 관측이 부족한가(n↓)"를 로그만으로 구분하기 위함 |

### pi5/prep/ (Python) — 학습 데이터 준비 (오프라인)

| 파일 | 담당 기능 |
|---|---|
| `camera.py` | **카메라를 바꿀 때 손대는 유일한 파일.** 프로파일(해상도/fps/노출/게인/캘리브레이션 모델)을 `SPECS`에 두고 Picamera2↔OpenCV를 같은 인터페이스로 감싼다. 자동 노출 측정 후 고정(`auto_lock`), 셔터 우선 상한, 노이즈 리덕션 차단(MOG2에 잔상을 남긴다)까지 처리 |
| `check_setup.py` | 패키지·카메라·디스크 사전 점검. 밝기 진단도 출력 |
| `calibrate.py` | `capture`(체스보드 촬영, 3×3 커버리지 추적) + `solve`(계산 → JSON + config 스니펫). 프로파일의 `calib_model`을 보고 `cv2.calibrateCamera` / `cv2.fisheye.calibrate`를 자동 선택 |
| `capture_dataset.py` | MOG2 배경차분으로 투척 물체를 자동 검출해 **YOLO 라벨(.txt)까지 자동 생성**. 확정 전 프레임을 모아뒀다 확정 시 함께 저장하고, 확정 후에는 `--track-grace`만큼 놓쳐도 추적을 유지 |
| `review_labels.py` | 자동 라벨을 사람이 검수·수정·기각. **수집은 라파이 헤드리스, 검수는 PC** 분업 |
| `prepare_dataset.py` | **세션 단위**로 train/val/test 분할 + `dataset.yaml` 생성. 프레임 단위 무작위 분할은 같은 투척이 양쪽에 들어가 검증 점수를 부풀린다 |
| `train_yolo.py` | YOLOv8n COCO 사전학습에서 전이학습. Colab용 `--project`(Drive 저장)·`--resume` 지원 |
| `measure_sigma_w.py` | bbox 폭 분산 측정. **합격 기준으로는 쓰지 않는다** — 깊이 추정이 폭을 안 쓴다. 검출 흔들림의 간접 지표로만 참고 |

### pico/ (C, Pico SDK) — 실시간 모터 제어

| 파일 | 담당 기능 |
|---|---|
| `main.c` | 1 ms 고정 주기(`sleep_until`) 루프: 엔코더 읽기 → 바퀴속도 변환 → 순기구학+칼만으로 오도메트리 갱신 → 파이 명령 확인 → **받은 속도를 그대로 사용**(나눗셈 없음) → 안전 클램프 → 역기구학 → PID → PWM → 오도메트리 회신 |
| `config.h` | 모터별 핀 배치, 로봇 물리 상수(휠 지름·장착각·중심거리·기어비), 엔코더 스펙, 제어주기, PID 게인, `MAX_BODY_SPEED_MPS` |
| `encoder/` | PIO로 A상 상승엣지만 카운트하고 그 순간 B상 레벨(`JMP PIN`)로 방향 판단하는 1x 쿼드러처 디코딩. CPU 개입 없이 하드웨어가 누적 |
| `kinematics.c/.h` | 역기구학(body 속도 → 바퀴 3개 선속도)과 순기구학(3×3 역행렬) |
| `odometry_kalman.c/.h` | vx, vy, omega 축별 독립 1D 칼만으로 엔코더 양자화 노이즈를 스무딩하고, body-frame 속도를 로봇 방향(theta) 기준으로 world-frame으로 회전 변환해 적분 |
| `motor_control.c/.h` | 바퀴별 PID → 부호에 따라 BTS7960의 RPWM/LPWM 중 하나에 PWM 출력 |
| `communication.c/.h` | 논블로킹 상태머신으로 프레임 수신, 오도메트리 송신 |

---

## 3. 파이5 ↔ 피코 역할 분담

| | 파이5 | 피코 |
|---|---|---|
| 궤적 추정 | ✅ | |
| 착지 예측 | ✅ | |
| 남은 거리 계산 (오도메트리 차감) | ✅ | |
| 목표 속도 산출 | ✅ | |
| 최대속도 클램프 | ✅ (1차) | ✅ (2차 안전) |
| 오도메트리 적분 | | ✅ |
| 역기구학 + PID + PWM | | ✅ |

**파이5가 계획을 전담하고 피코는 "이 속도로 돌려라"만 실행한다.**
**피코는 궤적도 착지점도 모른다** — 이 경계를 지키는 것이 설계의 핵심이다.
**피코는 나눗셈을 하지 않는다.**

---

## 4. 파이프라인 흐름과 구현 상태

### 4-0. 부팅 시점

- 피코 `main.c`는 켜지자마자 1 ms 주기 슈퍼루프 시작. 파이5가 아직 아무것도 안 보냈어도
  계속 돌면서 엔코더를 읽고 오도메트리를 갱신한다 (명령 없으면 목표속도 0, 정지 유지). ✅
- 파이5 `main.py`는 `run()` 호출 시점부터 (카메라·시리얼 초기화). ⏳

### 4-1. 프레임 루프 — 파이5 `main.py` (60 fps, 매 프레임 동일)

```
   ┌──────────────────────────────────────────────────────────┐
   │  ① 캡처 → ② 검출 → ③ 누적·재피팅 → ④ 예측 → ⑤ 명령      │
   └────────────────────────┬─────────────────────────────────┘
                            └──── 다음 프레임에서 그대로 반복
```

| 단계 | 파일 / 함수 | 상태 |
|---|---|---|
| ① 프레임 캡처 (센서 타임스탬프) | `vision.Camera.capture` | ⏳ Picamera2 연동 필요 |
| ② Hailo YOLO 추론 → 최고 신뢰도 1개 | `vision.detect` | ⏳ `.hef` 학습·변환 후 |
| ②' bbox 중심 왜곡 보정 (점 하나만) | `vision._undistort` / `fisheye.py` | ✅ 로직 완성, ⏳ 왜곡계수 필요 |
| ③ 관측 누적 + 매 프레임 재피팅 | `trajectory.Tracker.add` | ✅ **합성 투척 검증 완료** |
| ③' 깊이 수렴 판정 | `Tracker._depth_converged` | ✅ |
| ④ 착지점·남은시간 예측 | `trajectory.predict_landing` | ✅ |
| ⑤ 오도메트리 수신 (논블로킹) | `communication.try_receive_odometry` | ✅ 로직, ⏳ pyserial 연결 |
| ⑤' 남은거리÷남은시간 → 목표속도 | `control.to_drive_command` | ✅ |
| ⑤" 프레임 전송 | `communication.send_command` | ✅ 로직, ⏳ pyserial 연결 |

### 4-2. USB 전송

프레임: `[START 0xAA][LEN][PAYLOAD][CHECKSUM]`, `CHECKSUM = sum(PAYLOAD) % 256` ✅

| 방향 | PAYLOAD | 크기 |
|---|---|---|
| 파이5 → 피코 | `<fff>` target_vx, target_vy, timeout_s | 12 B |
| 피코 → 파이5 | `<ffffff>` x, y, theta, vx, vy, omega | 24 B |

상세는 [`docs/protocol.md`](docs/protocol.md).

### 4-3. 피코 실시간 루프 — `main.c` (1 ms마다 반복)

| 단계 | 파일 | 상태 |
|---|---|---|
| 엔코더 카운트 읽기 (PIO, 1x 쿼드러처) | `encoder/encoder_pio.c` | ✅ 로직, ⏳ 핀 번호 |
| 카운트 → 바퀴속도 | `main.c` | ✅, ⏳ `ENCODER_COUNTS_PER_REV` |
| 바퀴속도 → 로봇속도 (순기구학) | `kinematics.c` | ✅ 실행 검증 완료 |
| 속도 스무딩 + 위치 적분 | `odometry_kalman.c` | ✅ 로직, ⏳ 노이즈 파라미터 |
| 파이5 명령 확인 (논블로킹) | `communication.c` | ✅ |
| 받은 속도 그대로 사용 + 안전 클램프 | `main.c` | ✅ |
| 로봇속도 → 바퀴 3개 목표속도 (역기구학) | `kinematics.c` | ✅ 실행 검증 완료 |
| PID + PWM 출력 | `motor_control.c` | ✅ 구조, ⏳ 게인·스케일 |
| 오도메트리 회신 | `communication.c` | ✅ |

최대속도 클램프는 **크기만 깎고 방향은 보존**한다. 포화 상태에서는 세 바퀴의 속도 비율이
깨져서 크기뿐 아니라 진행 방향까지 틀어지기 때문이다.

---

## 5. 지금 막혀 있는 지점

**로직상으로는 막힌 데가 없다.** 남은 것은 전부 **실측값과 하드웨어 연동**이다.
우선순위 순:

| 순위 | 항목 | 없으면 어떻게 되나 |
|---|---|---|
| 1 | **카메라 캘리브레이션 실행** (`prep/calibrate.py`는 작성 완료) ⏳ | f가 틀린 만큼 깊이가 그대로 틀어진다 |
| 2 | **YOLO 학습 + Hailo 변환** ⏳ | 검출 자체가 안 된다 |
| 3 | **`vision.Camera` Picamera2 연동** ⏳ | 프레임을 못 받는다 |
| 4 | **`CAMERA_OFFSET_M` 상수·보정 코드** ⛔ | 매번 같은 방향으로 15 cm 어긋난다 |
| 5 | **pyserial 연결** ⏳ | 피코와 통신 불가 |
| 6 | **모터 실측** (최대속도/가속/정지거리) ⏳ | 부품 미도착 |
| 7 | **피코 핀 번호·PID 게인** ⏳ | 부품 미도착 |

파일별 상세 항목은 [`pi5/TODO.md`](pi5/TODO.md) · [`pico/TODO.md`](pico/TODO.md).

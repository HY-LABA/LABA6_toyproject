# 소프트웨어 아키텍처

> 2026-09 개정. 파이5↔피코 프로토콜(**월드 좌표 목표점 전송**)이 양쪽 다 구현 완료됐다
> (이전 판은 "파이5 완료, 피코 미반영"이었음). 피코 쪽은 MicroPython 벤치 테스트
> (`pico_micropython/`)로 바퀴 배치·엔코더·모터 부호·PID를 실기 검증한 뒤 C 펌웨어에
> 반영했다. 파이5 쪽은 다중 가설 추적(`tracker.py`)이 새로 생겨 `trajectory.py`에서
> 상태를 분리했다. 계약은 `docs/protocol.md`, 알고리즘 자체는 `algorithm.md` 참고.
> **실기 UF2 빌드까지 검증 완료, 파이5-피코 통신 왕복은 아직 미검증.**

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
│   ├── main.py                   # 프레임 루프: 캡처 → 검출후보 → 가설갱신 → 채택 → 예측 → 목표점 전송
│   ├── config.py                 # 카메라 내부파라미터, 궤적/제어 파라미터 전체
│   ├── vision.py                 # 카메라 캡처 + Hailo YOLO → 프레임의 **검출 후보 전체** + 왜곡보정
│   ├── trajectory.py             # 순수 계산만: 중력 기반 궤적 최소제곱 + 착지 예측 (상태 없음)
│   ├── tracker.py                # ★ 상태를 갖는 부분 전부 — 후보마다 가설(Tracker) 추적,
│   │                             #   물리를 통과한 가설만 채택(TrackerPool), IDLE/TRACKING/COOLDOWN 사이클
│   ├── fisheye.py                # 어안 → 핀홀 등가 좌표 변환
│   ├── control.py                # 착지점 → 월드 좌표 목표점(TargetCommand) + 도달판정·로그용 속도 예측
│   ├── communication.py          # USB 시리얼 프레임 인코딩/디코딩 — 목표점(16B)/속도(12B)/오도메트리(24B)
│   ├── utils.py                  # 로깅 + 타이밍
│   ├── teleop_test.py            # Xbox 컨트롤러로 body-frame vx/vy 직접 전송 — 피코 브링업 테스트
│   ├── test_accuracy.py          # 로봇 정지 상태로 궤적 예측 정확도만 측정 (실기 로그 저장 + 리플레이)
│   ├── TODO.md                   # 실측·연동 필요 항목
│   └── prep/                     # 학습 데이터 준비 (로봇 구동과 무관, 오프라인)
│       ├── camera.py             # 카메라 추상화 — 카메라 교체 시 여기만 손댄다
│       ├── check_setup.py        # 실행 전 환경 점검
│       ├── capture_dataset.py    # MOG2 자동 검출·라벨링으로 YOLO 데이터셋 수집
│       ├── review_labels.py      # 자동 라벨 수동 검수 (PC에서)
│       ├── prepare_dataset.py    # 세션 단위 train/val/test 분할 + dataset.yaml
│       ├── train_yolo.py         # YOLOv8n 전이학습 (PC/Colab)
│       ├── measure_sigma_w.py    # 참고용 — 합격 기준으로는 쓰지 않는다
│       ├── calibrate.py          # 체스보드 캘리브레이션(capture+solve) — config.py 실측값의 출처
│       └── TROUBLESHOOTING.md    # 실기 작업 기록 — 겪은 문제와 해결
│
├── pico/                         # 라즈베리파이 피코 (C, Pico SDK) — 실시간 모터 제어. 실기 UF2 빌드 검증됨
│   ├── main.c                    # 20ms(50Hz) 실시간 제어 루프 — 목표점/속도 두 명령 경로 처리
│   ├── config.h                  # 핀 배치, 물리 상수, MOTOR_SIGN, PID 게인, 최대속도 — 전부 실기 확정값
│   ├── encoder/
│   │   ├── encoder_pio.c/.h      # PIO 기반 A/B상 엔코더 카운팅
│   │   └── encoder.pio           # PIO 프로그램 (별도 빌드 대상이라 분리)
│   ├── kinematics.c/.h           # 3륜 옴니 순/역기구학 + 방향별 바퀴기준 속도 상한
│   ├── odometry_kalman.c/.h      # 속도 노이즈 스무딩 + 오도메트리 적분
│   ├── motor_control.c/.h        # PID(안티와인드업+데드밴드 보상) + BTS7960 PWM 출력
│   ├── communication.c/.h        # 파이5와 왕복 USB CDC 통신 — 목표점(16B)/속도(12B) 둘 다 수용
│   ├── CMakeLists.txt / pico_sdk_import.cmake / .vscode/  # 빌드 설정 (VS Code Pico 확장으로 검증)
│   └── TODO.md                   # 실측·연동 필요 항목
│
├── pico_micropython/             # MicroPython 벤치 테스트 리그 — UF2 재플래시 없이 값 튜닝용.
│   └── goto_xy_test.py           # 좌표 입력 → 폐루프 이동. pico/config.h 확정값의 실측 출처
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
| `main.py` | 프레임 단위 루프 하나로 전체를 돌린다: 캡처→**검출 후보 전체**→`TrackerPool.step`(가설 갱신·채택·예측)→**월드 좌표 목표점 전송**. 채택된 가설이 없거나 사이클이 끝나면(착지/유실/타임아웃) 리셋하고 로봇을 세운다. 어떤 경로로 빠져나가도 `finally`에서 정지 명령을 보낸다 |
| `config.py` | 카메라 내부파라미터(`CAMERA_FX/FY/CX/CY`, 왜곡, 모델), 해상도·fps, YOLO 경로·임계값·단일 클래스, 궤적 파라미터(`MIN_OBSERVATIONS`, `MIN_TIME_SPAN_S`, `MAX_RESIDUAL_PX`, `DEPTH_STABILITY_RATIO`, `Z_RANGE_M`), 바퀴 배치·최대속도·워치독·도착 허용오차, 시리얼 설정. **실측으로 정한 값에는 근거 표가 주석으로 붙어 있다** |
| `vision.py` | ① Picamera2 캡처 ② Hailo NPU로 YOLO 추론 ③ **신뢰도 임계값을 넘는 검출 후보 전부**의 bbox 중심을 취함(최고점 하나만 쓰면 오탐이 진짜를 가릴 수 있어 후보를 다 넘기고 `tracker`가 물리로 거른다) ④ 후보마다 `cv2.undistortPoints`로 **그 점만** 왜곡 보정. bbox 크기는 로그용으로만 남긴다 |
| `trajectory.py` | **순수 계산만, 상태 없음.** `fit_trajectory`(2N×6 선형 최소제곱으로 위치·속도 6개 동시 추정, 카메라 이동 보정 포함), `landing_time`/`predict_landing`(캐치 평면 통과 시각·좌표), `project`(연관용 화면 좌표 역산) |
| `tracker.py` | ★ 상태를 갖는 부분 전부. `Tracker`(가설 하나 — 관측 누적, 카메라 월드 위치 동시 기록, 매 프레임 재피팅, 깊이 수렴 판정), `TrackerPool`(후보마다 가설을 굴려 물리를 통과한 것만 채택, IDLE/TRACKING/COOLDOWN 사이클 관리). 정적 오탐이 짧은 관측 구간에서 우연히 fit.ok를 통과하는 사례가 실기에서 확인돼, 화면상 이동량 사전필터·풀린 속도 상한 사후필터를 추가로 둔다 |
| `control.py` | 착지점(트랙 원점 기준) + 트랙 시작 시점 오도메트리 = **월드 좌표 목표점**(`TargetCommand`). 남은 거리를 빼는 것도 속도를 만드는 것도 피코가 한다 — 파이가 미리 빼면 두 번 빠진다. 방향별 바퀴 속도 상한(`max_body_speed`)도 여기 있다 — 로그·도달판정용 예측이 피코와 같은 식을 써야 갈리지 않는다. `verify_wheel_config()`가 `pico/config.h`를 직접 읽어 바퀴 설정 불일치를 잡아낸다 |
| `communication.py` | `[START 0xAA][LEN][PAYLOAD][CHECKSUM]` 프레임. LEN으로 목표점(16B)/속도(12B)를 구분해 보낸다. 수신은 **논블로킹**이고 버퍼에 쌓인 것 중 최신만 쓴다 — 여기서 기다리면 카메라 프레임을 놓치고 그건 곧 관측 손실이다 |
| `utils.py` | `run.log` append + 콘솔 동시 출력. `log_cycle`이 한 사이클의 판단 근거(관측수, 잔차, 깊이, 착지점, 남은시간, 오도메트리, 명령)를 한 줄로 남긴다 — 예측이 이상할 때 "궤적이 안 맞나(잔차↑) 관측이 부족한가(n↓)"를 로그만으로 구분하기 위함 |
| `teleop_test.py` | Xbox 컨트롤러 왼쪽 스틱으로 body-frame `target_vx/vy`를 직접 전송(12 B 속도 프레임). 카메라/YOLO 없이 피코 모터 응답만 먼저 검증할 수 있다 |
| `test_accuracy.py` | 로봇을 세워둔 채(오도메트리 고정) 카메라→YOLO→궤적피팅까지 `main.py`와 같은 함수로 돌려 **예측 정확도만** 측정. 관측을 전부 저장해 `--replay`로 파라미터를 바꿔가며 재실행할 수 있다. 실측 착지점을 입력받아 좌표 부호가 뒤집혔는지도 자동 진단 |

### pi5/prep/ (Python) — 학습 데이터 준비 (오프라인)

| 파일 | 담당 기능 |
|---|---|
| `camera.py` | **카메라를 바꿀 때 손대는 유일한 파일.** 프로파일(해상도/fps/노출/게인/캘리브레이션 모델)을 `SPECS`에 두고 Picamera2↔OpenCV를 같은 인터페이스로 감싼다. 자동 노출 측정 후 고정(`auto_lock`), 셔터 우선 상한, 노이즈 리덕션 차단(MOG2에 잔상을 남긴다)까지 처리 |
| `check_setup.py` | 패키지·카메라·디스크 사전 점검. 밝기 진단도 출력 |
| `capture_dataset.py` | MOG2 배경차분으로 투척 물체를 자동 검출해 **YOLO 라벨(.txt)까지 자동 생성**. 확정 전 프레임을 모아뒀다 확정 시 함께 저장하고, 확정 후에는 `--track-grace`만큼 놓쳐도 추적을 유지한다. `--no-display`(원격 접속 시 필수), 실효 fps 출력 |
| `review_labels.py` | 자동 라벨을 사람이 검수·수정·기각. **수집은 라파이 헤드리스, 검수는 PC** 분업 |
| `prepare_dataset.py` | **세션 단위**로 train/val/test 분할 + `dataset.yaml` 생성. 프레임 단위 무작위 분할은 같은 투척이 양쪽에 들어가 검증 점수를 부풀린다 |
| `train_yolo.py` | YOLOv8n COCO 사전학습에서 전이학습. Colab용 `--project`(Drive 저장)·`--resume` 지원. 크기·회전 증강을 좁게 잡는다 |
| `measure_sigma_w.py` | (구) bbox 폭 분산 측정. **합격 기준으로서는 폐기** — 깊이 추정이 폭을 더 이상 쓰지 않는다. 검출 흔들림의 간접 지표로만 참고 |
| `TROUBLESHOOTING.md` | 실기 작업 기록. 형광등 맥동, X11 프레임 드롭, 노이즈 리덕션 잔상, 설계 전환 근거와 실측 표 |

### pico/ (C, Pico SDK) — 실시간 모터 제어

| 파일 | 담당 기능 |
|---|---|
| `main.c` | 20ms(50Hz) 고정 주기(`sleep_until`) 루프: 엔코더 읽기 → `MOTOR_SIGN` 보정 → 바퀴속도 변환 → 순기구학+칼만으로 오도메트리 갱신 → 파이 명령 확인(목표점/속도 두 경로, 최근 받은 쪽이 활성) → **목표점이면 남은거리÷남은시간을 직접 계산**, 속도면 그대로 사용 → 방향별 바퀴기준 속도 상한 → 역기구학 → PID → PWM → 오도메트리 회신 |
| `config.h` | 모터별 핀 배치, `MOTOR_SIGN`(모터별 부호 보정), 로봇 물리 상수(휠 지름·장착각·중심거리·기어비), 엔코더 CPR(실측), 제어주기, PID 게인, `WHEEL_MAX_SPEED_MPS`, `POSITION_TOLERANCE_M` — 전부 MicroPython 벤치 테스트로 실기 확정 |
| `encoder/` | PIO로 A상 상승엣지만 카운트하고 그 순간 B상 레벨(`JMP PIN`)로 방향 판단하는 1x 쿼드러처 디코딩. CPU 개입 없이 하드웨어가 누적 |
| `kinematics.c/.h` | 역기구학(body 속도 → 바퀴 3개 선속도), 순기구학(3×3 역행렬), 방향별 바퀴기준 최대 body 속도(`max_body_speed`, pi5 `control.py`와 같은 식) |
| `odometry_kalman.c/.h` | vx, vy, omega 축별 독립 1D 칼만으로 엔코더 양자화 노이즈를 스무딩하고, body-frame 속도를 로봇 방향(theta) 기준으로 world-frame으로 회전 변환해 적분 |
| `motor_control.c/.h` | 바퀴별 PID(안티와인드업 + 데드밴드 보상) → `MOTOR_SIGN`으로 하드웨어 부호 복원 → BTS7960의 RPWM/LPWM 중 하나에 PWM 출력 |
| `communication.c/.h` | 논블로킹 상태머신으로 프레임 수신 — LEN으로 목표점(16B)/속도(12B) 구분, 오도메트리 송신 |

---

## 3. 파이5 ↔ 피코 역할 분담

| | 파이5 | 피코 |
|---|---|---|
| 궤적 추정 | ✅ | |
| 착지 예측 | ✅ | |
| 남은 거리 계산 (오도메트리 차감) | | ✅ |
| 목표 속도 산출 (남은거리÷남은시간) | | ✅ |
| 방향별 바퀴기준 속도 상한 | ✅ (로그·도달판정용 예측) | ✅ (실제 적용) |
| 오도메트리 적분 | | ✅ |
| 역기구학 + PID + PWM | | ✅ |

**파이5가 착지점(월드 좌표)과 남은시간만 넘기고, "거기까지 남은 거리를 계산해 얼마나
빨리 가야 하는지"는 피코가 자기 오도메트리로 직접 계산한다** (2026-09, `docs/protocol.md`
목표점 방식). 텔레옵·정지용 속도 경로만 예전처럼 피코가 나눗셈 없이 그대로 쓴다.
**피코는 여전히 궤적도 착지점도 모른다** — 받는 건 좌표 하나와 시간 하나뿐이다.

---

## 4. 파이프라인 흐름과 구현 상태

### 4-0. 부팅 시점

- 피코 `main.c`는 켜지자마자 20 ms(50 Hz) 주기 슈퍼루프 시작. 파이5가 아직 아무것도
  안 보냈어도 계속 돌면서 엔코더를 읽고 오도메트리를 갱신한다 (명령 없으면 목표속도 0,
  정지 유지). ✅ 실기 UF2 빌드로 확인됨
- 파이5 `main.py`는 `run()` 호출 시점부터 (카메라·시리얼 초기화). 카메라+YOLO는
  `live_predict.py`로 별도 확인됨 ✅. **`main.py`로 피코까지 붙인 왕복은 아직 미검증**

### 4-1. 프레임 루프 — 파이5 `main.py` (카메라 fps 기준, 매 프레임 동일)

```
   ┌──────────────────────────────────────────────────────────────┐
   │  ① 캡처 → ② 검출후보 → ③ 가설갱신·채택 → ④ 예측 → ⑤ 명령      │
   └────────────────────────┬─────────────────────────────────────┘
                            └──── 다음 프레임에서 그대로 반복
```

| 단계 | 파일 / 함수 | 상태 |
|---|---|---|
| ① 프레임 캡처 | `vision.Camera.capture` | ✅ Picamera2 연동 완료 |
| ② Hailo YOLO 추론 → 신뢰도 임계값 통과 후보 전체 | `vision.detect_all` | ✅ `.hef` 학습·변환 완료 |
| ②' bbox 중심 왜곡 보정 (후보마다 점 하나) | `vision._undistort` / `fisheye.py` | ✅ 캘리브레이션 완료(실측 어안 계수) |
| ③ 후보별 가설 갱신 + 물리 통과분만 채택 | `tracker.TrackerPool.step` | ✅ 실기 로그로 필터 보강 중 (2026-09) |
| ③' 깊이 수렴 판정 | `Tracker._depth_converged` | ✅ |
| ④ 착지점·남은시간 예측 | `trajectory.predict_landing` | ✅ |
| ⑤ 오도메트리 수신 (논블로킹) | `communication.try_receive_odometry` | ✅ 로직·연결 완료, **피코와의 실측 왕복 미검증** |
| ⑤' 목표점(월드 좌표) 조립 | `control.to_target_command` | ✅ |
| ⑤" 프레임 전송 | `communication.send_target` | ✅ 로직·연결 완료, **피코와의 실측 왕복 미검증** |

### 4-2. USB 전송

프레임: `[START 0xAA][LEN][PAYLOAD][CHECKSUM]`, `CHECKSUM = sum(PAYLOAD) % 256` ✅

| 방향 | PAYLOAD | 크기 |
|---|---|---|
| 파이5 → 피코 ① | `<ffff>` target_x, target_y, time_remaining_s, timeout_s (월드 좌표) | 16 B |
| 파이5 → 피코 ② | `<fff>` target_vx, target_vy, timeout_s (텔레옵·정지) | 12 B |
| 피코 → 파이5 | `<ffffff>` x, y, theta, vx, vy, omega | 24 B |

**양방향 다 구현 완료** (2026-09 — 이전엔 피코가 ①을 조용히 무시했다). 상세는
[`docs/protocol.md`](docs/protocol.md).

### 4-3. 피코 실시간 루프 — `main.c` (20 ms마다 반복)

| 단계 | 파일 | 상태 |
|---|---|---|
| 엔코더 카운트 읽기 (PIO, 1x 쿼드러처) | `encoder/encoder_pio.c` | ✅ 핀·CPR 실기 확정 |
| 카운트 → 바퀴속도 (`MOTOR_SIGN` 보정 포함) | `main.c` | ✅ |
| 바퀴속도 → 로봇속도 (순기구학) | `kinematics.c` | ✅ 실행 검증 완료 |
| 속도 스무딩 + 위치 적분 | `odometry_kalman.c` | ✅ 로직, ⏳ 노이즈 파라미터 실측 |
| 파이5 명령 확인 (목표점/속도, 논블로킹) | `communication.c` | ✅ |
| 목표점이면 남은거리÷남은시간 직접 계산, 속도면 그대로 + 안전 클램프 | `main.c` | ✅ 로직, ⏳ 실기 UF2로는 미검증 |
| 로봇속도 → 바퀴 3개 목표속도 (역기구학) | `kinematics.c` | ✅ 실행 검증 완료 |
| PID(안티와인드업+데드밴드) + PWM 출력 | `motor_control.c` | ✅ 게인 확정(MicroPython 벤치), ⏳ 실기 C 재확인 |
| 오도메트리 회신 | `communication.c` | ✅ |

속도 상한은 **크기만 깎고 방향은 보존**한다. 포화 상태에서는 세 바퀴의 속도 비율이
깨져서 크기뿐 아니라 진행 방향까지 틀어지기 때문이다. 상한 자체도 body 속력 상수 하나가
아니라 **방향별 바퀴기준**으로 계산한다(`kinematics.c: max_body_speed`).

---

## 5. 지금 막혀 있는 지점

**로직·펌웨어 빌드는 다 됐다.** 남은 건 실기 검증과 몇 개 남은 실측값이다. 우선순위 순:

| 순위 | 항목 | 없으면 어떻게 되나 |
|---|---|---|
| 1 | **`main.py` ↔ 피코 왕복 실기 검증** (UF2는 빌드·플래시 완료, 통신은 아직) ⏳ | 지금 유일하게 실제로 안 돌려본 연결점 |
| 2 | **`CAMERA_OFFSET_M` 실측** (지금 (0,0) placeholder) ⛔ | 매번 같은 방향으로 약 15 cm 어긋난다 |
| 3 | **모터 실측** (최대속도/가속시간/정지거리) ⏳ | `WHEEL_MAX_SPEED_MPS`가 아직 이론값(1.22)이라 실제 한계와 다를 수 있다 |
| 4 | **`odometry_kalman.c` 노이즈 파라미터** ⏳ | 오도메트리가 과하게 흔들리거나 반응이 느릴 수 있다 |
| 5 | **바퀴 장착 반경(`WHEEL_MOUNT_RADIUS_M`) 실측** ⏳ | 지금 0.15m 이론값, 회전(ω) 쓸 때만 영향 |

카메라·YOLO·바퀴 배치·엔코더·모터 부호·PID·프로토콜은 전부 실기로 확인됐다.
파일별 상세 항목은 [`pi5/TODO.md`](pi5/TODO.md) · [`pico/TODO.md`](pico/TODO.md).

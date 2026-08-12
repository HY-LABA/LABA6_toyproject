# 소프트웨어 아키텍처

> 2026-08-10 개정. 깊이 추정 방식 전환(크기비율 → 중력 기반 궤적 최소제곱)과 파이5↔피코
> 프로토콜 변경(좌표 → 속도)을 반영했다. 알고리즘 자체는 `algorithm.md` 참고.

## 파일 구조

```
LABA6_toyproject/
├── README.md
├── architecture.md               # 이 문서 — 파일 구조, 파일별 책임
├── algorithm.md                  # 궤적 추정 수식, 제어 루프, 검증된 성능
├── system_flow.md                # pi5↔pico 파이프라인 흐름 + 구현 상태
├── docs/
│   └── coordinate_system.md      # (비어 있음 — 좌표계 정의는 pi5/trajectory.py 상단에)
│
├── pi5/                          # 라즈베리파이5 (Python)
│   ├── main.py                   # 프레임 루프: 관측 → 재피팅 → 예측 → 속도 명령
│   ├── config.py                 # 카메라 내부파라미터, 궤적/제어 파라미터 전체
│   ├── vision.py                 # 카메라 캡처 + YOLO → bbox 중심 + 왜곡보정
│   ├── trajectory.py             # ★ 중력 기반 궤적 최소제곱 + 착지 예측 + Tracker
│   ├── control.py                # 남은거리÷남은시간 → 목표 속도 (+ 최대속도 클램프)
│   ├── communication.py          # USB 시리얼 프레임 인코딩/디코딩
│   ├── utils.py                  # 로깅 + 타이밍
│   ├── TODO.md                   # 실측·연동 필요 항목
│   └── prep/                     # 학습 데이터 준비 (로봇 구동과 무관, 오프라인)
│       ├── camera.py             # 카메라 추상화 — 카메라 교체 시 여기만 손댄다
│       ├── check_setup.py        # 실행 전 환경 점검
│       ├── capture_dataset.py    # MOG2 자동 검출·라벨링으로 YOLO 데이터셋 수집
│       ├── review_labels.py      # 자동 라벨 수동 검수 (PC에서)
│       ├── prepare_dataset.py    # 세션 단위 train/val/test 분할 + dataset.yaml
│       ├── train_yolo.py         # YOLOv8n 전이학습 (PC/Colab)
│       ├── measure_sigma_w.py    # (구) σ_w 측정 — 합격 기준 폐기, 참고용만
│       └── TROUBLESHOOTING.md    # 실기 작업 기록 — 겪은 문제와 해결
│
├── pico/                         # 라즈베리파이 피코 (C, Pico SDK)
│   ├── main.c                    # 1ms 실시간 제어 루프
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
└── data_collection/              # ⚠ 실험용 — 현재 파이프라인에서 사용하지 않음
    ├── brain/                    # 팀원이 별도로 작성한 수집 도구
    └── tools/                    # 크롭 기반 이진분류 데이터셋을 만든다 (YOLO 라벨 아님)
```

> **`data_collection/`은 pi5와 완전히 독립**이며 import 관계가 없다. 같은 목적(투척 물체
> 자동 라벨링)을 다르게 구현한 것으로, `brain/planning/trajectory.py`는 `pi5/trajectory.py`와
> **같은 수식**이다. 다만 이쪽에만 있는 것이 몇 개 있어 남겨뒀다:
> 축소 해상도 MOG2(더 빠름), 추적+포물선 탄도 게이트, 물체 단위 누수 검증 분할,
> **카메라 없이 돌리는 합성 투척 생성기**(`SyntheticThrowSource`).

> `pi/kalman.py`(구 버전 3D 포물선 칼만필터)는 **삭제됐다** (2026-08-10). 현재
> `trajectory.py`는 매 프레임 배치 최소제곱으로 처음부터 다시 푼다 — 이 규모에선 연산이
> 마이크로초라 아낄 게 없고, 배치 피팅은 물리를 매 프레임 정확히 강제해서 프로세스
> 노이즈가 포물선을 서서히 밀어내는 일이 없다.

## 파일별 상세 설명

### pi5/ (Python) — 실시간 캐치

| 파일 | 담당 기능 |
|---|---|
| `main.py` | 프레임 단위 루프 하나로 전체를 돌린다: 캡처→검출→관측 누적→재피팅→착지 예측→오도메트리 수신→목표 속도 전송. 물체를 `TRACK_MAX_GAP_S` 이상 놓치면 트랙을 리셋하고 로봇을 세운다. 어떤 경로로 빠져나가도 `finally`에서 정지 명령을 보낸다 |
| `config.py` | 카메라 내부파라미터(`CAMERA_FX/FY/CX/CY`, 왜곡, 모델), 해상도·fps, YOLO 경로·임계값·단일 클래스, 궤적 파라미터(`MIN_OBSERVATIONS`, `MIN_TIME_SPAN_S`, `MAX_RESIDUAL_PX`, `DEPTH_STABILITY_RATIO`, `Z_RANGE_M`), 로봇 최대속도·워치독·도착 허용오차, 시리얼 설정. **실측으로 정한 값에는 근거 표가 주석으로 붙어 있다** |
| `vision.py` | ① Picamera2 캡처 (센서 타임스탬프 사용 — 파이썬 수신 시각은 스케줄링 지터가 섞여 궤적 피팅이 그걸 운동으로 읽는다) ② Hailo NPU로 YOLO 추론 ③ 신뢰도 최고 검출 하나의 **bbox 중심**만 취함 ④ `cv2.undistortPoints`로 그 **점 하나만** 왜곡 보정 (프레임 전체를 펴는 건 낭비다). bbox 크기는 로그용으로만 남긴다 |
| `trajectory.py` | ★ 핵심. `fit_trajectory`(2N×6 선형 최소제곱으로 위치·속도 6개 동시 추정), `landing_time`/`predict_landing`(캐치 평면 통과 시각·좌표), `Tracker`(관측 누적 + 매 프레임 재피팅 + **깊이 수렴 판정**). 좌표계 정의도 이 파일 상단에 있다 |
| `control.py` | 착지점 − 오도메트리 = 남은 거리, 나누기 남은 시간 = 목표 속도. 최대속도 초과 시 **크기만 깎고 방향은 보존**(포화 상태에서는 세 바퀴 속도 비율이 깨져 진행 방향까지 틀어진다). 도착 허용오차 안이면 정지 |
| `communication.py` | `[START 0xAA][LEN][PAYLOAD][CHECKSUM]` 프레임. 송신 `<fff>`=목표 vx,vy,워치독 / 수신 `<ffffff>`=오도메트리. 수신은 **논블로킹**이고 버퍼에 쌓인 것 중 최신만 쓴다 — 여기서 기다리면 카메라 프레임을 놓치고 그건 곧 관측 손실이다 |
| `utils.py` | `run.log` append + 콘솔 동시 출력. `log_cycle`이 한 사이클의 판단 근거(관측수, 잔차, 깊이, 착지점, 남은시간, 오도메트리, 명령)를 한 줄로 남긴다 — 예측이 이상할 때 "궤적이 안 맞나(잔차↑) 관측이 부족한가(n↓)"를 로그만으로 구분하기 위함 |

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
| `main.c` | 1ms 고정 주기(`sleep_until`) 루프: 엔코더 읽기 → 바퀴속도 변환 → 순기구학+칼만으로 오도메트리 갱신 → 파이 명령 확인 → **받은 속도를 그대로 사용**(나눗셈 없음) → 안전 클램프 → 역기구학 → PID → PWM → 오도메트리 회신 |
| `config.h` | 모터별 핀 배치, 로봇 물리 상수(휠 지름·장착각·중심거리·기어비), 엔코더 스펙, 제어주기, PID 게인, `MAX_BODY_SPEED_MPS` |
| `encoder/encoder_pio.c/.h`, `encoder.pio` | PIO로 A상 상승엣지만 카운트하고 그 순간 B상 레벨(`JMP PIN`)로 방향 판단하는 1x 쿼드러처 디코딩. CPU 개입 없이 하드웨어가 누적 |
| `kinematics.c/.h` | 역기구학(body 속도 → 바퀴 3개 선속도)과 순기구학(3x3 역행렬). 실행 검증 완료 |
| `odometry_kalman.c/.h` | vx,vy,omega 축별 독립 1D 칼만으로 엔코더 양자화 노이즈를 스무딩하고, body-frame 속도를 로봇 방향(theta) 기준으로 world-frame으로 회전 변환해 적분 |
| `motor_control.c/.h` | 바퀴별 PID → 부호에 따라 BTS7960의 RPWM/LPWM 중 하나에 PWM 출력 |
| `communication.c/.h` | 논블로킹 상태머신으로 프레임 수신, 오도메트리 송신. **payload 의미가 좌표에서 속도로 바뀌었다** — `pi5/communication.py`와 반드시 같이 봐야 한다 |

## 파이5 ↔ 피코 역할 분담

프로토콜 변경(2026-08-10)으로 책임이 명확해졌다.

| | 파이5 | 피코 |
|---|---|---|
| 궤적 추정 | ✅ | |
| 착지 예측 | ✅ | |
| 남은 거리 계산 (오도메트리 차감) | ✅ | |
| 목표 속도 산출 | ✅ | |
| 최대속도 클램프 | ✅ (1차) | ✅ (2차 안전) |
| 오도메트리 적분 | | ✅ |
| 역기구학 + PID + PWM | | ✅ |

**파이5가 계획을 전담하고 피코는 "이 속도로 돌려라"만 실행한다.** 이전에는 피코가
`목표좌표 ÷ 구동시간`으로 속도를 계산했는데, 자기 오도메트리를 빼지 않아 매 사이클
처음부터의 거리를 다시 가려 했고(구조적 오버슈트), 구동시간이 짧게 잘려 목표속도가
물리 한계를 넘어 PID가 영구 포화됐다.

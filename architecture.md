# 소프트웨어 아키텍처

## 파일 구조

```
LABA6_toyproject/
├── README.md
├── architecture.md               # 이 문서
├── algorithm.md                  # 관측·예측 로직, 재보정 루프, vx/vy/vz 정의
├── system_flow.md                # pi5-pico 전체 파이프라인 흐름 + 구현 상태 표
├── docs/
│   └── coordinate_system.md      # 카메라/세계/로봇 좌표 정의 (아직 미작성)
│
├── pi5/                          # 라즈베리파이5 (Python)
│   ├── main.py                   # 관측·예측 → 재보정 루프 오케스트레이션
│   ├── config.py                 # 설정값 + 클래스별 기준사이즈 DB
│   ├── vision.py                 # 카메라캡처 + YOLO(Hailo)/z추정 + 이동벡터
│   ├── trajectory.py             # 궤적 예측 + 재보정 계산
│   ├── communication.py          # USB 시리얼 왕복 송수신 + 프로토콜
│   ├── control.py                # 목표좌표 산출 + (추후) 안전장치
│   ├── utils.py                  # 로깅 + 타이밍
│   └── TODO.md                   # 실측·연동 필요 항목 정리
│
└── pico/                         # 라즈베리파이 피코 (C, Pico SDK)
    ├── CMakeLists.txt            # 빌드 설정
    ├── pico_sdk_import.cmake     # Pico SDK 부트스트랩 (SDK 설치 후 원본으로 교체 권장)
    ├── main.c                    # 실시간 제어 메인 루프
    ├── config.h                  # 핀 배치, 물리 상수, PID 게인
    ├── encoder/
    │   ├── encoder_pio.c/.h      # PIO 기반 A/B상 엔코더 카운팅
    │   └── encoder.pio           # PIO 프로그램 (별도 빌드 대상이라 분리 유지)
    ├── kinematics.c/.h           # 순기구학 + 역기구학 통합
    ├── odometry_kalman.c/.h      # 엔코더 속도 노이즈 필터링 + 오도메트리
    ├── motor_control.c/.h        # PID + 모터드라이버 통합
    ├── communication.c/.h        # 파이와 왕복 USB 시리얼 통신 + 프로토콜
    └── TODO.md                   # 실측·연동 필요 항목 정리
```

> `pi/kalman.py`(구 버전 3D 포물선 칼만필터)는 재구조화 이전 프로토타입 폴더에 남은
> 파일로, 현재 `pi5/` 어디에서도 사용하지 않는다. `trajectory.recalibrate()` 설계 시
> 재활용할지 별도로 새로 설계할지 아직 결정 전.

## 파일별 상세 설명

### pi5/ (Python)

| 파일 | 담당 기능 |
|---|---|
| `main.py` | 전체 실행 순서 제어: 카메라·시리얼 초기화 → 관측·예측 파트 1회 실행 → 재보정 루프 반복 실행 → z≤0 도달 시 종료 |
| `config.py` | 카메라 해상도/fps/샘플링 간격, YOLO 모델 경로·신뢰도 임계값·대상 클래스, 클래스별 1m 기준 사이즈 DB, 확정 프레임 수(`CONFIRM_FRAMES`), 이동벡터 계산 프레임 수(`VELOCITY_SAMPLE_FRAMES`), 카메라 초점거리, 물리상수(중력·캐치높이), 시리얼 포트, 재보정 구동시간 등 파라미터 전체를 상수로 정의 |
| `vision.py` | ① 카메라에서 설정된 fps로 프레임 샘플링 ② Hailo NPU로 YOLO 추론해 클래스·바운딩박스·신뢰도 획득 ③ 바운딩박스 크기를 기준 사이즈와 비교해 z(높이) 계산, 픽셀 오프셋을 초점거리 기반 핀홀 모델로 실제 x,y(m) 변환 ④ 같은 클래스가 `CONFIRM_FRAMES` 연속 감지되면 확정 ⑤ 확정 후 `VELOCITY_SAMPLE_FRAMES`장 캡처해 첫·마지막 프레임 위치차로 vx,vy(등속 가정)·vz(등가속도 보정, algorithm.md 참고) 계산 |
| `trajectory.py` | ① `predict_landing`: (위치, 속도)로 포물선 공식을 이용해 착지점·도달시간 예측 (완성) ② `recalibrate`: 재보정 루프에서 엔코더 이동량과 카메라 재관측값의 오차를 반영해 궤적을 다시 계산 — **미설계, 현재 `NotImplementedError`** |
| `communication.py` | ① 피코와의 USB CDC 시리얼 연결 ② `[START｜LEN｜PAYLOAD｜CHECKSUM]` 프레임으로 목표좌표+구동시간을 인코딩해 송신 ③ 피코가 보낸 오도메트리(위치+속도) 프레임을 수신·디코딩 ④ 디버깅용 raw 바이트 → 텍스트 변환 도구(`debug_decode`) 포함 |
| `control.py` | 착지점에서 로봇이 이동할 x,y 거리를 그대로 뽑아내고(좌표 변환 불필요), 재보정 사이클마다 구동시간을 `RECAL_DRIVE_TIME_S` 이내로 짧게 끊어 결정. (추후) 재보정 루프 최대 반복횟수·타임아웃 체크도 이 파일에서 담당 |
| `utils.py` | 단일 파일(`run.log`) append 로그 + 콘솔 동시 출력. 검출 결과·좌표·속도 등 디버깅 정보 기록, 프레임 캡처 시각(timestamp) 관리 — Δt 계산의 기준값을 제공 |

### pico/ (C, Pico SDK)

| 파일 | 담당 기능 |
|---|---|
| `main.c` | 초기화(GPIO, PIO, USB CDC) 후 고정 주기(1ms, `sleep_until` 기반 정밀 타이밍)로 도는 실시간 제어 루프: 엔코더 읽기 → 바퀴속도 변환 → 순기구학+칼만 필터로 오도메트리 갱신 → 파이 명령 확인(구동시간 만료 시 자동 정지) → 목표 body 속도 계산 → 역기구학 → PID → PWM 출력 → 오도메트리 회신을 매 사이클 반복 |
| `config.h` | 모터별 핀 배치(rpwm/lpwm/en 공통/enc_a/enc_b), 로봇 물리 상수(휠 지름, 바퀴 장착각·중심거리, 기어비), 엔코더 스펙, 제어주기, 모터별 PID 게인을 배열(모터 인덱스 0~2)로 정의 |
| `encoder/encoder_pio.c/.h`, `encoder.pio` | PIO 하드웨어로 A상 상승엣지만 카운트하고 그 순간 B상 레벨(`JMP PIN`)로 방향을 판단하는 1x 쿼드러처 디코딩. CPU 개입 없이 하드웨어가 카운트를 누적하며, 상위 코드는 `encoder_get_count()`로 값만 읽어감 |
| `kinematics.c/.h` | ① 역기구학: 목표 body 속도(vx,vy,omega) → 바퀴 3개 목표 선속도 ② 순기구학: 바퀴 3개 선속도 → body 속도(vx,vy,omega), 역기구학 행렬의 3x3 역행렬(여인수/수반행렬 공식)로 계산. 실행 검증 완료 (2026-07-31) |
| `odometry_kalman.c/.h` | vx,vy,omega 축별 독립 1D 칼만필터로 순기구학 결과에 낀 엔코더 양자화 노이즈를 스무딩하고, body-frame 속도를 로봇 현재 방향(theta) 기준으로 world-frame으로 회전 변환해 적분, 위치(x,y,theta) 오도메트리 갱신 |
| `motor_control.c/.h` | ① PID: 바퀴별 목표속도와 엔코더 기반 실제속도를 비교해 오차 기반 출력 계산 ② 출력 부호에 따라 BTS7960의 RPWM/LPWM 중 하나에 PWM 출력, EN 핀은 상시 활성화 |
| `communication.c/.h` | 파이와 USB CDC로 왕복 통신: `[START｜LEN｜PAYLOAD｜CHECKSUM]` 프레임 수신 시 목표좌표+구동시간을 파싱(non-blocking 상태머신), 송신 시 오도메트리(위치+속도)를 같은 프레임 포맷으로 인코딩해 전송 |
| `CMakeLists.txt`, `pico_sdk_import.cmake` | Pico SDK 빌드 설정. 실제 빌드하려면 Pico SDK 설치 + `PICO_SDK_PATH` 환경변수 설정 필요 (아직 미설치) |

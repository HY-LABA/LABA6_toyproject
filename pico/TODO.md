# pico 남은 작업

> 채워야 할 값과 실측 항목. 제어 설계 설명은 [`../docs/pico-control.md`](../docs/pico-control.md).

## 1. 부품 도착·배선 후 값 채우기 (config.h)

MicroPython 벤치 테스트(`pico_micropython/goto_xy_test.py`, 2026-09)로 아래 값들
확정 완료, config.h에 반영됨:

- [x] `MOTOR_PINS[3]` — rpwm/lpwm/enc_a/enc_b (en은 3.3V 직결이라 필드 자체를 없앰)
- [x] `ENCODER_COUNTS_PER_REV` = 687.5 (1체배 PIO 기준)
- [x] `WHEEL_ANGLES_RAD[3]` — 배치 확정(M1=90°/전방, M2=330°/우측 뒤, M3=210°/좌측 뒤)
      MicroPython 벤치에서 도면으로 실물 대조까지 완료.
      **반경(`WHEEL_MOUNT_RADIUS_M`)은 아직 이론값(0.15m) — 자로 실측 필요**
- [x] `MOTOR_SIGN[3]` — 모터 3개 다 -1 (원인 불명, MicroPython 테스트로 발견·보정)
- [x] `MOTOR_PID[3]` — {60, 40, 0} (MicroPython 50Hz 기준 검증값. `CONTROL_PERIOD_MS`도
      1kHz에서 20ms로 맞춤 — Ki/Kd가 dt에 비례하므로 검증 안 된 주기로 쓰면 거동이 달라짐)
- [ ] **`MAX_BODY_SPEED_MPS`** — 아직 이론값(1.22). 몸체 완성 후 실측 필요.
      **`pi5/config.py`의 `ROBOT_MAX_SPEED_MPS`와 같은 값으로 맞출 것**

## 1-2. 프로토콜 변경 — 목표점 명령 받기 (완료, 2026-09)

파이5가 보내는 **월드 좌표 도착 지점**(16 B)을 이제 피코가 받는다. 계약 전문은
[`../docs/protocol.md`](../docs/protocol.md) 2장.

- [x] `communication.c`/`.h` — 16B(`RX_CMD_TARGET`)/12B(`RX_CMD_VELOCITY`) 둘 다
      수용. `RxCommand`에 `kind`를 태그로 둬서 프레임 길이로 구분한다. 다른 길이는
      예전처럼 조용히 버림(그 성질 유지)
- [x] `main.c` — `TargetCommand` 처리: `남은거리 = target − 자기 pose`,
      `v = 남은거리 ÷ 남은시간`(상한 `MAX_BODY_SPEED_MPS`), 도착하면 정지.
      12B 속도 경로(`teleop_test.py`/STOP)는 그대로 유지 — 최근 받은 쪽이 활성 모드
- [x] `POSITION_TOLERANCE_M` = 0.02 추가 (`pi5/config.py`와 값 맞춤)
- [ ] **실기 빌드로 아직 검증 안 됨** — SDK 설치 후 빌드·플래시해서 실제로 목표점
      명령에 반응하는지 확인 필요
- [ ] 상한이 아직 body 속력(`MAX_BODY_SPEED_MPS`) 기준이다 — 1-3번(바퀴 기준 클램프)
      이 끝나면 그쪽으로 교체할 것. 지금은 안전하게 도는 것 우선, 최적은 나중

## 1-3. ★ 속도 클램프를 바퀴 기준으로 옮기기 (지금 버그)

`main.c`가 body 속력을 `MAX_BODY_SPEED_MPS`(1.22)로 자른다. 그런데 모터가 제한하는 건
바퀴 속도이고, 3륜 옴니는 방향에 따라 body 1 m/s에 필요한 바퀴 속도가 0.866~1.000으로
다르다. 그래서 유리한 방향(0°, 90° 등)에서 실제 낼 수 있는 1.41 m/s를 1.22로 깎고 있다.

**단순한 성능 손실이 아니다.** 파이5의 `tracker._reach()`는 1.41 × REACH_MARGIN 1.15 =
1.62 m/s까지 도달 가능하다고 믿는다 — 실제 한계의 **33% 낙관**이다. 그 결과 못 잡을
표적을 채택하고, 그 사이 잡을 수 있었던 걸 놓친다.

- [ ] 클램프를 `inverse_kinematics` **뒤**로 옮기고 `max|wheel_target|`이 바퀴 한계를
      넘으면 세 값을 같은 비율로 줄인다 (방향 보존은 그대로)

## 2. 모터 돌려보며 실측할 것 ★ 도착하면 제일 먼저

세 가지를 한 번에 잰다. 오도메트리가 이미 파이로 회신되니 로그만 찍으면 된다.

- [ ] **최대속도** — PWM 100%로 직진, 엔코더 속도가 평평해지는 값
- [ ] **가속 시간** — 0에서 최대속도까지 몇 초.
      궤적 예측이 착지 0.25초 전에 나오므로(`algorithm.md` 4장), 그 안에 못 서면
      제어 전략을 다시 봐야 한다
- [ ] **정지 거리(오버슈트)** — 최대속도에서 명령 끊고 몇 cm 더 가는지.
      **쓰레기통 입구 폭보다 크면 아예 못 받는다** — 이게 감속 프로파일이 따로
      필요한지를 결정한다

## 3. 튜닝

- [x] `main.c`의 `wheel_speed_from_encoder_delta` — 변환식 완성, 값 채움
      (GEAR_RATIO 중복 나눗셈 버그도 같이 발견·수정 — ENCODER_COUNTS_PER_REV가
      이미 바퀴축 기준이라 또 나누면 안 됨)
- [x] `motor_control.c` — PID output → PWM 듀티 스케일 (MicroPython 벤치값으로 확정,
      안티와인드업·데드밴드 보상도 같이 이식)
- [ ] `odometry_kalman.c`의 `PROCESS_NOISE`/`MEASURE_NOISE` — 스무딩 정도
- [ ] 이 값들은 전부 MicroPython 50Hz 루프(DriveCommand/속도 경로)에서 검증된 것 —
      **실기 C 빌드로 최종 확인 필요.** 1-2번 TargetCommand 경로가 완성되면 그쪽
      기준으로도 재확인할 것

## 4. 확인 필요 (하드웨어 스펙)

- [x] `encoder_pio.c`의 `gpio_pull_up` — MicroPython 벤치 테스트에서 3.3V 내부
      풀업 직결로 3개 모터 전부 정상 카운트 확인됨. 그대로 유지

## 5. 빌드 설정

- [x] `CMakeLists.txt` / `pico_sdk_import.cmake` 작성 완료
- [ ] 실제 Pico SDK 설치 + `PICO_SDK_PATH` 설정 후 빌드 검증
      (`main.c`에 `sqrtf` 클램프가 추가됐다 — `config.h`가 `<math.h>`를 포함하므로
      링크는 되어야 하지만 실기 빌드로 확인할 것)

## 6. 구현 완료 항목

- [x] **목표점(16B) 경로 — 피코가 남은거리÷남은시간으로 직접 속도 계산** (1-2번)
- [x] **속도(12B) 경로 — 파이가 보낸 속도를 그대로 사용** (teleop/STOP 전용,
      피코는 여기선 나눗셈을 하지 않는다)
- [x] **최대속도 안전 클램프** — 크기만 깎고 방향은 보존한다. 포화 상태에서는 세
      바퀴의 속도 비율이 깨져 진행 방향까지 틀어지기 때문
- [x] `timeout_s` 워치독 — 정상 동작 중엔 매 프레임 새 명령이 오므로 만료되지 않는다.
      만료 = 파이가 죽었거나 링크가 끊긴 것 → 정지

> ⚠ `communication.c`의 payload는 **길이(LEN)로 종류가 갈린다** — 16B(목표점)와
> 12B(속도) 둘 다 유효하고 의미가 다르다. `pi5/communication.py`와 반드시 같이 볼 것.
> 경위는 `../CHANGELOG.md`, 계약 전문은 `../docs/protocol.md`.

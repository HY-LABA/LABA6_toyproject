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
- [x] `MOTOR_PID[3]` — **{0.6, 0.4, 0}**. MicroPython 50Hz 검증값 {60, 40, 0}을
      **100으로 나눈 값**이다 — 벤치의 PID 출력은 duty **퍼센트(0~100)**였고 C의
      `set_pwm()`은 duty **비율(0~1)**을 받는다. 환산 안 하면 속도 오차 0.017 m/s에
      duty가 포화돼 비례 제어가 아니라 뱅뱅 제어가 된다.
      (`MIN_DRIVE_DUTY` 15.0→0.15, 안티와인드업 100/ki→1/ki 는 이미 환산돼 있었다.)
      `CONTROL_PERIOD_MS`도 1kHz에서 20ms로 맞춤 — 검증 안 된 주기로 쓰면 거동이 달라짐
- [x] `PWM_FREQ_HZ` = 10000 — 벤치 검증값(BTS7960 상한 25kHz, 저속 선형성 위해 낮춤).
      `motor_control.c`가 실제 `clk_sys`에서 분주를 역산한다. **분주를 안 걸면 이 보드
      (pico2_w = RP2350, 150MHz)에서 36.6kHz로 드라이버 상한을 넘는다**
- [ ] **`WHEEL_MAX_SPEED_MPS`** — 아직 이론값(1.22). 몸체 완성 후 실측 필요.
      **`pi5/config.py`의 `WHEEL_MAX_SPEED_MPS`와 같은 값으로 맞출 것**
      (옛 이름 `MAX_BODY_SPEED_MPS`에서 바뀌었다 — 1-3 참고)

## 1-2. 프로토콜 변경 — 목표점 명령 받기 (완료, 2026-09)

파이5가 보내는 **월드 좌표 도착 지점**(16 B)을 이제 피코가 받는다. 계약 전문은
[`../docs/protocol.md`](../docs/protocol.md) 2장.

- [x] `communication.c`/`.h` — 16B(`RX_CMD_TARGET`)/12B(`RX_CMD_VELOCITY`) 둘 다
      수용. `RxCommand`에 `kind`를 태그로 둬서 프레임 길이로 구분한다. 다른 길이는
      예전처럼 조용히 버림(그 성질 유지)
- [x] `main.c` — `TargetCommand` 처리: `남은거리 = target − 자기 pose`,
      `v = 남은거리 ÷ 남은시간`(상한은 아래 1-3의 방향별 바퀴 기준), 도착하면 정지.
      12B 속도 경로(`teleop_test.py`/STOP)는 그대로 유지 — 최근 받은 쪽이 활성 모드
- [x] `POSITION_TOLERANCE_M` = 0.02 추가 (`pi5/config.py`와 값 맞춤.
      `control.verify_wheel_config()`가 이 값도 대조한다)
- [ ] **실기 빌드로 아직 검증 안 됨** — SDK 설치 후 빌드·플래시해서 실제로 목표점
      명령에 반응하는지 확인 필요

## 1-3. 속도 클램프를 바퀴 기준으로 (완료, 2026-09)

예전엔 `main.c`가 body 속력을 `MAX_BODY_SPEED_MPS`(1.22)로 잘랐다. 그런데 모터가 제한하는 건
바퀴 속도이고, 3륜 옴니는 방향에 따라 body 1 m/s에 필요한 바퀴 속도가 0.866~1.000으로
다르다. 그래서 유리한 방향(0°, 90° 등)에서 실제 낼 수 있는 1.41 m/s를 1.22로 깎고 있다.

**단순한 성능 손실이 아니다.** 파이5의 `tracker._reach()`는 1.41 × REACH_MARGIN 1.15 =
1.62 m/s까지 도달 가능하다고 믿는다 — 실제 한계의 **33% 낙관**이다. 그 결과 못 잡을
표적을 채택하고, 그 사이 잡을 수 있었던 걸 놓친다.

- [x] `kinematics.c`에 `max_body_speed(vx, vy, omega)` 추가 — 진행 방향으로 단위 속도를
      넣어 나온 `max|wheel|`이 `WHEEL_MAX_SPEED_MPS`에 닿는 지점이 그 방향의 상한이다.
      IK 뒤에서 세 값을 같은 비율로 줄이는 것과 등가이면서 **파이의
      `control.max_body_speed()`와 같은 식**이라 로그와 실제가 갈리지 않는다.
      목표점 경로와 텔레옵(속도) 경로 둘 다 이걸 탄다.
      이름이 거짓말이던 `MAX_BODY_SPEED_MPS`는 `WHEEL_MAX_SPEED_MPS`로 개명했고,
      `pi5/control.verify_wheel_config()`도 새 이름을 파싱하도록 같이 고쳤다
      (**이름을 못 찾으면 그 자체를 불일치로 보고한다** — 예전엔 조용히 통과했다)

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
      안티와인드업·데드밴드 보상도 같이 이식). **게인 단위 환산은 1번 항목 참고**
- [x] **`communication_send_odometry`의 블로킹** (2026-09-04, 다른 세션 작업) —
      `tud_cdc_connected()` + `tud_cdc_write_available()`로 여유 확인 후 부족하면
      그 프레임을 건너뛰도록 수정. `communication_try_receive()`도 프레임 하나에서
      바로 return하지 않고 버퍼를 끝까지 비워 **마지막 완전 프레임만** 채택하게 바뀜
      (파이 쪽 `try_receive_odometry()`와 같은 방침)
- [x] `odometry_kalman.c`의 `PROCESS_NOISE`/`MEASURE_NOISE` (2026-09-04, 다른 세션 작업) —
      파라미터를 재튜닝하는 대신 **pose 적분을 raw 속도로 되돌렸다.** 이 조합(Q=0.01,
      R=1.0, dt=0.02)의 칼만 시정수가 약 1.4초로 나오는데, 캐치 기동은 0.5초라 필터를
      통과한 속도로 pose를 적분하면 실제 이동량을 구조적으로 과소보고한다(0.5초 지점
      기준 41cm 과소보고 — GEAR_RATIO 이중나눗셈 버그와 같은 계열의 오버슈트 원인).
      MicroPython 벤치는 애초에 이 필터 없이 순수 오일러 적분으로 검증됐으므로, 검증된
      동작으로 되돌리는 쪽을 택함. 칼만필터는 파이5로 보고하는 속도(텔레메트리) 스무딩
      용도로만 남음. 스무딩을 pose에 다시 넣고 싶어지면 새 파라미터의 시정수가
      0.1초 이하인지 먼저 확인할 것
- [x] **모드 전환 시 `motor_control_reset()` 호출 누락** (2026-09-04, 다른 세션 발견·수정) —
      PID 적분 상태가 모드(목표점/속도) 전환 시 안 지워지고 있었다. 정지해 있다 다음
      명령 첫 틱에 쌓여있던 적분항 때문에 전력으로 튀어나갈 수 있는 버그. 벤치는
      명령마다 `pid_reset()`을 불렀지만, 여기선 TargetCommand가 프레임마다(30~60Hz)
      갱신되므로 "명령마다"가 아니라 **모드가 실제로 바뀔 때만**(+워치독 만료 시)
      리셋하도록 대응시킴 — 매 프레임 리셋하면 Ki가 정지마찰 극복하려고 쌓이는 걸
      계속 지워버려서 목표 근처에서 못 감
- [x] **world→body 회전 누락** (2026-09-04, 다른 세션 발견·수정) — `main.c`가 목표
      방향(`rx,ry`, world frame)을 회전 없이 그대로 `inverse_kinematics`(body frame
      기대)에 넣고 있었다. theta≈0 가정으로 생략됐던 건데, theta는 리셋 수단이 없어
      엔코더 노이즈만으로도 부팅 후 계속 드리프트한다. 벤치의 `drive_to()`에 있던
      R(-theta) 회전을 추가. 방향별 바퀴 속도 상한(`max_body_speed`) 계산도 회전
      **후** 값을 넣도록 같이 수정 — 바퀴 포화는 body 방향 기준이라 예전엔 theta≠0일 때
      상한 자체가 틀렸음
- [ ] 이 값들은 전부 MicroPython 50Hz 루프(DriveCommand/속도 경로)에서 검증된 것 —
      **실기 C 빌드로 최종 확인 필요.** 위 4건은 코드 리뷰로 잡은 것이지 실기로
      확인된 게 아니다 — pi5-피코 실측 왕복 테스트가 우선순위 1번(architecture.md 5장)

## 4. 확인 필요 (하드웨어 스펙)

- [x] `encoder_pio.c`의 `gpio_pull_up` — MicroPython 벤치 테스트에서 3.3V 내부
      풀업 직결로 3개 모터 전부 정상 카운트 확인됨. 그대로 유지

## 5. 빌드 설정

- [x] `CMakeLists.txt` / `pico_sdk_import.cmake` 작성 완료
- [x] Pico SDK 설치 + 빌드 검증 (2026-09) — VS Code Pico 확장으로 SDK 2.3.0 설치,
      `cmake -G Ninja` + `ninja`로 98/98 빌드 성공(에러 0), UF2 생성·플래싱 확인.
      이후 위 3번 4건 수정도 재빌드 통과(경고 0). **아직 실물 로봇 구동 테스트는 안 함**

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

# pico 남은 작업

> 2026-08-10 갱신. 프로토콜이 좌표 전송 → **속도 전송**으로 바뀌었다 (`algorithm.md` 3장).

## 1. 부품 도착·배선 후 값 채우기 (config.h)

MicroPython 벤치 테스트(`pico_micropython/goto_xy_test.py`, 2026-09)로 아래 값들
확정 완료, config.h에 반영됨:

- [x] `MOTOR_PINS[3]` — rpwm/lpwm/enc_a/enc_b (en은 3.3V 직결이라 필드 자체를 없앰)
- [x] `ENCODER_COUNTS_PER_REV` = 687.5 (1체배 PIO 기준)
- [x] `WHEEL_ANGLES_RAD[3]` — 각도 순서 확정(90/330/210도, 도면으로 실물 대조).
      **반경(`WHEEL_MOUNT_RADIUS_M`)은 아직 이론값(0.15m) — 자로 실측 필요**
- [x] `MOTOR_SIGN[3]` — 모터 3개 다 -1 (원인 불명, MicroPython 테스트로 발견·보정)
- [x] `MOTOR_PID[3]` — {60, 40, 0} (MicroPython 50Hz 기준 검증값. `CONTROL_PERIOD_MS`도
      1kHz에서 20ms로 맞춤 — Ki/Kd가 dt에 비례하므로 검증 안 된 주기로 쓰면 거동이 달라짐)
- [ ] **`MAX_BODY_SPEED_MPS`** — 아직 이론값(1.22). 몸체 완성 후 실측 필요.
      **`pi5/config.py`의 `ROBOT_MAX_SPEED_MPS`와 같은 값으로 맞출 것**

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
- [x] `motor_control.c` — PID output → PWM 듀티 스케일 (MicroPython 벤치값으로 확정,
      안티와인드업·데드밴드 보상도 같이 이식)
- [ ] `odometry_kalman.c`의 `PROCESS_NOISE`/`MEASURE_NOISE` — 스무딩 정도
- [ ] 이 값들은 전부 MicroPython 50Hz 루프에서 검증된 것 — **실기 C 빌드로 최종
      확인 필요** (특히 PID는 같은 dt를 써도 PIO 인터럽트 특성이 달라 재확인 권장)

## 4. 확인 필요 (하드웨어 스펙)

- [x] `encoder_pio.c`의 `gpio_pull_up` — MicroPython 벤치 테스트에서 3.3V 내부
      풀업 직결로 3개 모터 전부 정상 카운트 확인됨. 그대로 유지

## 5. 빌드 설정

- [x] `CMakeLists.txt` / `pico_sdk_import.cmake` 작성 완료
- [ ] 실제 Pico SDK 설치 + `PICO_SDK_PATH` 설정 후 빌드 검증
      (`main.c`에 `sqrtf` 클램프가 추가됐다 — `config.h`가 `<math.h>`를 포함하므로
      링크는 되어야 하지만 실기 빌드로 확인할 것)

## 6. 완료 (2026-08-10)

- [x] **좌표 → 속도 프로토콜 전환.** 피코가 `target_x / drive_time_s`로 속도를
      계산하던 것을 제거. 자기 오도메트리를 빼지 않아 매 사이클 처음부터의 거리를
      다시 가려 했고(구조적 오버슈트), 구동시간이 0.1초로 잘려 목표속도가 최대속도의
      3배가 되어 PID가 영구 포화됐다. 이제 파이5가 남은거리÷남은시간으로 계산해 보낸다
- [x] **최대속도 안전 클램프** — 크기만 깎고 방향은 보존한다. 포화 상태에서는 세
      바퀴의 속도 비율이 깨져 진행 방향까지 틀어지기 때문
- [x] `timeout_s` 워치독 — 정상 동작 중엔 매 프레임 새 명령이 오므로 만료되지 않는다.
      만료 = 파이가 죽었거나 링크가 끊긴 것 → 정지

> ⚠ `communication.c`의 payload 의미가 바뀌었다. 바이트 수(12B)가 같아서
> **한쪽만 고치면 조용히 엉뚱한 속도로 돈다.** `pi5/communication.py`와 반드시 같이 볼 것.

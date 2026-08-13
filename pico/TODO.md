# pico 남은 작업

> 채워야 할 값과 실측 항목. 제어 설계 설명은 [`../docs/pico-control.md`](../docs/pico-control.md).

## 1. 부품 도착·배선 후 값 채우기 (config.h)

- [ ] `MOTOR_PINS[3]` — 모터 3개 각각 rpwm/lpwm/en/enc_a/enc_b 핀 번호
- [ ] `ENCODER_COUNTS_PER_REV` — 엔코더 실물 스펙(1회전당 펄스 수)
- [ ] `WHEEL_ANGLES_RAD[3]` / `WHEEL_MOUNT_RADIUS_M` — 조립 후 자·각도기로 실측
      (지금은 이론값 90/210/330도, 0.15m)
- [ ] `MOTOR_PID[3]` — 모터 실측 튜닝 (지금은 3개 다 동일 placeholder)
- [ ] **`MAX_BODY_SPEED_MPS`** — 지금 1.22는 이론값
      (FIT0186 251RPM × 0.925 부하감쇠 × π × 0.1m).
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

- [x] `main.c`의 `wheel_speed_from_encoder_delta` — 변환식 완성, 값만 채우면 됨
- [ ] `motor_control.c` — PID output(속도 오차) → PWM 듀티(0~1) 스케일 계수
- [ ] `odometry_kalman.c`의 `PROCESS_NOISE`/`MEASURE_NOISE` — 스무딩 정도

## 4. 확인 필요 (하드웨어 스펙)

- [ ] `encoder_pio.c`의 `gpio_pull_up` — 엔코더가 오픈드레인인지 푸시풀인지 확인 후
      필요 없으면 제거

## 5. 빌드 설정

- [x] `CMakeLists.txt` / `pico_sdk_import.cmake` 작성 완료
- [ ] 실제 Pico SDK 설치 + `PICO_SDK_PATH` 설정 후 빌드 검증
      (`main.c`에 `sqrtf` 클램프가 추가됐다 — `config.h`가 `<math.h>`를 포함하므로
      링크는 되어야 하지만 실기 빌드로 확인할 것)

## 6. 구현 완료 항목

- [x] **파이5가 보낸 속도를 그대로 사용** (피코는 나눗셈을 하지 않는다)
- [x] **최대속도 안전 클램프** — 크기만 깎고 방향은 보존한다. 포화 상태에서는 세
      바퀴의 속도 비율이 깨져 진행 방향까지 틀어지기 때문
- [x] `timeout_s` 워치독 — 정상 동작 중엔 매 프레임 새 명령이 오므로 만료되지 않는다.
      만료 = 파이가 죽었거나 링크가 끊긴 것 → 정지

> ⚠ `communication.c`의 payload는 **속도**(`target_vx, target_vy, timeout_s`)다.
> 예전 좌표 방식과 바이트 수(12B)가 같아서 **한쪽만 옛 의미로 해석하면 조용히 엉뚱한
> 속도로 돈다.** `pi5/communication.py`와 반드시 같이 볼 것. 경위는 `../CHANGELOG.md`.

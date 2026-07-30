# pico 남은 작업

## 1. 부품 구매·배선 후 값 채우기 (config.h)

- [ ] `MOTOR_PINS[3]` — 모터 3개 각각 rpwm/lpwm/en/enc_a/enc_b 핀 번호
- [ ] `ENCODER_COUNTS_PER_REV` — 엔코더 실물 스펙(1회전당 펄스 수)
- [ ] `WHEEL_ANGLES_RAD[3]` / `WHEEL_MOUNT_RADIUS_M` — 로봇 조립 후 자/각도기로 실측 (지금은 이론값 90/210/330도, 0.15m)
- [ ] `MOTOR_PID[3]` — 모터 실측 튜닝 (지금은 3개 다 동일 placeholder)

## 2. 실제로 모터 돌려보며 튜닝해야 하는 값

- [x] `main.c`의 `wheel_speed_from_encoder_delta` — 변환식 구현 완료, `ENCODER_COUNTS_PER_REV` 값만 채우면 됨
- [ ] `motor_control.c` — PID output(속도 오차 기반) → PWM 듀티(0~1) 변환 스케일 계수
- [ ] `odometry_kalman.c`의 `PROCESS_NOISE`/`MEASURE_NOISE` — 칼만필터 스무딩 정도 튜닝

## 3. 정책 결정 필요

- [x] `main.c` — `drive_time_s` 만료 후 정지 정책 구현 완료 (새 명령 없으면 자동 정지)

## 4. 확인 필요 (하드웨어 스펙)

- [ ] `encoder_pio.c`의 `gpio_pull_up` — 엔코더가 오픈드레인인지 푸시풀인지 확인 후 필요 없으면 제거

## 5. 빌드 설정

- [x] `CMakeLists.txt` / `pico_sdk_import.cmake` 작성 완료
- [ ] 실제 Pico SDK 설치 + `PICO_SDK_PATH` 설정 후 빌드 검증

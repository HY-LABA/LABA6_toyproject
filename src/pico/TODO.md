# pico 남은 작업

> 2026-09-15 갱신. **지금 남은 것만** 적는다. 제어 설계는 [`../docs/design/pico-control.md`](../../docs/design/pico-control.md),
> 통신 계약은 [`../docs/design/protocol.md`](../../docs/design/protocol.md), 경위(버그 수정 이력)는 [`../CHANGELOG.md`](../../CHANGELOG.md).
>
> 펌웨어가 두 판이다 — C(`src/pico/`, UF2) 와 MicroPython(`src/pico_micropython/`, Thonny).
> 값을 바꾸면 **`src/pico/config.h`, `src/pico_micropython/config.py`, `src/pi5/config.py` 셋을 같이** 본다.

## 1. 확정된 값

- [x] `MOTOR_PINS[3]`, `ENCODER_COUNTS_PER_REV = 687.5` (바퀴축 기준 1체배 — `GEAR_RATIO` 로 또 나누지 말 것)
- [x] `WHEEL_ANGLES_RAD` — M1=90°(전방), M2=330°, M3=210°. 몸체 실물 대조 (2026-09-07)
- [x] `WHEEL_MOUNT_RADIUS_M = 0.15` — 실측 약 15 cm, 그대로 둠. `omega` 를 안 쓰므로 영향 없음
- [x] `MOTOR_PID = {0.6, 0.4, 0}`, `MIN_DRIVE_DUTY = 0.15`, `PWM_FREQ_HZ = 10000`, `CONTROL_PERIOD_MS = 20` — 벤치 검증값
- [x] `POSITION_TOLERANCE_M = 0.02` — pi5와 일치
- [x] 실물 로봇 구동 — `src/pi5/main.py` 목표점 명령으로 주행 확인 (2026-09-09~)

## 2. 아직 확정 안 된 값 ★

- [ ] **`MOTOR_SIGN = {1, 1, 1}` 과 진행 방향** — 벤치에서는 `{-1,-1,-1}` 이었고 2026-09-09 `+1` 로
      바꾼 뒤 속도는 빨라졌지만 방향이 틀어졌다. 이후 파이의 `CAMERA_YAW_RAD` 를 180°→0° 로 바꿔
      실기에서 맞게 가게 됐는데, 사진 측정값은 180° 다 → **구동 쪽 반전과 카메라 설정이 상쇄 중**일 수 있다.
      카메라 없이 판정:
      ```bash
      python src/pi5/pico_test.py --port /dev/ttyACM0 --dir front --dist 0.5 --speed 0.3
      ```
      M1 방향으로 가면 정상. 반대로 가면 `MOTOR_SIGN` 을 뒤집고 **파이 `CAMERA_YAW_RAD` 도 180° 로 같이** 바꾼다
- [ ] **`WHEEL_MAX_SPEED_MPS = 1.8`** — 파이·피코 둘 다 1.8 이지만 **실측값이 아니다**
      (최대속도 시험용으로 올린 클램프, 이론값 1.22). 실제 한계보다 높으면 세 바퀴가 제각각 포화돼
      진행 방향이 틀어진다. 실측 후 세 파일 통일, C 판은 UF2 재굽기

## 3. 실측할 것

- [ ] **최대속도** — `python src/pi5/teleop_test.py --port /dev/ttyACM0 --speed-test 1.8 --speed-test-duration 1.0`
- [ ] **가속시간·정지거리** — `python src/pi5/teleop_test.py --port /dev/ttyACM0 --go-distance 1.0` /
      `python src/pi5/pico_test.py --port /dev/ttyACM0 --dir front --dist 1.0 --max`.
      정지거리가 통 입구 반지름(13 cm)을 넘으면 파이 `DRIVE_AGGRESSION` 부터 낮춘다
- [ ] **오도메트리 스케일** — 위 `--go-distance` 출력 거리와 줄자 대조 (슬립 포함)
- [ ] **M1 배선** — `front` 에서 M1 이 안 도는 건 정상(진행 방향과 수직)이다. `left`/`right` 에서도
      안 돌면 배선·드라이버 확인

## 4. 코드

- [ ] (선택) 바퀴 가속 제한(slew rate) — 전속 출발에서 슬립이 크면. 지금은 없다
- [ ] (선택) `theta` 리셋 수단 — 부팅 후 엔코더 노이즈로 드리프트한다. 파이는 `theta≈0` 을 가정한다
- [ ] (정리) `src/pi5/control.verify_wheel_config()` 는 `src/pico/config.h` 만 읽는다 — MicroPython 판과
      실제로 구운 UF2 는 대조하지 않는다

## 참고

- 빌드: VS Code Pico 확장(SDK 2.3.0), `cmake -G Ninja` + `ninja` → `src/pico/build/*.uf2`. BOOTSEL 로 드래그
- MicroPython 판을 Thonny 로 올리면 UF2(C 판)는 덮어써진다. 되돌리려면 UF2 를 다시 굽는다
- MicroPython 판은 USB 로 `0x03` 바이트가 오면 Ctrl-C 로 먹으므로 `micropython.kbd_intr(-1)` 이 필수다
  ([`../pico_micropython/README.md`](../pico_micropython/README.md))

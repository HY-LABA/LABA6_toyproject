# 시스템 전체 흐름 (pi5 ↔ pico)

YOLO가 쓰레기를 확인했을 때부터 로봇이 실제로 움직이기까지, pi5와 pico가 주고받는 전체
파이프라인. 각 단계 옆에 지금 상태를 표시함 (✅ 실제 로직 완성 / ⏳ 구조는 있으나 실측·구현 필요).

## 0. 부팅 시점

- pico `main.c`가 켜지자마자 1ms 주기 슈퍼루프 시작. pi5가 아직 아무것도 안 보냈어도
  계속 돌면서 엔코더 읽고 오도메트리 갱신함 (cmd 없으면 목표속도 0, 정지 상태 유지).
- pi5 `main.py`는 `run()` 호출 시점부터 시작 (카메라·시리얼 초기화).

## 1. 확정 및 관측 — pi5 `vision.py`

1. 30fps 샘플링(60fps 중 2프레임마다 1장, `config.FRAME_SKIP`)하며 매 프레임 Hailo YOLO 추론 ⏳
2. `TARGET_CLASSES`(pet_bottle/can/paper_cup) 중 신뢰도 임계값 이상이 `CONFIRM_FRAMES`(1)
   연속 감지되면 확정 ✅ (로직 완성, 실제 감지는 YOLO 연동 후)
3. 확정 후 `VELOCITY_SAMPLE_FRAMES`(3)장 캡처, 첫/마지막 프레임의 bbox로 z(크기비율) +
   x,y(픽셀→미터, 핀홀모델) 계산 ⏳ (`REFERENCE_SIZE_AT_1M`, `FOCAL_LENGTH_PX` 실측 필요)
4. vx,vy = 위치차/Δt (등속 가정) ✅, vz = 위치차/Δt − 0.5·g·Δt (등가속도 보정) ✅
5. `Observation(class_name, position, velocity, t)` 반환 (마지막 프레임 시점 기준)

## 2. 착지 예측 — pi5 `trajectory.py`

- `predict_landing(position, velocity, g, z_catch)` — 포물선 공식으로 착지 시각·좌표 계산 ✅
- `valid=False`(z 추정 오류로만 발생 가능)면 조용히 안 넘기고 `main.py`에서 즉시
  `RuntimeError` ✅

## 3. 목표 변환 — pi5 `control.py`

- 착지점 (x,y)를 그대로 "로봇 기준 이동거리"로 사용 (카메라가 로봇 위를 보므로 좌표
  변환 불필요) ✅
- 구동시간 = `min(RECAL_DRIVE_TIME_S, time_to_land)` — 짧게 끊어서 재보정 루프가 매번
  다시 확인 ✅ (단, `RECAL_DRIVE_TIME_S` 자체는 실측 전 임시값 0.1초)

## 4. USB 전송 — pi5 `communication.py` → pico `communication.c`

- 프레임: `[START 0xAA][LEN][PAYLOAD][CHECKSUM]`, PAYLOAD=`<fff>` target_x,target_y,
  drive_time_s (12바이트) ✅ 인코딩/디코딩 로직 완성, pyserial 실제 연결만 ⏳

## 5. 피코 실시간 루프 — pico `main.c` (1ms마다 반복)

| 단계 | 파일 | 상태 |
|---|---|---|
| 엔코더 카운트 읽기 (PIO, 1x 쿼드러처) | `encoder/encoder_pio.c` | ✅ 로직 완성, 핀 번호 ⏳ |
| 카운트 → 바퀴속도 변환 | `main.c` (`wheel_speed_from_encoder_delta`) | ✅ 변환식 완성, `ENCODER_COUNTS_PER_REV` 값만 필요 |
| 바퀴속도 → 로봇속도(vx,vy,omega) | `kinematics.c` (`forward_kinematics`) | ✅ 3x3 역행렬, 실행 검증 완료 |
| 속도 노이즈 스무딩 + 위치 적분 | `odometry_kalman.c` | ✅ 로직 완성, 노이즈 파라미터 ⏳ |
| pi5 명령 확인 (non-blocking) | `communication.c` | ✅ |
| 목표속도 = target/drive_time | `main.c` | ✅ |
| 로봇속도 → 바퀴 3개 목표속도 | `kinematics.c` (`inverse_kinematics`) | ✅ 실행 검증 완료 |
| PID + PWM 출력 | `motor_control.c` | ✅ 구조 완성, 게인·스케일 ⏳ |
| 오도메트리 회신 | `communication.c` | ✅ |

## 6. 재보정 루프 — pi5 `main.py` (z ≤ 0 될 때까지)

1. `communication.send_target` 재전송
2. `communication.receive_odometry` — 피코의 엔코더 기반 이동량 수신 ✅
3. `vision.reobserve` — 카메라 재관측 (단일 프레임, 속도는 미계산) ✅
4. `trajectory.recalibrate(odometry, reobs, previous_landing_point)` — **미설계, 현재
   `NotImplementedError`** ⛔ 다음 작업 대상
5. `control.to_drive_command`로 목표 갱신 → 1번으로 반복

## 지금 막혀있는 지점

`trajectory.recalibrate()`(pi5)가 없어서 실제로 돌리면 재보정 루프 첫 사이클에서 멈춘다.
그 앞(0~5단계, 부팅→확정→예측→변환→전송→피코 1ms 루프)까지는 로직상 완전히 연결되어
있고, 나머지는 전부 실측값·하드웨어 연동만 채우면 되는 상태 (2026-07-31 재확인, pico
`wheel_speed_from_encoder_delta`/`drive_time_s` 만료 정책 구현 완료 후).

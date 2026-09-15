# 피코 — 모터 실시간 제어

피코가 하는 일은 하나다. **파이5가 준 목표점(또는 목표 속도)을 따라 바퀴 3개를 돌린다.**
물체가 어디 떨어지는지는 [파이5](../algorithm.md)가 판단한다.

**피코는 궤적도 착지점도 모른다.** 받는 건 좌표 하나와 시간 하나뿐이다.
이 경계를 지키는 것이 설계의 핵심이다.

펌웨어는 두 판이 있고 **동작이 같다** — 한 번에 하나만 보드에 올라간다.

| 판 | 위치 | 쓰는 때 |
|---|---|---|
| C (Pico SDK) | [`pico/`](../pico/) | 기본. `pico/build/moving_trash_bin.uf2` 를 BOOTSEL 드래그로 굽는다 |
| MicroPython | [`pico_micropython/`](../pico_micropython/) | 값을 자주 바꿔가며 튜닝할 때. 굽기·업로드 절차와 C 와의 차이 4군데는 [그 폴더 README](../pico_micropython/README.md) |

관련 문서: [하드웨어](hardware.md) · [통신 프로토콜](protocol.md) · [물리 계산](physics.md)

---

## 1. 왜 별도 MCU인가

리눅스는 실시간 OS가 아니라 제어 루프의 주기를 보장할 수 없다. 스케줄러 지터가 수 ms
단위로 튄다. PID의 미분·적분항은 `dt`를 쓰기 때문에 dt가 흔들리면 출력이 요동친다.

피코는 슈퍼루프로 주기를 지키고, **엔코더는 PIO 하드웨어**, **PWM은 PWM 슬라이스**가
CPU와 무관하게 처리한다. 그래서 MicroPython 판도 제어 성능은 사실상 같다
([`pico_micropython/README.md` 3장](../pico_micropython/README.md)).

---

## 2. 제어 루프 — 20 ms (50 Hz)

```
매 20 ms (sleep_until 고정 주기):
  1. 엔코더 카운트 읽기 (PIO 누적값)
  2. 카운트 차분 → 바퀴 선속도 3개  × MOTOR_SIGN
  3. 순기구학 → body 속도 (vx, vy, ω)
  4. 오도메트리 — pose 는 raw 속도로 적분, 칼만은 보고용 속도에만
  5. 파이5 명령 수신 — 버퍼를 끝까지 비우고 마지막 완전 프레임만 채택
       · 목표점(16B) 이면 DRIVE_TARGET, 속도(12B) 면 DRIVE_VELOCITY
       · **모드가 바뀌면 PID 상태 리셋**
       · 새 명령이 없으면 워치독 누적 → 만료 시 DRIVE_NONE + PID 리셋
  6. 목표 body 속도 결정
       · DRIVE_TARGET: 남은거리 = 목표 − pose  (world)
                       → R(−θ) 로 body 축 회전
                       → v = 남은거리 ÷ 남은시간, 방향별 바퀴 상한으로 클램프
                       → 남은시간 −= dt
       · DRIVE_VELOCITY: 받은 값 그대로
  7. 안전 클램프 (크기만 깎고 방향 보존)
  8. 역기구학 → 바퀴 목표속도 3개
  9. 바퀴별 PID → PWM (RPWM / LPWM)
 10. 오도메트리 회신 — 송신 버퍼 여유 있을 때만
```

**50 Hz인 이유**: PID 게인이 MicroPython 벤치(50 Hz)에서 튜닝됐다. Ki·Kd는 dt에 비례해
누적·변화하므로 같은 게인을 다른 주기에 쓰면 거동이 달라진다.

**부팅 직후부터 계속 돈다.** 파이5가 아무것도 안 보냈어도 엔코더를 읽고 오도메트리를
갱신한다 (명령이 없으면 목표 속도 0).

---

## 3. 기구학

바퀴 장착각 `β_i`, 중심-바퀴 거리 `L`.

```
역기구학:  w_i = −sin(β_i)·vx + cos(β_i)·vy + L·ω
순기구학:  위 3×3 행렬의 역행렬 (여인수/수반행렬 공식)
```

장착각은 **M1 = 90°(전방, +Y) / M2 = 330°(우측 뒤) / M3 = 210°(좌측 뒤)**, L = 0.15 m다
(몸체 완성 후 실물 대조·자로 확인). β_i는 중심에서 바퀴를 본 위치각이고 옴니 바퀴가 구르는
방향은 그 접선(β_i+90°)이라, 위 식의 앞 두 항이 곧 접선 단위벡터다.

```
            전방 (+Y)
               M1
               |
      M3 ------+------ M2   → 우측 (+X)
```

**순수 전진(+Y)에서 M1은 돌지 않는다** — `cos 90° = 0`. M1은 롤러로 옆으로 미끄러지고
M2·M3가 **서로 반대 방향으로** 돈다. 브링업 때 "안 도는 바퀴 쪽으로 로봇이 가는가"로
배선·부호를 한 번에 확인할 수 있다 (9절).

### 방향별 속도 상한 (`max_body_speed`)

모터가 제한하는 건 **바퀴 속도**다. 같은 body 속력이라도 방향에 따라 가장 빨리 도는 바퀴가
달라서, body 1 m/s에 필요한 최대 바퀴 속도가 0.866~1.000으로 15.5% 차이 난다. 그래서 상한을
상수 하나로 자르지 않고 **가려는 방향에 단위 속도를 넣어 나온 `max|w_i|` 가
`WHEEL_MAX_SPEED_MPS` 에 닿는 지점**으로 정한다. 파이5 `control.max_body_speed()` 와
**같은 식**이어야 한다 — 파이의 도달 판정(`tracker._reach`)이 이 값을 믿는다.

⚠ 포화될 때 **크기만 깎고 방향은 보존**한다. 한 바퀴만 잘리면 세 바퀴 비율이 깨져서 속도뿐
아니라 진행 방향까지 틀어진다.

---

## 4. 오도메트리

```
body 속도를 로봇 방향(θ)으로 회전시켜 world frame 으로 바꾼 뒤 적분:

  x     += (vx·cosθ − vy·sinθ) · dt
  y     += (vx·sinθ + vy·cosθ) · dt
  θ     += ω · dt
```

**pose 는 raw 속도로 적분한다.** 축별 1D 칼만필터는 파이5로 **보고하는 속도(텔레메트리)**
에만 쓴다. 지금 파라미터(Q=0.01, R=1.0, dt=0.02)는 정지 상태로 몇 초만 지나도 칼만 이득이
0.014로 굳어 시정수가 약 1.4초가 되는데, 캐치 기동은 0.5초짜리라 그 속도로 pose를 적분하면
이동량을 크게 과소보고한다. 그래서:

- 오도메트리 `(x, y, θ)` — 짧은 구간 측정에 **써도 된다**
- 오도메트리 `(vx, vy, ω)` — 1.4초 지연이 걸려 있어 **가속·정지거리 측정에 쓰면 안 된다.**
  pose 차분으로 다시 계산할 것 (`pico_test.py`, `teleop_test.py --go-distance` 가 그렇게 한다)

pose 원점은 **부팅 시점**이고 리셋하지 않는다. 파이5는 트랙 시작 시점의 pose를 기준으로
착지점을 절대좌표로 바꿔 보내고, **남은 거리는 피코가 자기 pose로 뺀다**
([protocol.md 2장](protocol.md)).

**⚠ 옴니휠은 슬립이 있다.** 엔코더는 바퀴 회전만 세므로 미끄러진 만큼 모른다. 줄자로 잰
이동거리가 오도메트리보다 크면 그 차이가 슬립이다.

---

## 5. 모터 제어 — 바퀴별 PID

```c
error    = wheel_target − wheel_actual;
integral = clamp(integral + error·dt, ±1/ki);     // 안티와인드업
output   = kp·error + ki·integral + kd·d(error)/dt;
if (|wheel_target| > 1e-4 && |output| < MIN_DRIVE_DUTY)
    output = ±MIN_DRIVE_DUTY;                      // 데드밴드 보상
output  *= MOTOR_SIGN;
```

| 항목 | 값 | 이유 |
|---|---|---|
| 게인 `{kp, ki, kd}` | `{0.6, 0.4, 0}` ×3 | MicroPython 벤치 검증값 `{60, 40, 0}` 을 **100으로 나눈 값**. 벤치는 duty를 퍼센트로, C는 비율(0~1)로 받는다. 안 나누면 속도 오차 0.017 m/s에 duty가 포화돼 뱅뱅 제어가 된다 |
| 안티와인드업 | 적분항 단독으로 duty 100% 초과 금지 | 오래 멈췄다 풀릴 때 튀어나가는 걸 막는다 |
| 데드밴드 보상 | `MIN_DRIVE_DUTY = 0.15` | 목표가 0이 아닌데 duty가 작으면 정지마찰을 못 이겨 목표 근처에서 영원히 못 간다 |
| **모드 전환 시 PID 리셋** | `motor_control_reset()` | 이전 모드의 적분항이 남으면 정지 상태에서 다음 명령 첫 틱에 전력으로 튀어나갈 수 있다. **매 명령마다가 아니라 모드가 바뀔 때만** 지운다 — 목표점은 프레임마다 갱신되므로 매번 지우면 Ki가 정지마찰을 이기려고 쌓이는 걸 계속 버리게 된다 |

### PWM 출력

BTS7960은 RPWM / LPWM 2채널이다. 출력 부호에 따라 한쪽에만 PWM을 주고 다른 쪽은 0.
R_EN/L_EN은 3.3 V에 직결돼 있어 GPIO를 쓰지 않는다.

- 반송파 **10 kHz** (`PWM_FREQ_HZ`). BTS7960 상한이 25 kHz이고, 분주를 안 걸면 이 보드
  (pico2_w = RP2350, 150 MHz)에서 36.6 kHz가 나와 상한을 넘는다. `motor_control.c` 가
  실제 `clk_sys` 에서 분주를 역산한다
- 분해능 12비트 (`PWM_WRAP = 4095`)

### `MOTOR_SIGN`

엔코더 측정값과 PWM 출력 **양쪽에 똑같이** 곱해진다. 그래서 값이 틀려도 PID는 자기 자신과
멀쩡히 맞물려 돌고 오도메트리도 명령과 일치하는데 **로봇만 반대로 간다.** 이 값의 판정은
반드시 **눈으로 본 방향**으로 해야 한다. 현재 `{1, 1, 1}` — 판정 방법은 9절.

---

## 6. 엔코더 (PIO)

PIO로 **A상 상승엣지만 세고, 그 순간 B상 레벨로 방향을 판단**하는 1x 쿼드러처 디코딩이다.
CPU 개입 없이 X 레지스터에 누적하고, 상위 코드는 최신 값만 읽는다.

- PIO에는 덧셈 명령이 없어 증가는 `반전 → 감소 → 반전` 트릭을 쓴다
- FIFO가 차서 push가 버려져도 무해하다 — X에 누적 절대값이 들어 있고 최신 값만 읽는다

### ★ CPR은 **출력축(바퀴축) 기준 실측값**이다

```c
wheel_revs = delta_counts / ENCODER_COUNTS_PER_REV;   // 687.5, 기어비로 나누지 않는다
speed      = wheel_revs * (WHEEL_DIAMETER_M * M_PI) / dt;
```

`ENCODER_COUNTS_PER_REV = 687.5` 는 손회전·정밀정지로 잰 **바퀴 1회전당 카운트**다
(2체배 실측 1375 ÷ 2, 우리 PIO가 1체배라서). 이미 감속기 이후 값이므로 **`GEAR_RATIO`
(43.8)로 또 나누면 안 된다** — 나누면 이동거리를 43.8배 작게 계산해서 "1 m 줬는데 한참 더
가는" 버그가 된다(실제로 겪었다). `GEAR_RATIO` 는 config에 기록용으로만 남아 있다.

데이터시트의 "16 CPR(모터축)"은 실측과 맞지 않아 쓰지 않는다. 4체배로 바꾸면 1375×2 = 2750.

**⚠ 엔코더 VCC는 3.3 V에 물린다.** 출력이 VCC 기준으로 스윙하므로 3.3 V 급전이면 피코에
직결해도 안전하다. 5 V로 물리면 피코가 죽는다 ([hardware.md 3.2장](hardware.md#32-엔코더)).
`encoder_pio.c` 의 내부 풀업은 그대로 둔다.

---

## 7. 파일 구조

```
pico/
├── CMakeLists.txt          # 빌드 설정 (PICO_BOARD pico2_w)
├── pico_sdk_import.cmake   # SDK 부트스트랩
├── main.c                  # 20 ms 제어 루프 (2절)
├── config.h                # 핀 배치, 물리 상수, MOTOR_SIGN, PID 게인, 속도 상한
├── encoder/
│   ├── encoder_pio.c/.h    # PIO 엔코더 인터페이스
│   └── encoder.pio         # PIO 프로그램 (CMake가 헤더 생성)
├── kinematics.c/.h         # 순/역기구학 + 방향별 속도 상한 (3절)
├── odometry_kalman.c/.h    # pose 적분 + 보고용 속도 스무딩 (4절)
├── motor_control.c/.h      # PID + PWM + 모드 전환 리셋 (5절)
└── communication.c/.h      # 프레임 송수신 (protocol.md)
```

MicroPython 판은 파일이 1:1 대응한다 ([`pico_micropython/README.md` 1장](../pico_micropython/README.md)).

---

## 8. 파라미터

`pico/config.h` (MicroPython은 `pico_micropython/config.py`, **값을 고치면 둘 다** 고친다).

| 값 | 현재 | 상태 |
|---|---|---|
| `MOTOR_PINS[3]` | `{0,1,7,8}` `{2,3,10,11}` `{4,5,12,13}` (rpwm, lpwm, enc_a, enc_b) | ✅ 실기 확정 |
| `WHEEL_ANGLES_RAD[3]` | 90° / 330° / 210° | ✅ 몸체 실물 대조 |
| `WHEEL_MOUNT_RADIUS_M` | 0.15 m | ✅ 자로 확인 (ω를 쓸 때만 영향) |
| `WHEEL_DIAMETER_M` | 0.100 m | ✅ |
| `ENCODER_COUNTS_PER_REV` | 687.5 (바퀴축, 1체배) | ✅ 실측 |
| `MOTOR_SIGN[3]` | `{1, 1, 1}` | ⚠ 9절 "방향 판정" 참고 — 카메라 설정과 상쇄 중일 수 있다 |
| `MOTOR_PID[3]` | `{0.6, 0.4, 0}` | ✅ 벤치 검증값 환산 |
| `MIN_DRIVE_DUTY` | 0.15 | ✅ 벤치 검증 |
| `PWM_FREQ_HZ` | 10000 | ✅ 벤치 검증 |
| `CONTROL_PERIOD_MS` | 20 | ✅ 게인이 이 주기 기준 |
| `POSITION_TOLERANCE_M` | 0.02 | ✅ pi5와 일치 |
| `WHEEL_MAX_SPEED_MPS` | **1.8** | ⚠ pi5와 일치(9/15)하지만 **실측값이 아니다** — 이론값 1.22. 실측 후 세 파일 통일 |
| `PROCESS_NOISE` / `MEASURE_NOISE` | 0.01 / 1.0 | 보고용 속도에만 영향 |

`WHEEL_ANGLES_RAD`, `WHEEL_MOUNT_RADIUS_M`, `WHEEL_MAX_SPEED_MPS`, `POSITION_TOLERANCE_M` 은
파이5 `pi5/config.py` 에도 있다. `python pi5/control.py` 가 `pico/config.h` 를 직접 읽어
대조한다 (MicroPython `config.py` 는 대조하지 않는다).

---

## 9. 브링업 절차

**순서대로 한다. 앞 단계가 안 되면 뒷 단계는 디버깅이 안 된다.**

1. **빌드·굽기** — C: `pico/build` 에서 `ninja` → `moving_trash_bin.uf2` 를 BOOTSEL 드래그.
   MicroPython: [README 2장](../pico_micropython/README.md)
2. **엔코더** — 바퀴를 손으로 1회전 → 687~688 카운트, 역회전 시 부호 반전
3. **방향 판정** ★ — 카메라 없이 구동만:
   ```bash
   python pi5/pico_test.py --port /dev/ttyACM0 --dir front --dist 0.5 --speed 0.3
   ```
   | 볼 것 | 정상 |
   |---|---|
   | 안 도는 바퀴가 하나 있나 | 있다 (그게 코드의 M1) |
   | 로봇이 그 바퀴 쪽으로 가나 | 간다 |
   | 나머지 둘이 반대로 도나 | 반대 |

   그 바퀴 **반대쪽으로** 가면 `MOTOR_SIGN` 이 뒤집힌 것이다. 안 도는 바퀴가 실제 앞바퀴가
   아니면 `MOTOR_PINS` 순서가 배선과 다른 것이다. `--all` 로 8방향 왕복을 돌리면 끝에 누적
   드리프트도 나온다
4. **거리·최대속도·정지거리** — 줄자와 대조:
   ```bash
   python pi5/teleop_test.py --port /dev/ttyACM0 --go-distance 1.0
   python pi5/pico_test.py --port /dev/ttyACM0 --dir front --dist 1.0 --max
   ```
   `--max` 는 톨러런스 직전까지 전속으로 붙어서 **정지거리(오버슈트)가 그대로 드러난다.**
   통 입구 반지름(13 cm)을 넘으면 경고한다
5. **통신 왕복** — [protocol.md](protocol.md) 5장

### 마찰계수 μ

정지에서 전속까지의 초기 가속 기울기가 견인 한계 `a = 0.5·μ·g` 다 → 여기서 μ를 역산하고
[physics.md 6장](physics.md#6-통-입구-크기--설계의-지렛대) 표에서 유효 캐치 반경을 읽는다.
**아직 안 쟀다.** 가속 구간은 오도메트리 **pose 차분**으로 봐야 한다(4절).

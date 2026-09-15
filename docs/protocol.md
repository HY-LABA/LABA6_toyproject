# 통신 프로토콜 (파이5 ↔ 피코)

**양쪽이 반드시 일치해야 하는 계약이다.** 그래서 파이5 문서에도 피코 문서에도 넣지 않고
따로 뒀다. 한쪽만 고치면 조용히 깨진다.

구현: [`../pi5/communication.py`](../pi5/communication.py)
↔ [`pico/communication.c`](../pico/communication.c)
(MicroPython 판: [`pico_micropython/communication.py`](../pico_micropython/communication.py) — 프레임 포맷 동일)

관련 문서: [동작 로직](../algorithm.md) · [피코 제어](pico-control.md)

---

## 1. 프레임 포맷

양방향 공통이다.

```
[START 0xAA][LEN][PAYLOAD ...][CHECKSUM]

CHECKSUM = sum(PAYLOAD) % 256
```

수신 측은 논블로킹 상태머신으로 파싱한다:
`WAIT_START → WAIT_LEN → WAIT_PAYLOAD → WAIT_CHECKSUM`.
체크섬이 틀리면 프레임을 버리고 `WAIT_START`로 돌아간다.

전송 매체는 **USB CDC**다. 하드웨어 UART 핀이 아니다. 파이5에서는 `/dev/ttyACM0`, 115200.

> **⚠ MicroPython 판에서는 0x03 을 조심한다.** float 이진 데이터에는 0x03 바이트가 반드시
> 섞이는데, MicroPython 은 기본적으로 stdin 의 0x03 을 Ctrl-C 로 해석해 제어 루프를 죽인다
> (모터는 마지막 듀티로 계속 돈다). 그래서 `communication_init()` 이 `kbd_intr(-1)` 로 끈다
> ([`pico_micropython/README.md` 2-4장](../pico_micropython/README.md)).

---

## 2. 페이로드

파이5 → 피코는 **두 종류**다. 프레임에 이미 LEN 바이트가 있으므로 **페이로드 길이가
곧 명령 종류**다 (16 B vs 12 B — 섞일 수 없다).

### ① 목표점 — 평상시 구동 (16 B, `<ffff`)

| 필드 | 타입 | 설명 |
|---|---|---|
| `target_x` | float32 | 도착 지점 X — **world frame 절대 좌표** (m) |
| `target_y` | float32 | 도착 지점 Y — **world frame 절대 좌표** (m) |
| `time_remaining_s` | float32 | 이 명령을 만든 순간 기준 착지까지 남은 시간 (s) |
| `timeout_s` | float32 | 워치독 (s) |

**"여기서부터 얼마"가 아니라 절대 좌표다.** 피코가 `남은거리 = target − 자기 pose`를
직접 계산하므로, 파이는 이미 간 거리를 빼서 보내지 않는다. 예전 프로토콜은 상대
거리를 보냈고 피코가 아무도 빼지 않아서 **매 사이클 처음부터 다시 갔다**(구조적
오버슈트). 절대 좌표에서는 이중으로 뺄 값 자체가 없다.

> **world frame = 피코 오도메트리 원점(부팅 시점).** 파이는 트랙 시작 시점에 피코가
> 보고한 pose를 착지점(로봇 body 축)에 더해서 만든다. 이 덧셈은 **θ≈0일 때만** 성립한다 —
> 착지점은 body 축이고 pose는 world 축이다. 지금은 ω를 명령하지 않아 거의 같다.
> (피코 쪽은 남은거리를 body 축으로 돌릴 때 R(−θ)를 적용한다 — 아래 코드.)

> **`time_remaining_s` 는 실제 남은시간이 아니다.** 파이가 `config.DRIVE_AGGRESSION`(현재
> 8.0)으로 나눠서 보낸다 — 초반부터 최고속도로 붙게 하려는 의도다
> ([algorithm.md 5장](../algorithm.md#5-제어-루프-pi5mainpy-controlpy)). 피코는 받은 값을
> 그대로 쓰므로 프로토콜은 바뀌지 않았다.

피코가 해야 하는 계산:

```c
rx = target_x - pose.x;   ry = target_y - pose.y;          // world
dist = hypotf(rx, ry);
if (dist <= POSITION_TOLERANCE_M) { 정지; }
bx =  cos(θ)*rx + sin(θ)*ry;  by = -sin(θ)*rx + cos(θ)*ry;  // body 축으로 R(−θ)
limit = max_body_speed(bx, by);                            // ← 바퀴 속도 기준, body 방향으로
v = (time_remaining_s > 1e-3) ? dist / time_remaining_s : limit;
v = fminf(v, limit);
inverse_kinematics(bx/dist*v, by/dist*v, 0, wheel_target);
time_remaining_s -= dt;   // 다음 명령이 오면 덮어쓴다
```

`time_remaining_s` 가 0 이하면 "이미 늦었다"고 보고 **상한 속도로 붙는다.**

**남은시간을 같이 보내는 이유:** 없으면 피코가 최대속도로 가다 급정지해서 정지거리만큼
구조적으로 오버슈트한다. `남은거리 ÷ 남은시간`이면 목표에 가까워질수록 목표속도가
저절로 줄어서, 별도 감속 프로파일 없이 P 제어가 감속기 역할을 한다.

> **⚠ 상한은 body 속력이 아니라 바퀴 속도 기준이어야 한다.** 3륜 옴니는 방향에 따라
> 같은 body 속도에 필요한 바퀴 속도가 0.866~1.000으로 다르다. body 속력을 상수 하나로
> 자르면 유리한 방향에서 13.4%를 버리고, 파이의 `tracker._reach()`가 그만큼 낙관해서
> **못 잡을 표적을 쫓는다.**

### ② 속도 — 텔레옵·정지 (12 B, `<fff`)

| 필드 | 타입 | 설명 |
|---|---|---|
| `target_vx` | float32 | 목표 body 속도 X (m/s) |
| `target_vy` | float32 | 목표 body 속도 Y (m/s) |
| `timeout_s` | float32 | 워치독 (s) |

`teleop_test.py`(게임패드)와 `control.STOP` 전용이다. 스틱 입력은 본질적으로 속도지
좌표가 아니고, 모터 최대속도·가속시간·정지거리 실측이 이 경로에 걸려 있다.

`omega`는 보내지 않는다. 현재 설계는 로봇 회전을 쓰지 않는다 (ω = 0 고정,
[open-questions.md](open-questions.md#2-결정해야-할-것들) 참고).

### 피코 → 파이5 (24 B, `<ffffff`)

| 필드 | 타입 | 설명 |
|---|---|---|
| `x`, `y`, `theta` | float32 ×3 | world frame 자세 — **raw 속도 적분** |
| `vx`, `vy`, `omega` | float32 ×3 | body frame 속도 — **칼만 스무딩, 시정수 약 1.4초** |

> **⚠ 속도 필드로 가속·정지거리를 재면 안 된다.** 1.4초 지연이 걸려 있다. 짧은 구간의
> 속도는 pose 차분으로 다시 계산할 것 ([pico-control.md 4장](pico-control.md#4-오도메트리)).

> **타임스탬프가 없다.** 카메라 프레임 시각과 오도메트리 시각이 어긋나면 위치 보정이
> 그만큼 틀린다 — 1 m/s에서 10 ms면 1 cm다. `t_ms`(uint32)를 앞에 붙여 28 B로 늘리면
> 파이5가 지연을 측정하고 자세를 보간할 수 있다. **미구현이다.**

---

## 3. 워치독 (`timeout_s`)

두 명령 모두 `timeout_s` 를 싣는다. 피코는 **새 명령이 없는 틱마다** 경과시간을 누적하고
그게 `timeout_s` 를 넘으면 구동을 멈춘다.

```c
if      (new_cmd.kind == RX_CMD_TARGET)   { if (mode != DRIVE_TARGET)   pid_reset(); mode = DRIVE_TARGET;   elapsed = 0; ... }
else if (new_cmd.kind == RX_CMD_VELOCITY) { if (mode != DRIVE_VELOCITY) pid_reset(); mode = DRIVE_VELOCITY; elapsed = 0; ... }
else if (mode != DRIVE_NONE)              { elapsed += dt;
                                            if (elapsed >= timeout_s) { mode = DRIVE_NONE; pid_reset(); } }
```

**이게 유일한 안전장치다.** 파이5가 죽거나, USB가 빠지거나, 프로그램이 멈춰도 로봇은
`timeout_s` 안에 정지한다. 없으면 마지막 명령대로 계속 달린다.

현재 파이5는 `config.DRIVE_TIMEOUT_S = 0.15` 를 보낸다. 파이5 루프 주기(60 fps에서 16.7 ms)의
몇 배로 잡는다 — 너무 짧으면 프레임 하나만 늦어도 끊기고, 너무 길면 워치독 의미가 없다.
**⚠ 파이5 루프가 150 ms 보다 느려지면 매 프레임 만료돼 로봇이 가다 서다를 반복한다.**

예외: `main.py --once --hold N` 은 단발 시험용으로 `timeout_s` 를 N초로 늘려 보낸다.

---

## 4. 송신 주기와 백로그

**원칙: 양쪽 다 버퍼를 끝까지 비우고 마지막 완전 프레임만 쓴다.** 쌓인 프레임은 낡은
정보이고, 오래된 것부터 하나씩 읽으면 지연이 무한히 누적된다.

| 방향 | 주기 | 수신 측 처리 |
|---|---|---|
| 피코 → 파이5 | 매 제어 틱 = **50 Hz** (28 B × 50 = 1.4 KB/s) | `SerialLink.try_receive_odometry()` — **절대 블로킹하지 않는다.** 읽을 게 있으면 전부 읽고 최신 하나, 없으면 `None` |
| 파이5 → 피코 | 카메라 프레임마다 (실측 60 fps) | `communication_try_receive()` — 버퍼를 끝까지 비우고 **마지막 완전 프레임만** 채택 |

> 피코 수신이 "프레임 하나 읽고 return" 이면 **틱당 1프레임(50 Hz)** 밖에 소비를 못 해서,
> 파이가 60 fps로 보내는 순간 백로그가 쌓이기 시작한다. 그래서 끝까지 비운다.
> 부수효과로, 같은 틱에 목표점과 STOP 이 함께 도착하면 **뒤에 온 STOP 이 이긴다.**

### 피코 송신은 논블로킹

`putchar_raw` 는 호스트가 안 읽으면 **블로킹한다.** 제어 루프 안에서 걸리면 루프가 멈추고
모터가 마지막 듀티로 계속 돈다. 그래서 **송신 여유를 먼저 확인하고, 없으면 이번 틱은
건너뛴다** — 오도메트리 한 프레임은 빠져도 되지만 제어 루프는 멈추면 안 된다.

| 판 | 여유 확인 방법 |
|---|---|
| C | `tud_cdc_connected()` + `tud_cdc_write_available() >= 27` |
| MicroPython | `select.poll(sys.stdout, POLLOUT)` |

> **타이밍 주의 (파이5 쪽, 미해결):** `main.py` 는 카메라 프레임을 캡처·추론한 **뒤에**
> 오도메트리를 읽는다. 그 프레임과 짝지어지는 pose가 추론 시간만큼 늦다(로봇 1.2 m/s, 추론
> 30 ms 가정 시 약 3.6 cm). 오도메트리 프레임에 타임스탬프가 없어서(2장) 보간도 못 한다.

---

## 5. 테스트

> **⚠ 아래 자동 테스트(`tests/test_protocol.py`)는 아직 없다.** 지금 확인 수단은 실기 왕복
> (`pi5/pico_test.py`, `pi5/teleop_test.py`)과 6장의 디코더뿐이다. 만들 때의 목록으로 둔다.

프로토콜은 양쪽 구현이 갈라지기 쉬운 지점이라 테스트로 강제하는 게 맞다.

| 테스트 | 내용 |
|---|---|
| 인코딩 왕복 | `decode(encode(x)) == x` (양방향 페이로드 모두) |
| 바이트 수준 고정 | 알려진 입력의 바이트열을 하드코딩해 비교 — C 쪽 `memcpy` 오프셋과 대조 |
| 체크섬 오류 | 1비트 손상 프레임을 거부하는지 |
| 프레임 재동기 | 쓰레기 바이트 뒤에 정상 프레임이 와도 파싱되는지 |
| 부분 수신 | 프레임이 여러 번에 나눠 도착해도 상태머신이 조립하는지 |
| 백로그 | 프레임 100개를 밀어넣고 `try_receive_odometry()`가 마지막 것을 주는지 |

**바이트 수준 고정 테스트가 핵심이다.** Python `struct`와 C `memcpy`의 정렬·엔디안이
어긋나는 것을 잡아낸다. 둘 다 리틀엔디안 packed를 가정한다.

---

## 6. 디버깅

`pi5/communication.py` 의 `debug_decode(raw)` 가 프레임 하나를 사람이 읽는 텍스트로 바꾼다.
페이로드 길이(24/16/12 B)로 종류를 구분한다.

```
$ python pi5/communication.py < frame.bin
[odometry] x=0.031 y=-0.002 theta=0.001 vx=0.420 vy=0.000 omega=0.000
```

프레임 하나만 넣어야 한다 (스트림 전체를 넣으면 첫 프레임 뒤는 무시된다).

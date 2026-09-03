# 통신 프로토콜 (파이5 ↔ 피코)

**양쪽이 반드시 일치해야 하는 계약이다.** 그래서 파이5 문서에도 피코 문서에도 넣지 않고
따로 뒀다. 한쪽만 고치면 조용히 깨진다.

구현: [`../pi5/communication.py`](../pi5/communication.py)
↔ [`pico/communication.c`](../pico/communication.c)

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

전송 매체는 **USB CDC**다. 하드웨어 UART 핀이 아니다.

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
> 보고한 pose를 트랙 좌표에 더해서 만든다. 이 덧셈은 **θ≈0일 때만** 성립한다 —
> 트랙 좌표는 body 축이고 pose는 world 축이다. 지금은 ω=0이라 같다.

피코가 해야 하는 계산:

```c
rx = target_x - pose.x;   ry = target_y - pose.y;
dist = hypotf(rx, ry);
if (dist <= POSITION_TOLERANCE_M) { 정지; }
v = (time_remaining_s > 0) ? dist / time_remaining_s : INFINITY;
v = fminf(v, 이 방향에서 바퀴가 포화되지 않는 상한);   // ← 바퀴 속도 기준
inverse_kinematics(rx/dist*v, ry/dist*v, 0, wheel_target);
time_remaining_s -= dt;   // 다음 명령이 오면 덮어쓴다
```

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
| `x`, `y`, `theta` | float32 ×3 | world frame 자세 |
| `vx`, `vy`, `omega` | float32 ×3 | body frame 속도 |

> **타임스탬프가 없다.** 카메라 프레임 시각과 오도메트리 시각이 어긋나면 위치 보정이
> 그만큼 틀린다 — 1 m/s에서 10 ms면 1 cm다. `t_ms`(uint32)를 앞에 붙여 28 B로 늘리면
> 파이5가 지연을 측정하고 자세를 보간할 수 있다. **미구현이다.**

---

## 3. TTL 워치독

파이5는 매 명령에 유효시간 `ttl_s`(예: 0.1 s)를 실어 보낸다. 피코는 매 사이클 TTL을 감소시키고
**만료되면 목표 속도를 0으로 만든다.**

```c
if (new_cmd.valid)      { cmd = new_cmd; ttl_remaining = new_cmd.ttl_s; }
else if (cmd.valid)     { ttl_remaining -= dt;
                          if (ttl_remaining <= 0) cmd.valid = false; }

target_vx = cmd.valid ? cmd.cmd_vx : 0.0f;
```

**이게 유일한 안전장치다.** 파이5가 죽거나, USB가 빠지거나, 프로그램이 멈춰도 로봇은
0.1초 안에 정지한다. TTL이 없으면 마지막 명령대로 계속 달린다.

TTL은 파이5 루프 주기(16.7 ms @ 60 fps)의 3~6배 정도로 잡는다. 너무 짧으면 프레임 하나만 늦어도 끊기고,
너무 길면 워치독 의미가 없다.

---

## 4. 송신 주기와 백로그

### 왜 신경 써야 하나

피코가 1 ms마다 프레임을 보내면 파이5가 못 따라 읽는다. 파이5는 루프당 한 번 읽는데
`read()`는 **가장 오래된** 프레임을 돌려주므로, 버퍼에 쌓일수록 **지연이 무한히 누적된다.**
오도메트리가 낡으면 남은 거리 계산이 틀리고, 그게 그대로 목표 속도 오차가 된다.

### 대책 (둘 다 적용)

**1. 피코 송신 주기를 100 Hz로 낮춘다.** 제어 루프는 1 kHz를 유지하고, 10 사이클마다 한 번만
송신한다. 대역폭이 2.8 KB/s로 떨어진다. 파이5는 30~60 Hz로 읽으므로 100 Hz면 충분하다.

**2. 파이5는 입력 버퍼를 비우고 마지막 완전한 프레임만 쓴다** (`latest_odometry()`).
쌓인 프레임은 어차피 낡은 정보다.

```python
def latest_odometry(self):
    latest = None
    while self._port.in_waiting > 0:
        frame = self._read_one_frame()
        if frame is not None:
            latest = frame
    return latest if latest else self._blocking_read_one()
```

### 피코 송신은 논블로킹이어야 한다

`putchar_raw`는 호스트가 안 읽으면 **블로킹한다.** 1 kHz 루프 안에서 이게 걸리면 제어가
멈춘다 — 모터가 마지막 듀티로 계속 돌아간다.

**송신 가능 공간을 먼저 확인하고, 없으면 이번 사이클 송신을 건너뛴다.**
오도메트리는 한 프레임 빠져도 되지만 제어 루프는 멈추면 안 된다.

---

## 5. 테스트

프로토콜은 양쪽 구현이 갈라지기 쉬운 지점이라 테스트로 강제한다 (`tests/test_protocol.py`).

| 테스트 | 내용 |
|---|---|
| 인코딩 왕복 | `decode(encode(x)) == x` (양방향 페이로드 모두) |
| 바이트 수준 고정 | 알려진 입력의 바이트열을 하드코딩해 비교 — C 쪽 `memcpy` 오프셋과 대조 |
| 체크섬 오류 | 1비트 손상 프레임을 거부하는지 |
| 프레임 재동기 | 쓰레기 바이트 뒤에 정상 프레임이 와도 파싱되는지 |
| 부분 수신 | 프레임이 여러 번에 나눠 도착해도 상태머신이 조립하는지 |
| 백로그 | 프레임 100개를 밀어넣고 `latest_odometry()`가 마지막 것을 주는지 |

**바이트 수준 고정 테스트가 핵심이다.** Python `struct`와 C `memcpy`의 정렬·엔디안이
어긋나는 것을 잡아낸다. 둘 다 리틀엔디안 packed를 가정한다.

---

## 6. 디버깅

`debug_decode(raw: bytes) -> str` 같은 디코더를 두면 배선 문제를 빨리 찾을 수 있다.
raw 바이트를 사람이 읽는 텍스트로 바꾼다. 페이로드 길이로 방향을 구분한다.

```
$ cat /dev/ttyACM0 | python -m protocol
[odometry] t=1234 x=0.031 y=-0.002 theta=0.001 vx=0.42 vy=0.00 omega=0.00
```

# 통신 프로토콜 (파이5 ↔ 피코)

**양쪽이 반드시 일치해야 하는 계약이다.** 그래서 파이5 문서에도 피코 문서에도 넣지 않고
따로 뒀다. 한쪽만 고치면 조용히 깨진다.

구현: [`pi5/communication.py`](../pi5/communication.py) ↔ [`pico/communication.c`](../pico/communication.c)

관련 문서: [파이5 알고리즘](pi5-algorithm.md) · [피코 제어](pico-control.md)

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

### 파이5 → 피코 (12 B, `<fff`)

| 필드 | 타입 | 설명 |
|---|---|---|
| `cmd_vx` | float32 | 목표 body 속도 X (m/s) |
| `cmd_vy` | float32 | 목표 body 속도 Y (m/s) |
| `ttl_s` | float32 | 명령 유효시간 (s) |

> **⚠ 기존 설계에서 의미가 바뀌었다.**
> 기존은 `target_x, target_y, drive_time_s`를 보내고 피코가 `target / drive_time`으로 속도를
> 만들었다. 그런데 0.3 m를 0.1 s에 가라는 명령은 **3 m/s**로, 최고 속도 1.83 m/s를 훨씬
> 넘는다. 피코에는 궤적 정보가 없어 이게 가능한지 판단할 수 없다.
>
> **속도 계산은 전체 그림을 아는 파이5가 한다.** 페이로드 크기가 12 B로 같아서 프레임 포맷
> 자체는 변경이 없다.

`omega`는 보내지 않는다. 현재 설계는 로봇 회전을 쓰지 않는다 (ω = 0 고정,
[open-questions.md](open-questions.md#2-결정해야-할-것들) 참고).

### 피코 → 파이5 (28 B, `<Iffffff`)

| 필드 | 타입 | 설명 |
|---|---|---|
| `t_ms` | uint32 | 피코 부팅 후 경과 ms |
| `x`, `y`, `theta` | float32 ×3 | world frame 자세 |
| `vx`, `vy`, `omega` | float32 ×3 | body frame 속도 |

> **타임스탬프를 추가한 이유:** 카메라 프레임 시각과 오도메트리 시각이 어긋나면
> body → world 변환이 틀린다. 로봇이 1 m/s로 움직일 때 10 ms 어긋나면 1 cm 오차다.
> 타임스탬프가 있으면 파이5가 지연을 측정하고 자세를 보간할 수 있다.

`t_ms`는 피코 부팅 기준이라 파이5의 시계와 원점이 다르다. 파이5는 **차분**만 쓴다
(프레임 간 경과시간, 지연 추정).

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

TTL은 파이5 루프 주기(33 ms)의 3배 정도로 잡는다. 너무 짧으면 프레임 하나만 늦어도 끊기고,
너무 길면 워치독 의미가 없다.

---

## 4. 송신 주기와 백로그

### 문제 — 현재 설계의 결함

피코가 1 ms마다 27 B를 보내면 **27 KB/s**다. 파이5는 루프당 한 프레임만 읽으므로 버퍼에
프레임이 쌓이고, `read()`는 **가장 오래된** 프레임을 반환한다.

1초만 지나도 1초 묵은 자세를 읽게 되고, **지연이 무한히 누적된다.** 현재 코드가 이 상태다.
오도메트리가 낡으면 body → world 변환이 틀리고, 칼만필터에 오염된 관측이 들어간다.

### 대책 (둘 다 적용)

**1. 피코 송신 주기를 100 Hz로 낮춘다.** 제어 루프는 1 kHz를 유지하고, 10 사이클마다 한 번만
송신한다. 대역폭이 2.8 KB/s로 떨어진다. 파이5는 30 Hz로 읽으므로 100 Hz면 충분하다.

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

[`communication.py`](../pi5/communication.py)에 `debug_decode(raw: bytes) -> str`가 있다.
raw 바이트를 사람이 읽는 텍스트로 바꾼다. 페이로드 길이로 방향을 구분한다.

```
$ cat /dev/ttyACM0 | python -m communication
[odometry] t=1234 x=0.031 y=-0.002 theta=0.001 vx=0.42 vy=0.00 omega=0.00
```

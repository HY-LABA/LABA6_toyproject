# pico_micropython — 피코 펌웨어의 MicroPython 판

`pico/` 의 C 펌웨어를 MicroPython 으로 그대로 옮긴 것이다. 프로토콜·기구학·PID·
오도메트리·핀 배치가 전부 같으므로 **pi5 쪽은 한 줄도 고칠 필요가 없다**.

---

## 1. 파일 구성

| MicroPython | 대응하는 C | 비고 |
|---|---|---|
| `config.py` | `pico/config.h` | 값 동일. ⚠ pi5 `control.verify_wheel_config()` 는 여전히 **`pico/config.h` 를 읽는다** — 값을 고치면 양쪽 다 고칠 것 |
| `kinematics.py` | `kinematics.c` | sin/cos·역행렬을 import 때 한 번만 계산 (결과 동일) |
| `encoder_pio.py` | `encoder/encoder.pio` + `encoder_pio.c` | `rp2.asm_pio` 로 1:1 이식 |
| `motor_control.py` | `motor_control.c` | duty 단위 0~1 로 C 와 동일 |
| `odometry_kalman.py` | `odometry_kalman.c` | 칼만은 보고용, pose 는 raw 적분 |
| `communication.py` | `communication.c` | 프레임 포맷 동일 |
| `main.py` | `main.c` | **전원 넣으면 자동 실행되는 진입점** |
| `bench_loop.py` | — | 한 틱이 몇 us 걸리는지 실측 (3장) |
| `goto_xy_test.py` | — | 기존 벤치 스크립트. 이번 포팅과 무관, 그대로 둠 |

---

## 2. Thonny 로 올리는 법

> **"그냥 업로드만 하면 되는 거겠지?" → 파일 복사 자체는 맞는데, 그 전에 보드에
> MicroPython 펌웨어를 한 번 구워야 한다.** 지금 보드에는 C UF2 가 들어 있어서
> 파이썬 파일을 둘 곳(파일시스템) 자체가 없다.

### 2-1. MicroPython 펌웨어 굽기 (최초 1회)

1. BOOTSEL 버튼을 누른 채 USB 를 꽂는다 → `RPI-RP2` 드라이브가 보인다.
2. Thonny → `Tools ▸ Options ▸ Interpreter` → 인터프리터를 **MicroPython (Raspberry Pi Pico)** 로,
   우하단 `Install or update MicroPython` 을 눌러 **Pico 2 W (RP2350)** 용을 설치한다.
   (직접 받으려면 micropython.org/download 의 `RPI_PICO2_W` UF2 를 드라이브에 복사.)
3. ⚠ **보드 종류를 꼭 확인할 것.** 이 프로젝트는 `pico2_w` = RP2350 이다
   (`pico/CMakeLists.txt` 의 `set(PICO_BOARD pico2_w)`). RP2040 용 UF2 를 구우면 안 올라간다.
4. ⚠ 이 순간 **C 펌웨어는 지워진다.** 되돌리려면 C UF2 를 다시 구우면 되고,
   그때는 반대로 파이썬 파일시스템이 지워진다. 둘은 공존하지 못한다.

### 2-2. 파일 올리기

Thonny 왼쪽 파일 패널에서 이 폴더의 파일들을 선택 → 우클릭 → `Upload to /`.
올릴 파일은 **7개**다:

```
config.py  kinematics.py  encoder_pio.py  motor_control.py
odometry_kalman.py  communication.py  main.py
```

(`bench_loop.py`, `goto_xy_test.py` 는 필요할 때만. `README.md` 는 올릴 필요 없다.)

보드 루트(`/`)에 평평하게 두면 된다 — 서로 `import` 로 찾는다.

### 2-3. 실행

- **자동 실행**: 이름이 `main.py` 라서 전원이 들어오면 알아서 돈다. 파이5를 꽂기만 하면 된다.
- **수동 실행**: Thonny 에서 `main.py` 를 열고 Run.

### 2-4. ⚠ 꼭 알아야 할 함정 — Ctrl-C 가 꺼진다

파이5가 보내는 건 float 이진 데이터라 **0x03 바이트가 언젠가 반드시 섞인다.**
MicroPython 은 기본적으로 stdin 의 0x03 을 Ctrl-C 로 해석해서 `KeyboardInterrupt` 를
던지는데, 그러면 제어 루프가 죽고 **모터는 마지막 듀티로 계속 돈다.** 그래서
`communication.communication_init()` 이 `micropython.kbd_intr(-1)` 로 이걸 끈다.

끄고 나면 **Thonny 의 정지 버튼(Ctrl-C)도 안 먹는다.** 그래서 `main.py` 는 시작하고
`BOOT_GRACE_S`(기본 3초) 동안 아무것도 안 하고 기다린다 — 잘못 올렸을 때 멈출 수 있는
창구가 그 3초다. 놓치면 USB 를 뽑았다 BOOTSEL 로 다시 꽂아야 한다.

### 2-5. 파이5 연결

포트는 그대로 `/dev/ttyACM0`, 115200 (`pi5/config.py`). MicroPython 도 USB CDC 로
잡히므로 바뀌는 게 없다. 두 가지만:

- **VID/PID 는 바뀐다.** udev 규칙을 VID/PID 로 걸어뒀다면 거기만 갱신.
- **Thonny 가 포트를 물고 있으면 파이5가 못 연다.** 같은 PC 에서 테스트할 때는
  Thonny 를 닫거나 `Stop/Disconnect` 할 것.

---

## 3. C 랑 파이썬이랑 속도 차이가 많이 날까

**결론부터: 절대 시간으로는 수십 배 차이가 나지만, 이 로봇의 제어 성능은 사실상
같다.** 이유는 느려지는 부분이 제어 성능을 결정하는 부분이 아니기 때문이다.

### 3-1. 안 느려지는 것 (= 중요한 것들)

| | 어디서 도는가 | C vs 파이썬 |
|---|---|---|
| 엔코더 카운팅 | **PIO 스테이트머신** | 완전히 동일. CPU 와 무관하게 sysclk 속도로 엣지를 센다 |
| PWM 생성 | **PWM 하드웨어 슬라이스** | 완전히 동일. 10 kHz 반송파는 인터프리터와 무관 |
| USB CDC 전송 | **USB 하드웨어 + DMA** | 동일 |
| 제어 주기 | 20 ms (50 Hz) | 동일 — 애초에 MicroPython 벤치에서 튜닝한 값이다 |

특히 엔코더가 PIO 라는 게 핵심이다. 옛 벤치(`goto_xy_test.py`)가 `Pin.irq()` 로 세다가
고속에서 엣지를 놓쳐 1 m 명령에 1.8 m 를 가버린 적이 있는데, 그건 "파이썬이라서"가
아니라 "인터럽트 핸들러라서"였다. PIO 로 옮기면 그 문제 자체가 사라진다.

### 3-2. 느려지는 것 (= 계산 부분)

한 틱에서 파이썬이 직접 하는 일은 순기구학·칼만·역기구학·PID·프레임 파싱이다.
RP2350 150 MHz 기준 대략:

| | 한 틱 계산 시간 | 20 ms 예산 대비 |
|---|---|---|
| C (`-O2`) | 약 20~50 us | 0.1 ~ 0.3 % |
| MicroPython | 약 1.5~3 ms | **8 ~ 15 %** |

즉 **비율로는 50배쯤 느린데, 예산의 10% 대 0.2% 라 둘 다 널널하다.** 남는 시간은
어차피 `sleep_ms` 로 버리는 시간이었다.

> 위 수치는 추정이다. 실제 값은 보드에서 `bench_loop.py` 를 Run 하면 구성요소별로
> 찍힌다 (모터 전원 내리고 돌릴 것). 합계가 20000 us 보다 충분히 작으면 문제없다.

### 3-3. 그래서 진짜로 손해 보는 것 세 가지

1. **지터.** C 는 틱 간격이 마이크로초 단위로 일정한데, 파이썬은 GC·인터프리터
   변동으로 수백 us ~ 1 ms 씩 흔들린다. 바퀴 속도를 `카운트 / dt` 로 구하는데 `dt` 는
   상수 20 ms 로 박혀 있으므로, 틱이 21 ms 걸리면 그 틱의 속도가 5% 과대평가된다.
   → GC 는 `main.py` 의 `GC_EVERY_N_TICKS` 로 예측 가능한 자리에 몰아넣어 완화했다.
   → 그래도 모자라면 `dt` 를 상수 대신 `ticks_diff` 로 잰 실측값으로 바꾸면 된다.
     (C 와 달라지는 변경이라 기본값으로는 넣지 않았다.)

2. **제어 주파수를 못 올린다.** C 라면 1 kHz 도 가능하지만 MicroPython 은 100~200 Hz
   가 현실적인 상한이다. 다만 이 프로젝트는 PID 게인이 50 Hz 에서 튜닝돼 있어서
   (`config.py` 의 `CONTROL_PERIOD_MS` 주석) 올릴 계획 자체가 없다 — 지금은 무해하다.

3. **최악값(worst case)을 보장 못 한다.** 힙이 찼을 때의 GC, 예외 처리 경로 등에서
   한 틱이 수 ms 튈 수 있다. 자동차/항공이면 문제지만, 20 ms 주기에 한두 틱 늦는
   정도는 이 로봇에서는 표적 추종 오차에 묻힌다.

### 3-4. 얻는 것

반복 속도다. C 는 고칠 때마다 CMake 빌드 → UF2 굽기 → 리셋인데, 파이썬은 Thonny 에서
파일 저장하고 Run 이다. `config.py` 의 게인·부호(`MOTOR_SIGN`)·`WHEEL_MAX_SPEED_MPS`
같은 **실측으로 정해야 하는 값이 아직 여러 개 남아 있는** 지금 단계에서는 이게 훨씬 크다.

---

## 4. C 와 의도적으로 다르게 한 곳 (전부 4군데)

값이나 제어식은 하나도 안 바꿨고, 아래만 MicroPython 사정에 맞춰 손봤다.

1. **`micropython.kbd_intr(-1)`** — 2-4 장. C 에는 대응물이 없다(필요가 없었다).
2. **stdout 논블로킹 방식** — C 는 `tud_cdc_write_available()` 로 쟀는데, MicroPython 은
   그 API 가 없어서 `select.poll(sys.stdout, POLLOUT)` 으로 같은 판단을 한다.
   목적은 동일: 호스트가 안 읽을 때 제어 루프가 멈추지 않게 한다.
3. **모터 방향 전환 시 PWM 쓰는 순서** — C 는 구동측을 먼저 올렸는데, 반대측을 먼저
   0 으로 내리도록 바꿨다. 방향이 바뀌는 틱에 BTS7960 양쪽이 동시에 high 가 되는
   순간을 없애기 위함이다 (`motor_control.py` 주석).
4. **틱 밀림 복구** — 한 주기 넘게 밀리면 기준 시각을 다시 잡는다. C 의 `sleep_until`
   에는 없던 보호인데, 인터프리터 쪽이 튈 여지가 더 커서 넣었다.

그 외 `encoder_pio.py` 의 `set(x, 0)` 한 줄(카운터 초기화)과, 파이썬 int 가 무한정밀도라
필요해진 int32 랩어라운드 처리(`encoder_read_deltas`)가 있다.

---

## 5. 아직 실기로 확인 안 한 것

CPython 에서 검증한 건 기구학(순/역 왕복 + C 공식 일치), 칼만 수열, 프레임
파싱·송신(pi5 의 실제 디코더로 디코드 확인)까지다. **보드에서 확인해야 하는 건:**

- [ ] `rp2.asm_pio` 로 옮긴 엔코더가 C PIO 와 같은 카운트를 내는가
      (바퀴를 손으로 한 바퀴 돌려서 687~688 카운트가 나오는지)
- [ ] 세 모터의 회전 방향이 명령과 맞는가 (`config.py` 의 `MOTOR_SIGN`)
- [ ] `bench_loop.py` 합계가 20000 us 안에 충분히 들어오는가
- [ ] 파이5 와 붙였을 때 오도메트리 프레임이 끊기지 않는가

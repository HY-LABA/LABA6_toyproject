# 파이5 — 관측과 궤적 계산

파이5가 하는 일은 하나다. **"어디로, 얼마나 빨리 가야 하는가"를 결정한다.**
모터를 어떻게 돌릴지는 [피코](pico-control.md)가 맡는다.

관련 문서: [비전 파이프라인](vision-pipeline.md) · [물리 계산](physics.md) · [하드웨어](hardware.md) · [통신 프로토콜](protocol.md)

---

## 1. 역할과 경계

```
카메라 프레임
   ↓  vision      YOLO 추론 → bbox → 로봇 기준 (x, y, z)
   ↓  frames      오도메트리로 world 좌표 변환
   ↓  estimator   포물선 칼만필터로 궤적 상태 갱신
   ↓  trajectory  착지점 · 착지시각 예측
   ↓  control     목표 속도 (body frame) + TTL
피코로 전송
```

파이5는 궤적 수준의 판단만 하고, 피코는 그 속도를 정확히 내는 것만 한다.
**피코는 궤적도 착지점도 모른다.**

---

## 2. 좌표계 정의

**세 좌표계가 있고, 이걸 헷갈리면 시스템 전체가 조용히 틀린 답을 낸다.**

### 2.1 World frame (W) — 관성계

- 원점: 피코 부팅 시점의 로봇 중심
- X_W: 부팅 시점 로봇 전방 / Y_W: 좌측 / Z_W: 위(중력 반대)
- 피코 오도메트리 `(x, y, theta)`가 그대로 W 기준이다
- **포물선 피팅은 반드시 이 좌표계에서 한다.** 로봇이 움직이는 좌표계에서 피팅하면
  로봇 자신의 가속도가 물체의 가속도로 잘못 들어간다

### 2.2 Body frame (B) — 로봇 기준

- 원점: **로봇 회전 중심** (통 중앙). 카메라는 여기서 약 15 cm 떨어져 있다 — 2.5절
- X_B 전방 / Y_B 좌측 / Z_B 위
- `theta` = W 기준 B의 회전각

```
p_W = R(theta) · p_B + (x_odom, y_odom)
p_B = R(−theta) · (p_W − (x_odom, y_odom))

R(θ) = [[cosθ, −sinθ],
        [sinθ,  cosθ]]
```

Z는 회전 영향이 없으므로 `Z_W = Z_B` (로봇이 기울지 않는다는 가정).

### 2.3 Camera / Image frame (C)

- 광축은 +Z_B (수직 위)
- 이미지 좌표 (u, v): u는 오른쪽, v는 **아래쪽**이 증가 방향
- 물리적 장착 규약은 [hardware.md 3.1장](hardware.md#31-카메라)

**⚠ 핀홀 역투영을 쓰면 안 된다.** 번들 렌즈는 대각 화각 140°의 어안계다
([physics.md 7.2장](physics.md#72--이-렌즈는-핀홀이-아니다)). 등입체각 모델을 쓴다.

```
① 픽셀 -> 입사 방향
   r     = hypot(u − u₀, v − v₀)
   θ     = 2·asin( r / (2·f_px) )        ← 등입체각 역변환 (핀홀이면 atan(r/f_px))
   φ     = atan2( −(v − v₀), u − u₀ )    ← v축 부호 반전에 주의

② bbox 폭 -> 직선거리 R
   f_local = f_px · cos(θ/2)             ← 국소 배율. 가장자리에서 최대 18% 작다
   R       = f_local · W_real / w_px

③ 방향 × 거리 -> body 좌표
   x_B = R·sinθ·cosφ ,  y_B = R·sinθ·sinφ ,  z_B = R·cosθ
```

θ가 작으면 핀홀 식으로 수렴한다. 화면 중앙만 쓰면 차이가 없지만, 광각의 이점을 살리려면
반드시 이 형태를 써야 한다 — 핀홀로 계산하면 입사각 42°에서 z가 79% 틀린다.

`(u₀, v₀)`는 **주점(principal point)**이며 1456×1088에서 대략 이미지 중심 (728, 544)이다.
정확한 값은 **`cv2.fisheye.calibrate`**로 구한다.

> ⚠ **구코드([`pi5/vision.py`](../pi5/vision.py))의 버그:** `_pixel_to_meters_xy`가
> `cx_px * z / f`로 주점을 빼지 않고, y축 부호 반전도 없으며, 핀홀을 가정한다.
> 새 구현([`catcher/frames.py`](../catcher/frames.py))에서 모두 해결했다.

조립 후 카메라가 규약에서 ψ만큼 틀어졌다면 `config.CAMERA_YAW_RAD`에 넣고 2D 회전으로
보정한다.

### 2.4 z 추정 — 실물 치수 기반

현재 코드는 "1 m 거리에서의 bbox 픽셀 크기" DB(`REFERENCE_SIZE_AT_1M`)를 쓴다. 이 방식은
렌즈나 해상도를 바꾸면 DB 전체를 다시 실측해야 한다. **실물 치수(m)를 넣고 f_px로 계산하는
방식으로 바꾼다:**

```
R = f_px·cos(θ/2) × W_real / w_px     ← w_px 는 bbox '폭' (높이 아님)
```

(θ는 2.3절의 입사각. 중심부에서는 `R ≈ z = f_px·W_real/w_px`로 수렴한다)

`config.OBJECT_SIZE_M = {"can": {"w": 0.066, "h": 0.122}, ...}` — 줄자로 한 번 재면 끝이고,
렌즈를 바꿔도 `f_px`만 갱신하면 된다.

**bbox 폭을 쓴다 — 높이가 아니다.**
글로벌 셔터라 기하 왜곡은 없으므로(롤링 셔터였다면 높이가 2% 왜곡됐다,
[physics.md 7.1장](physics.md#71-글로벌-셔터의-이점)) 선택 기준은 **물체 자세**다.
폭이 물체의 짧은 치수(캔 66 mm)에 대응해 회전에 덜 민감하다.

**자세 의존성 문제:** 캔을 눕히면 보이는 폭이 66 mm가 아니라 122 mm다. 물체가 회전하며
떨어지면 z 추정이 흔들린다. 대책:

- 폭이 물체의 **짧은 치수**에 가까운지 확인하고, bbox 종횡비로 자세를 판별해
  해당 치수를 골라 쓴다
- 이 오차는 백색잡음이 아니라 상관 잡음이라 칼만필터가 완전히는 못 지운다.
  측정 노이즈 R을 넉넉히 잡고, 리스크로 인지해 둔다
  ([open-questions.md](open-questions.md#1-리스크))

**단안 카메라에서 크기 정보는 유일한 스케일 소스다.** 각도만으로는 "가까운 작은 물체"와
"먼 큰 물체"를 구분할 수 없다. z 추정을 포기할 수 없고, 이 정확도가 시스템 성능의 상한을
결정한다 ([physics.md 8.3장](physics.md#83-x-y-오차는-z보다-훨씬-작다)).

---

### 2.5 카메라 오프셋

카메라가 **통 입구 림의 가장자리**에 있으므로 광학 중심이 로봇 회전 중심에서
**약 15 cm 떨어져 있다.** 관측은 카메라 기준이고 제어 목표는 로봇 중심 기준이므로 보정한다.

```
p_robot = p_camera + CAMERA_OFFSET_M        (body frame, 상수 벡터 덧셈)
```

로봇이 회전하지 않으므로(ω = 0) 회전 항이 없어 단순 덧셈이다. 만약 나중에 θ 제어를
넣는다면 이 오프셋도 함께 회전시켜야 한다.

**대가:** 통 중앙으로 떨어지는 물체가 화면 가장자리로 밀려 **z ≈ 0.36 m에서 프레임을
벗어난다.** 착지 0.09초 전이고 예측이 이미 수렴한 뒤라 칼만필터 외삽으로 충분하다
([physics.md 6장](physics.md#카메라를-입구에-두는-대가--오프셋)).

---

## 3. 왜 2단계 구조를 버리는가

기존 설계는 `① 확정 → 속도 3프레임 측정 → 예측` 후 `② 재보정 루프`로 나뉘어 있었다.
문제는 **3프레임 차분으로 구한 vz의 오차가 6.5 m/s**라는 것이다. vz 자체가 수 m/s인데
오차가 그 수준이면 착지 예측이 성립하지 않는다.
(프레임레이트를 올릴수록 Δt가 짧아져 **오히려 나빠진다** — 차분 방식의 근본 한계다.)

같은 관측을 비행 전체에 걸쳐 중력 제약 하에 피팅하면 오차가 **0.183 m/s로 35배** 좋아진다.
계산 근거는 [physics.md 8장](physics.md#8-속도-추정-오차--설계를-바꾼-계산).

그래서 **관측·추정·예측·명령을 매 프레임 한꺼번에 하는 단일 루프**로 바꾼다.
부수 효과로 **`recalibrate()`라는 함수가 아예 필요 없어진다.** 재보정이 "같은 업데이트를
한 번 더"가 되기 때문이다. 지금 막혀 있는 지점이 설계 변경으로 사라진다.

---

## 4. 메인 루프

### 4.1 상태 기계

루프는 두 상태를 오간다. **탐색은 물체를 찾고, 추적은 정밀하게 따라간다.**

```
        ┌──────────────────────────────────────────┐
        │                                          │
        ▼                                          │
   ┌─────────┐   대상 클래스 검출    ┌──────────┐   │ 착지 / 소실 / 타임아웃
   │ SEARCH  │ ────────────────────▶│  TRACK   │───┘   (정지 명령 후 복귀)
   │ 전체프레임 │                     │ ROI 크롭  │
   │ 640 축소 │ ◀────────────────────│  네이티브  │
   └─────────┘   ROI에서 검출 실패     └──────────┘
                 (칼만 상태는 유지)
```

**왜 두 상태인가:** 전체 프레임을 640으로 줄여 추론하면 bbox가 2 m에서 11 px까지 작아져
거리 오차가 2.3배로 늘고 YOLO 검출률도 떨어진다. ROI 크롭은 네이티브 해상도를 유지해
bbox 26 px를 지킨다
([vision-pipeline.md 2장](vision-pipeline.md#2--추론-해상도가-z-정확도를-2배-이상-좌우한다)).

SEARCH의 부정확한 첫 관측은 문제되지 않는다. 칼만필터가 어차피 큰 불확실성으로 시작하고,
2~3프레임 뒤 TRACK으로 넘어가면서 정밀도가 확보된다.

### 4.2 루프

```python
def run(cam, link, est, clock):
    state = State.SEARCH

    while True:
        frame, t_cap = cam.capture()                       # 60 Hz
        pose = link.latest_odometry()                      # drain-to-latest

        # ── 관측 ────────────────────────────────────────
        if state is State.SEARCH:
            det = vision.detect_full(frame)                # 640 리사이즈
        else:
            uv = est.predict_image_position(t_cap, pose)   # 칼만 → 이미지 좌표
            det = vision.detect_roi(frame, uv)             # 640 네이티브 크롭
            if det is None:
                state = State.SEARCH                       # 재획득 (est는 유지)
                continue

        if det is None:
            continue

        # ── 추정 ────────────────────────────────────────
        p_body  = vision.to_body_xyz(det)                  # 2.3, 2.4절
        p_world = frames.body_to_world(p_body, pose)       # 2.2절
        est.update(p_world, t_cap, det)                    # 5절
        state = State.TRACK

        # ── 예측 · 명령 ──────────────────────────────────
        if not est.confident:
            continue
        land, t_land, ok = trajectory.predict_landing(est.state, ...)
        if not ok:
            continue
        link.send(control.to_velocity_command(land, t_land, pose, est.confidence))

        # ── 종료 판정 ────────────────────────────────────
        if (reason := termination_reason(est, t_cap, clock)) is not None:
            link.send(control.stop_command())
            utils.log(f"cycle end: {reason}")
            est.reset()
            state = State.SEARCH                           # 다음 물체 대기
```

**한 번의 반복이 관측·추정·예측·명령을 모두 한다.** 별도의 "확정 단계"도 "재보정 루프"도
없다. 코드가 줄고, 지연이 줄고, 정확도가 오른다.

### 4.3 종료 조건

| 조건 | 의미 | 후속 |
|---|---|---|
| `est.z ≤ Z_CATCH` | 착지 (정상 종료) | 정지 → SEARCH |
| 관측 소실 후 `LOST_TIMEOUT_S` 경과 | 물체를 놓침 | 정지 → SEARCH |
| 사이클 경과 `CYCLE_TIMEOUT_S` 초과 | 무한 루프 방지 | 정지 → SEARCH |

뒤의 둘은 기존 설계에 없던 안전장치다. **어느 경우든 정지 명령을 보내고 SEARCH로 돌아간다** —
`break`로 프로그램을 끝내지 않으므로 연속 캐치가 자연스럽게 동작한다.

성공/실패는 판정하지 않는다 (범위 밖).

### 4.4 타이밍 예산

센서는 60 fps(프레임당 **16.7 ms**)를 낸다. 파이프라인 추정치는:

| 단계 | 예산 | 비고 |
|---|---|---|
| 캡처 (Picamera2) | ~3 ms | 1456×1088 |
| ROI 크롭 + 전처리 | ~2 ms | numpy 슬라이스 |
| Hailo 추론 | ~10 ms | YOLO11n @ 640 |
| 후처리 (NMS, 좌표 역변환) | ~1 ms | |
| 좌표 변환 + 칼만 + 제어 | ~2 ms | 6×6 행렬, 무시할 수준 |
| 시리얼 송수신 | ~1 ms | |
| **합계** | **~19 ms** | **16.7 ms를 초과한다** |

**전 프레임 처리는 빠듯하다.** 초과하면 격프레임(30 fps)으로 떨어뜨린다 — σ_vz가
0.183 → 0.259 m/s가 되지만 시스템은 성립한다
([physics.md 7.4장](physics.md#74-프레임레이트와-처리-예산)).
**Phase 4에서 추론 지연을 재고 결정한다.**

---

## 5. 궤적 추정기 (ProjectileEstimator)

[`pi/kalman.py`](../pi/kalman.py)의 `ProjectileKalman`을 [`catcher/estimator.py`](../catcher/estimator.py)로 이식하고
아래 4가지를 개선한다.

- **상태:** `[X, Y, Z, VX, VY, VZ]` (world frame, 6차원)
- **예측 모델:** 포물선 운동 — 가속도 `(0, 0, −g_eff)`, `g_eff`는 클래스별
  ([physics.md 1장](physics.md#1-운동-모델--포물선-운동) ·
  [4장](physics.md#4-항력--포물선-모델이-양쪽-축에서-흔들린다))
- **관측:** world 좌표 위치 3개

### 개선 1 — 프레임별 측정 노이즈 R

현재는 고정 스칼라인데, 실제 오차는 z와 화면상 위치에 따라 크게 달라진다. 매 프레임 계산한다:

```
σ_z = z · σ_w / w_px
σ_x = √( (z·σ_u/f)²  +  ((u−u₀)·σ_z/f)²  +  σ_odom² )
σ_y = √( (z·σ_v/f)²  +  ((v−v₀)·σ_z/f)²  +  σ_odom² )
```

- 첫 항: 픽셀 위치 지터
- 둘째 항: **z 오차가 x, y로 전파되는 성분** — 화면 가장자리일수록 커진다
- 셋째 항: 로봇 오도메트리 자체의 불확실성 (옴니휠은 슬립이 있다)

세 성분은 실제로 서로 상관되어 있으나 대각 근사로 충분하다.

### 개선 2 — 클래스별 프로세스 노이즈 q

항력 모델 오차를 흡수한다. 종이컵 > 캔 ≈ 페트병 순으로 크게 잡는다.
q를 키우면 필터가 최신 관측을 더 신뢰하게 되어 모델 편향이 줄어든다.

**⚠ 수평 축(X, Y)에도 충분한 q를 준다.** 항력은 수평 속도도 감속시킨다 (100 g 기준
0.52초간 1.5~2.6 cm). q를 0으로 두면 필터가 "수평 등속"을 확신해 관측을 무시하게 된다
([physics.md 4장](physics.md#4-항력--포물선-모델이-양쪽-축에서-흔들린다)).

### 개선 3 — 초기화

현재 `_init_state`는 속도를 0으로 두는데, 낙하물의 속도는 0이 아니다. 속도 공분산을
`(10 m/s)²`로 크게 두면 2~3프레임 안에 수렴한다. 그 전까지는 `confident`가 False라
명령을 내지 않는다.

**confident 판정:** 속도 공분산의 trace가 임계 이하일 때. 대략 3프레임이면 통과한다.

### 개선 4 — ROI 중심 예측

추적 단계가 크롭 위치를 알아야 하므로, 다음 프레임의 **이미지 좌표**를 내주는 메서드가
필요하다. world 상태를 현재 로봇 자세로 body에 되돌린 뒤 핀홀 정투영한다.

```python
def predict_image_position(self, t: float, pose) -> tuple[float, float]:
    p_world = self.predict_state(t)[:3]          # 등가속 외삽
    x, y, z = frames.world_to_body(p_world, pose)
    u = u0 + x * f_px / z
    v = v0 - y * f_px / z                        # v축 부호 반전
    return u, v
```

크롭이 프레임 경계를 벗어나면 안쪽으로 밀어 넣는다 (clamp).

---

## 6. 착지 예측

```
Z(t) = Z_now + VZ·t − 0.5·g_eff·t²  =  Z_CATCH   를 t에 대해 푼다
```

양의 실근 중 **큰 쪽**을 택한다 (위로 던져 올린 경우 하강 구간에서 잡아야 하므로).
근이 없으면 물체가 `Z_CATCH`까지 내려오지 않는다는 뜻이므로 명령을 내지 않고 넘긴다.

착지점 (world frame):

```
X_land = X_now + VX · t_land
Y_land = Y_now + VY · t_land
```

`Z_CATCH`는 착지 판정 평면이다. **카메라를 통 입구 림 높이에 두므로 `Z_CATCH = 0`**이다
([hardware.md 4장](hardware.md#4-통-쓰레기통)).

> ⚠ 현재 `config.CATCH_HEIGHT_M = 0.0`은 "물체가 카메라에 닿는 순간"을 뜻해서 틀렸다.

---

## 7. 목표 속도 생성

```python
def to_velocity_command(land, t_land, pose, confidence):
    d_world = land[:2] - (pose.x, pose.y)                 # 남은 이동 벡터 (world)
    T       = max(t_land - latency_estimate, T_MIN)       # 남은 시간
    v_world = d_world / T                                 # 필요 평균 속도
    v_world = clamp_norm(v_world, V_MAX)                  # 최고속도 제한
    v_world *= confidence_gain(confidence)                # 초기 불확실 구간 감쇠
    v_body  = rotate(v_world, -pose.theta)                # body frame으로
    return VelocityCommand(v_body[0], v_body[1], ttl_s=COMMAND_TTL_S)
```

**왜 이렇게 단순해도 되는가:** 30~60 Hz로 매번 다시 계산하므로 일종의 비례 유도(proportional
guidance)가 된다. 궤적을 미리 계획할 필요가 없다. 정교한 가감속 프로파일을 넣어도 다음
프레임에 덮어쓰이므로 이득이 없다.

### 세 가지 안전장치

| 장치 | 목적 |
|---|---|
| `T_MIN` (예: 0.08 s) | 착지 직전 T→0에서 속도가 발산하는 것을 막는다 |
| `clamp_norm(V_MAX)` | 방향은 유지하고 크기만 자른다. **축별로 자르면 방향이 틀어진다** |
| `confidence_gain` | 칼만필터 수렴 전에는 명령을 줄여 엉뚱한 방향으로 튀는 것을 막는다 |

### 도달 가능성 판정

`|d_world| > d_max(T)`([physics.md 3장](physics.md#3-가속도--2구간-모델)의 공식)이면
물리적으로 불가능하다. **경고를 로그에 남기되 명령은 그대로 낸다** — 최대한 가까이 가는
것이 최선이기 때문이다.

`ttl_s`는 명령 유효시간(예: 0.1 s)이다. 피코는 TTL이 만료되면 자동 정지한다.
파이5가 죽거나 USB가 빠져도 로봇이 폭주하지 않는다 — **워치독이다**
([protocol.md](protocol.md#3-ttl-워치독)).

---

## 8. 파일 구조와 모듈 책임

```
pi5/
├── main.py            # 연속 추정 루프 (4절)
├── config.py          # 전 파라미터
├── vision.py          # 캡처 + YOLO(detect_full/detect_roi) + 픽셀→body 좌표
├── frames.py          # ★신규  body ↔ world 변환
├── estimator.py       # ★신규  포물선 칼만필터 (pi/kalman.py 이식)
├── trajectory.py      # 착지 예측 (recalibrate 삭제)
├── control.py         # 목표 속도 + 도달가능성 판정
├── communication.py   # USB 시리얼 + 프로토콜
├── utils.py           # 로깅 + 타이밍
└── sim/               # ★신규  시뮬레이터
    ├── world.py       #   물체 낙하 + 로봇 운동 모델
    ├── fake_camera.py #   Camera 대체 (bbox 직접 합성)
    └── fake_link.py   #   SerialLink 대체 (피코 모델)
```

| 파일 | 책임 | 의존 |
|---|---|---|
| `main.py` | 상태 기계(SEARCH/TRACK), 루프 오케스트레이션, 종료 판정 | 전부 |
| `config.py` | 모든 파라미터. **다른 파일에 상수 하드코딩 금지** | 없음 |
| `vision.py` | 프레임 → `Detection` → `body_xyz`. ROI 크롭·좌표 역변환을 내부에서 끝낸다 | config |
| `frames.py` | 순수 좌표 변환 함수. 상태 없음 | 없음 |
| `estimator.py` | 관측 누적 → 궤적 상태 + ROI 중심 예측. **유일한 상태 보유자** | config, frames |
| `trajectory.py` | 상태 → 착지점/착지시각. 순수 함수 | config |
| `control.py` | 착지점 + 자세 → 속도 명령. 순수 함수 | config |
| `communication.py` | 시리얼 I/O + 프레임 인코딩 | config |

**설계 원칙:** `frames`, `trajectory`, `control`은 상태 없는 순수 함수 모듈이다. 하드웨어
없이 단위 테스트가 가능하다. 상태를 가지는 것은 `estimator`(궤적), `communication`(포트),
`vision`(카메라 핸들)뿐이다.

`main.py`는 진짜 구현과 시뮬레이터를 **의존성 주입**으로 갈아끼울 수 있어야 한다.

---

## 9. 현재 코드 대비 변경점

| 파일 | 변경 | 이유 |
|---|---|---|
| `main.py` | **재작성** — 2단계 → 연속 루프 + SEARCH/TRACK 상태 기계 | 3, 4절 |
| `vision.py` | 주점 보정, y축 부호, z는 bbox 폭 기준, 속도 계산 제거,
  **`detect_full`/`detect_roi` 2단계 추론** | 2.3, 2.4절 · [vision-pipeline](vision-pipeline.md) |
| `trajectory.py` | `recalibrate()` **삭제**, `predict_landing` 유지 | 3절 |
| `estimator.py` | **신규** — `pi/kalman.py` 이식 + R/q/초기화/ROI예측 개선 | 5절 |
| `frames.py` | **신규** — body ↔ world 변환 | 2.2절 |
| `control.py` | 목표좌표 → **목표 속도** 산출로 변경, 도달가능성 판정 추가 | 7절 |
| `communication.py` | 페이로드 의미 변경, 타임스탬프, drain-to-latest | [protocol.md](protocol.md) |
| `config.py` | 파라미터 대폭 추가·개편 | 10절 |
| `sim/` | **신규** | 시뮬레이터 |
| `pi/kalman.py` | `estimator.py`로 이식 후 **폴더 삭제** | — |

---

## 10. 파라미터

`pi5/config.py`에 모은다.

### 캘리브레이션으로 구하는 값

| 값 | 방법 |
|---|---|
| `FOCAL_LENGTH_PX` | **어안** 체커보드 캘리브레이션 (이론값 792) |
| `PROJECTION_MODEL` | 캘리브레이션으로 확인 (기본 `equisolid`) |
| `PRINCIPAL_POINT` (u₀, v₀) | 체커보드 카메라 캘리브레이션 |
| `CAMERA_YAW_RAD` | 조립 후 회전 오차 보정 |
| `CAMERA_OFFSET_M` | 로봇 중심 → 카메라 광학 중심 벡터 (약 0.15 m). 자로 실측 |
| `OBJECT_SIZE_M` | 줄자 실측 (**지금 바로 가능**) |
| `GRAVITY_BY_CLASS` (g_eff) | 낙하 영상 피팅 |
| `Z_CATCH` | **0** — 카메라가 통 입구 평면에 있다 |

### 실측 또는 튜닝이 필요한 값

| 값 | 임시값 | 확정 시점 |
|---|---|---|
| `V_MAX` | 1.22 m/s | 무부하 속도 실측 |
| `A_MAX` | 2.45 m/s² (μ=0.5 견인한계) | **마찰계수 μ 실측** — 캐치 반경을 좌우한다 |
| `CAMERA_FPS` | 60 (캡처) | 확정 |
| `PROCESS_EVERY_N` | 1 또는 2 | 추론 지연 실측 후 결정 (4.4절) |
| `SIGMA_BBOX_PX` (σ_w, σ_u) | 2 px | 정지 물체 촬영 통계 ([vision-pipeline.md 7장](vision-pipeline.md#7-검증-지표)) |
| `ROI_SIZE_PX` | 640 (추론 입력과 일치) | 확정 |
| `SIGMA_ODOM_M` | 미정 | 오도메트리 드리프트 측정 |
| `PROCESS_NOISE_BY_CLASS` | 미정 | 시뮬레이터 + 실측 튜닝 |
| `T_MIN` | 0.08 s | 시뮬레이터 |
| `COMMAND_TTL_S` | 0.1 s | 시뮬레이터 |
| `LOST_TIMEOUT_S` | 미정 | 시뮬레이터 |
| `CYCLE_TIMEOUT_S` | 미정 | 시뮬레이터 |
| `CONFIDENCE_THRESHOLD` | 미정 | 시뮬레이터 |

### 연동 후 채우는 값

| 값 | 비고 |
|---|---|
| `YOLO_MODEL_PATH` | YOLO11n 학습 → Hailo 컴파일 후 `.hef` 경로 ([vision-pipeline.md](vision-pipeline.md)) |
| `SERIAL_PORT` | 피코 연결 후 확인 (`/dev/ttyACM0` 등) |

### 구현이 필요한 연동 코드

| 항목 | 내용 |
|---|---|
| `vision.Camera` | Picamera2 실제 캡처 |
| `vision._HailoYolo` | HailoRT 추론 (⚠ API는 실제 버전 문서로 검증 필요) |
| `communication.SerialLink.__init__` | pyserial 포트 오픈 |

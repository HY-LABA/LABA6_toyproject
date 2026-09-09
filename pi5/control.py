"""착지 예측 → 피코에 보낼 **목표 속도 벡터**.

역할 분담
---------
    파이5   궤적을 풀고 **착지점(월드 좌표)과 남은 시간**을 안다 → 그것만 보낸다
    피코    "목표 − 자기 오도메트리"로 남은 거리를 직접 구하고, 속도를 만들어
            바퀴 3개로 분해(역기구학)한 뒤 각 바퀴에 PID를 건다

★ 왜 상대 거리가 아니라 **절대 좌표**인가
------------------------------------------
예전 프로토콜은 "여기서부터 얼마"를 보냈고, 피코가 자기 오도메트리를 빼지 않아
매 사이클 이미 간 거리를 다시 갔다 — **오버슈트가 구조적으로 보장돼 있었다.**
절대 좌표를 보내면 피코가 `남은거리 = 목표 − 자기 pose` 를 계산하므로 그 실패가
**구조적으로 불가능하다.** 이중으로 뺄 값 자체가 없다.

⚠ 여기서 "월드"는 **피코 오도메트리 원점(부팅 시점)** 이다. 피코는 pose 를 θ 로
  회전시켜 적분하는데(`odometry_kalman.c`), 우리는 ω=0 으로만 굴리므로 θ≈0 이고
  world 축 ≡ body 축이다. **회전을 쓰기 시작하면 이 등식이 깨진다.**

남은 시간을 같이 보내는 이유
----------------------------
피코가 "언제까지" 가야 하는지 모르면 최대속도로 가다 급정지한다 — 정지거리만큼
구조적으로 오버슈트한다. **남은거리 ÷ 남은시간**이면 목표에 가까워질수록 목표속도가
저절로 줄어서, 별도 감속 프로파일 없이 P 제어가 감속기 역할을 한다:

    남은거리 0.50m / 남은시간 0.50s → 1.00 m/s
    남은거리 0.20m / 남은시간 0.25s → 0.80 m/s
    남은거리 0.03m / 남은시간 0.10s → 0.30 m/s

★ 속도 상한은 방향에 따라 다르다
--------------------------------
3륜 옴니에서 모터가 제한하는 건 **바퀴 속도**이지 body 속도가 아니다. 같은 body
속도라도 진행 방향에 따라 어느 한 바퀴가 더 빨리 돌아야 한다. 실측 계산으로
body 1 m/s 를 내는 데 필요한 최대 바퀴 속도가 **0.866 ~ 1.000 m/s** — 방향에 따라
15.5% 차이다.

그래서 `ROBOT_MAX_SPEED_MPS` 하나로 자르면 유리한 방향에서 13.4%를 손해 본다.
여기서는 **가고 싶은 방향에 대해 실제 바퀴 속도를 계산해서** 상한을 정한다.

**포화되면 크기만 깎고 방향은 보존해야 한다.** 세 바퀴 중 하나만 잘리면 비율이
깨져서 속도뿐 아니라 **진행 방향까지 틀어진다.** 그래서 세 값을 같은 비율로 줄인다.

⚠ **이 상한 계산은 이제 피코가 해야 한다** — 속도를 만드는 쪽이 피코이기 때문이다.
  파이에 남은 `max_body_speed()` 는 두 용도뿐이다: ① 표적 도달 가능성 판정
  (`tracker._reach`), ② 로그용 예측. 피코가 body 속력을 상수 하나로 자르면
  방향별 이득(최대 15.5%)이 사라지고, 그러면 ①이 실제보다 낙관하게 된다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import config


@dataclass
class TargetCommand:
    """★ 피코로 실제로 보내는 명령. **월드 좌표계의 도착 지점**이다.

    `target_x/y` 는 피코 오도메트리 원점(부팅 시점) 기준 절대 좌표다. 피코가
    `남은거리 = (target − 자기 pose)` 를 스스로 계산하므로, 파이는 "이미 간 만큼"을
    빼서 보내지 않는다 — 빼서 보내면 피코가 또 빼서 두 번 빠진다.

    `time_remaining_s` 는 **이 명령을 만든 순간** 기준으로 착지까지 남은 시간이다.
    피코는 매 틱 `dt` 만큼 깎아 쓰고, 다음 명령이 오면 새 값으로 덮는다.
    0 이하이면 이미 늦은 것이므로 상한 속도로 붙는다.

    `timeout_s` 는 구동시간이 아니라 **워치독**이다 — 이 시간 안에 새 명령이 안 오면
    피코가 알아서 멈춘다. 파이가 죽었을 때 로봇이 계속 굴러가는 걸 막는 장치다.
    """

    target_x: float
    target_y: float
    time_remaining_s: float
    timeout_s: float


@dataclass
class DriveCommand:
    """body frame 속도(m/s) 명령.

    ⚠ **구동 경로는 이제 `TargetCommand` 다.** 이 타입은 두 군데서만 쓴다:
      ① `teleop_test.py` — 게임패드 스틱은 본질적으로 속도지 좌표가 아니다.
         모터 응답·최대속도·정지거리를 재는 데 이 경로가 필요하다
      ② 정지(`STOP`)와 로그용 예측 — "피코가 이 목표점으로 낼 속도"를 미리 계산해
         어느 바퀴가 한계에 붙는지 로그에 남긴다

    timeout_s는 구동시간이 아니라 **워치독**이다 — 이 시간 안에 새 명령이 안 오면
    피코가 알아서 멈춘다. 파이가 죽었을 때 로봇이 계속 굴러가는 걸 막는 장치이고,
    정상 동작 중에는 매 사이클 새 명령이 오므로 만료되지 않는다.
    """

    target_vx: float
    target_vy: float
    timeout_s: float

    @property
    def speed(self) -> float:
        return math.hypot(self.target_vx, self.target_vy)

    @property
    def heading_deg(self) -> float:
        """진행 방향(도). body +X(로봇 우측)에서 반시계로 잰다 — 90°가 전방(M1)."""
        return math.degrees(math.atan2(self.target_vy, self.target_vx))


STOP = DriveCommand(target_vx=0.0, target_vy=0.0, timeout_s=config.DRIVE_TIMEOUT_S)


# ── 역기구학 (피코와 같은 식) ─────────────────────────────────────────────

def wheel_speeds(vx: float, vy: float, omega: float = 0.0) -> tuple[float, float, float]:
    """body 속도 → 바퀴 3개의 선속도(m/s). `pico/kinematics.c` 와 같은 식이다.

        w_i = -sin(b_i)·vx + cos(b_i)·vy + L·omega

    ⚠ **이 값을 피코로 보내지는 않는다.** 실제 구동용 역기구학은 피코가 한다.
      여기서 계산하는 이유는 두 가지다:
        ① 방향별 속도 상한을 정하려면 어느 바퀴가 먼저 포화되는지 알아야 한다
        ② 실기에서 "어느 바퀴가 한계에 붙었나"를 로그로 봐야 튜닝이 된다

    ⚠ 부호 규약이 피코와 같아야 한다. `verify_wheel_config()` 로 대조할 것.
    """
    L = config.WHEEL_MOUNT_RADIUS_M
    return tuple(
        -math.sin(b) * vx + math.cos(b) * vy + L * omega
        for b in config.WHEEL_ANGLES_RAD
    )  # type: ignore[return-value]


def max_body_speed(vx: float, vy: float, omega: float = 0.0) -> float:
    """**이 방향으로** 갈 때 바퀴가 포화되지 않는 최대 body 속력(m/s).

    단위 속도로 가정하고 바퀴 속도를 구한 뒤, 가장 큰 것이 바퀴 한계에 닿는
    지점을 찾는다. 방향이 유리하면 `ROBOT_MAX_SPEED_MPS` 보다 크게 나온다.
    """
    n = math.hypot(vx, vy)
    if n < 1e-9:
        return config.ROBOT_MAX_SPEED_MPS
    peak = max(abs(w) for w in wheel_speeds(vx / n, vy / n, omega / n if omega else 0.0))
    if peak < 1e-9:
        return config.ROBOT_MAX_SPEED_MPS
    return config.WHEEL_MAX_SPEED_MPS / peak


# ── 계획 ──────────────────────────────────────────────────────────────────

def to_target_command(landing_xy: tuple[float, float], origin_odom: tuple[float, float],
                      time_remaining: float) -> TargetCommand:
    """착지점(트랙 원점 기준) + 트랙 원점의 오도메트리 → **월드 좌표 목표점**.

    `landing_xy` 는 트랙이 시작된 시점의 **로봇 중심**을 원점으로 한 좌표이고
    (`tracker.Tracker.landing()`), `origin_odom` 은 바로 그 시점에 피코가 보고한
    pose 다. 둘을 더하면 피코 오도메트리 원점 기준 절대 좌표가 된다.

    ⚠ 이 덧셈은 **둘이 같은 축일 때만** 성립한다. `landing_xy` 는 `tracker.landing()`
      이 이미 `cam_to_robot()` 으로 로봇 body 축까지 돌려서 준 값이다 —
      `trajectory` 가 내놓는 카메라 이미지 축 원값을 여기로 바로 넘기면 안 된다
      (거리는 맞고 방향만 `config.CAMERA_YAW_RAD` 만큼 틀어진다).

    ⚠ 이 덧셈은 **θ≈0** 일 때만 성립한다. 트랙 좌표는 body 축이고 오도메트리는
      world 축이라, 로봇이 돌면 더하기 전에 회전시켜야 한다. 지금은 ω=0 이다.

    ⚠ 여기서 "이미 간 거리"를 빼지 않는다. 그걸 하는 건 피코다.
    """
    return TargetCommand(
        target_x=landing_xy[0] + origin_odom[0],
        target_y=landing_xy[1] + origin_odom[1],
        # ★ 남은시간을 DRIVE_AGGRESSION 으로 나눠서 보낸다 (2026-09-07).
        #   피코는 `속도 = 남은거리 ÷ 남은시간` 이므로, 남은시간을 줄여 보내면 그만큼
        #   빨리 간다. 8.0 이면 사실상 "항상 최대속도" 이고, 어차피 피코가 방향별
        #   바퀴 상한으로 자르므로 못 내는 속도를 요구하는 일은 없다.
        #   근거와 튜닝 기준은 config.DRIVE_AGGRESSION 주석 참고.
        time_remaining_s=time_remaining / config.DRIVE_AGGRESSION,
        timeout_s=config.DRIVE_TIMEOUT_S,
    )


def held_target(first: TargetCommand, elapsed_s: float,
                timeout_s: float | None = None) -> TargetCommand:
    """★ 테스트 전용 — **첫 예측을 고정한 채** 재전송할 명령을 만든다.

    `main.py --once` 가 쓴다. 목표점(x, y)은 첫 예측 그대로 두고 **남은 시간만 실제로
    흐른 만큼 깎는다.** 남은 시간까지 얼려버리면 착지 시각이 영영 안 오는 셈이라
    피코가 감속을 시작하는 시점이 실제와 달라진다 — 고정하고 싶은 건 좌표지 시계가
    아니다.

    프로덕션 루프는 이 함수를 쓰지 않는다. 매 프레임 새로 푼 예측을 보낸다.
    """
    return TargetCommand(
        target_x=first.target_x,
        target_y=first.target_y,
        time_remaining_s=first.time_remaining_s - elapsed_s,
        timeout_s=first.timeout_s if timeout_s is None else timeout_s,
    )


def to_drive_command(landing_xy: tuple[float, float], time_remaining: float,
                     odometry_xy: tuple[float, float] = (0.0, 0.0),
                     omega: float = 0.0) -> DriveCommand:
    """착지점 + 남은 시간 + 지금까지 이동한 거리 → 목표 속도 벡터.

    ⚠ **이 값을 피코로 보내지 않는다.** 구동 명령은 `to_target_command()` 다.
      여기 있는 건 피코가 같은 입력으로 낼 속도의 **예측치**이고, 두 군데서 쓴다:
        ① 로그 — `describe()` 로 어느 바퀴가 한계에 붙는지 보려면 속도가 있어야 한다
        ② 도달 가능성 판정 — `tracker._reach()` 가 같은 상한 계산을 쓴다
      **피코가 이것과 다른 식으로 속도를 만들면 로그와 실제가 조용히 갈린다.**
      피코 쪽 식이 바뀌면 여기도 같이 바꿀 것.

    `landing_xy`는 **트랙을 시작한 시점의 로봇 중심** 기준이고, `odometry_xy`는
    같은 기준의 누적 이동량이다(`tracker.moved_since_start()`가 만들어 준다).
    피코가 주는 값은 부팅 이후 누적이라 **그대로 넣으면 두 번째 투척부터 어긋난다.**

    `omega`는 지금 쓰지 않는다(항상 0). 회전이 필요해질 때 구조를 안 바꾸고 값만
    넣을 수 있도록 자리만 만들어 뒀다 — 넣으면 상한 계산에도 반영된다.
    """
    remain_x = landing_xy[0] - odometry_xy[0]
    remain_y = landing_xy[1] - odometry_xy[1]
    distance = math.hypot(remain_x, remain_y)

    # 도착했으면 멈춘다. 이 여유가 없으면 남은거리가 0 근처에서 부호가 뒤집히며
    # 앞뒤로 덜컹거린다.
    if distance <= config.POSITION_TOLERANCE_M:
        return STOP

    # 남은 시간이 0이거나 음수면(이미 착지 시각을 지났으면) 시간으로 나눌 수 없다.
    # 이 경우엔 갈 수 있는 만큼 간다 — 늦었으니 최대로 붙는다.
    if time_remaining <= 1e-3:
        speed = math.inf
    else:
        speed = distance / time_remaining

    # ★ 상한은 **이 방향에서** 바퀴가 포화되지 않는 값이다. 방향에 따라 다르다.
    limit = max_body_speed(remain_x, remain_y, omega)
    if speed > limit:
        speed = limit

    scale = speed / distance
    return DriveCommand(
        target_vx=remain_x * scale,
        target_vy=remain_y * scale,
        timeout_s=config.DRIVE_TIMEOUT_S,
    )


# ── 진단 ──────────────────────────────────────────────────────────────────

def describe(cmd: DriveCommand, omega: float = 0.0) -> str:
    """로그용 한 줄. 어느 바퀴가 한계에 붙었는지 보인다."""
    w = wheel_speeds(cmd.target_vx, cmd.target_vy, omega)
    lim = config.WHEEL_MAX_SPEED_MPS
    parts = " ".join(f"w{i}={x:+.2f}{'!' if abs(x) > lim * 0.98 else ''}"
                     for i, x in enumerate(w, 1))
    return (f"v=({cmd.target_vx:+.2f},{cmd.target_vy:+.2f}) "
            f"|v|={cmd.speed:.2f} {cmd.heading_deg:+.0f}°  {parts}")


def verify_wheel_config(pico_config: str | None = None) -> list[str]:
    """`pico/config.h` 를 읽어 바퀴 설정이 일치하는지 대조한다. 불일치 목록을 반환.

    같은 물리 상수가 파이와 피코 두 곳에 있으면 **한쪽만 고쳤을 때 조용히 틀린다.**
    파일을 파싱하는 게 투박하지만, 값이 어긋난 채로 굴러가는 것보다 낫다.
    """
    import pathlib
    import re

    path = pathlib.Path(pico_config) if pico_config else (
        pathlib.Path(__file__).resolve().parent.parent / "pico" / "config.h")
    if not path.is_file():
        return [f"pico/config.h 를 못 찾았다: {path}"]

    raw = path.read_text(encoding="utf-8", errors="replace")
    text = re.sub(r"//.*", "", raw)   # 값 뒤에 붙은 주석은 걷어내고 파싱한다
    problems: list[str] = []

    m = re.search(r"WHEEL_ANGLES_RAD\s*\[[^\]]*\]\s*=\s*\{(.*?)\}", text, re.S)
    if not m:
        problems.append("pico/config.h 에서 WHEEL_ANGLES_RAD 를 못 읽었다")
    else:
        # (float)(M_PI * 7.0 / 6.0) 같은 식을 M_PI만 치환해 평가한다.
        exprs = [e.strip() for e in m.group(1).split(",") if e.strip()]
        vals = []
        for e in exprs:
            e = e.replace("(float)", "").replace("M_PI", str(math.pi)).strip()
            try:
                vals.append(float(eval(e, {"__builtins__": {}}, {})))  # noqa: S307
            except Exception:  # noqa: BLE001
                problems.append(f"각도 식을 해석 못 했다: {e}")
        if len(vals) == len(config.WHEEL_ANGLES_RAD):
            for i, (a, b) in enumerate(zip(config.WHEEL_ANGLES_RAD, vals), 1):
                if abs(a - b) > 1e-6:
                    problems.append(
                        f"바퀴{i} 장착각 불일치: pi5={math.degrees(a):.2f}° "
                        f"pico={math.degrees(b):.2f}°")
        elif not problems:
            problems.append(f"바퀴 개수 불일치: pi5={len(config.WHEEL_ANGLES_RAD)} "
                            f"pico={len(vals)}")

    m = re.search(r"#define\s+WHEEL_MOUNT_RADIUS_M\s+([0-9.]+)", text)
    if m and abs(float(m.group(1)) - config.WHEEL_MOUNT_RADIUS_M) > 1e-6:
        problems.append(f"WHEEL_MOUNT_RADIUS_M 불일치: pi5={config.WHEEL_MOUNT_RADIUS_M} "
                        f"pico={m.group(1)}")

    # 이름이 없으면 **불일치를 못 잡고 조용히 통과한다.** 그래서 못 찾은 것 자체를
    # 문제로 보고한다 — 피코에서 상수 이름이 바뀌면 여기가 먼저 시끄러워야 한다.
    for name, expected in (("WHEEL_MAX_SPEED_MPS", config.WHEEL_MAX_SPEED_MPS),
                           ("POSITION_TOLERANCE_M", config.POSITION_TOLERANCE_M)):
        m = re.search(rf"#define\s+{name}\s+([0-9.]+)f?", text)
        if not m:
            problems.append(f"pico/config.h 에서 {name} 를 못 읽었다")
        elif abs(float(m.group(1)) - expected) > 1e-6:
            problems.append(f"{name} 불일치: pi5={expected} pico={m.group(1)}")
    return problems


if __name__ == "__main__":
    bad = verify_wheel_config()
    print("바퀴 설정 대조:", "일치" if not bad else "불일치 " + str(len(bad)) + "건")
    for b in bad:
        print("  ⚠", b)
    print()
    print(f"{'방향':>6}{'최대 body속도':>14}{'그때 바퀴속도':>28}")
    for deg in range(0, 360, 30):
        th = math.radians(deg)
        v = max_body_speed(math.cos(th), math.sin(th))
        w = wheel_speeds(math.cos(th) * v, math.sin(th) * v)
        print(f"{deg:>5}°{v:>12.3f} m/s   " + " ".join(f"{x:+.3f}" for x in w))

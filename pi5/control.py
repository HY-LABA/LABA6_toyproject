"""착지 예측 → 피코에 보낼 **목표 속도 벡터**.

역할 분담
---------
    파이5   궤적을 풀고 착지점·남은시간을 안다 → "어느 방향으로 몇 m/s"를 정한다
    피코    그 속도 벡터를 바퀴 3개로 분해(역기구학)하고 각 바퀴에 PID를 건다

**파이가 내는 건 바퀴별 속도가 아니라 body 속도 벡터 하나다.** `(vx, vy)` 두 값이
곧 "로봇 전체가 우측(+X)/전방(+Y)으로 각각 몇 m/s"이고, 이걸 12바이트로 보낸다.
바퀴별 분해는 `pico/kinematics.c` 의 `inverse_kinematics()` 가 한다.

**남은 거리 ÷ 남은 시간**이라 목표에 가까워질수록 목표속도가 저절로 줄어든다.
별도 감속 프로파일 없이 P 제어가 감속기 역할을 한다:

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
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import config


@dataclass
class DriveCommand:
    """피코로 보내는 명령. target_*는 **body frame 속도(m/s)** 다.

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

def to_drive_command(landing_xy: tuple[float, float], time_remaining: float,
                     odometry_xy: tuple[float, float] = (0.0, 0.0),
                     omega: float = 0.0) -> DriveCommand:
    """착지점 + 남은 시간 + 지금까지 이동한 거리 → 목표 속도 벡터.

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

    m = re.search(r"#define\s+MAX_BODY_SPEED_MPS\s+([0-9.]+)f?", text)
    if m and abs(float(m.group(1)) - config.WHEEL_MAX_SPEED_MPS) > 1e-6:
        problems.append(f"속도 한계 불일치: pi5 WHEEL_MAX_SPEED_MPS="
                        f"{config.WHEEL_MAX_SPEED_MPS} pico MAX_BODY_SPEED_MPS={m.group(1)}")
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

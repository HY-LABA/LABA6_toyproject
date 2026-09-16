"""피코 구동만 따로 검증한다 — 카메라·YOLO·궤적예측 없이.

파이와 피코를 USB 로 연결하고 돌리면 된다. 8방향(전/후/좌/우 + 대각선 4)으로
지정 거리만큼 갔다가 되돌아온다.

    python pico_test.py --port /dev/ttyACM0 --all
    python pico_test.py --port /dev/ttyACM0 --dir front --dist 1.0
    python pico_test.py --port /dev/ttyACM0 --all --dist 0.5 --speed 0.2   # 좁은 곳

★ 왜 속도 명령이 아니라 목표점 명령인가
----------------------------------------
"0.3 m/s 로 3.3초" 식으로 속도 명령(12B)을 주면 가감속 구간 때문에 실제 거리가
안 맞는다. 대신 **목표점 명령(16B)** 을 쓴다 — `main.py` 가 실제로 쓰는 바로 그
경로다. 피코가 `남은거리 = 목표 − 자기 pose` 를 스스로 계산하고
`POSITION_TOLERANCE_M`(2cm) 안에 들어오면 알아서 선다.

그래서 이 스크립트는 **구동 경로 전체를 실기로 검증한다**:
기구학 → PID → 모터 부호 → 엔코더 → 오도메트리 → 목표점 폐루프.
`main.py` 에서 남는 미검증 부분은 카메라·YOLO·궤적피팅뿐이 된다.

★ 목표점은 절대좌표다
---------------------
`TargetCommand` 의 x,y 는 **피코 오도메트리 원점(부팅 시점) 기준 절대좌표**다.
그래서 "여기서 1m 앞으로"를 하려면 지금 pose 를 읽어서 더해야 한다 — 이 스크립트가
매 구간 시작마다 그렇게 한다.

`time_remaining_s` 는 매 틱 **남은거리 ÷ 목표속도**로 다시 계산해서 보낸다.
고정값으로 보내면 0 이 되는 순간 피코가 "이미 늦었다"고 보고 상한 속도로 붙는다
(`pico/main.c` DRIVE_TARGET 블록).

★ 무엇을 보고 판정하나
----------------------
    ① 눈으로 본 방향   — 명령한 방향으로 실제로 갔나. **이게 1순위 판정이다.**
    ② 오도메트리 이동  — 피코가 "내가 이만큼 갔다"고 보고한 값
    ③ 줄자로 잰 거리   — 실제로 간 거리

⚠ ②가 명령과 맞아도 ①이 틀릴 수 있다. `MOTOR_SIGN` 이 엔코더 측정과 PWM 출력
  **양쪽에** 곱해지기 때문이다(`pico/main.c`, `motor_control.c`). 배선이 통째로
  뒤집혀 있으면 PID 는 자기 자신과 멀쩡히 맞물려 돌고 오도메트리도 명령과 일치하는데
  로봇만 정반대로 간다. 2026-09 MicroPython 벤치에서 실제로 겪은 버그가 이것이다.

⚠ ②와 ③이 다르면 바퀴가 미끄러진 것이다. 엔코더는 바퀴 회전만 세므로 슬립을
  모른다. ③ < ② 면 그 차이가 곧 미끄러진 양이다.

★ 왕복 오차가 오도메트리 신뢰도다
---------------------------------
기본으로 갔다가 되돌아온다(`--no-return` 으로 끌 수 있다). 8방향을 다 돌고 나서
출발점으로 얼마나 돌아왔는지가 곧 **오도메트리 누적 오차**다. 이 값이 크면
`main.py` 의 목표점 추종도 그만큼 어긋난다 — 같은 pose 를 쓰기 때문이다.

안전
----
- 주변 치우고, 사람이 지켜보는 상태에서만. 장애물 감지가 없다.
- 처음엔 `--dist 0.5 --speed 0.2` 로 짧고 느리게 시작할 것.
- Ctrl+C 로 언제든 정지. 어느 경로로 끝나도 STOP 을 두 번 보낸다.
- 스크립트가 죽어도 피코 워치독(`config.DRIVE_TIMEOUT_S`)이 알아서 세운다.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import communication
import config
import control

_S = math.sqrt(0.5)     # 대각선 성분 — 크기가 1 이 되도록

# ⚠ 2026-09-16 — 이 파일은 config.CAMERA_YAW_RAD 를 전혀 안 쓴다(카메라 자체가
#   없는 테스트라서). 그런데 오늘 이 테스트로 "front/right/left 전부 정확히 반대로
#   간다"(균일 반전)를 확인했다 — 09-09 에는 이 테스트에서 정상이었던 것이 UF2
#   재빌드·배선 등으로 그사이 다시 뒤집힌 것으로 보인다(원인 미상). UF2 를 이번엔
#   건드리지 않기로 해서, 이 스크립트만의 구동 보정을 여기 하나로 모아 둔다.
#   구동을 다시 실측했는데 결과가 바뀌면(=UF2 를 고쳤다면) 제일 먼저 이 값을
#   +1 로 되돌릴 것 — 안 그러면 이 테스트 도구 자체가 반대로 보고한다.
DRIVE_SIGN = -1  # 2026-09-16: 균일 반전 확인, 상쇄. UF2 고치면 +1 로.

# body frame: +X = 로봇 우측, +Y = 로봇 전방(M1). theta 를 안 쓰므로 world 축과 같다.
DIRECTIONS: dict[str, tuple[float, float, str]] = {
    "front": (0.0, +1.0, "전방 (+Y, M1 쪽)"),
    "back":  (0.0, -1.0, "후방 (-Y)"),
    "right": (+1.0, 0.0, "우측 (+X)"),
    "left":  (-1.0, 0.0, "좌측 (-X)"),
    "fr":    (+_S, +_S, "전방-우측 대각"),
    "fl":    (-_S, +_S, "전방-좌측 대각"),
    "br":    (+_S, -_S, "후방-우측 대각"),
    "bl":    (-_S, -_S, "후방-좌측 대각"),
}
ORDER = ["front", "back", "right", "left", "fr", "fl", "br", "bl"]


def _latest_pose(link, last, timeout_s: float = 1.0):
    """최신 오도메트리를 받아 (x, y) 로. 없으면 `last` 를 그대로 돌려준다."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        odom = link.try_receive_odometry()
        if odom is not None:
            return odom.xy
        if last is not None:
            return last
        time.sleep(0.005)
    return last


def goto(link, target_xy, speed: float, hz: float, timeout_s: float, last_pose,
         t_floor: float = 0.05):
    """목표 절대좌표까지 간다. → (도착했나, 마지막 pose)

    매 틱 `time_remaining = 남은거리 / speed` 로 다시 계산해 보낸다. 그래야 피코가
    계산하는 목표속도(`남은거리 ÷ 남은시간`)가 `speed` 로 일정하게 유지된다.

    `t_floor` 는 그 값의 하한이다. 이게 있으면 목표 근처에서 자연스럽게 감속한다 —
    남은거리가 `speed * t_floor` 아래로 내려가면 속도가 거리에 비례해 줄기 때문이다.
    **최대속도 시험(`--max`)에서는 이걸 거의 0 으로 줘서** 톨러런스 직전까지 전속으로
    붙게 한다. 그래야 정지거리(오버슈트)가 실제 값으로 드러난다.
    """
    tx, ty = target_xy
    period = 1.0 / hz
    t0 = time.monotonic()
    next_tick = t0
    pose = last_pose

    while time.monotonic() - t0 < timeout_s:
        pose = _latest_pose(link, pose, timeout_s=0.02)
        if pose is None:
            print("   ⚠ 오도메트리를 못 받는다 — 포트/펌웨어 확인")
            return False, None

        rx, ry = tx - pose[0], ty - pose[1]
        dist = math.hypot(rx, ry)
        if dist <= config.POSITION_TOLERANCE_M:
            return True, pose

        link.send_target(control.TargetCommand(
            target_x=tx, target_y=ty,
            time_remaining_s=max(dist / speed, t_floor),
            timeout_s=config.DRIVE_TIMEOUT_S))

        print(f"   pose=({pose[0]:+.3f},{pose[1]:+.3f})  남은거리={dist * 100:5.1f}cm  ",
              end="\r", flush=True)

        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = time.monotonic()

    return False, pose


def run_leg(link, name: str, dist_m: float, speed: float, hz: float,
            go_back: bool, last_pose, t_floor: float = 0.05):
    ux, uy, label = DIRECTIONS[name]
    dx, dy = ux * dist_m * DRIVE_SIGN, uy * dist_m * DRIVE_SIGN

    print(f"\n── {name}  {label}  {dist_m:.2f} m ──")
    for n in (3, 2, 1):
        print(f"   {n}...", end="\r", flush=True)
        time.sleep(1.0)

    start = _latest_pose(link, last_pose, timeout_s=2.0)
    if start is None:
        print("   ⚠ 오도메트리를 하나도 못 받았다 — 중단")
        return None
    print(f"   출발 pose=({start[0]:+.3f},{start[1]:+.3f})            ")

    target = (start[0] + dx, start[1] + dy)
    timeout_s = dist_m / min(speed, 2.0) * 4.0 + 3.0     # 넉넉하게
    ok, pose = goto(link, target, speed, hz, timeout_s, start, t_floor)
    link.send_command(control.STOP)

    # 멈출 때까지 계속 읽는다 — 관성으로 더 가는 만큼이 곧 정지거리(오버슈트)다.
    t_stop = time.monotonic()
    while time.monotonic() - t_stop < 1.2:
        pose = _latest_pose(link, pose, timeout_s=0.05)

    overshoot = math.hypot(pose[0] - target[0], pose[1] - target[1])
    moved = math.hypot(pose[0] - start[0], pose[1] - start[1])
    print(" " * 70, end="\r")
    print(f"   {'도착' if ok else '⚠ 타임아웃'} — 오도메트리 이동 {moved * 100:.1f} cm "
          f"(명령 {dist_m * 100:.0f} cm)")
    radius_cm = config.CATCH_OPENING_DIAMETER_M / 2 * 100
    mark = "" if overshoot * 100 <= radius_cm else f"   ← 통 입구 반지름 {radius_cm:.0f}cm 초과!"
    print(f"   목표점과의 최종 오차(정지거리 포함): {overshoot * 100:.1f} cm{mark}")
    print(f"   ★ 눈으로 본 방향이 '{label}' 이 맞나?  줄자로 잰 거리는?")

    if go_back:
        print("   되돌아간다...")
        ok2, pose = goto(link, start, speed, hz, timeout_s, pose)
        link.send_command(control.STOP)
        time.sleep(0.4)
        pose = _latest_pose(link, pose, timeout_s=0.5)
        back_err = math.hypot(pose[0] - start[0], pose[1] - start[1])
        print(" " * 70, end="\r")
        print(f"   복귀 {'완료' if ok2 else '⚠ 타임아웃'} — 출발점과 "
              f"{back_err * 100:.1f} cm 차이")
    return pose


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=config.SERIAL_PORT, help="피코 시리얼 포트")
    ap.add_argument("--dir", choices=ORDER, default=None, help="한 방향만")
    ap.add_argument("--all", action="store_true", help="8방향 전부 (사이에 멈춤)")
    ap.add_argument("--dist", type=float, default=1.0, help="구간 거리(m)")
    ap.add_argument("--speed", type=float, default=0.3,
                    help="목표 속도(m/s). 처음엔 낮게 (기본 0.3)")
    ap.add_argument("--max", action="store_true",
                    help="피코가 낼 수 있는 최대 속도로 간다 (방향별 1.22~1.41 m/s). "
                         "톨러런스 직전까지 전속이라 정지거리가 그대로 드러난다. "
                         "공간을 넉넉히 비우고 쓸 것")
    ap.add_argument("--hz", type=float, default=30.0, help="명령 전송 주기")
    ap.add_argument("--no-return", action="store_true",
                    help="되돌아오지 않는다 (기본: 갔다가 복귀)")
    args = ap.parse_args()

    if not args.all and args.dir is None:
        ap.error("--dir 또는 --all 중 하나는 지정할 것")
    if args.port is None:
        ap.error("--port 를 지정할 것 (예: --port /dev/ttyACM0)")
    if 1.0 / args.hz >= config.DRIVE_TIMEOUT_S:
        print(f"⚠ --hz={args.hz:.0f} 가 워치독({config.DRIVE_TIMEOUT_S * 1000:.0f}ms)보다 "
              f"느리다 — 명령 사이에 로봇이 자꾸 멈춘다.")

    # --max: 피코가 자기 상한(max_body_speed)으로 자르도록 아주 큰 속도를 요구하고,
    #        time_remaining 하한도 없애서 톨러런스 직전까지 감속하지 않게 한다.
    speed = 50.0 if args.max else args.speed
    t_floor = 1e-3 if args.max else 0.05

    names = ORDER if args.all else [args.dir]
    speed_txt = "최대 (피코 상한 1.22~1.41 m/s)" if args.max else f"{args.speed:.2f} m/s"
    print(f"포트 {args.port} | 거리 {args.dist:.2f} m | 속도 {speed_txt} | "
          f"{'왕복' if not args.no_return else '편도'}")
    if args.max:
        print("⚠ 최대속도 모드 — 감속 프로파일이 없어 목표를 지나쳤다가 선다.")
        print(f"  진행 방향으로 최소 {args.dist + 1.0:.1f} m 는 비워둘 것.")
    print("★ 판정 기준은 **눈으로 본 방향**이다 — 오도메트리가 명령과 맞아도")
    print("  MOTOR_SIGN 때문에 로봇만 반대로 갈 수 있다 (docstring 참고).")

    link = communication.SerialLink(port=args.port)
    pose = None
    origin = None
    try:
        for i, name in enumerate(names):
            pose = run_leg(link, name, args.dist, speed, args.hz,
                           not args.no_return, pose, t_floor)
            if pose is None:
                break
            if origin is None:
                origin = pose
            if i < len(names) - 1:
                input("\n   Enter 를 누르면 다음 방향> ")
    except KeyboardInterrupt:
        print("\n인터럽트 — 정지")
    finally:
        try:
            link.send_command(control.STOP)
            time.sleep(0.05)
            link.send_command(control.STOP)   # 워치독 안전마진
        except Exception as exc:  # noqa: BLE001 - 정지 시도는 실패해도 계속 정리한다
            print(f"정지 명령 실패: {exc}")
        link.close()

    if origin is not None and pose is not None and len(names) > 1 and not args.no_return:
        drift = math.hypot(pose[0] - origin[0], pose[1] - origin[1])
        print(f"\n전체 누적 오도메트리 드리프트: {drift * 100:.1f} cm")
        print("  이 값이 크면 main.py 의 목표점 추종도 같은 만큼 어긋난다 — 같은 pose 를 쓴다.")
    print("\n끝.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

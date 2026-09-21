"""Xbox 컨트롤러로 피코 모터 응답을 수동으로 테스트한다.

카메라·YOLO 없이 피코 브링업만 따로 검증하기 위한 도구. 왼쪽 스틱을 body-frame
목표속도(vx, vy)로 매핑해 매 틱 피코로 쏜다 — main.py가 쓰는 것과 동일한 시리얼
프로토콜(communication.py)을 그대로 타므로, 이걸로 검증한 값은 실제 파이프라인
에서도 그대로 유효하다.

pico/TODO.md 2번(최대속도·가속시간·정지거리 실측)을 반복 재현 가능하게 하려고
만들었다. 회전(omega)은 프로토콜에 없으므로 다루지 않는다 — pico/communication.h 참고.

    python teleop_test.py --port /dev/ttyACM0                  # 기본 0.5 m/s 상한
    python teleop_test.py --port /dev/ttyACM0 --max-speed 1.8   # 최대속도 실측 시

★ 거리 실측 모드 (--go-distance) — 2026-09-07 추가
----------------------------------------------------
조이스틱 없이, **정해진 속도로 정해진 거리만큼 직진시키고 자동으로 멈춘 뒤 결과를
찍어주는 모드.** pico/TODO.md의 "오도메트리 스케일·최대속도·가속도 실측"을 사람이
스틱으로 어림잡지 않고 반복 가능하게 재려고 만들었다.

    python teleop_test.py --port /dev/ttyACM0 --go-distance 1.0

**원리** — 이 스크립트는 "1m 갔다"를 스스로 증명할 방법이 없다. 로봇은 자기
오도메트리가 1.0m를 봤다고 판단하는 순간 멈출 뿐이다. 그래서 오도메트리 자체가
맞는지는 **사람이 줄자로 실제 이동 거리를 재서, 이 스크립트가 출력하는 값과
비교해야** 확인된다. 화면에 나오는 "오도메트리 기준 이동거리"가 줄자 실측과
가까우면 인코더 스케일이 맞는 것이고, 계속 짧게/길게 나오면 그 비율만큼
`pico/config.h`의 `ENCODER_COUNTS_PER_REV`(또는 `WHEEL_DIAMETER_M`)가 틀린 것이다.

같은 실행에서 위치의 시간 미분(오도메트리 pose 차분 — 피코의 텔레메트리 속도
`odom.vx/vy`는 시정수 ~1.4초짜리 칼만 스무딩이 걸려 있어(`odometry_kalman.c`)
이런 짧은 구간 측정에는 못 쓴다, 그래서 여기서는 pose로 직접 재계산한다)로
가속 구간의 도달 속도·도달 시간도 같이 추정해서, 정지 후 관성으로 더 미끄러진
거리(정지거리)까지 한 번에 보여준다 — 우선순위 1·2·3번(오도메트리·최대속도·가속도
실측)을 한 스크립트로 커버한다.

    python teleop_test.py --port /dev/ttyACM0 --go-distance 1.0 --test-speed 0.5 --heading 90

`--heading`은 `control.DriveCommand.heading_deg`와 같은 규약이다 — body +X(로봇
우측)에서 반시계로 재고, 90°가 전방(M1)이다. 기본값 90°(전방)로 두면 보통은
줄자를 로봇 앞에 놓고 재기 편하다.

★ 고정시간 속도 테스트 모드 (--speed-test) — 2026-09-15 추가
--------------------------------------------------------------
"이 속도로 N초 동안 가라"고만 명령하고, 실제로 낸 속도를 재는 모드. `--go-distance`가
"거리 목표"를 기준으로 서는 것과 달리 이건 **거리와 무관하게 딱 정해진 시간만** 밀어붙인다
— 도달 여부를 안 따지므로 "명령한 속도를 실제로 낼 수 있는가" 자체를 보는 데 더 직접적이다.

    python teleop_test.py --port /dev/ttyACM0 --speed-test 1.8 --speed-test-duration 1.0

⚠ **읽고 쓸 것 — 이 모드는 1.8 m/s를 그대로 시험하지 못할 수 있다.**
피코 펌웨어(`pico/main.c`)는 어떤 경로로 온 명령이든 마지막에 `max_body_speed()`로
방향별 바퀴 상한을 걸어 **소프트웨어적으로 자른다** — 그 상한의 기준값이
`pico/config.h`의 `WHEEL_MAX_SPEED_MPS`(현재 1.22, 유리한 방향이면 최대 1.41까지)다.
1.8 m/s를 명령해도 이 값을 넘는 순간 무조건 잘려서 나간다. 즉:

  · 실측 결과가 이 상한 근처(1.22~1.41 m/s대)에서 **평평하게 멈춘다** → 지금은
    소프트웨어 클램프가 병목이다. 진짜 모터 한계가 1.8인지는 `pico/config.h`의
    `WHEEL_MAX_SPEED_MPS`를 올리고(`pi5/config.py`도 같은 값으로 맞추고)
    재플래시한 뒤에야 실측할 수 있다 — 이 스크립트 혼자서는 그 이상을 못 낸다.
  · 그 상한에도 못 미치고 더 낮은 값에서 멈춘다 → 클램프보다 먼저 다른 무언가
    (모터 토크·배터리 전압 강하·PID 게인·바퀴 슬립)가 병목이라는 뜻이라, 상한을
    올려봐야 소용없고 그쪽을 봐야 한다.

이 스크립트는 매 틱 명령한 속도와, 오도메트리 pose 차분으로 잰 실제 순간속도를
같이 찍고, 마지막에 그 방향의 소프트웨어 상한(`control.max_body_speed`)과 비교해서
위 두 경우 중 어느 쪽인지 판정까지 같이 보여준다.

의존성: pip install pygame pyserial (또는 apt python3-pygame) — **단, --go-distance
--speed-test 모드는 조이스틱을 안 쓰므로 pygame이 없어도 동작한다** (아래에서 지연
import로 분기했다).

컨트롤러는 USB-A로 라즈베리파이에 유선 연결한다. 축 부호가 몸체와 안 맞으면
(예: 스틱을 앞으로 밀었는데 로봇이 옆으로 감) 실행 중 찍히는 cmd=(vx,vy) 부호를
보고 --invert-x/--invert-y/--swap-xy로 맞출 것 — 정확한 배선 방향은 조립 후가
아니면 알 수 없다.

정지: Ctrl+C, 또는 컨트롤러 B 버튼(SDL 매핑에 따라 다를 수 있음 — 안 먹으면
Ctrl+C 쓸 것). 어느 경로로 종료해도 STOP을 두 번 보내고 닫는다. 스크립트가
죽어도 피코 워치독(config.DRIVE_TIMEOUT_S)이 알아서 정지시킨다.

⚠ 안전 — 1.8m/s는 이 로봇 기준으로 빠르다. 장애물 감지가 없으므로, 반드시 앞뒤로
  최소 2~3m 이상 뚫린 공간을 확보하고, 사람이 바로 옆에서 지켜보며 Ctrl+C를 쥐고
  있는 상태에서만 --speed-test를 쓸 것.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # 헤드리스 라파이에서 디스플레이 요구 안 하게

import communication
import config
import control


def _open_joystick(index: int):
    try:
        import pygame
    except ImportError:
        sys.exit("pygame이 없다 — pip install pygame (또는 apt install python3-pygame)")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= index:
        sys.exit(f"컨트롤러를 못 찾았다 (index={index}). USB-A로 라즈베리파이에 "
                 f"연결됐는지, `lsusb`에 잡히는지 확인할 것.")
    js = pygame.joystick.Joystick(index)
    js.init()
    print(f"컨트롤러: {js.get_name()}  축 {js.get_numaxes()}개  버튼 {js.get_numbuttons()}개")
    return pygame, js


def _apply_deadzone(v: float, deadzone: float) -> float:
    if abs(v) < deadzone:
        return 0.0
    # 데드존 경계에서 뚝 끊기지 않게 나머지 구간을 0~1로 재스케일
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - deadzone) / (1.0 - deadzone)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=config.SERIAL_PORT,
                     help="피코 시리얼 포트 (예: /dev/ttyACM0)")
    ap.add_argument("--max-speed", type=float, default=0.5,
                     help=f"스틱 최대치에서 낼 속도(m/s). 로봇 이론상한은 "
                          f"{config.ROBOT_MAX_SPEED_MPS} — 처음엔 낮게 잡을 것 (기본 0.5)")
    ap.add_argument("--deadzone", type=float, default=0.15)
    ap.add_argument("--hz", type=float, default=30.0,
                     help="명령 전송 주기. 워치독(timeout_s)보다 충분히 빨라야 안 끊긴다")
    ap.add_argument("--joystick-index", type=int, default=0)
    ap.add_argument("--invert-x", action="store_true")
    ap.add_argument("--invert-y", action="store_true")
    ap.add_argument("--swap-xy", action="store_true", help="축이 뒤바뀌어 있으면")

    g = ap.add_argument_group("거리 실측 모드 (--go-distance 를 주면 조이스틱 대신 이걸 쓴다)")
    g.add_argument("--go-distance", type=float, default=None, metavar="M",
                    help="이 거리(m)만큼 오도메트리 기준으로 직진시키고 자동 정지 후 "
                         "결과를 출력한다. 주면 조이스틱 모드 대신 이 모드로 진입한다")
    g.add_argument("--test-speed", type=float, default=0.3, metavar="M/S",
                    help="--go-distance 에서 낼 순항 속도 (기본 0.3 — 처음엔 낮게)")
    g.add_argument("--heading", type=float, default=90.0, metavar="DEG",
                    help="--go-distance/--speed-test 진행 방향(도). "
                         "control.DriveCommand.heading_deg와 같은 규약: body +X(우측)에서 "
                         "반시계, 90°=전방(M1). 기본 90°")
    g.add_argument("--settle-s", type=float, default=0.6, metavar="초",
                    help="--go-distance/--speed-test 정지 명령 뒤에도 이만큼 더 오도메트리를 "
                         "받아 정지거리(관성 미끄러짐)를 측정한다 (기본 0.6s)")
    g.add_argument("--timeout-s", type=float, default=10.0, metavar="초",
                    help="--go-distance 가 이 시간 안에 목표 거리에 못 미치면 강제 종료 "
                         "(도달 불가 상황에서 무한 루프 방지, 기본 10s)")

    g2 = ap.add_argument_group("고정시간 속도 테스트 모드 (--speed-test 를 주면 이걸 쓴다)")
    g2.add_argument("--speed-test", type=float, default=None, metavar="M/S",
                     help="이 속도(m/s)로 --speed-test-duration 초 동안 직진 명령만 계속 "
                          "보낸다 (거리 목표 없음). 주면 조이스틱/--go-distance 대신 이 "
                          "모드로 진입한다. ⚠ 피코가 WHEEL_MAX_SPEED_MPS로 방향별 상한을 "
                          "걸어 넘는 값은 자동으로 잘린다 — 모듈 docstring 참고")
    g2.add_argument("--speed-test-duration", type=float, default=1.0, metavar="초",
                     help="--speed-test 지속 시간 (기본 1.0초)")

    args = ap.parse_args()

    if args.port is None:
        sys.exit("--port를 지정할 것 (예: --port /dev/ttyACM0). config.SERIAL_PORT도 아직 비어 있다.")

    if args.speed_test is not None:
        return _run_speed_test(args)

    if args.go_distance is not None:
        return _run_distance_test(args)

    period = 1.0 / args.hz
    if period >= config.DRIVE_TIMEOUT_S:
        print(f"⚠ --hz={args.hz:.0f}가 너무 느리다 (주기 {period * 1000:.0f}ms >= "
              f"워치독 {config.DRIVE_TIMEOUT_S * 1000:.0f}ms) — 명령 사이에 로봇이 자꾸 멈춘다.")

    pygame, js = _open_joystick(args.joystick_index)
    link = communication.SerialLink(port=args.port)

    print(f"시작 — 최대 {args.max_speed:.2f} m/s, {args.hz:.0f}Hz. Ctrl+C 또는 B 버튼으로 정지.")
    try:
        next_tick = time.monotonic()
        while True:
            pygame.event.pump()

            if js.get_numbuttons() > 1 and js.get_button(1):  # Xbox B (SDL xinput 기준)
                print("\nB 버튼 — 정지하고 종료")
                break

            x = _apply_deadzone(js.get_axis(0), args.deadzone)
            y = _apply_deadzone(js.get_axis(1), args.deadzone)
            if args.swap_xy:
                x, y = y, x
            if args.invert_x:
                x = -x
            if args.invert_y:
                y = -y

            # body frame: +X = 로봇 우측, +Y = 로봇 전방(M1). 스틱 위(-1)가 전진이다.
            # 배선이 뒤바뀌어 로봇이 엉뚱하게 가면 --invert-*/--swap-xy 로 맞출 것.
            target_vx = x * args.max_speed          # 스틱 우 → +X (우측)
            target_vy = -y * args.max_speed         # 스틱 위 → +Y (전방)

            cmd = control.DriveCommand(target_vx=target_vx, target_vy=target_vy,
                                        timeout_s=config.DRIVE_TIMEOUT_S)
            link.send_command(cmd)

            odom = link.try_receive_odometry()
            status = f"cmd=({target_vx:+.2f},{target_vy:+.2f})"
            if odom is not None:
                status += (f"  odom pos=({odom.x:+.3f},{odom.y:+.3f}) "
                           f"vel=({odom.vx:+.2f},{odom.vy:+.2f})")
            print(status + " " * 10, end="\r")

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()  # 밀렸으면 다시 기준을 잡는다 (계속 밀리지 않게)
    except KeyboardInterrupt:
        print("\n인터럽트 — 정지")
    finally:
        try:
            link.send_command(control.STOP)
            time.sleep(0.05)
            link.send_command(control.STOP)  # 워치독 안전마진: 한 번 더
        except Exception as exc:  # noqa: BLE001 - 정지 시도는 실패해도 계속 정리한다
            print(f"정지 명령 실패: {exc}")
        link.close()
        pygame.joystick.quit()
        pygame.quit()
    return 0


def _run_distance_test(args) -> int:
    """조이스틱 없이 --go-distance 만큼 직진시키고 결과를 출력한다.

    조이스틱 모드와 시리얼 프로토콜을 완전히 공유한다(`communication.py`의
    `send_command`/`try_receive_odometry`, `control.DriveCommand`) — 여기서 잰
    값이 실제 캐치 루프(`main.py`)에서도 그대로 유효하다는 근거가 그것이다.

    ⚠ 이 함수는 **오도메트리 자체의 정확도를 증명하지 못한다.** "목표 거리에
    도달했다"는 판정 자체가 오도메트리를 보고 내리는 판정이라 순환 논리다. 이
    스크립트가 하는 일은 딱 하나 — **오도메트리가 스스로 "1.0m 갔다"고 믿는 순간을
    정확히 잡아서 로봇을 세우는 것**뿐이다. 그 순간 로봇이 물리적으로 몇 m
    갔는지는 사람이 줄자로 재서 이 스크립트의 출력과 비교해야 한다. 그 차이가
    바로 인코더 스케일 오차다.
    """
    import math
    import time as _time

    import communication
    import config
    import control

    heading_rad = math.radians(args.heading)
    vx_dir, vy_dir = math.cos(heading_rad), math.sin(heading_rad)

    period = 1.0 / 50.0    # 피코 제어주기(20ms)보다 넉넉히 빠르게, 워치독 여유 있게
    if period >= config.DRIVE_TIMEOUT_S:
        print(f"⚠ 명령 주기 {period*1000:.0f}ms가 워치독 {config.DRIVE_TIMEOUT_S*1000:.0f}ms"
              f"보다 느리다 — 명령 사이에 로봇이 자꾸 멈출 수 있다.")

    link = communication.SerialLink(port=args.port)

    def _await_odometry(timeout_s: float = 2.0):
        """첫 오도메트리가 올 때까지 대기(짧게 폴링). 없으면 None."""
        deadline = _time.monotonic() + timeout_s
        while _time.monotonic() < deadline:
            link.send_command(control.STOP)
            odom = link.try_receive_odometry()
            if odom is not None:
                return odom
            _time.sleep(0.02)
        return None

    print(f"시작 전 오도메트리 확인 중...")
    start_odom = _await_odometry()
    if start_odom is None:
        link.close()
        sys.exit("피코에서 오도메트리를 못 받았다 — 배선/포트/워치독을 확인할 것.")

    x0, y0 = start_odom.x, start_odom.y
    print(f"시작 pose = ({x0:+.4f}, {y0:+.4f})  "
          f"목표: {args.go_distance:.3f}m, {args.heading:.0f}° 방향, "
          f"{args.test_speed:.2f} m/s 순항")
    print("Ctrl+C로 언제든 중단 가능 (STOP 두 번 보내고 종료).\n")

    # (로컬시각, 오도메트리 pose로부터 계산한 누적 이동거리) 샘플. 정지거리·가속도
    # 추정에 쓴다. odom.vx/vy(칼만 스무딩, 시정수 ~1.4s)는 이런 <1s 구간 측정엔
    # 못 미더워서 안 쓴다 — pose 차분으로 직접 속도를 낸다.
    samples: list[tuple[float, float, float, float]] = []   # (t, dist, x, y)
    t_start = _time.monotonic()
    cmd = control.DriveCommand(target_vx=vx_dir * args.test_speed,
                                target_vy=vy_dir * args.test_speed,
                                timeout_s=config.DRIVE_TIMEOUT_S)

    next_tick = _time.monotonic()
    reached_at = None
    try:
        while True:
            now = _time.monotonic()
            if now - t_start > args.timeout_s:
                print(f"\n⚠ {args.timeout_s:.0f}초 안에 목표 거리에 못 미쳤다 — 강제 종료.")
                break

            link.send_command(cmd)
            odom = link.try_receive_odometry()
            if odom is not None:
                dist = math.hypot(odom.x - x0, odom.y - y0)
                samples.append((now - t_start, dist, odom.x, odom.y))
                print(f"  t={now - t_start:5.2f}s  오도메트리 이동거리={dist:6.3f}m"
                      f"  pose=({odom.x:+.3f},{odom.y:+.3f})" + " " * 6, end="\r")
                if dist >= args.go_distance:
                    reached_at = now - t_start
                    break

            next_tick += period
            sleep_for = next_tick - _time.monotonic()
            if sleep_for > 0:
                _time.sleep(sleep_for)
            else:
                next_tick = _time.monotonic()
    except KeyboardInterrupt:
        print("\n인터럽트 — 정지")

    # ── 정지 + 정지거리(관성 미끄러짐) 측정 ──────────────────────────────
    link.send_command(control.STOP)
    _time.sleep(0.05)
    link.send_command(control.STOP)
    stop_t = _time.monotonic() - t_start
    settle_deadline = _time.monotonic() + args.settle_s
    while _time.monotonic() < settle_deadline:
        odom = link.try_receive_odometry()
        if odom is not None:
            dist = math.hypot(odom.x - x0, odom.y - y0)
            samples.append((_time.monotonic() - t_start, dist, odom.x, odom.y))
        _time.sleep(0.02)
    link.close()

    print("\n" + "=" * 60)
    if not samples:
        print("오도메트리 샘플을 하나도 못 받았다 — 측정 실패.")
        return 1

    final_t, final_dist, final_x, final_y = samples[-1]
    at_stop = max((s for s in samples if s[0] <= stop_t), key=lambda s: s[0], default=samples[0])
    overshoot = final_dist - at_stop[1]

    print(f"오도메트리 기준 이동거리 (정지 명령 시점) = {at_stop[1]:.4f} m"
          f"  (목표 {args.go_distance:.3f}m)")
    if overshoot > 1e-4:
        print(f"정지 후 관성으로 더 이동 (정지거리)        = {overshoot:.4f} m"
              f"  (settle {args.settle_s:.1f}s 동안)")
    print(f"소요 시간 (시작 -> 정지 명령)                = {stop_t:.3f} s")
    if stop_t > 1e-6:
        print(f"평균 속도 (목표거리 / 소요시간)              = {args.go_distance / stop_t:.3f} m/s"
              f"  (명령 속도 {args.test_speed:.2f} m/s 대비 "
              f"{100*(args.go_distance/stop_t)/args.test_speed:.0f}%)")

    # 가속 구간 추정: 연속 샘플 간 pose 차분으로 순간속도를 내고, 순항속도의
    # 90%에 처음 도달한 시각을 '가속 완료 시각'으로 잡는다 (거친 추정 — 짧은
    # 구간 유한차분이라 노이즈가 있다. 참고용으로만 쓸 것).
    target_v90 = 0.9 * args.test_speed
    rise_t = None
    for i in range(1, len(samples)):
        t_a, d_a, _, _ = samples[i - 1]
        t_b, d_b, _, _ = samples[i]
        if t_b <= t_a or t_b > stop_t:
            continue
        v = (d_b - d_a) / (t_b - t_a)
        if v >= target_v90:
            rise_t = t_b
            break
    if rise_t is not None:
        print(f"순항속도 90%(={target_v90:.2f} m/s) 도달 시각         = {rise_t:.3f} s"
              f"  (대략적 가속도 추정 ≈ {target_v90 / rise_t:.2f} m/s²  — 유한차분 기반, 참고용)")
    else:
        print("순항속도 90%에 도달한 기록을 못 찾았다 — 거리/속도가 너무 작거나 "
              "샘플 간격이 성기다.")

    print("=" * 60)
    print("다음 단계: 줄자로 로봇이 실제로 이동한 거리를 재서 위 "
          f"'오도메트리 기준 이동거리'({at_stop[1]:.3f}m)와 비교할 것.")
    print("  실측이 이보다 짧다 -> 피코가 실제보다 더 갔다고 과대보고 -> 그 비율만큼")
    print("                        캐치 목표점에 못 미친다 (지금 겪는 증상과 같은 방향).")
    print("  실측이 이보다 길다 -> 피코가 과소보고 -> 캐치가 오버슈트하는 방향.")
    return 0


def _run_speed_test(args) -> int:
    """조이스틱 없이 --speed-test 속도로 --speed-test-duration 초만 직진시킨다.

    `--go-distance`와 달리 **거리 목표가 없다** — "도달했으니 정지"가 아니라 "정해진
    시간 동안 이 속도를 계속 요구하면 실제로 뭐가 나오는가"만 본다. 모터/배터리/
    펌웨어 클램프 중 어디가 병목인지 가르는 게 목적이라, 오히려 거리 판정이 섞이면
    안 된다(도달 여부가 속도 자체와는 다른 얘기이기 때문).

    ⚠ 피코 펌웨어의 `max_body_speed()` 클램프(WHEEL_MAX_SPEED_MPS 기준)를 이 함수가
    우회할 방법은 없다 — 모듈 docstring의 "고정시간 속도 테스트 모드" 항목 참고.
    """
    import math
    import time as _time

    import communication
    import config
    import control

    heading_rad = math.radians(args.heading)
    vx_dir, vy_dir = math.cos(heading_rad), math.sin(heading_rad)
    cap = control.max_body_speed(vx_dir, vy_dir)

    period = 1.0 / 50.0
    if period >= config.DRIVE_TIMEOUT_S:
        print(f"⚠ 명령 주기 {period*1000:.0f}ms가 워치독 {config.DRIVE_TIMEOUT_S*1000:.0f}ms"
              f"보다 느리다 — 명령 사이에 로봇이 자꾸 멈출 수 있다.")

    if args.speed_test > cap:
        print(f"⚠ 요청 속도 {args.speed_test:.2f}m/s가 이 방향({args.heading:.0f}°)의 현재 "
              f"소프트웨어 상한 {cap:.2f}m/s(config.WHEEL_MAX_SPEED_MPS 기준)보다 크다 — "
              f"피코가 자동으로 {cap:.2f}m/s까지 잘라서 내보낼 것이다. 그 이상을 실측하려면 "
              f"pico/config.h의 WHEEL_MAX_SPEED_MPS를 올리고 재플래시해야 한다.")

    link = communication.SerialLink(port=args.port)

    def _await_odometry(timeout_s: float = 2.0):
        deadline = _time.monotonic() + timeout_s
        while _time.monotonic() < deadline:
            link.send_command(control.STOP)
            odom = link.try_receive_odometry()
            if odom is not None:
                return odom
            _time.sleep(0.02)
        return None

    print("시작 전 오도메트리 확인 중...")
    start_odom = _await_odometry()
    if start_odom is None:
        link.close()
        sys.exit("피코에서 오도메트리를 못 받았다 — 배선/포트/워치독을 확인할 것.")

    x0, y0 = start_odom.x, start_odom.y
    print(f"시작 pose = ({x0:+.4f}, {y0:+.4f})  "
          f"명령: {args.speed_test:.2f}m/s, {args.heading:.0f}° 방향, "
          f"{args.speed_test_duration:.2f}초간  (이 방향 소프트웨어 상한 {cap:.2f}m/s)")
    print("Ctrl+C로 언제든 중단 가능 (STOP 두 번 보내고 종료).\n")

    # (t, x, y) 샘플 — go_distance와 같은 이유로 pose 차분으로 속도를 직접 낸다
    # (odom.vx/vy는 칼만 스무딩 시정수 ~1.4s라 <1s 구간에는 못 믿는다).
    samples: list[tuple[float, float, float]] = []
    t_start = _time.monotonic()
    cmd = control.DriveCommand(target_vx=vx_dir * args.speed_test,
                                target_vy=vy_dir * args.speed_test,
                                timeout_s=config.DRIVE_TIMEOUT_S)

    next_tick = _time.monotonic()
    try:
        while True:
            now = _time.monotonic()
            elapsed = now - t_start
            if elapsed >= args.speed_test_duration:
                break

            link.send_command(cmd)
            odom = link.try_receive_odometry()
            if odom is not None:
                samples.append((elapsed, odom.x, odom.y))
                print(f"  t={elapsed:5.2f}s  pose=({odom.x:+.3f},{odom.y:+.3f})  "
                      f"odom.vel(스무딩됨)=({odom.vx:+.2f},{odom.vy:+.2f})" + " " * 6,
                      end="\r")

            next_tick += period
            sleep_for = next_tick - _time.monotonic()
            if sleep_for > 0:
                _time.sleep(sleep_for)
            else:
                next_tick = _time.monotonic()
    except KeyboardInterrupt:
        print("\n인터럽트 — 정지")

    cmd_end_t = _time.monotonic() - t_start
    link.send_command(control.STOP)
    _time.sleep(0.05)
    link.send_command(control.STOP)

    settle_deadline = _time.monotonic() + args.settle_s
    while _time.monotonic() < settle_deadline:
        odom = link.try_receive_odometry()
        if odom is not None:
            samples.append((_time.monotonic() - t_start, odom.x, odom.y))
        _time.sleep(0.02)
    link.close()

    print("\n" + "=" * 60)
    if len(samples) < 2:
        print("오도메트리 샘플이 너무 적다 — 측정 실패.")
        return 1

    # 연속 샘플 간 pose 차분으로 순간속도를 낸다. 명령 구간(t <= cmd_end_t)에서
    # 최고값·마지막 구간(순항 안정 여부 확인용) 평균을 본다.
    speeds: list[tuple[float, float]] = []   # (t, |v|)
    for i in range(1, len(samples)):
        t_a, xa, ya = samples[i - 1]
        t_b, xb, yb = samples[i]
        dt = t_b - t_a
        if dt <= 1e-4:
            continue
        v = math.hypot(xb - xa, yb - ya) / dt
        speeds.append((t_b, v))

    during = [v for t, v in speeds if t <= cmd_end_t]
    if not during:
        print("명령 구간 안의 속도 샘플이 없다 — 측정 실패.")
        return 1

    peak = max(during)
    peak_t = next(t for t, v in speeds if v == peak)
    # '순항 안정' 추정: 명령 구간 마지막 30%의 평균 — 계속 가속 중이면 이 값이
    # peak보다 뚜렷이 낮게 나온다(=가속이 안 끝났다는 뜻), 비슷하면 이미 정상상태.
    tail_start = cmd_end_t * 0.7
    tail = [v for t, v in speeds if t >= tail_start and t <= cmd_end_t]
    steady = sum(tail) / len(tail) if tail else peak

    print(f"명령 속도                         = {args.speed_test:.3f} m/s"
          f"  ({args.heading:.0f}° 방향, 소프트웨어 상한 {cap:.3f} m/s)")
    print(f"실측 최고 순간속도 (pose 차분)     = {peak:.3f} m/s  (t={peak_t:.2f}s)")
    print(f"명령 구간 마지막 30% 평균속도      = {steady:.3f} m/s"
          f"  (peak 대비 {100*steady/peak:.0f}%" +
          (", 정상상태로 보임)" if steady >= peak * 0.9 else ", 아직 가속 중이었을 가능성)"))

    print("-" * 60)
    if peak >= cap * 0.9:
        print(f"→ 실측 최고속도가 이 방향 소프트웨어 상한({cap:.2f}m/s)의 90% 이상이다.")
        if args.speed_test > cap * 1.05:
            print(f"  요청({args.speed_test:.2f}m/s)이 상한보다 컸는데도 상한 근처에서 "
                  f"막혔다 — 지금은 **소프트웨어 클램프(WHEEL_MAX_SPEED_MPS)가 병목**이다. "
                  f"1.8m/s 자체가 모터로 가능한지는 아직 이 실측으로는 알 수 없다 — "
                  f"pico/config.h의 WHEEL_MAX_SPEED_MPS를 올리고 재플래시한 뒤 다시 재볼 것.")
        else:
            print(f"  요청한 속도만큼(또는 그 근처까지) 잘 나왔다 — 이 속도 대역에서는 "
                  f"모터/배터리가 병목이 아니다.")
    else:
        print(f"→ 소프트웨어 상한({cap:.2f}m/s)까지도 못 미치고 {peak:.2f}m/s에서 멈췄다. "
              f"클램프가 아니라 **모터 토크·배터리 전압강하·PID 게인·바퀴 슬립 쪽이 "
              f"먼저 병목**일 가능성이 크다 — 상한을 올려도 이 속도 자체는 안 오를 수 있다.")

    print("=" * 60)
    print("⚠ 참고: 위 '실측 최고 순간속도'는 20ms 오도메트리 샘플 간 pose 차분이라 "
          "노이즈가 있다 — 여러 번 반복해서 값이 일관되는지 볼 것. odom.vel(스무딩됨)은 "
          "칼만 시정수(~1.4s) 때문에 이렇게 짧은 구간에서는 실제보다 낮게 보일 수 있어 "
          "참고용으로만 같이 찍었다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

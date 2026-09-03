"""Xbox 컨트롤러로 피코 모터 응답을 수동으로 테스트한다.

카메라·YOLO 없이 피코 브링업만 따로 검증하기 위한 도구. 왼쪽 스틱을 body-frame
목표속도(vx, vy)로 매핑해 매 틱 피코로 쏜다 — main.py가 쓰는 것과 동일한 시리얼
프로토콜(communication.py)을 그대로 타므로, 이걸로 검증한 값은 실제 파이프라인
에서도 그대로 유효하다.

pico/TODO.md 2번(최대속도·가속시간·정지거리 실측)을 반복 재현 가능하게 하려고
만들었다. 회전(omega)은 프로토콜에 없으므로 다루지 않는다 — pico/communication.h 참고.

    python teleop_test.py --port /dev/ttyACM0                  # 기본 0.5 m/s 상한
    python teleop_test.py --port /dev/ttyACM0 --max-speed 1.8   # 최대속도 실측 시

의존성: pip install pygame pyserial (또는 apt python3-pygame)

컨트롤러는 USB-A로 라즈베리파이에 유선 연결한다. 축 부호가 몸체와 안 맞으면
(예: 스틱을 앞으로 밀었는데 로봇이 옆으로 감) 실행 중 찍히는 cmd=(vx,vy) 부호를
보고 --invert-x/--invert-y/--swap-xy로 맞출 것 — 정확한 배선 방향은 조립 후가
아니면 알 수 없다.

정지: Ctrl+C, 또는 컨트롤러 B 버튼(SDL 매핑에 따라 다를 수 있음 — 안 먹으면
Ctrl+C 쓸 것). 어느 경로로 종료해도 STOP을 두 번 보내고 닫는다. 스크립트가
죽어도 피코 워치독(config.DRIVE_TIMEOUT_S)이 알아서 정지시킨다.
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
    args = ap.parse_args()

    if args.port is None:
        sys.exit("--port를 지정할 것 (예: --port /dev/ttyACM0). config.SERIAL_PORT도 아직 비어 있다.")

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


if __name__ == "__main__":
    raise SystemExit(main())

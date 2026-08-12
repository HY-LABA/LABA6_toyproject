"""전체 실행 순서 오케스트레이션.

**구조가 바뀌었다 (2026-08-10).** 예전에는 "관측·예측을 1회 하고 → 재보정 루프를
반복"하는 2단 구조였고, 재보정을 어떻게 할지 설계가 안 돼서 `recalibrate()`가
비어 있었다. 궤적 추정이 중력 기반 최소제곱으로 바뀌면서 그 두 단계가 하나로
합쳐졌다 — **관측을 하나 더 넣고 다시 피팅하면 그게 곧 재보정**이다.

    프레임마다:
      ① 카메라 캡처 → YOLO → bbox 중심 (u,v,t)
      ② 관측 누적 → 궤적 재피팅 (시간 스팬이 충분해지면 해가 나온다)
      ③ 착지점·남은시간 예측
      ④ 오도메트리 수신 → 남은 거리 계산 → 목표 속도 → 피코 전송

예측은 프레임이 쌓일수록 정확해지고, 로봇은 첫 예측이 나온 순간부터 움직인다.
초기 예측이 거리는 좀 틀려도 **방향은 대체로 맞기** 때문에(깊이 불확실성은 X와 Y를
같은 배율로 함께 틀리므로 비율=방향은 보존된다) 일찍 출발하는 게 이득이다.
모터 가속에 시간이 걸리므로 관측 시간과 가속 시간을 겹치게 쓰는 셈이다.
"""

from __future__ import annotations

import config
import communication
import control
import trajectory
import utils


def run() -> None:
    cam = vision_open()
    link = communication.SerialLink()
    tracker = trajectory.Tracker()

    odom_xy = (0.0, 0.0)
    last_seen_t: float | None = None
    commanded = False

    utils.log("catch loop start")
    try:
        while True:
            det = vision.observe(cam)

            # ── 트랙 유지/리셋 판단 ──────────────────────────────────────
            if det is None:
                if last_seen_t is not None and tracker.times:
                    # 마지막 관측으로부터 얼마나 지났는지로 판단한다. 프레임 수로
                    # 세면 검출이 느려질 때 기준이 같이 흔들린다.
                    gap = utils.timestamp() - last_seen_t
                    if gap > config.TRACK_MAX_GAP_S:
                        utils.log(f"track lost (gap={gap:.3f}s) — reset")
                        tracker.reset()
                        last_seen_t = None
                        if commanded:
                            link.send_command(control.STOP)
                            commanded = False
                continue

            last_seen_t = det.t

            # ── 관측 누적 + 재피팅 ───────────────────────────────────────
            fit = tracker.add(det.u, det.v, det.t)
            if fit is None:
                # 아직 관측이 부족하거나 시간 스팬이 짧다. 조용히 다음 프레임.
                # 여기서 억지로 예측하면 깊이가 발산해 엉뚱한 방향으로 출발한다.
                continue

            landing = tracker.landing()
            if landing is None:
                # 궤적이 캐치 평면에 도달하지 못한다 (위로 던진 게 아니거나
                # 이미 지나갔다). 잔차는 통과했으니 검출 자체는 정상이다.
                utils.log(f"no landing solution: {fit}")
                continue

            landing_x, landing_y, time_remaining = landing

            # ── 오도메트리 반영 → 목표 속도 ──────────────────────────────
            odom = link.try_receive_odometry()
            if odom is not None:
                odom_xy = odom.xy

            cmd = control.to_drive_command(
                landing_xy=(landing_x, landing_y),
                time_remaining=time_remaining,
                odometry_xy=odom_xy,
            )
            link.send_command(cmd)
            commanded = True

            utils.log_cycle(fit, (landing_x, landing_y), time_remaining, odom_xy, cmd)

            # ── 종료 판단 ────────────────────────────────────────────────
            if time_remaining <= 0.0:
                utils.log("catch moment passed — track complete")
                tracker.reset()
                last_seen_t = None
                link.send_command(control.STOP)
                commanded = False
    except KeyboardInterrupt:
        utils.log("interrupted")
    finally:
        # 어떤 경로로 빠져나가도 로봇은 세운다.
        try:
            link.send_command(control.STOP)
        except Exception as exc:  # noqa: BLE001 - 정지 시도는 실패해도 계속 정리한다
            utils.log(f"stop command failed: {exc}")
        link.close()
        cam.close()
        utils.log("catch loop end")


def vision_open():
    """vision을 늦게 import한다 — Picamera2/HailoRT가 없는 PC에서도 이 파일을
    읽고 문법 검사할 수 있게 하기 위함."""
    global vision
    import vision as _vision

    vision = _vision
    return vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)


if __name__ == "__main__":
    run()

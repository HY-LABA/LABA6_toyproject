"""전체 실행 순서 오케스트레이션.

    프레임마다:
      ① 카메라 캡처 → YOLO → **검출 후보 전체**            [vision.observe]
      ② 후보마다 가설을 돌린다 (미리 고르지 않는다)         [pool.update]
      ③ 물리를 통과한 가설을 채택                          [pool.best]
      ④ 착지점·남은시간                                    [track.landing]
      ⑤ 남은거리 ÷ 남은시간 → 목표 속도 → 피코 전송        [control]
      ⑥ 사이클 종료 판단 → 리셋                            [pool.cycle_end_reason]

★ 왜 ②에서 하나를 고르지 않나
------------------------------
YOLO가 에어컨(conf 0.88)을 실제 쓰레기(0.62)보다 높게 준다. 여기서 하나를 고르는
휴리스틱을 쓰면 틀렸을 때 회복이 안 된다 — 오탐이 트랙에 들어가는 순간 재투영 잔차가
무한대로 튀어 사이클 전체가 죽는다.

대신 **후보마다 가설을 돌리고 물리가 판정하게 한다.** 정적 오탐은 화소가 고정돼
포물선 제약을 만족할 수 없으므로 착지해를 영영 못 낸다. 에어컨 가설은 저절로 탈락한다.

★ 오도메트리를 두 곳에 쓴다 — 역할이 다르다
-------------------------------------------
  피팅 입력  카메라가 움직이면 관측 (u,v)에 물체의 운동과 카메라의 운동이 섞인다.
             보정 없이 풀면 **등속 주행에서는 잔차가 작은 채로 답만 틀린다**
             (합성 검증: 1.2 m/s에서 잔차 1.48px인데 착지 57cm 오차).
             `Tracker`가 각 관측과 함께 카메라 위치를 쌓는다.
  남은 거리  착지점에서 "이미 간 만큼"을 빼야 다시 그만큼 가지 않는다. `control`이 쓴다.

**둘 다 가설이 시작된 시점을 원점으로 한 상대 이동량이다.** 피코가 주는 값은 부팅 이후
누적이라, 사이클마다 원점을 다시 잡지 않으면 두 번째 투척부터 좌표가 어긋난다.
"""

from __future__ import annotations

import config
import communication
import control
import tracker as tracker_mod
import utils


def run() -> None:
    cam = vision_open()
    link = communication.SerialLink()
    pool = tracker_mod.TrackerPool()

    odom_xy = (0.0, 0.0)
    commanded = False

    utils.log("catch loop start")
    try:
        while True:
            now = utils.timestamp()
            pool.tick(now)                          # COOLDOWN 만료 → IDLE

            # ── ① 캡처 + 검출 (후보 전체) ────────────────────────────────
            candidates = vision.observe(cam)

            # ── 오도메트리 갱신 (논블로킹) ───────────────────────────────
            odom = link.try_receive_odometry()
            if odom is not None:
                odom_xy = odom.xy

            # ── ② 후보마다 가설 갱신/스폰 ───────────────────────────────
            pool.update(candidates, odom_xy, now)

            # ── ③ 물리를 통과한 가설 채택 ───────────────────────────────
            track = pool.best()

            # ── ④ 예측 ──────────────────────────────────────────────────
            landing = track.landing() if track is not None else None

            if landing is not None:
                landing_x, landing_y, time_remaining = landing

                # ── ⑤ 명령 ──────────────────────────────────────────────
                cmd = control.to_drive_command(
                    landing_xy=(landing_x, landing_y),
                    time_remaining=time_remaining,
                    odometry_xy=pool.moved_since_start(odom_xy),
                )
                link.send_command(cmd)
                commanded = True
                utils.log_cycle(track.fit, (landing_x, landing_y), time_remaining,
                                odom_xy, cmd)
            else:
                time_remaining = None

            # ── ⑥ 사이클 종료 ───────────────────────────────────────────
            reason = pool.cycle_end_reason(now, time_remaining)
            if reason:
                utils.log(f"cycle end — {reason} (관측 {pool.n}, "
                          f"스팬 {pool.time_span:.2f}s, 가설 {pool.n_tracks}개)")
                pool.reset(now, reason)
                if commanded:
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

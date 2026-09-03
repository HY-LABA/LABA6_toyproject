"""전체 실행 순서 오케스트레이션.

    프레임마다:
      ① 카메라 캡처 → YOLO → **검출 후보 전체**            [vision.observe]
      ② 후보마다 가설을 돌린다 (미리 고르지 않는다)         [pool.update]
      ③ 물리를 통과한 가설을 채택                          [pool.best]
      ④ 착지점·남은시간                                    [track.landing]
      ⑤ 착지점을 월드 좌표로 옮겨 피코에 전송              [control]
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
  도달 판정  "남은 거리를 남은 시간 안에 갈 수 있나"로 표적을 고른다. `tracker._reach`.
             (실제 구동에서 남은 거리를 빼는 건 파이가 아니라 피코다 — 아래 참고)

**둘 다 가설이 시작된 시점을 원점으로 한 상대 이동량이다.** 피코가 주는 값은 부팅 이후
누적이라, 사이클마다 원점을 다시 잡지 않으면 두 번째 투척부터 좌표가 어긋난다.

★ 남은 거리는 파이가 빼지 않는다
--------------------------------
피코에는 **월드 좌표 도착 지점**만 보내고, "이미 간 만큼"을 빼는 건 피코가 자기
pose 로 한다. 파이가 미리 빼서 보내면 **두 번 빠진다** — 예전 프로토콜이 정확히 그
반대 방향으로 터졌었다(피코가 아무도 안 빼서 매번 처음부터 다시 갔다).
여기서 `moved_since_start()` 를 여전히 쓰는 곳은 **로그용 예측 속도**뿐이다.
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
            # ── ① 캡처 + 검출 (후보 전체) ────────────────────────────────
            #    시각은 캡처 시점 것을 쓴다 — 피팅 입력과 같은 시계여야 한다.
            candidates, now, _frame = vision.observe(cam)

            # ── 오도메트리 갱신 (논블로킹) ───────────────────────────────
            odom = link.try_receive_odometry()
            if odom is not None:
                odom_xy = odom.xy

            # ── ②③④ 가설 갱신 → 채택 → 예측 ──────────────────────────
            #    test_accuracy.py가 **이 함수를 그대로 쓴다.**
            track, landing, reason = pool.step(candidates, odom_xy, now)

            if landing is not None:
                landing_x, landing_y, time_remaining = landing

                # ── ⑤ 명령 — 도착 지점(월드 좌표)만 보낸다 ──────────────
                target = control.to_target_command(
                    landing_xy=(landing_x, landing_y),
                    origin_odom=track.origin_odom,
                    time_remaining=time_remaining,
                )
                link.send_target(target)
                commanded = True

                # 속도는 보내지 않지만(피코가 계산한다) **예측치를 로그에 남긴다** —
                # 어느 바퀴가 한계에 붙었는지 봐야 튜닝이 되고, 실기에서 피코가 실제로
                # 낸 속도(오도메트리 회신)와 이 예측을 비교하면 양쪽 식이 갈렸는지
                # 바로 보인다. 붙은 바퀴엔 ! 표시.
                predicted = control.to_drive_command(
                    landing_xy=(landing_x, landing_y),
                    time_remaining=time_remaining,
                    odometry_xy=pool.moved_since_start(odom_xy),
                )
                utils.log_cycle(track.fit, (landing_x, landing_y), time_remaining,
                                odom_xy, predicted, target)
                utils.log(control.describe(predicted))

            # ── ⑥ 사이클 종료 ───────────────────────────────────────────
            if reason:
                utils.log(f"cycle end — {reason} (관측 {pool.n}, "
                          f"스팬 {pool.time_span:.2f}s, 가설 {pool.n_tracks}개)")
                # ★ 움직이지 않았으면 COOLDOWN 없이 바로 다음 궤적을 기다린다.
                #   COOLDOWN은 "움직인 뒤 관성/튐"을 위한 것이라, 가만히 있었으면
                #   0.8초를 쉬는 건 그 사이 날아오는 걸 놓치는 것뿐이다.
                pool.reset(now, reason, cooldown=commanded)
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

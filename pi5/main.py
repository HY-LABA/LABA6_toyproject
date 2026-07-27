"""전체 실행 순서 오케스트레이션 (algorithm.md 1, 2번 참고).

관측·예측 파트(1회) -> 재보정 루프(z=0까지 반복).
아래 함수 시그니처는 초안이며, 실제 vision/trajectory/communication/control
구현과 맞춰 확정한다. ❓ 표시는 아직 결정 안 된 부분.
"""

from __future__ import annotations

import config
import communication
import control
import trajectory
import utils
import vision


def run() -> None:
    cam = vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)
    link = communication.SerialLink()  # ❓ 포트/보드레이트 아직 미정 (config에 없음)

    # ── 1. 관측 및 예측 파트 (물체 확정 시 1회) ──────────────
    obs = vision.wait_for_confirmed_object(cam)
    # obs: class_name, position=(x, y, z), vx, vy
    # vz는 별도 측정이 아니라 frame1/frame3의 z(=bbox 크기 기반) 값 자체로 역산한다:
    #   vz = (z3 - z1)/Δt + 0.5*g*Δt   (등가속도 보정. vx,vy와 달리 z는 등속이 아니므로)
    # -> vision.py 또는 trajectory.py 둘 중 어디서 계산할지는 구현하면서 정한다.
    landing_point, time_to_land, valid = trajectory.predict_landing(
        position=obs.position,       # (x, y, z)
        velocity=obs.velocity,       # (vx, vy, vz)
        g=config.GRAVITY,
        z_catch=config.CATCH_HEIGHT_M,
    )
    # valid=False는 z 추정이 잘못돼서(z0가 z_catch 이하로 나옴 등) 착지시간을 못 구했다는 뜻.
    # 정상 관측이면 나올 수 없는 상황이라 조용히 넘기지 않고 바로 잡는다.
    if not valid:
        raise RuntimeError(
            f"착지시간 계산 실패 (z 추정 오류로 추정): position={obs.position} velocity={obs.velocity}"
        )

    # target = (target_x, target_y, drive_time).
    # 카메라가 로봇 기준으로 위를 보므로 landing_point.x/y가 곧 로봇 기준 이동거리와 같다
    # (좌표 원점 변환 불필요). drive_time은 이번 구동에 얼마나 시간을 쓸지 control.py가 결정.
    target = control.to_drive_command(landing_point, time_to_land)
    utils.log(f"predicted landing={landing_point} time_to_land={time_to_land:.2f}s target={target}")
    link.send_target(target)

    # ── 2. 재보정 루프 (z = 0 될 때까지 반복) ────────────────
    z = obs.z
    while z > 0:
        link.send_target(target)              # 지정 시간(❓ RECAL_DRIVE_TIME_S) 동안 구동
        odometry = link.receive_odometry()     # 피코 -> 파이: 엔코더 기반 이동량 + 추정 위치
        reobs = vision.reobserve(cam, obs.class_name)  # 카메라 재관측

        landing_point, time_to_land, z = trajectory.recalibrate(
            odometry=odometry,
            reobs=reobs,
            previous_landing_point=landing_point,
        )
        target = control.to_drive_command(landing_point, time_to_land)
        utils.log(f"recalibrated landing={landing_point} time_to_land={time_to_land:.2f}s target={target} z={z:.3f}")

    utils.log("catch cycle complete")


if __name__ == "__main__":
    run()

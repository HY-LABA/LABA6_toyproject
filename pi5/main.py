"""전체 실행 순서 오케스트레이션 (algorithm.md 1, 2번)."""

from __future__ import annotations

import config
import communication
import control
import trajectory
import utils
import vision


def run() -> None:
    cam = vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)
    link = communication.SerialLink()

    obs = vision.wait_for_confirmed_object(cam)

    landing_point, time_to_land, valid = trajectory.predict_landing(
        position=obs.position,
        velocity=obs.velocity,
        g=config.GRAVITY,
        z_catch=config.CATCH_HEIGHT_M,
    )
    if not valid:
        raise RuntimeError(f"착지시간 계산 실패: position={obs.position} velocity={obs.velocity}")

    target = control.to_drive_command(landing_point, time_to_land)
    utils.log(f"predicted landing={landing_point} time_to_land={time_to_land:.2f}s target={target}")
    link.send_target(target)

    z = obs.position[2]
    while z > 0:
        link.send_target(target)
        odometry = link.receive_odometry()
        reobs = vision.reobserve(cam, obs.class_name)

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

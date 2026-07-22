from camera import Camera             # Camera: read() -> (frame, t), close()
from detector import Detector         # Detector: detect(frame) -> [(cls, conf, bbox)]
from coordinate import pixel_to_world # pixel_to_world(det, t) -> (position[x,y,z], t)
from kalman import ProjectileKalman   # ProjectileKalman: update(pos, t), .position, .velocity
from trajectory import predict_landing # predict_landing(pos, vel, g, z) -> (point, ttl, valid)

GRAVITY = 9.81
Z_CATCH = 0.45
PROCESS_STD = 0.5
MEASURE_STD = 0.03
MIN_TRACK_POINTS = 3


def run():
    camera = Camera()
    detector = Detector()
    kf = ProjectileKalman(GRAVITY, PROCESS_STD, MEASURE_STD)

    n_obs = 0
    while True:
        item = camera.read()
        if item is None:
            break
        frame, t = item

        detections = detector.detect(frame)
        if not detections:
            continue
        det = detections[0]

        position, ts = pixel_to_world(det, t)
        kf.update(position, ts)
        n_obs += 1
        if n_obs < MIN_TRACK_POINTS:
            continue

        point, time_to_land, valid = predict_landing(kf.position, kf.velocity, GRAVITY, Z_CATCH)
        if not valid:
            continue

        # TODO: Pico로 낙하점 전송
        print(f"[예측] 낙하 ({point[0]:+.3f}, {point[1]:+.3f}) m, {time_to_land:.3f}s 뒤")

    camera.close()


if __name__ == "__main__":
    run()

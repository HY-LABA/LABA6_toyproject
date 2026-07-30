#include "odometry_kalman.h"

#include <math.h>

// 축(vx,vy,omega)별 독립 1D 칼만필터. 상태=속도 자체(가속도 모델 없음),
// 엔코더 양자화 노이즈를 스무딩하는 용도.
typedef struct {
    float x;
    float p;
} Kalman1D;

#define PROCESS_NOISE 0.01f  // 정해야함: 실측하며 튜닝
#define MEASURE_NOISE 1.0f   // 정해야함: 실측하며 튜닝

static Kalman1D kf_vx, kf_vy, kf_omega;
static RobotPose pose;

static void kalman1d_init(Kalman1D *kf) {
    kf->x = 0.0f;
    kf->p = 1.0f;
}

static float kalman1d_update(Kalman1D *kf, float measurement, float dt) {
    kf->p += PROCESS_NOISE * dt;
    float k = kf->p / (kf->p + MEASURE_NOISE);
    kf->x += k * (measurement - kf->x);
    kf->p *= (1.0f - k);
    return kf->x;
}

void odometry_kalman_init(void) {
    kalman1d_init(&kf_vx);
    kalman1d_init(&kf_vy);
    kalman1d_init(&kf_omega);
    pose.x = 0.0f;
    pose.y = 0.0f;
    pose.theta = 0.0f;
}

RobotVelocity odometry_kalman_update(RobotVelocity raw_velocity, float dt) {
    RobotVelocity smooth;
    smooth.vx = kalman1d_update(&kf_vx, raw_velocity.vx, dt);
    smooth.vy = kalman1d_update(&kf_vy, raw_velocity.vy, dt);
    smooth.omega = kalman1d_update(&kf_omega, raw_velocity.omega, dt);

    // body frame(로봇 기준) 속도를 world frame으로 회전 변환 후 적분
    float cos_t = cosf(pose.theta);
    float sin_t = sinf(pose.theta);
    pose.x += (smooth.vx * cos_t - smooth.vy * sin_t) * dt;
    pose.y += (smooth.vx * sin_t + smooth.vy * cos_t) * dt;
    pose.theta += smooth.omega * dt;

    return smooth;
}

RobotPose odometry_get_pose(void) {
    return pose;
}

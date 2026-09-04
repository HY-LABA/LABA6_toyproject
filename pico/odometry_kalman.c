#include "odometry_kalman.h"

#include <math.h>

// 축(vx,vy,omega)별 독립 1D 칼만필터. 상태=속도 자체(가속도 모델 없음),
// 엔코더 양자화 노이즈를 스무딩하는 용도.
//
// ★★ 이 필터의 출력은 **파이5로 보고하는 속도에만** 쓴다. pose 적분에는 절대
//    쓰지 않는다 (2026-09-04 수정). 이유:
//
//    이 파라미터 조합에서 사전분산 p 는 정지 상태로 몇 초만 지나도 정상상태로
//    수렴하고, 그때 칼만 이득이 k≈0.014 로 굳는다 (x² − Q·dt·x − Q·dt·R = 0 →
//    x≈0.0142, K = x/(x+R) ≈ 0.0140). 시정수가 τ = dt/K ≈ 1.4초다.
//
//    캐치 기동은 0.5초짜리다. 시정수 1.4초짜리 필터를 통과한 속도를 적분하면
//    pose 가 실제 이동량을 이만큼 과소보고한다 (1.0 m/s 스텝, 정지 2초 후):
//
//        t=0.20s   실제 20cm  ->  pose  1.6cm  ( 8%)
//        t=0.50s   실제 50cm  ->  pose  8.9cm  (18%)   ← 41cm 과소보고
//        t=1.00s   실제 100cm ->  pose 30.6cm  (31%)
//
//    main.c 의 목표점 경로는 `남은거리 = target − pose` 이므로, 50cm 를 갔는데
//    9cm 갔다고 믿고 계속 최고속도로 달린다 — **구조적으로 보장된 오버슈트**다.
//    앞서 잡은 GEAR_RATIO 이중 나눗셈(43.8배 축소) 버그와 같은 계열이다.
//
//    실기 검증된 MicroPython 벤치(`pico_micropython/goto_xy_test.py`)는 이 필터를
//    아예 쓰지 않고 **순수 오일러 적분**으로 pose 를 만들었다 (그 파일의
//    "오도메트리 (단순 오일러 적분, C쪽 칼만필터 없이 우선 확인용)" 주석 참고).
//    즉 이 필터는 어떤 실측 테스트에서도 돌아간 적이 없는 컴포넌트다.
//    그래서 파라미터를 새로 고르는 대신 **검증된 동작으로 되돌린다** —
//    pose 는 raw 적분, 칼만은 텔레메트리 스무딩 전용.
//
//    ⚠ 스무딩을 pose 에 다시 넣고 싶어지면, 먼저 위 표를 이 주기(dt=0.02)와
//      새 파라미터로 다시 계산해서 시정수가 0.1초 이하인지 확인할 것.
typedef struct {
    float x;
    float p;
} Kalman1D;

#define PROCESS_NOISE 0.01f  // 정해야함: 실측하며 튜닝 (텔레메트리 스무딩에만 영향)
#define MEASURE_NOISE 1.0f   // 정해야함: 실측하며 튜닝 (텔레메트리 스무딩에만 영향)

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
    // 반환값(= 파이5로 보고할 속도)만 스무딩한다.
    RobotVelocity smooth;
    smooth.vx = kalman1d_update(&kf_vx, raw_velocity.vx, dt);
    smooth.vy = kalman1d_update(&kf_vy, raw_velocity.vy, dt);
    smooth.omega = kalman1d_update(&kf_omega, raw_velocity.omega, dt);

    // ★ pose 는 **raw 속도**로 적분한다 (위 주석 참고). 벤치와 같은 식이다.
    //   body frame(로봇 기준) 속도를 world frame으로 회전 변환 후 적분.
    float cos_t = cosf(pose.theta);
    float sin_t = sinf(pose.theta);
    pose.x += (raw_velocity.vx * cos_t - raw_velocity.vy * sin_t) * dt;
    pose.y += (raw_velocity.vx * sin_t + raw_velocity.vy * cos_t) * dt;
    pose.theta += raw_velocity.omega * dt;

    return smooth;
}

RobotPose odometry_get_pose(void) {
    return pose;
}

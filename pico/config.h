#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>

#define NUM_MOTORS 3

typedef struct {
    int rpwm;
    int lpwm;
    int en;  // R_EN, L_EN 공통
    int enc_a;
    int enc_b;
} MotorPins;

static const MotorPins MOTOR_PINS[NUM_MOTORS] = {
    {-1, -1, -1, -1, -1},  // 정해야함
    {-1, -1, -1, -1, -1},  // 정해야함
    {-1, -1, -1, -1, -1},  // 정해야함
};

// 파이5 통신: USB CDC (하드웨어 UART 핀 아님)

#define WHEEL_DIAMETER_M 0.100f
#define GEAR_RATIO 43.8f      // FIT0186
#define MOTOR_RATED_RPM 251.0f  // FIT0186 무부하 @12V

// body frame 최대 속도(m/s). 물리적으로 불가능한 목표가 들어오면 PID가 영구
// 포화되고, 그러면 세 바퀴의 속도 비율이 깨져서 진행 방향까지 틀어진다.
// FIT0186 기준: 251 RPM 무부하에 부하 감쇠 92.5%를 반영해 232 RPM,
//   (232/60) 회전/초 × π × 0.1m ≈ 1.22 m/s  (docs/physics.md 0장·3장)
// ⚠ 정해야함: 실측. 모터 도착 후 PWM 100%로 직진시켜 엔코더 속도가 평평해지는
//   값을 넣고, pi5/config.py의 ROBOT_MAX_SPEED_MPS도 같은 값으로 맞출 것.
#define MAX_BODY_SPEED_MPS 1.22f

// 바퀴 장착각(rad, 120도 간격 이상값) — 정해야함: 실제 조립 후 자/각도기로 재측정
static const float WHEEL_ANGLES_RAD[NUM_MOTORS] = {
    (float)(M_PI * 0.5),
    (float)(M_PI * 7.0 / 6.0),
    (float)(M_PI * 11.0 / 6.0),
};
#define WHEEL_MOUNT_RADIUS_M 0.15f  // 중심-바퀴 거리(m) — 정해야함: 실측

// FIT0186 모터축 16 CPR (1x 디코딩 기준). 데이터시트의 64 CPR은 4x 디코딩 값이다.
// PIO를 4x로 바꾸면 이 값도 64로 올릴 것 — docs/pico-control.md 6장.
#define ENCODER_COUNTS_PER_REV 16

#define CONTROL_PERIOD_MS 1

typedef struct {
    float kp;
    float ki;
    float kd;
} PidGains;

static const PidGains MOTOR_PID[NUM_MOTORS] = {
    {1.0f, 0.0f, 0.0f},  // 정해야함
    {1.0f, 0.0f, 0.0f},  // 정해야함
    {1.0f, 0.0f, 0.0f},  // 정해야함
};

#endif  // CONFIG_H

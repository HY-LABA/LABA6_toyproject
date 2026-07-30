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
#define GEAR_RATIO 34.0f
#define MOTOR_RATED_RPM 350.0f

// 바퀴 장착각(rad, 120도 간격 이상값) — 정해야함: 실제 조립 후 자/각도기로 재측정
static const float WHEEL_ANGLES_RAD[NUM_MOTORS] = {
    (float)(M_PI * 0.5),
    (float)(M_PI * 7.0 / 6.0),
    (float)(M_PI * 11.0 / 6.0),
};
#define WHEEL_MOUNT_RADIUS_M 0.15f  // 중심-바퀴 거리(m) — 정해야함: 실측

#define ENCODER_COUNTS_PER_REV -1  // 정해야함

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

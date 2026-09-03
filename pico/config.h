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

// 바퀴 장착각(rad). **pi5/config.py 의 WHEEL_ANGLES_RAD 와 반드시 같아야 한다**
// (control.verify_wheel_config() 가 이 파일을 직접 읽어 대조한다).
//
// body frame: M1 방향이 +Y(전방), 그 오른쪽 직각이 +X(= M1 과 M2 사이로 나가는 방향).
// 오른손 좌표계라 omega 는 반시계가 +. M1 이 앞, M1 에서 보아 오른쪽 아래가 M2,
// 왼쪽 아래가 M3 다:
//
//                    +Y (전방)
//                       M1  90°
//                        |
//          210° M3 ------+------ M2 330°   --> +X (우측)
//
// b_i 는 중심에서 바퀴를 본 위치각이고, 옴니 바퀴가 구르는 방향은 그 접선(b_i+90°)
// 이다. 그래서 kinematics.c 의 식이 w_i = -sin(b_i)*vx + cos(b_i)*vy + L*omega 다.
//
// 120도 등간격 이상값이다 — 조립 오차가 있으면 각도기로 재서 갱신할 것.
static const float WHEEL_ANGLES_RAD[NUM_MOTORS] = {
    (float)(M_PI * 0.5),        //  90° — M1 (전방)
    (float)(M_PI * 11.0 / 6.0), // 330° — M2 (우측 뒤)
    (float)(M_PI * 7.0 / 6.0),  // 210° — M3 (좌측 뒤)
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

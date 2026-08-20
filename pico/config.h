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

// 핀 배치 근거:
//   - RPWM/LPWM은 모터별로 같은 PWM 슬라이스의 A/B 채널에 둔다(짝수/홀수 쌍).
//     같은 슬라이스 = 같은 주파수, 채널은 듀티 독립 -> motor_control.c 구조에 맞음.
//   - RP2040은 slice=(gpio>>1)&7, channel=gpio&1 이라 GPIO n과 n+16이 슬라이스·채널까지
//     겹친다. 아래 배치는 그 충돌을 피한다.
//   - enc_a/enc_b는 인접할 필요 없음 (encoder_pio.c가 in_pins/jmp_pin을 따로 지정).
//   - GP0/GP1은 UART 디버그 콘솔용으로 비워둠.
// 인덱스 = 바퀴 번호 = WHEEL_ANGLES_RAD 인덱스. 드라이버·모터에 물리 라벨을 꼭 붙일 것.
static const MotorPins MOTOR_PINS[NUM_MOTORS] = {
    { 2,  3,  6, 10, 11},  // 0: 전방      (b=90도)
    { 4,  5,  7, 12, 13},  // 1: 후방 좌   (b=210도)
    { 8,  9, 14, 15, 16},  // 2: 후방 우   (b=330도)
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

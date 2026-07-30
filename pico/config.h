// 핀 배치, 로봇 물리 상수, PID 게인 초기값 (architecture.md 참고)
//
// 부품 구매 전이라 모터별 핀 번호 / PID 게인은 전부 placeholder(-1, 임시값)이다.
// 모터 3개를 항상 배열(인덱스 0,1,2)로 다뤄서, 나중에 실측값이 모터마다 달라져도
// 숫자만 채우면 되게 구조를 잡아둔다 (kinematics.c의 WHEEL_ANGLES[3]와 같은 패턴).
#ifndef CONFIG_H
#define CONFIG_H

#define NUM_MOTORS 3

// ── GPIO 핀 배치 (모터별) ─────────────────────────────────
// TODO: 부품 구매·배선 후 실제 핀 번호로 교체.
typedef struct {
    int rpwm;
    int lpwm;
    int r_en;
    int l_en;
    int enc_a;
    int enc_b;
} MotorPins;

static const MotorPins MOTOR_PINS[NUM_MOTORS] = {
    {-1, -1, -1, -1, -1, -1},  // TODO: 모터 0
    {-1, -1, -1, -1, -1, -1},  // TODO: 모터 1
    {-1, -1, -1, -1, -1, -1},  // TODO: 모터 2
};

// ── UART (파이5 <-> 피코) ────────────────────────────────
#define UART_TX_PIN -1       // TODO
#define UART_RX_PIN -1       // TODO
#define UART_BAUDRATE 115200 // ❓ pi5 쪽 config.py의 SERIAL_BAUDRATE와 반드시 일치해야 함

// ── 로봇 물리 상수 (README 기준, 이미 정해진 값) ─────────
#define WHEEL_DIAMETER_M 0.100f  // 100mm 옴니휠
#define GEAR_RATIO 34.0f         // 34:1
#define MOTOR_RATED_RPM 350.0f   // FIT0493 정격 RPM

// TODO: 엔코더 실물 스펙 확인 후 (홀센서 1회전당 펄스 수, 모터축 기준인지 출력축 기준인지도 확인)
#define ENCODER_COUNTS_PER_REV -1

// ── 제어 주기 ─────────────────────────────────────────────
#define CONTROL_PERIOD_MS 1  // architecture.md 예시값. 실측하면서 조정 가능

// ── PID 게인 (모터별, 실측 전 임시로 전부 동일값) ─────────
typedef struct {
    float kp;
    float ki;
    float kd;
} PidGains;

// TODO: 모터 개체차로 인해 실측하면 모터마다 값이 달라질 수 있음. 지금은 동일 placeholder.
static const PidGains MOTOR_PID[NUM_MOTORS] = {
    {1.0f, 0.0f, 0.0f},  // 모터 0
    {1.0f, 0.0f, 0.0f},  // 모터 1
    {1.0f, 0.0f, 0.0f},  // 모터 2
};

#endif  // CONFIG_H

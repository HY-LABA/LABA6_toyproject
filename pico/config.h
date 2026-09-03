#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>

#define NUM_MOTORS 3

typedef struct {
    int rpwm;
    int lpwm;
    int enc_a;
    int enc_b;
} MotorPins;

// 실측 확정값 (2026-09, MicroPython 벤치 테스트 리그 배선 = 최종 로봇 배선 확인됨).
// R_EN/L_EN은 GPIO가 아니라 3.3V에 직결(항상 활성)돼 있어서 en 핀 자체가 없다.
static const MotorPins MOTOR_PINS[NUM_MOTORS] = {
    {0, 1, 7, 8},
    {2, 3, 10, 11},
    {4, 5, 12, 13},
};

// 파이5 통신: USB CDC (하드웨어 UART 핀 아님)

#define WHEEL_DIAMETER_M 0.100f  // 실측 대신 대략값으로 확정 (2026-09)
#define GEAR_RATIO 43.8f      // FIT0186 실제 스펙 (이전 34.0f는 다른 모터 값이 잘못 남아있던 것)
                              // 참고용 상수 — 오도메트리 계산엔 안 쓴다. ENCODER_COUNTS_PER_REV가
                              // 이미 출력축(바퀴축) 기준이라 기어비를 또 나누면 이동거리가
                              // 43.8배 적게 계산된다 (main.c 참고).
#define MOTOR_RATED_RPM 251.0f  // FIT0186 무부하 출력축 RPM @12V (이전 350.0f 오기 수정)

// body frame 최대 속도(m/s). 물리적으로 불가능한 목표가 들어오면 PID가 영구
// 포화되고, 그러면 세 바퀴의 속도 비율이 깨져서 진행 방향까지 틀어진다.
// 이론값: 251 RPM 무부하에 부하 감쇠 92.5%를 반영해 232 RPM,
//   (232/60) 회전/초 × π × 0.1m ≈ 1.22 m/s
// ⚠ 정해야함: 실측. 부하·전압강하·바닥 마찰로 실제로는 더 낮다. 모터 도착 후
//   PWM 100%로 직진시켜 엔코더 속도가 평평해지는 값을 넣고, pi5/config.py의
//   ROBOT_MAX_SPEED_MPS도 같은 값으로 맞출 것.
#define MAX_BODY_SPEED_MPS 1.22f

// 바퀴 장착각: 90도=정면(M1), 330도=우측 뒤(M2), 210도=좌측 뒤(M3).
// MicroPython 벤치 테스트(2026-09)에서 도면으로 실물 대조 확인 완료.
// 반경(WHEEL_MOUNT_RADIUS_M)은 아직 실측 전 — 각도 순서만 확정된 값.
static const float WHEEL_ANGLES_RAD[NUM_MOTORS] = {
    (float)(M_PI * 0.5),
    (float)(M_PI * 11.0 / 6.0),
    (float)(M_PI * 7.0 / 6.0),
};
#define WHEEL_MOUNT_RADIUS_M 0.15f  // 중심-바퀴 거리(m) — 정해야함: 실측

// 모터별 부호 보정. wheel_speed_from_encoder_delta 결과와 motor_control의
// PWM 출력 양쪽에 곱해서 명령-측정 부호를 맞춘다.
// MicroPython 벤치 테스트(2026-09) 결과: x,y 이동 방향이 커맨드와 정반대로
// 나와서 모터 3개 전부 -1로 보정. PID가 안정적으로(폭주 없이) 수렴하면서
// 방향만 반대였던 걸로 봐서 개별 모터 배선 실수가 아니라 3개 다 일관된
// 방향 규칙(어느 쪽이었든)이 공식 가정과 정반대였던 것 — 원인 특정은 안 됐지만
// 이 보정으로 완전히 해결됨.
static const int MOTOR_SIGN[NUM_MOTORS] = {-1, -1, -1};

// 실측 확정값 (2026-09, MicroPython 테스트 리그 · encoder_test.py 핸드오프 문서).
// 데이터시트 표기(16 CPR 모터축)는 부정확한 것으로 확인됨 — 손회전/정밀정지
// 테스트로 실측한 결과 2체배(A채널 양쪽 엣지, XOR 방향판별) 디코딩 기준
// COUNTS_PER_REV = 1375, **출력축(바퀴축) 1회전 기준** (감속기 이후. 핸드오프
// 문서 공식이 "거리 = 카운트/1375 × 바퀴둘레"로 기어비를 안 나눔).
// 이 프로젝트의 PIO(quadrature_1x.pio)는 1체배(A 상승엣지만 카운트)라
// 카운트가 정확히 절반이므로 1375 / 2 = 687.5를 쓴다.
// ⚠ 여기서 나온 값은 이미 바퀴축 기준이므로 GEAR_RATIO로 다시 나누면 안 된다.
// 나중에 4체배(A/B 양쪽 엣지) 디코딩으로 바꾸면 1375 * 2 = 2750으로 재조정할 것.
#define ENCODER_COUNTS_PER_REV 687.5f

// MicroPython 벤치 테스트가 50Hz(20ms)에서 튜닝됐다 — PID의 Ki/Kd는 dt에
// 비례해서 누적/변화하므로, 같은 게인 값을 다른 주기에 그대로 쓰면 거동이
// 달라진다. 1kHz(1ms)는 검증된 적 없는 값이라 검증된 주기로 맞춘다.
#define CONTROL_PERIOD_MS 20

typedef struct {
    float kp;
    float ki;
    float kd;
} PidGains;

// MicroPython 벤치 테스트(2026-09)에서 확인된 값. 목표 근처 저속 구간에서
// Ki=0이면 정지마찰을 못 이기고 영원히 멈추는 문제가 있어서 Ki를 넣었다.
static const PidGains MOTOR_PID[NUM_MOTORS] = {
    {60.0f, 40.0f, 0.0f},
    {60.0f, 40.0f, 0.0f},
    {60.0f, 40.0f, 0.0f},
};

// 목표 속도가 0이 아닌데 계산된 duty가 이보다 작으면 이 값까지 끌어올린다
// (0~1 스케일, motor_control.c의 set_pwm과 동일 단위). 정지마찰 못 이기고
// 목표 근처에서 바퀴가 아예 안 굴러 영원히 멈추는 문제를 막는 데드밴드 보상.
#define MIN_DRIVE_DUTY 0.15f

#endif  // CONFIG_H

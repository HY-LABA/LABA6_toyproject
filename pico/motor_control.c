#include "motor_control.h"

#include <math.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define PWM_WRAP 4095

typedef struct {
    float integral;
    float prev_error;
} PidState;

static PidState pid_state[NUM_MOTORS];

static void setup_pwm_pin(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_enabled(slice, true);
}

void motor_control_init(void) {
    // R_EN/L_EN은 3.3V에 직결돼 있어 GPIO로 켤 필요가 없다 (config.h 참고).
    for (int i = 0; i < NUM_MOTORS; i++) {
        pid_state[i].integral = 0.0f;
        pid_state[i].prev_error = 0.0f;

        setup_pwm_pin(MOTOR_PINS[i].rpwm);
        setup_pwm_pin(MOTOR_PINS[i].lpwm);
    }
}

static void set_pwm(int pin, float duty_0_to_1) {
    if (duty_0_to_1 < 0.0f) duty_0_to_1 = 0.0f;
    if (duty_0_to_1 > 1.0f) duty_0_to_1 = 1.0f;
    pwm_set_gpio_level(pin, (uint16_t)(duty_0_to_1 * PWM_WRAP));
}

void motor_control_update(const float wheel_target[NUM_MOTORS], const float wheel_actual[NUM_MOTORS], float dt) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        float error = wheel_target[i] - wheel_actual[i];

        pid_state[i].integral += error * dt;
        // 안티와인드업: 적분항 하나만으로 duty 100%(1.0)를 넘길 수 없게 클램프.
        // 없으면 오래 멈춰있다 풀릴 때 적분이 쌓일 대로 쌓여서 확 튀어나간다.
        if (MOTOR_PID[i].ki != 0.0f) {
            float integral_limit = 1.0f / MOTOR_PID[i].ki;
            if (pid_state[i].integral > integral_limit) pid_state[i].integral = integral_limit;
            if (pid_state[i].integral < -integral_limit) pid_state[i].integral = -integral_limit;
        }
        float derivative = (error - pid_state[i].prev_error) / dt;
        pid_state[i].prev_error = error;

        PidGains gains = MOTOR_PID[i];
        float output = gains.kp * error + gains.ki * pid_state[i].integral + gains.kd * derivative;

        // 데드밴드 보상: 목표 속도가 0이 아닌데 duty가 정지마찰을 못 이길 만큼
        // 작아서 바퀴가 아예 안 구르는 상태(목표 근처에서 영원히 도착 못 함)를 막는다.
        if (fabsf(wheel_target[i]) > 1e-4f && fabsf(output) < MIN_DRIVE_DUTY) {
            output = (output >= 0.0f) ? MIN_DRIVE_DUTY : -MIN_DRIVE_DUTY;
        }

        // MOTOR_SIGN: 기구학 공식 기준 출력을 하드웨어가 요구하는 부호로 되돌린다.
        output *= MOTOR_SIGN[i];

        if (output >= 0.0f) {
            set_pwm(MOTOR_PINS[i].rpwm, output);
            set_pwm(MOTOR_PINS[i].lpwm, 0.0f);
        } else {
            set_pwm(MOTOR_PINS[i].rpwm, 0.0f);
            set_pwm(MOTOR_PINS[i].lpwm, -output);
        }
    }
}

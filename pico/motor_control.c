#include "motor_control.h"

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
    for (int i = 0; i < NUM_MOTORS; i++) {
        pid_state[i].integral = 0.0f;
        pid_state[i].prev_error = 0.0f;

        setup_pwm_pin(MOTOR_PINS[i].rpwm);
        setup_pwm_pin(MOTOR_PINS[i].lpwm);

        gpio_init(MOTOR_PINS[i].en);
        gpio_set_dir(MOTOR_PINS[i].en, GPIO_OUT);
        gpio_put(MOTOR_PINS[i].en, 1);
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
        float derivative = (error - pid_state[i].prev_error) / dt;
        pid_state[i].prev_error = error;

        PidGains gains = MOTOR_PID[i];
        float output = gains.kp * error + gains.ki * pid_state[i].integral + gains.kd * derivative;
        // 정해야함: output(속도 오차 기반) -> PWM 듀티(0~1) 스케일 계수, 실측 튜닝 필요

        if (output >= 0.0f) {
            set_pwm(MOTOR_PINS[i].rpwm, output);
            set_pwm(MOTOR_PINS[i].lpwm, 0.0f);
        } else {
            set_pwm(MOTOR_PINS[i].rpwm, 0.0f);
            set_pwm(MOTOR_PINS[i].lpwm, -output);
        }
    }
}

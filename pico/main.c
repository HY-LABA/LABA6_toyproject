#include "pico/stdlib.h"

#include "config.h"
#include "kinematics.h"
#include "odometry_kalman.h"
#include "motor_control.h"
#include "communication.h"
#include "encoder/encoder_pio.h"

static float wheel_speed_from_encoder_delta(int32_t delta_counts, float dt) {
    // 엔코더는 모터축(기어박스 이전) 기준이라 GEAR_RATIO로 나눠 바퀴 회전수로 변환
    float motor_revs = (float)delta_counts / ENCODER_COUNTS_PER_REV;
    float wheel_revs = motor_revs / GEAR_RATIO;
    float distance_m = wheel_revs * (WHEEL_DIAMETER_M * (float)M_PI);
    return distance_m / dt;
}

int main(void) {
    stdio_init_all();

    encoder_init_all();
    motor_control_init();
    communication_init();
    odometry_kalman_init();

    int32_t prev_counts[NUM_MOTORS] = {0};
    DriveCommand cmd = {0};
    float cmd_elapsed_s = 0.0f;

    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    absolute_time_t next_tick = get_absolute_time();

    while (true) {
        next_tick = delayed_by_ms(next_tick, CONTROL_PERIOD_MS);

        int32_t counts[NUM_MOTORS];
        float wheel_speed[NUM_MOTORS];
        for (int i = 0; i < NUM_MOTORS; i++) {
            counts[i] = encoder_get_count(i);
            wheel_speed[i] = wheel_speed_from_encoder_delta(counts[i] - prev_counts[i], dt);
            prev_counts[i] = counts[i];
        }

        float vx, vy, omega;
        forward_kinematics(wheel_speed, &vx, &vy, &omega);
        RobotVelocity raw_velocity = {vx, vy, omega};
        RobotVelocity smooth_velocity = odometry_kalman_update(raw_velocity, dt);

        DriveCommand new_cmd = communication_try_receive();
        if (new_cmd.valid) {
            cmd = new_cmd;
            cmd_elapsed_s = 0.0f;
        } else if (cmd.valid) {
            cmd_elapsed_s += dt;
            if (cmd_elapsed_s >= cmd.drive_time_s) {
                cmd.valid = false;
            }
        }

        float target_vx = cmd.valid ? (cmd.target_x / cmd.drive_time_s) : 0.0f;
        float target_vy = cmd.valid ? (cmd.target_y / cmd.drive_time_s) : 0.0f;
        float target_omega = 0.0f;

        float wheel_target[NUM_MOTORS];
        inverse_kinematics(target_vx, target_vy, target_omega, wheel_target);

        motor_control_update(wheel_target, wheel_speed, dt);
        communication_send_odometry(odometry_get_pose(), smooth_velocity);

        sleep_until(next_tick);
    }

    return 0;
}

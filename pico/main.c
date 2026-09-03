#include "pico/stdlib.h"

#include "config.h"
#include "kinematics.h"
#include "odometry_kalman.h"
#include "motor_control.h"
#include "communication.h"
#include "encoder/encoder_pio.h"

static float wheel_speed_from_encoder_delta(int32_t delta_counts, float dt) {
    // ENCODER_COUNTS_PER_REV는 실측으로 이미 출력축(바퀴축) 1회전 기준 값이라
    // (핸드오프 문서: "거리 = 카운트/1375 × 바퀴둘레", GEAR_RATIO 안 나눔),
    // 여기서 GEAR_RATIO로 또 나누면 실제 이동거리를 43.8배 적게 계산하게 된다
    // (2026-09, "1 1" 줬는데 개멀리 가는 버그로 발견).
    float wheel_revs = (float)delta_counts / ENCODER_COUNTS_PER_REV;
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
            // MOTOR_SIGN: 엔코더가 실제로 측정한 값을 기구학 공식 기준 부호로 맞춘다.
            wheel_speed[i] = wheel_speed_from_encoder_delta(counts[i] - prev_counts[i], dt) * MOTOR_SIGN[i];
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
            // timeout_s는 구동시간이 아니라 워치독이다. 정상 동작 중에는 파이5가
            // 매 프레임 새 명령을 보내므로 만료되지 않는다. 만료됐다는 건 파이가
            // 죽었거나 링크가 끊겼다는 뜻이므로 세우는 게 맞다.
            cmd_elapsed_s += dt;
            if (cmd_elapsed_s >= cmd.timeout_s) {
                cmd.valid = false;
            }
        }

        // 파이5가 이미 "남은거리 ÷ 남은시간"으로 계산해 보낸 속도다.
        // 여기서 나눗셈을 하지 않는다.
        float target_vx = cmd.valid ? cmd.target_vx : 0.0f;
        float target_vy = cmd.valid ? cmd.target_vy : 0.0f;
        float target_omega = 0.0f;

        // 안전 클램프. 파이5도 클램프하지만 여기서 한 번 더 막는다 — 통신 오류나
        // 파이 쪽 버그로 말도 안 되는 값이 들어오면 PID가 영구 포화되고, 그 상태에선
        // 세 바퀴의 속도 비율이 깨져서 크기만이 아니라 **진행 방향까지** 틀어진다.
        // 그래서 크기만 깎고 방향은 보존한다.
        float speed = sqrtf(target_vx * target_vx + target_vy * target_vy);
        if (speed > MAX_BODY_SPEED_MPS) {
            float scale = MAX_BODY_SPEED_MPS / speed;
            target_vx *= scale;
            target_vy *= scale;
        }

        float wheel_target[NUM_MOTORS];
        inverse_kinematics(target_vx, target_vy, target_omega, wheel_target);

        motor_control_update(wheel_target, wheel_speed, dt);
        communication_send_odometry(odometry_get_pose(), smooth_velocity);

        sleep_until(next_tick);
    }

    return 0;
}

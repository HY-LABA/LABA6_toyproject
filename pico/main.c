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

    // 현재 "구동 모드" — 목표점(TargetCommand)과 속도(teleop/정지) 두 경로 중
    // 가장 최근에 받은 쪽이 활성화된다 (docs/protocol.md 2장).
    typedef enum { DRIVE_NONE, DRIVE_TARGET, DRIVE_VELOCITY } DriveMode;
    DriveMode mode = DRIVE_NONE;
    float target_x = 0.0f, target_y = 0.0f, time_remaining_s = 0.0f;
    float vel_vx = 0.0f, vel_vy = 0.0f;
    float cmd_elapsed_s = 0.0f;
    float cmd_timeout_s = 0.0f;

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

        RxCommand new_cmd = communication_try_receive();
        if (new_cmd.kind == RX_CMD_TARGET) {
            mode = DRIVE_TARGET;
            target_x = new_cmd.target_x;
            target_y = new_cmd.target_y;
            time_remaining_s = new_cmd.time_remaining_s;
            cmd_timeout_s = new_cmd.timeout_s;
            cmd_elapsed_s = 0.0f;
        } else if (new_cmd.kind == RX_CMD_VELOCITY) {
            mode = DRIVE_VELOCITY;
            vel_vx = new_cmd.target_vx;
            vel_vy = new_cmd.target_vy;
            cmd_timeout_s = new_cmd.timeout_s;
            cmd_elapsed_s = 0.0f;
        } else if (mode != DRIVE_NONE) {
            // timeout_s는 구동시간이 아니라 워치독이다. 정상 동작 중에는 파이5가
            // 매 프레임 새 명령을 보내므로 만료되지 않는다. 만료됐다는 건 파이가
            // 죽었거나 링크가 끊겼다는 뜻이므로 세우는 게 맞다.
            cmd_elapsed_s += dt;
            if (cmd_elapsed_s >= cmd_timeout_s) {
                mode = DRIVE_NONE;
            }
        }

        float target_vx = 0.0f;
        float target_vy = 0.0f;
        const float target_omega = 0.0f;

        if (mode == DRIVE_TARGET) {
            // docs/protocol.md 2장: 파이는 절대좌표만 보내고, "이미 간 만큼"을
            // 빼는 건 피코가 자기 오도메트리로 한다 — 그래야 이중으로 안 빠진다.
            RobotPose pose = odometry_get_pose();
            float rx = target_x - pose.x;
            float ry = target_y - pose.y;
            float dist = sqrtf(rx * rx + ry * ry);

            if (dist > POSITION_TOLERANCE_M) {
                // 상한은 **이 방향에서** 바퀴가 포화되지 않는 값이다. 방향에 따라
                // 1.22~1.41 m/s 로 다르고, 파이의 tracker._reach() 가 믿는 값도
                // 이것이다 (docs/protocol.md 2장의 ⚠ 상자).
                float limit = max_body_speed(rx, ry, 0.0f);
                // 남은거리 ÷ 남은시간 — 목표에 가까워질수록 속도가 저절로 줄어서
                // 별도 감속 프로파일 없이 P 제어가 감속기 역할을 한다.
                // 남은시간이 0 이하면 이미 착지 시각을 지난 것이라 최대로 붙는다.
                // (0 이 아니라 1e-3 으로 거르는 건 아주 작은 양수로 나눠 v 가
                //  발산하는 걸 막기 위해서다 — 어차피 아래에서 잘리지만.)
                float v = (time_remaining_s > 1e-3f) ? (dist / time_remaining_s) : limit;
                if (v > limit) {
                    v = limit;
                }
                target_vx = (rx / dist) * v;
                target_vy = (ry / dist) * v;
            }
            // 다음 명령이 오면 덮어쓴다 — 새 명령이 안 오는 동안만 스스로 깎는다.
            time_remaining_s -= dt;
        } else if (mode == DRIVE_VELOCITY) {
            target_vx = vel_vx;
            target_vy = vel_vy;
        }

        // 안전 클램프. 파이5도 클램프하지만 여기서 한 번 더 막는다 — 통신 오류나
        // 파이 쪽 버그로 말도 안 되는 값이 들어오면 PID가 영구 포화되고, 그 상태에선
        // 세 바퀴의 속도 비율이 깨져서 크기만이 아니라 **진행 방향까지** 틀어진다.
        // 그래서 크기만 깎고 방향은 보존한다.
        //
        // ★ 상한이 방향별이다. 예전엔 상수(1.22)로 잘랐는데, 모터가 제한하는 건
        //   body 속도가 아니라 바퀴 속도라 유리한 방향(0°/90°…)에서 실제로 낼 수
        //   있는 1.41 m/s 를 버리고 있었다. 텔레옵(속도 명령) 경로도 여기를 탄다.
        float speed = sqrtf(target_vx * target_vx + target_vy * target_vy);
        if (speed > 1e-9f) {
            float limit = max_body_speed(target_vx, target_vy, target_omega);
            if (speed > limit) {
                float scale = limit / speed;
                target_vx *= scale;
                target_vy *= scale;
            }
        }

        float wheel_target[NUM_MOTORS];
        inverse_kinematics(target_vx, target_vy, target_omega, wheel_target);

        motor_control_update(wheel_target, wheel_speed, dt);
        communication_send_odometry(odometry_get_pose(), smooth_velocity);

        sleep_until(next_tick);
    }

    return 0;
}

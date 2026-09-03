#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>

#include "odometry_kalman.h"

// 파이5가 보내는 명령. 두 종류이고 LEN(바이트 수)으로 구분한다 — 섞일 수 없다.
// 계약 전문: docs/protocol.md 2장.
typedef enum {
    RX_CMD_NONE,      // 이번 사이클에 받은 게 없음
    RX_CMD_TARGET,    // 16B — 평상시 구동 (목표점, world frame 절대 좌표)
    RX_CMD_VELOCITY,  // 12B — 텔레옵(teleop_test.py)·정지(STOP) 전용
} RxCommandKind;

typedef struct {
    RxCommandKind kind;

    // RX_CMD_TARGET: 파이가 남은거리를 빼서 보내지 않는다 — 여기 이 값 그대로
    // "목표 - 자기 pose"를 피코가 계산한다 (main.c).
    float target_x;
    float target_y;
    float time_remaining_s;

    // RX_CMD_VELOCITY: body frame 목표 속도, 그대로 유지한다.
    float target_vx;
    float target_vy;

    float timeout_s;  // 워치독: 이 시간 안에 새 명령이 없으면 정지. 두 종류 다 있음.
} RxCommand;

void communication_init(void);
RxCommand communication_try_receive(void);
void communication_send_odometry(RobotPose pose, RobotVelocity velocity);

#endif  // COMMUNICATION_H

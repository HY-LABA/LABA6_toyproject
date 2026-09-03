#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>

#include "odometry_kalman.h"

// 파이5가 보내는 명령. **좌표가 아니라 속도다.** 파이5가 남은거리÷남은시간으로
// 이미 계산해서 보내므로 피코는 그 속도를 유지하기만 한다.
typedef struct {
    float target_vx;     // body frame 목표 속도 (m/s)
    float target_vy;     // body frame 목표 속도 (m/s)
    float timeout_s;     // 워치독: 이 시간 안에 새 명령이 없으면 정지
    bool valid;
} DriveCommand;

void communication_init(void);
DriveCommand communication_try_receive(void);
void communication_send_odometry(RobotPose pose, RobotVelocity velocity);

#endif  // COMMUNICATION_H

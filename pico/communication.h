#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>

#include "odometry_kalman.h"

typedef struct {
    float target_x;
    float target_y;
    float drive_time_s;
    bool valid;
} DriveCommand;

void communication_init(void);
DriveCommand communication_try_receive(void);
void communication_send_odometry(RobotPose pose, RobotVelocity velocity);

#endif  // COMMUNICATION_H

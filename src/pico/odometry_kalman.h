#ifndef ODOMETRY_KALMAN_H
#define ODOMETRY_KALMAN_H

typedef struct {
    float vx;
    float vy;
    float omega;
} RobotVelocity;

typedef struct {
    float x;
    float y;
    float theta;
} RobotPose;

void odometry_kalman_init(void);
RobotVelocity odometry_kalman_update(RobotVelocity raw_velocity, float dt);
RobotPose odometry_get_pose(void);

#endif  // ODOMETRY_KALMAN_H

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"

void motor_control_init(void);
void motor_control_update(const float wheel_target[NUM_MOTORS], const float wheel_actual[NUM_MOTORS], float dt);

#endif  // MOTOR_CONTROL_H

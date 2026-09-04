#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"

void motor_control_init(void);

// PID 적분·미분 상태를 지운다. **구동 모드가 바뀔 때마다 불러야 한다.**
// 실기 검증된 벤치(`pico_micropython/goto_xy_test.py`)는 새 명령마다
// `pid_reset()` 을 불렀는데 C 포팅에서 그 호출이 빠져 있었다 (2026-09-04 수정).
void motor_control_reset(void);
void motor_control_update(const float wheel_target[NUM_MOTORS], const float wheel_actual[NUM_MOTORS], float dt);

#endif  // MOTOR_CONTROL_H

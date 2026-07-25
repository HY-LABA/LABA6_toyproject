// 3륜 옴니 역/순기구학. (kinematics.py 재활용 -> C 이식)
// 공식: v_i = -sin(b_i)*vx + cos(b_i)*vy + L*omega
#include "kinematics.h"
#include <math.h>

// TODO: config.h 로 옮길 값들 (실측 후 조정)
static const float WHEEL_ANGLES[3] = {              // 바퀴 장착각(rad), 120도 간격
    (float)(M_PI * 0.5),                            // 90
    (float)(M_PI * 7.0 / 6.0),                      // 210
    (float)(M_PI * 11.0 / 6.0),                     // 330
};
static const float L = 0.15f;                       // 중심-바퀴 거리(m)

void inverse_kinematics(float vx, float vy, float omega, float wheel_out[3]) {
    for (int i = 0; i < 3; i++) {
        float b = WHEEL_ANGLES[i];
        wheel_out[i] = -sinf(b) * vx + cosf(b) * vy + L * omega;
    }
}

void forward_kinematics(const float wheel[3], float *vx, float *vy, float *omega) {
    // TODO: inverse_kinematics 행렬의 3x3 역행렬로 (vx,vy,omega) 복원
    (void)wheel; (void)vx; (void)vy; (void)omega;
}

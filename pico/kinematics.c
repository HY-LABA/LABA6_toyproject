// 3륜 옴니 역/순기구학. (kinematics.py 재활용 -> C 이식)
// 공식: v_i = -sin(b_i)*vx + cos(b_i)*vy + L*omega
#include "kinematics.h"
#include <math.h>

#include "config.h"

void inverse_kinematics(float vx, float vy, float omega, float wheel_out[3]) {
    for (int i = 0; i < 3; i++) {
        float b = WHEEL_ANGLES_RAD[i];
        wheel_out[i] = -sinf(b) * vx + cosf(b) * vy + WHEEL_MOUNT_RADIUS_M * omega;
    }
}

void forward_kinematics(const float wheel[3], float *vx, float *vy, float *omega) {
    // inverse_kinematics와 같은 행 [-sin(b_i), cos(b_i), L]로 이루어진 3x3 행렬 M을 세우고,
    // wheel = M * [vx,vy,omega] 를 여인수/수반행렬 공식으로 역산.
    float m[3][3];
    for (int i = 0; i < 3; i++) {
        float b = WHEEL_ANGLES_RAD[i];
        m[i][0] = -sinf(b);
        m[i][1] = cosf(b);
        m[i][2] = WHEEL_MOUNT_RADIUS_M;
    }

    float det =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    float inv[3][3];
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;

    *vx = inv[0][0] * wheel[0] + inv[0][1] * wheel[1] + inv[0][2] * wheel[2];
    *vy = inv[1][0] * wheel[0] + inv[1][1] * wheel[1] + inv[1][2] * wheel[2];
    *omega = inv[2][0] * wheel[0] + inv[2][1] * wheel[1] + inv[2][2] * wheel[2];
}

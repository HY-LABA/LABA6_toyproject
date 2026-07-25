// 3륜 옴니 역/순기구학. (kinematics.py 재활용 -> C 이식)
#ifndef KINEMATICS_H
#define KINEMATICS_H

// 역기구학: 바디 속도(vx, vy, omega) -> 바퀴 선속도 wheel_out[3]
void inverse_kinematics(float vx, float vy, float omega, float wheel_out[3]);

// 순기구학: 바퀴 선속도 wheel[3] -> 바디 속도(*vx, *vy, *omega)
// TODO: 3x3 역행렬 필요 — 추후 구현
void forward_kinematics(const float wheel[3], float *vx, float *vy, float *omega);

#endif

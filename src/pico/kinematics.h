// 3륜 옴니 역/순기구학. (kinematics.py 재활용 -> C 이식)
#ifndef KINEMATICS_H
#define KINEMATICS_H

// 역기구학: 바디 속도(vx, vy, omega) -> 바퀴 선속도 wheel_out[3]
void inverse_kinematics(float vx, float vy, float omega, float wheel_out[3]);

// 순기구학: 바퀴 선속도 wheel[3] -> 바디 속도(*vx, *vy, *omega)
void forward_kinematics(const float wheel[3], float *vx, float *vy, float *omega);

// **이 방향으로** 갈 때 바퀴가 포화되지 않는 최대 body 속력(m/s).
// (vx, vy)는 크기가 무시되고 방향만 쓰인다. 방향이 유리하면 WHEEL_MAX_SPEED_MPS
// 보다 크게 나온다 — 3륜 옴니에서 body 1 m/s 에 필요한 최대 바퀴 속도가 방향에 따라
// 0.866~1.000 으로 다르기 때문이다.
//
// ⚠ pi5 `control.max_body_speed()` 와 **같은 식이어야 한다.** 파이는 이 값으로
//   표적 도달 가능성을 판정한다(`tracker._reach()`). 여기가 더 짜게 자르면 파이는
//   못 잡을 표적을 쫓고, 더 후하게 자르면 바퀴가 포화돼 진행 방향이 틀어진다.
float max_body_speed(float vx, float vy, float omega);

#endif

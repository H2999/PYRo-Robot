#ifndef PYRO_ROBOT_GIMBAL_CONFIG_H
#define PYRO_ROBOT_GIMBAL_CONFIG_H

constexpr float yaw_motor_max_value = 1.5f;
constexpr float yaw_motor_min_value = -1.1f;

constexpr float pitch_max_value = 0.77f;
constexpr float pitch_min_value = -0.28f;

constexpr float control_dt = 0.001f;
constexpr float yaw_k_ff = 0.8f;        //用于遥控器控制
constexpr float pitch_k_ff = 0.65f;     //用于遥控器控制模式下

constexpr float yaw_torque_k_ff = 0.005f; //用于自瞄模式下乘上TD输出算出的加速度 作为加速度前馈

constexpr float YAW_OFFSET_RAD = 2.60700035f;

#endif //PYRO_ROBOT_GIMBAL_CONFIG_H
//
// Created by 1 on 2026/5/4.
//

#ifndef PYRO_ROBOT_GIMBAL_CONFIG_H
#define PYRO_ROBOT_GIMBAL_CONFIG_H

constexpr float yaw_motor_max_value = 1.5f;
constexpr float yaw_motor_min_value = -1.1f;

constexpr float pitch_max_value = 0.635f;
constexpr float pitch_min_value = -0.23f;

constexpr float control_dt = 0.001f;
constexpr float yaw_k_ff = 0.48f;
constexpr float pitch_k_ff = 0.7f;

constexpr float YAW_OFFSET_RAD = 2.60700035f;

#endif //PYRO_ROBOT_GIMBAL_CONFIG_H
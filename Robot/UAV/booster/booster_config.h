//
// Created by 1 on 2026/5/4.
//

#ifndef PYRO_ROBOT_BOOSTER_CONFIG_H
#define PYRO_ROBOT_BOOSTER_CONFIG_H
#include <stdint.h>

constexpr float FRIC1_RADIUS = 0.03f;
constexpr float FRIC2_RADIUS = 0.03f;

constexpr float shoot_torque_threshold = 20.0f;
constexpr float shoot_time_threshold = 12.0f;
constexpr uint8_t _17mm_ball_heat = 10;

constexpr float reduction_ratio = 36.0f;
constexpr float reciprocal_reduction_ratio =  0.0277777777777777f;

constexpr float filter_num[3] = {1.562916920892f, -0.6413063774028f, 0.07838945651057f};

#endif //PYRO_ROBOT_BOOSTER_CONFIG_H
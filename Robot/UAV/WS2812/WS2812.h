#ifndef PYRO_ROBOT_WS2812_H
#define PYRO_ROBOT_WS2812_H

#include <cstdint>
#include "stm32h7xx.h"
#include "tim.h"


static constexpr uint8_t PWM_LOW = 25;  // 逻辑“0”的占空比
static constexpr uint8_t PWM_HIGH = 50; // 逻辑“1”的占空比

void WS2812_init();
// 将RGB颜色数据转换为PWM占空比数据
inline void rgb_to_pwm(std::uint8_t r, uint8_t g, uint8_t b, uint16_t *buffer, int start_index);

#endif //PYRO_ROBOT_WS2812_H
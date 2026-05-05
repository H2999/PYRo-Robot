// #include "WS2812.h"
//
// void WS2812_init()
// {
//     // 这里可以添加任何必要的初始化代码，例如配置定时器或DMA
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 启动定时器PWM输出
// }
//
// void rgb_to_pwm(uint8_t r, uint8_t g, uint8_t b, uint16_t *buffer, int start_index)
// {
//     // WS2812数据顺序为GRB
//     uint8_t data[24];
//     for (int i = 0; i < 8; i++) {
//         data[i] = (g >> (7 - i)) & 0x01;
//     }
//     for (int i = 0; i < 8; i++) {
//         data[i + 8] = (r >> (7 - i)) & 0x01;
//     }
//     for (int i = 0; i < 8; i++) {
//         data[i + 16] = (b >> (7 - i)) & 0x01;
//     }
//
//     for (int i = 0; i < 24; i++) {
//         if (data[i] == 0) {
//             buffer[start_index + i] = PWM_LOW;
//         } else {
//             buffer[start_index + i] = PWM_HIGH;
//         }
//     }
// }
//
// volatile uint8_t led_busy = 0; // 发送状态标志位
//
// void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim->Instance == TIM3)
//     {
//         // 传输完成，停止 DMA 并清除标志位
//         HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
//         led_busy = 0;
//     }
// }
//
// void update_leds()
// {
//     if (led_busy) return; // 如果正在发送，跳过本次更新
//
//     // 1. 填充数据...
//     fill_pwm_buffer();
//
//     led_busy = 1;
//     HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)pwm_buffer, BIT_COUNT);
// }
//

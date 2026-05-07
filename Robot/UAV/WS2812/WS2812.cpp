#include "WS2812.h"



namespace pyro
{
    WS2812_drv_t* WS2812_drv_t::get_instance()
    {
        static WS2812_drv_t instance;
        return &instance;
    }

    WS2812_drv_t::WS2812_drv_t():led_busy(0)
    {
        std::memset(pwm_buffer, 0, sizeof(pwm_buffer));
        memset(led_colors, 0, sizeof(led_colors));
    }

    void WS2812_drv_t::WS2812_Init()
    {
        // 启动PWM输出
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        //高级定时器TIM1还要再使能一次
        TIM1->BDTR |= TIM_BDTR_MOE;  // 或者用 HAL 函数
    }

    /**
     * @brief 将RGB值转换为PWM占空比序列
     * @param r 红色分量 (0-255)
     * @param g 绿色分量 (0-255)
     * @param b 蓝色分量 (0-255)
     * @param buffer 输出缓冲区
     * @param start_index 起始索引
     */
    void WS2812_drv_t::rgb_to_PWM(uint8_t r, uint8_t g, uint8_t b, uint16_t *buffer, uint16_t start_index)
    {
        // WS2812数据顺序: GRB (绿色, 红色, 蓝色)
        uint32_t data = 0;
        data = (static_cast<uint32_t>(g) << 16) | (static_cast<uint32_t>(r) << 8) | b;

        // 将24位数据转换为PWM占空比序列
        for (int i = 0; i < 24; i++)
        {
            if (data & (0x800000 >> i))
            {
                buffer[start_index + i] = PWM_OUTPUT_1;
            }
            else
            {
                buffer[start_index + i] = PWM_OUTPUT_0;
            }
        }
    }

    //设置所有灯的颜色
    void WS2812_drv_t::set_all(uint8_t r, uint8_t g, uint8_t b)
    {
        for (uint16_t i = 0; i < LED_COUNT; i++)
        {
            led_colors[i * 3 + 0] = g;//Green
            led_colors[i * 3 + 1] = r;//Red
            led_colors[i * 3 + 2] = b;//Blue
        }
    }

    //把数据传给缓冲区 然后通过DMA发送
    void WS2812_drv_t::fill_PWM_buffer()
    {
        uint16_t buffer_index = 0;

        for (uint16_t i = 0; i < LED_COUNT; i++)
        {
            // 1. 组合成 GRB 24位数据
            uint32_t data = (static_cast<uint32_t>(led_colors[i * 3 + 0]) << 16) |
                            (static_cast<uint32_t>(led_colors[i * 3 + 1]) << 8)  |
                             led_colors[i * 3 + 2];

            // 2. 展开为 PWM 信号
            for (uint8_t bit = 0; bit < 24; bit++)
            {
                pwm_buffer[buffer_index++] = (data & (0x800000 >> bit)) ? PWM_OUTPUT_1 : PWM_OUTPUT_0;
            }
        }

        // 3. 填充 RESET 信号
        for (int i = 0; i < RESET_PULSES; i++)
        {
            pwm_buffer[buffer_index + i] = 0;
        }
    }

    //更新LED显示刷新 非阻塞式
    uint8_t WS2812_drv_t::update_light()
    {
        if (led_busy)
        {
            return 1;  // 繁忙，稍后再试
        }

        // 填充PWM缓冲区
        fill_PWM_buffer();

        // 启动DMA传输
        led_busy = 1;
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
                             reinterpret_cast<uint32_t*>(pwm_buffer),
                             LED_COUNT * BITS_PER_LED + RESET_PULSES);

        return 0;  // 已开始发送
    }

    /**
     * @brief 检查是否正在发送数据
     * @return 1: 繁忙, 0: 空闲
     */
    uint8_t WS2812_drv_t::WS2812_isbusy() const
    {
        return led_busy;
    }

    void WS2812_drv_t::clear_busy()
    {
        led_busy = 0;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        // 停止DMA传输
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
        pyro::WS2812_drv_t::get_instance()->clear_busy();
    }
}

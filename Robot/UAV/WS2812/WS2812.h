#ifndef WS2812_H_
#define WS2812_H_
#include <cstring>
#include <cstdint>
#include "tim.h"

namespace pyro
{
    class WS2812_drv_t
    {
    public:
        static WS2812_drv_t* get_instance();

        void WS2812_Init();
        void clear_busy();

        void set_all(uint8_t r, uint8_t g, uint8_t b);

        uint8_t update_light();

        [[nodiscard]] uint8_t WS2812_isbusy() const;

        WS2812_drv_t(const WS2812_drv_t &) = delete;
        WS2812_drv_t& operator=(const WS2812_drv_t &) = delete;

    private:
        WS2812_drv_t();
        ~WS2812_drv_t()= default;

        static void fill_PWM_buffer();
        static void rgb_to_PWM(uint8_t r, uint8_t g, uint8_t b, uint16_t *buffer, uint16_t start_index);

        static constexpr uint8_t LED_COUNT = 60;
        static constexpr uint8_t BITS_PER_LED = 24;
        //ws2812在发完数据后要等待一会 也就是发一段时间0来表示发送完毕
        static constexpr uint16_t RESET_PULSES = 60;

        static constexpr uint16_t BUFFER_SIZE = LED_COUNT * BITS_PER_LED + RESET_PULSES;
        static constexpr uint16_t COLOR_BUFFER_SIZE = LED_COUNT * 3;

        // 定义PWM缓冲区和LED颜色缓冲区 别忘了把要通过DMA发送的数据放到RAM.D2中
        __attribute__((section(".dma_heap"))) static inline uint16_t pwm_buffer[BUFFER_SIZE]{};
        static inline uint8_t led_colors[COLOR_BUFFER_SIZE]{};

        volatile uint8_t led_busy = 0;

        // 预计算的PWM占空比数值
        //0.35us表示低电平 所以0.35e-6 * 480M就是灯带灭时候的PWM_pulse
        static inline uint8_t PWM_OUTPUT_0 = 168;
        //700ns 也就是0.7us表示高电平 所以0.7e-6 * 480M就是想让灯带亮时候的PWM_pulse
        static inline uint16_t PWM_OUTPUT_1 = 336;
    };
}

#endif

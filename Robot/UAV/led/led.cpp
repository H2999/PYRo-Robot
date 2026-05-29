#include "led.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void led_init()
{
    // 重新配置 GPIO（确保不被其他功能占用）
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 确保 GPIOE 时钟已使能
    __HAL_RCC_GPIOE_CLK_ENABLE();

    // 配置 PE9 和 PE13 为复用推挽
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // TIM1 高级定时器配置
    TIM1->BDTR |= TIM_BDTR_MOE | TIM_BDTR_OSSR | TIM_BDTR_OSSI;
    TIM1->BDTR &= ~TIM_BDTR_BKE;

    // 清除刹车标志
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    TIM1->SR = 0;  // 清除所有中断标志

    // 启动 PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // 设置占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, LED_OFF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, LED_OFF);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_OFF);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_OFF);

    // 调试：强制输出高电平测试
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}

void front_led_on()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_ON);
}

void front_led_off()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_OFF);
}

void back_led_on()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_ON);
}

void back_led_off()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_OFF);
}
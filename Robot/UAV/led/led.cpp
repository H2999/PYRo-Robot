#include "led.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// void led_init()
// {
//     TIM1->BDTR |= TIM_BDTR_MOE;      // 主输出使能
//     TIM1->BDTR |= TIM_BDTR_OSSR;     // 运行模式 Off-state 选择
//     TIM1->BDTR |= TIM_BDTR_OSSI;     // 空闲模式 Off-state 选择
//     TIM1->BDTR &= ~TIM_BDTR_BKE;     // 关闭刹车功能（如果不需要的话）
//
//     // 启动所有PWM通道
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//     HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//     HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//     // 初始化为关闭状态
//     // led_off();
//     // led_on();
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, LED_ON);
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, LED_ON);
//     __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_ON);
//     __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_ON);
//
// }

// 在 led_init() 中添加 GPIO 重新配置
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
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;  // STM32H7 系列需要确认 AF 编号
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
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, LED_ON);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, LED_ON);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_ON);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_ON);

    // 调试：强制输出高电平测试
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}

void front_led_on()
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, LED_ON);
}

void front_led_off()
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, LED_OFF);
}

void back_on()
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, LED_ON);
}

void back_off()
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, LED_OFF);
}

void left_on()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_ON);
}

void left_off()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_OFF);
}

void right_on()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_ON);
}

void right_off()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_OFF);
}
//亮前两个是往前
void move_forward()
{
    front_led_on();
    back_off();
    left_off();
    right_on();
}
//全亮是往后
void move_backward()
{
    front_led_on();
    back_on();
    left_on();
    right_off();
}
//亮左边三个是往左
void turn_left()
{
    front_led_on();
    back_on();
    left_on();
    right_off();
}
//亮右边三个是往右
void turn_right()
{
    front_led_on();
    back_on();
    left_off();
    right_on();
}

void led_off()
{
    front_led_off();
    back_off();
    left_off();
    right_off();
}

void led_on()
{
    front_led_on();
    back_on();
    left_on();
    right_on();
}

#include "led.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void led_init()
{
    // 对于高级定时器TIM1，必须先使能MOE
    TIM1->BDTR |= TIM_BDTR_MOE;

    // 启动所有PWM通道
   // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    // 初始化为关闭状态
    led_off();
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

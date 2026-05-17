#include "led.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void led_init()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, LED_OFF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, LED_OFF);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_OFF);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_OFF);
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

void move_forward()
{
    front_led_on();
    back_off();
    left_off();
    right_off();
}

void move_backward()
{
    front_led_off();
    back_on();
    left_off();
    right_off();
}

void turn_left()
{
    front_led_off();
    back_off();
    left_on();
    right_off();
}

void turn_right()
{
    front_led_off();
    back_off();
    left_off();
    right_on();
}

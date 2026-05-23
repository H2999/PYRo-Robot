#ifndef PYRO_ROBOT_LED_H
#define PYRO_ROBOT_LED_H

#include "tim.h"

static constexpr uint16_t LED_ON = 999;
constexpr uint16_t LED_OFF = 0;

void front_led_on();
void front_led_off();
void back_on();
void back_off();
void left_on();
void left_off();
void right_on();
void right_off();

void led_init();
void move_forward();
void move_backward();
void turn_left();
void turn_right();
void led_off();

#endif //PYRO_ROBOT_LED_H
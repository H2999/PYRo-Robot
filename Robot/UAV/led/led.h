#ifndef PYRO_ROBOT_LED_H
#define PYRO_ROBOT_LED_H

#include "tim.h"

static constexpr uint16_t LED_ON = 999;
constexpr uint16_t LED_OFF = 0;

void led_init();

void front_led_on();
void front_led_off();
void back_led_on();
void back_led_off();

#endif //PYRO_ROBOT_LED_H
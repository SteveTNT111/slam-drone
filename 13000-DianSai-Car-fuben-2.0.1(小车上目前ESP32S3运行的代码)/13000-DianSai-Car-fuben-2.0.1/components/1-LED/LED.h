#ifndef __LED_H_
#define __LED_H_

#include "driver/gpio.h"
void Led_Init(void);
void gpio_toggle(gpio_num_t gpio_num);
#endif
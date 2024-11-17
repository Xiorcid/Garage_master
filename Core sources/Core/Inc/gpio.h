#ifndef _GPIO_H
#define _GPIO_H

#include <stm32f4xx_hal_gpio.h>

#define gpio_NoPull     0
#define gpio_PullUp     1
#define gpio_PullDown   2


void gpio_SetGPIOmode_In(GPIO_TypeDef* GPIOx, uint16_t Mask, uint8_t PullMode);
void gpio_SetGPIOmode_Out(GPIO_TypeDef* GPIOx, uint16_t Mask);
void gpio_PortClockStart(GPIO_TypeDef *GPIOx);

#endif

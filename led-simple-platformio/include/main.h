#ifndef MAIN_H
#define MAIN_H

#include "stm32f7xx_hal.h"

#define LED_PIN                                GPIO_PIN_0
#define LED_GPIO_PORT                          GPIOB
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()

#endif // MAIN_H
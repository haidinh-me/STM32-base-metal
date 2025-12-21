/*
 * stm32_GPIO.h
 *
 *  Created on: Aug 6, 2025
 *      Author: DINH
 */

#ifndef INC_STM32_GPIO_H_
#define INC_STM32_GPIO_H_

#include "stm32f4xx.h"

// GPIO mode options
#define GPIO_MODE_INPUT       0x00
#define GPIO_MODE_OUTPUT      0x01
#define GPIO_MODE_ALTFN       0x02
#define GPIO_MODE_ANALOG      0x03

// GPIO output type
#define GPIO_OTYPE_PP         0x00
#define GPIO_OTYPE_OD         0x01

// GPIO speed
#define GPIO_SPEED_LOW        0x00
#define GPIO_SPEED_MEDIUM     0x01
#define GPIO_SPEED_HIGH       0x02
#define GPIO_SPEED_VERYHIGH   0x03

// GPIO pull-up/pull-down
#define GPIO_NOPULL           0x00
#define GPIO_PULLUP           0x01
#define GPIO_PULLDOWN         0x02

// EXTI
#define GPIO_IRQ_DISABLE      0
#define GPIO_IRQ_ENABLE       1

#define EXTI_RISING_EDGE      0x01
#define EXTI_FALLING_EDGE     0x02
#define EXTI_BOTH_EDGE        0x03

typedef struct{
	uint8_t pin;          // Pin number 0..15
	uint8_t mode;         // Input, Output, Alternate, Analog
	uint8_t speed;        // Speed setting
	uint8_t pull;         // Pull-up/down/none
	uint8_t outputType;   // Push-pull or open-drain
	uint8_t altFunction;  // Alternate function (if used)
	uint8_t irqEnable;    // 0: disable, 1: enable
	uint8_t irqTrigger;    // EXTI_RISING, EXTI_FALLING, EXTI_BOTH
}GPIO_Config_t;

typedef struct{
	GPIO_Register_t *port;
	GPIO_Config_t Config;
}GPIO_Handle_t;

// APIs
void GPIO_PCLK_Control(GPIO_Handle_t *gpio, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *gpio);
void GPIO_DeInit(GPIO_Handle_t *gpio);
void GPIO_WritePin(GPIO_Handle_t *gpio, uint8_t value);
void GPIO_TogglePin(GPIO_Handle_t *gpio);
uint8_t GPIO_ReadPin(GPIO_Handle_t *gpio);

void GPIO_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32_GPIO_H_ */

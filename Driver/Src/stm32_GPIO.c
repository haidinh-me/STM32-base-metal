/*
 * stm32_GPIO.c
 *
 *  Created on: Aug 6, 2025
 *      Author: DINH
 */
#include "stm32_GPIO.h"

void GPIO_PCLK_Control(GPIO_Handle_t *gpio, uint8_t EnorDi){
	if(EnorDi){
		GPIO_PCLK_EN(gpio->port);
	}
	else{
		GPIO_PCLK_DIS(gpio->port);
	}
}

void GPIO_Init(GPIO_Handle_t *gpio){
	uint8_t pin = gpio->Config.pin;

	// Enable clock
	GPIO_PCLK_Control(gpio,ENABLE);

	// Set MODER
	gpio->port->MODER &= ~(0x03 << (pin*2));
	gpio->port->MODER |= (gpio->Config.mode << (pin*2));

	// Set OTYPE
	gpio->port->OTYPER &= ~(1 << pin);
	gpio->port->OTYPER |= (gpio->Config.mode << pin);

	// Set SPEEDR
	gpio->port->OSPEDDR &= ~(0x3 << (pin*2));
	gpio->port->OSPEDDR |= (gpio->Config.speed << (pin*2));

	// Set PuPd
	gpio->port->PUPDR &= ~(0x3 << (pin*2));
	gpio->port->PUPDR |= (gpio->Config.pull << (pin*2));

	// Set Alternate function (if needed)
	if (gpio->Config.mode == GPIO_MODE_ALTFN) {
		uint8_t afr_index = pin / 8;
		uint8_t afr_pos = (pin % 8) * 4;
		gpio->port->AFR[afr_index] &= ~(0xF << afr_pos);
		gpio->port->AFR[afr_index] |= (gpio->Config.altFunction << afr_pos);
	}

	if(gpio->Config.irqEnable == ENABLE){

		// Enable SYSCFG clock
		SYSCFG_CLK_EN();

		// Configure edge

		/*			RISING_EDGE			*/
		if(gpio->Config.irqTrigger == EXTI_RISING_EDGE){
			EXTI->RTSR |= (1 << gpio->Config.pin);
			EXTI->FTSR &= ~(1 << gpio->Config.pin);
		}

		/*			FALLING_EDGE			*/
		if(gpio->Config.irqTrigger == EXTI_FALLING_EDGE){
			EXTI->FTSR |= (1 << gpio->Config.pin);
			EXTI->RTSR &= ~(1 << gpio->Config.pin);
		}

		/*			BOTH_EDGE			*/
		if(gpio->Config.irqTrigger == EXTI_BOTH_EDGE){
			EXTI->FTSR |= (1 << gpio->Config.pin);
			EXTI->RTSR |= (1 << gpio->Config.pin);
		}

		// Configure SYSCFG_EXTICR
		uint8_t temp= gpio->Config.pin / 4;
		uint8_t temp1= gpio->Config.pin % 4;
		SYSCFG->EXTICR[temp]= (GPIO_OFFSET(gpio->port) << (temp1 * 4));

		// Disable mask register
		EXTI->IMR |= (1 << gpio->Config.pin);
	}
}

void GPIO_DeInit(GPIO_Handle_t *gpio){
		GPIO_RCC_RESET(GPIO_OFFSET(gpio->port));
}

void GPIO_WritePin(GPIO_Handle_t *gpio, uint8_t value){
	gpio->port->BSRR |= (value << gpio->Config.pin);
}

void GPIO_TogglePin(GPIO_Handle_t *gpio){
	gpio->port->ODR ^= (1 << gpio->Config.pin);
}

uint8_t GPIO_ReadPin(GPIO_Handle_t *gpio){
	return (gpio->port->IDR << gpio->Config.pin) & 0x1 ;
}

void GPIO_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31)
			*NVIC_ISER(0) |= (1U << IRQNumber);
		else if(IRQNumber >=32 && IRQNumber <= 223 )
				*NVIC_ISER( (uint8_t)(IRQNumber - IRQNumber % 32)/32 ) |= (1U << (IRQNumber % 32) );
		else *NVIC_ISER(7) |= (1U << (IRQNumber %32) ) ;
	}
	else if(EnorDi == DISABLE){
		if(IRQNumber <= 31)
			*NVIC_ICER(0) |= (1U << IRQNumber);
		else if(IRQNumber >=32 && IRQNumber <= 223 )
			*NVIC_ICER( (uint8_t)(IRQNumber - IRQNumber % 32)/32 ) |= (1U << (IRQNumber % 32) );
		else *NVIC_ICER(7) |= (1U << (IRQNumber %32) ) ;
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;

	*(NVIC_IPR_BASEADDR + ipr) |= (IRQPriority << (irq*8));
}

void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}

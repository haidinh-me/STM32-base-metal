#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32_GPIO.h"

void delay(){
	for(uint32_t i=0; i<5000; i++);
}

int main(void)
{
	return 0;
}

GPIO_Handle_t ButtonPA13, LedPB11;
void Button_Interrup(){
	// Configuration Button
	ButtonPA13.port = GPIOA;
	ButtonPA13.Config.pin = 13;
	ButtonPA13.Config.mode = GPIO_MODE_INPUT;
	ButtonPA13.Config.speed = GPIO_SPEED_VERYHIGH;
	ButtonPA13.Config.outputType= GPIO_OTYPE_PP;
	ButtonPA13.Config.pull = GPIO_PULLUP;
	ButtonPA13.Config.irqEnable = GPIO_IRQ_ENABLE;
	ButtonPA13.Config.irqTrigger= EXTI_FALLING_EDGE;
	GPIO_Init(&ButtonPA13);
	GPIO_IRQInterrupConfig(EXTI15_10,ENABLE);
	GPIO_IRQPriorityConfig(EXTI15_10,13);

	// Configuration Led
	LedPB11.port = GPIOB;
	LedPB11.Config.pin = 11;
	LedPB11.Config.mode = GPIO_MODE_OUTPUT;
	LedPB11.Config.speed = GPIO_SPEED_VERYHIGH;
	LedPB11.Config.outputType = GPIO_OTYPE_PP;
	LedPB11.Config.pull = GPIO_NOPULL;
	GPIO_Init(&LedPB11);
	GPIO_WritePin(&LedPB11,LOW);

}

void EXTI15_10_IRQHandler(){
	delay();
	GPIO_IRQHandling(11);
	GPIO_TogglePin(&LedPB11);
}

void Toggle_Led(){
	GPIO_Handle_t LedPA5;
		LedPA5.port = GPIOA;
		LedPA5.Config.pin = 5;
		LedPA5.Config.mode = GPIO_MODE_OUTPUT;
		LedPA5.Config.speed = GPIO_SPEED_VERYHIGH;
		LedPA5.Config.pull = GPIO_NOPULL;
		LedPA5.Config.outputType = GPIO_OTYPE_PP;

		GPIO_Init(&LedPA5);

		while(1)
		{
			GPIO_TogglePin(&LedPA5);
			delay();
		}
}

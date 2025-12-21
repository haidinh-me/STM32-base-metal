/*
 * USART_Test.c
 *
 *  Created on: Aug 21, 2025
 *      Author: DINH
 */

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32_GPIO.h"
#include "stm32_USART.h"

USART_Handle_t USART2_test;
char Data[] = "Hello HDin \n";

void GPIO_InitUSART(){
	GPIO_Handle_t GPIO_USART;
	GPIO_USART.port = GPIOD;
	GPIO_USART.Config.mode = GPIO_MODE_ALTFN;
	GPIO_USART.Config.outputType = GPIO_OTYPE_PP;
	GPIO_USART.Config.altFunction = 7;
	GPIO_USART.Config.pull = GPIO_PULLUP;
	GPIO_USART.Config.speed = GPIO_SPEED_VERYHIGH;

	// Tx Pin
	GPIO_USART.Config.pin = 5;
	GPIO_Init(&GPIO_USART);

	// Rx Pin
	GPIO_USART.Config.pin = 6;
	GPIO_Init(&GPIO_USART);
}

void USART2_Init(){
	USART2_test.pUSARTx = USART2;
	USART2_test.Config.Mode = USART_MODE_TX;
	USART2_test.Config.HWFlow = USART_HWFLOW_NONE;
	USART2_test.Config.Parity = USART_PARITY_DIS;
	USART2_test.Config.StopBit = USART_STOPBIT_1;
	USART2_test.Config.WordLenth = USART_WORDLENTH_8BIT;
	USART2_test.Config.BaudRate = USART_BAUDRATE_115200;
	USART_Init(&USART2_test);
}

int main(){
	GPIO_InitUSART();
	USART2_Init();

	while(1){
		USART_SentData(&USART2_test,(uint8_t*)Data,strlen(Data));
		while(1);
	}
	return 0;
}

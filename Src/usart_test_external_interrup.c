/*
 * usart_test_external_interrup.c
 *
 *  Created on: Sep 24, 2025
 *      Author: DINH
 */

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32_USART.h"
#include "stm32_GPIO.h"

char buffer[7];
int a=0;
int b=0;
USART_Handle_t USART6_test;

void USART_GPIOInit(){
	GPIO_Handle_t GPIO;
	GPIO.port = GPIOA;
	GPIO.Config.mode = GPIO_MODE_ALTFN;
	GPIO.Config.altFunction = 8;
	GPIO.Config.outputType = GPIO_OTYPE_PP;
	GPIO.Config.speed = GPIO_SPEED_VERYHIGH;
	GPIO.Config.pull = GPIO_NOPULL;

	//USART6_TX
	GPIO.Config.pin = 11;
	GPIO_Init(&GPIO);

	//USART6_RX
	GPIO.Config.pin = 12;
	GPIO_Init(&GPIO);
}

void USART_Initialize(){
	USART6_test.pUSARTx = USART6;
	USART6_test.Config.Mode = USART_MODE_BOTH;
	USART6_test.Config.HWFlow = USART_HWFLOW_NONE;
	USART6_test.Config.Parity = USART_PARITY_DIS;
	USART6_test.Config.StopBit = USART_STOPBIT_1;
	USART6_test.Config.WordLenth = USART_WORDLENTH_8BIT;
	USART6_test.Config.BaudRate = USART_BAUDRATE_115200;
	USART_Init(&USART6_test);
}

int main(){
	USART_GPIOInit();
	USART_Initialize();
	USART_IRQInterrupConfig(IRQ_USART6,ENABLE);

	while(1){
		while(USART_ReciveDataIT(&USART6_test, (uint8_t *)buffer, 6) != USART_BUSY_READY);
		USART_SentDataIT(&USART6_test, (uint8_t *)buffer, 6);
	}

	return 0;
}

void USART6_IRQHandler (void){
	USART_IRQHandling(&USART6_test);
}

void USART_ApplicationEventCallBack(USART_Handle_t *pUSART, uint8_t AppEv){
	if(AppEv == USART_EVEN_RX_CMPLT){
		a=1;
	}
	else if(AppEv == USART_EVEN_TX_CMPLT){
		b=1;
	}
}

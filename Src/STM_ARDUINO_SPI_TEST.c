/*
 * STM_ARDUINO_SPI_TEST.c
 *
 *  Created on: Aug 14, 2025
 *      Author: DINH
 */

#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "stm32_SPI.h"
#include "stm32_GPIO.h"

#define LEN_MAX 500

/*
 * PB12		SPI2_NSS
 * PB13		SPI2_SCK
 * PB14		SPI2_MISO
 * PB15		SPI2_MOSI
 * Alternate Function: 5
 */

SPI_Handle_t pSPI2;
char RxBuffer[LEN_MAX];
uint8_t ReadByte;

void SPI_GPIO_Init(){
	GPIO_Handle_t GPIO_config;
	GPIO_config.port = GPIOB;
	GPIO_config.Config.mode = GPIO_MODE_ALTFN;
	GPIO_config.Config.altFunction = 5;
	GPIO_config.Config.outputType = GPIO_OTYPE_PP;
	GPIO_config.Config.pull		  = GPIO_NOPULL;
	GPIO_config.Config.speed	  = GPIO_SPEED_VERYHIGH;

	GPIO_config.Config.pin = 12;
	GPIO_Init(&GPIO_config);

	GPIO_config.Config.pin = 13;
	GPIO_Init(&GPIO_config);

	GPIO_config.Config.pin = 14;
	GPIO_Init(&GPIO_config);

	GPIO_config.Config.pin = 15;
	GPIO_Init(&GPIO_config);
}

void SPI2_Init(){
	pSPI2.pSPI = SPI2;
	pSPI2.SPIConfig.DeviceMode = DEVICE_MODE_MASTER;
	pSPI2.SPIConfig.BusConfig = BUS_CONFIG_FD;
	pSPI2.SPIConfig.SclkSpeed = SCLK_SPEED_DIV8;
	pSPI2.SPIConfig.SSM 		= DISABLE;
	pSPI2.SPIConfig.CPOL		= LOW;
	pSPI2.SPIConfig.CPHA		= LOW;

	SPI_Init(&pSPI2);
}

int main(void){
	SPI_GPIO_Init();
	SPI2_Init();

	SPI_SSOEConfig(SPI2,ENABLE);
	SPI_IRQInterrupConfig(IRQ_SPI2,ENABLE);

	while(1){
		SPI_PripheralControl(SPI2,ENABLE);
		while(SPI_ReciveDataIT(&pSPI2,&ReadByte,1) == SPI_BUSY_IN_RX);
		while(pSPI2.pSPI->SR & (1 << SR_BSY));
		SPI_PripheralControl(SPI2,DISABLE);
	}

	return 0;
}

void SPI2_IRQHandler(){
	SPI_IRQHandling(&pSPI2);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t event){
	static uint32_t i=0;
	if(event == SPI_EVENT_RX_CMPLT){
		RxBuffer[i++] = ReadByte;
		if(ReadByte == '\0' || (i == LEN_MAX)){
			RxBuffer[i-1] = '\0';
			i=0;
		}
	}
}

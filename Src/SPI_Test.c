/*
 * SPI_Test.c
 *
 *  Created on: Aug 10, 2025
 *      Author: DINH
 */

#include "stm32f4xx.h"
#include "stm32_GPIO.h"
#include "stm32_SPI.h"
#include "string.h"

/*
 * PB12		SPI2_NSS
 * PB13		SPI2_SCK
 * PB14		SPI2_MISO
 * PB15		SPI2_MOSI
 * Alternate Function: 5
 */

void SPI_GPIOInit()
{
	GPIO_Handle_t SPI_GPIO;
	SPI_GPIO.port = GPIOB;
	SPI_GPIO.Config.mode = GPIO_MODE_ALTFN;
	SPI_GPIO.Config.outputType = GPIO_OTYPE_PP;
	SPI_GPIO.Config.pull = GPIO_NOPULL;
	SPI_GPIO.Config.speed = GPIO_SPEED_VERYHIGH;
	SPI_GPIO.Config.altFunction = 5;

	// Init NSS PIN
	SPI_GPIO.Config.pin = 12;
	GPIO_Init(&SPI_GPIO);

	// Init SCK PIN
	SPI_GPIO.Config.pin = 13;
	GPIO_Init(&SPI_GPIO);

	// Init MISO PIN
	SPI_GPIO.Config.pin = 14;
	GPIO_Init(&SPI_GPIO);

	// Init MOSI PIN
	SPI_GPIO.Config.pin = 15;
	GPIO_Init(&SPI_GPIO);

}

void SPI2_Init()
{
	SPI_Handle_t SPI_GPIO;
	SPI_GPIO.pSPI = SPI2;
	SPI_GPIO.SPIConfig.DeviceMode	= DEVICE_MODE_MASTER;
	SPI_GPIO.SPIConfig.BusConfig	= BUS_CONFIG_FD;
	SPI_GPIO.SPIConfig.CPOL			= CPOL_HIGH;
	SPI_GPIO.SPIConfig.CPHA			= CPHA_LOW;
	SPI_GPIO.SPIConfig.DFF			= DFF_8BIT;
	SPI_GPIO.SPIConfig.SclkSpeed	= SCLK_SPEED_DIV2;
	SPI_GPIO.SPIConfig.SSM			= SSM_ENABLE;
	SPI_Init(&SPI_GPIO);
}

int main(){
	char data[] = "Hello Word!";
	SPI_GPIOInit();
	SPI2_Init();
	SPI_SSIConfig(SPI2,ENABLE);

	SPI_SendData(SPI2,(uint8_t*)data,strlen(data));

	while(!(SPI2->SR & (1 << SR_BSY)));

	SPI_PripheralControl(SPI2,DISABLE);

	return 0;
}

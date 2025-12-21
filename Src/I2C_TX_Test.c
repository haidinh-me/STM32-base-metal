/*
 * I2C_TX_Test.c
 *
 *  Created on: Sep 29, 2025
 *      Author: DINH
 */

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32_GPIO.h"
#include "stm32_I2C.h"

#define MASTER_ADDR	0x19
#define SLAVE_ADDR	0x04

I2C_Handle_t I2C1_Test;
uint8_t txBuffer[] = "Hello Hai Din\n";

void I2C_GPIO_Init(){
	GPIO_Handle_t GPIO;
	GPIO.port = GPIOB;

	GPIO.Config.mode 		= GPIO_MODE_ALTFN;
	GPIO.Config.altFunction = 4;
	GPIO.Config.outputType 	= GPIO_OTYPE_OD;
	GPIO.Config.pull 		= GPIO_PULLUP;
	GPIO.Config.speed 		= GPIO_SPEED_HIGH;

	//PB6 : I2C1_SCL
	GPIO.Config.pin			= 6;
	GPIO_Init(&GPIO);

	//PB7: I2C1_SDA
	GPIO.Config.pin			= 7;
	GPIO_Init(&GPIO);
}

void I2C1_Init(){
	I2C1_Test.pI2Cx = I2C1;

	I2C1_Test.Config.AckControl 	= I2C_ACK_CONTROL_EN;
	I2C1_Test.Config.FMDutyCycle 	= I2C_FM_DUTY_CYCLE_2;
	I2C1_Test.Config.SCLSpeed		= I2C_SCL_SPEED_SM;
	I2C1_Test.Config.DeviceAddress 	= MASTER_ADDR;

	I2C_Init(&I2C1_Test);
}

int main(void){
	I2C_GPIO_Init();
	I2C1_Init();

	while(1){
		I2C_MasterSendData(&I2C1_Test, txBuffer, strlen((char*)txBuffer), SLAVE_ADDR, 0);
		while(1);
	}

	return 0;
}


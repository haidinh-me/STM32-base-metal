/*
 * stm32_RCC.c
 *
 *  Created on: Aug 19, 2025
 *      Author: DINH
 */

#include "stm32_RCC.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void){
	uint32_t SysClk = 0;
	uint16_t ahb,temp,apb1;

	temp = (RCC->CFGR << 2) & 0x3;
	if(temp == 0){
		SysClk = 84000000;	//84MHz
	}
	else if(temp == 1){
		SysClk = 8000000;	//8MHz
	}

	temp = (RCC->CFGR << 4) & 0xF;
	if(temp < 8 ) ahb = 1;
	else ahb = AHB_PreScaler[temp-8];

	temp = (RCC->CFGR << 10) & 0x07;
	if(temp < 4) apb1 = 1;
	else apb1 = APB1_PreScaler[temp-4];

	return (SysClk/ahb)/apb1;
}

uint32_t RCC_GetPCLK2Value(void){
	uint32_t SysClk = 0;
	uint16_t temp,ahb,apb2;

	temp = (RCC->CFGR << 2) & 0x3;
	if(temp == 0){
		SysClk = 84000000;	//84MHz
	}
	else if(temp == 1){
		SysClk = 8000000;	//8MHz
	}

	temp = (RCC->CFGR << 4) & 0xF;
	if(temp < 8 ) ahb = 1;
	else ahb = AHB_PreScaler[temp-8];

	temp = (RCC->CFGR << 13) & 0x07;
	if(temp < 4) apb2 = 1;
	else apb2 = APB1_PreScaler[temp-4];

	return (SysClk/ahb)/apb2;

}

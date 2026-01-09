/*
 * MPU_Test.c
 *
 *  Created on: Jan 8, 2026
 *      Author: Admin
 */

#include <stdint.h>

#include <stm32_MPU.h>

int main()
{
	//Region 0
	MPU->RNR = 0;								//Region mumber
	MPU->RBAR = 0x00;							//Start Address
	MPU->RASR |= (0b11111 << MPU_RASR_SIZE);	//Size 4GB region
	MPU->RASR |= (0b001 << MPU_RASR_AP);		//Full Access Permission
	MPU->RASR |= (0x1Ul << MPU_RASR_ENABLE) ;	//Enable Region

	//Region 1
	MPU->RNR = 0;								//Region mumber
	MPU->RBAR = 0x40000000;						//Start Address
	MPU->RASR |= (0b01001 << MPU_RASR_SIZE);	//Size 1GB region
	MPU->RASR |= (0b001000 << MPU_RASR_B);		//Memory type nomal, Not Shareable
	MPU->RASR |= (0b111 << MPU_RASR_AP);		//Read only Permission
	MPU->RASR |= (0x1UL << MPU_RASR_ENABLE);		//Enable Region

	//Enable MPU
	MPU->CTRL |= (0x1Ul << MPU_CTRL_ENABLE) ;	//Enable MPU

	//Write to Read Only Memory
	*(uint32_t* )0x40000000 = 0x123456;

	for(;;)

	return 0;
}

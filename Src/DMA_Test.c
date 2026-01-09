/*
 * DMA_Test.c
 *
 *  Created on: Jan 8, 2026
 *      Author: Admin
 */

#include <stm32_DMA.h>

#define SRCADD		0x20016000
#define DESTADD		0x20017000

int main(void)
{
	DMA_Init();

	*(uint32_t *)SRCADD = 0x4A4A4A;

	DMA_Config(SRCADD, DESTADD, 4);

	Start_Tranfer();

	for(;;);

	return 0;
}

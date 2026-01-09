/*
 * stm32_DMA.c
 *
 *  Created on: Jan 7, 2026
 *      Author: Admin
 */

#include <stm32_DMA.h>

void DMA_Init(void)
{
	//Enable clock
	DMA2_PCLK_EN();

	//Chanel select

	//Data direction
	DMA2->STREAM[0].SCR |= (MEM_TO_MEM << DMA_SCR_DIR);

	//Increment mode
	DMA2->STREAM[0].SCR |= ( (1 << DMA_SCR_MINC) || (1 << DMA_SCR_PINC) );

	//Data size
	DMA2->STREAM[0].SCR &= ~(0xF << DMA_SCR_PSIZE);

	//Priority level
	DMA2->STREAM[0].SCR |= (1 << DMA_SCR_PL);
}

void DMA_Config(int32_t srcAdd, uint32_t destAdd, uint16_t dataSize)
{
	DMA2->STREAM[0].SNDTR = dataSize;

	DMA2->STREAM[0].SPAR = srcAdd;

	DMA2->STREAM[0].SMxAR[0] = destAdd;
}

void Start_Tranfer(void)
{
	DMA2->STREAM[0].SCR |=(1 << DMA_SCR_EN);
}


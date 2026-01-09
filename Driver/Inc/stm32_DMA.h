/*
 * stm32_DMA.h
 *
 *  Created on: Jan 7, 2026
 *      Author: Admin
 */

#ifndef INC_STM32_DMA_H_
#define INC_STM32_DMA_H_

#include <stm32f4xx.h>

#define DMA_SCR_EN		0		//Stream enable / flag stream ready when read low
#define DMA_SCR_DMEIE	1		//Direct mode error interrupt enable
#define DMA_SCR_TEIE	2		//Transfer error interrupt enable
#define DMA_SCR_HTIE	3		//Half tranfer interrupt enabale
#define DMA_SCR_TCIE	4		//Tranfer complete interrupt enable
#define DMA_SCR_PFCTRL	5		//Peripheral flow controller
#define DMA_SCR_DIR		6		//Data tranfer direction
#define DMA_SCR_CIRC	8		//Circular mode
#define DMA_SCR_PINC	9		//Peripheral increment mode
#define DMA_SCR_MINC	10		//Memory increment mode
#define DMA_SCR_PSIZE	11		//Peripheral data size
#define DMA_SCR_MSIZE	13		//Memory data size
#define DMA_SCR_PINCOS	15		//Peripheral increment offset size
#define DMA_SCR_PL		16		//Priority level
#define DMA_SCR_DBM		18		//Double buffer mode
#define DMA_SCR_CT		19		//Current target
#define DMA_SCR_PBURST	23		//Peripheral burst transfer configuration
#define DMA_SCR_MBURST	23		//Memory burst transfer configuration
#define DMA_SCR_CHSEL	25		//Chanel selection

#define DMA_SNDTR_NDT	0		//Number of data items to tranfer

#define PER_TO_MEM		0
#define MEM_TO_PER		1
#define MEM_TO_MEM		2

void DMA_Init(void);
void DMA_Config(int32_t srcAdd, uint32_t destAdd, uint16_t dataSize);
void Start_Tranfer(void);

#endif /* INC_STM32_DMA_H_ */

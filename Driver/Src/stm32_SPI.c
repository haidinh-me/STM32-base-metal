/*
 * stm32_SPI.c
 *
 *  Created on: Aug 9, 2025
 *      Author: DINH
 */

#include "stm32_SPI.h"

static void spi_txe_interrup_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrup_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrup_handle(SPI_Handle_t *pSPIHandle);

void SPI_PriClockControl(SPI_Handle_t *pSPIx, uint8_t EnorDi){
	if(EnorDi)
	{
		if(pSPIx->pSPI == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx->pSPI == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx->pSPI == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx->pSPI == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else if(!EnorDi){
		if(pSPIx->pSPI == SPI1)
			{
				SPI1_PCLK_DIS();
			}
			else if(pSPIx->pSPI == SPI2)
			{
				SPI2_PCLK_DIS();
			}
			else if(pSPIx->pSPI == SPI3)
			{
				SPI3_PCLK_DIS();
			}
			else if(pSPIx->pSPI == SPI4)
			{
				SPI4_PCLK_DIS();
			}
	}
}

void SPI_Init(SPI_Handle_t *pSPIx){
	uint32_t temreg = 0;

	/*-----------------------------------------Enable clock --------------------------------------*/
	SPI_PriClockControl(pSPIx,ENABLE);

	/*-----------------------------------------Configure Device Mode------------------------------*/
	temreg |= pSPIx->SPIConfig.DeviceMode << CR1_MSTR;

	/*-----------------------------------------Configure Bus--------------------------------------*/
	if(pSPIx->SPIConfig.BusConfig == BUS_CONFIG_FD)
	{
		temreg &= ~(1 << CR1_BIDIMODE);
	}
	else if(pSPIx->SPIConfig.BusConfig == BUS_CONFIG_HD)
	{
		temreg |= 1 << CR1_BIDIMODE;
	}
	else if(pSPIx->SPIConfig.BusConfig == BUS_CONFIG_SIMPLEX_RXONLY)
	{
		temreg &= ~(1 << CR1_BIDIMODE);
		temreg |= 1 << CR1_RXONLY;
	}

	/*----------------------------------------Configure Clock Speed(Baud rate)--------------------*/
	temreg |= pSPIx->SPIConfig.SclkSpeed << CR1_BR;

	/*----------------------------------------Data frame format-----------------------------------*/
	temreg |= pSPIx->SPIConfig.DFF << CR1_DFF;

	/*----------------------------------------Configure CPOL--------------------------------------*/
	temreg |= pSPIx->SPIConfig.CPOL << CR1_CPOL;

	/*----------------------------------------Configure CPHA--------------------------------------*/
	temreg |= pSPIx->SPIConfig.CPHA << CR1_CPHA;

	/*----------------------------------------Configure SSM---------------------------------------*/
	temreg |= pSPIx->SPIConfig.SSM << CR1_SSM;

	(pSPIx->pSPI)->CR1 = temreg;

	/*----------------------------------------Enable Pripheral SPIx-------------------------------*/
	SPI_PripheralControl(pSPIx->pSPI,ENABLE);

}

void SPI_DeInit(SPI_Register_t *pSPIx){

}

void SPI_SendData(SPI_Register_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	while(len > 0){
		// Wait for  TXE is set
		while(!(pSPIHandle->SR & (1 << SR_TXE)));

		// DFF is 16-bit frame
		if(pSPIHandle->CR1 & (1 << CR1_DFF)){
			pSPIHandle->DR = *((uint16_t *)pTxBuffer);
			pTxBuffer += 2;
			len -= 2;
		}

		// DFF is 8-bit frame
		else {
			pSPIHandle->DR = *pTxBuffer;
			pTxBuffer ++;
			len--;
		}
	}

	// Wait for Busy flag(BSY) is set
	while(!(pSPIHandle->SR & (1 << SR_BSY)));
}

void SPI_ReciveData(SPI_Register_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	while(len >0){
		// Wait for RXNE is set
		while(!(pSPIHandle->SR & (1 << SR_RXNE)));

		// DFF is 16-bit frame
		if(pSPIHandle->CR1 & (1 << CR1_DFF)){
			*((uint16_t *)pRxBuffer) = pSPIHandle->DR;
			pRxBuffer +=2;
			len -=2;
		}
		// DFF is 8-bit frame
		else{
			*(pRxBuffer) = pSPIHandle->DR;
			pRxBuffer ++;
			len --;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	if(pSPIHandle->TxState 		!= SPI_BUSY_IN_TX){
		pSPIHandle->pTxBuffer 	= pTxBuffer;
		pSPIHandle->TxLen		= len;
		pSPIHandle->TxState		= SPI_BUSY_IN_TX;
		pSPIHandle->pSPI->CR2	|= (1 << CR2_TXEIE);
	}
	return pSPIHandle->TxState;
}

uint8_t SPI_ReciveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	if(pSPIHandle->RxState 		!= SPI_BUSY_IN_RX){
		pSPIHandle->pRxBuffer 	= pRxBuffer;
		pSPIHandle->RxLen		= len;
		pSPIHandle->RxState		= SPI_BUSY_IN_RX;
		pSPIHandle->pSPI->CR2	|= (1 << CR2_RXNEIE);
	}
	return pSPIHandle->RxState;
}

void SPI_PripheralControl(SPI_Register_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << CR1_SPE);
	}
	else pSPIx->CR1 &= ~(1 << CR1_SPE);
}

void SPI_SSIConfig(SPI_Register_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << CR1_SSI);
	}
	else pSPIx->CR1 &= ~(1 << CR1_SSI);
}

void SPI_SSOEConfig(SPI_Register_t *pSPIx, uint8_t EnorDi){
	if(EnorDi){
		pSPIx->CR2 |= (1 << CR2_SSOE);
	}
	else {
		pSPIx->CR2 &= ~(1 << CR2_SSOE);
	}
}

void SPI_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31)
			*NVIC_ISER(0) |= (1U << IRQNumber);
		else if(IRQNumber >=32 && IRQNumber <= 223 )
				*NVIC_ISER( (uint8_t)(IRQNumber - IRQNumber % 32)/32 ) |= (1U << (IRQNumber % 32) );
		else *NVIC_ISER(7) |= (1U << (IRQNumber %32) ) ;
	}
	else if(EnorDi == DISABLE){
		if(IRQNumber <= 31)
			*NVIC_ICER(0) |= (1U << IRQNumber);
		else if(IRQNumber >=32 && IRQNumber <= 223 )
			*NVIC_ICER( (uint8_t)(IRQNumber - IRQNumber % 32)/32 ) |= (1U << (IRQNumber % 32) );
		else *NVIC_ICER(7) |= (1U << (IRQNumber %32) ) ;
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;

	*(NVIC_IPR_BASEADDR + ipr) |= (IRQPriority << (irq*8));
}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1,temp2;

	// TXE
	temp1 = pSPIHandle->pSPI->SR & (1 << SR_TXE);
	temp2 = pSPIHandle->pSPI->CR2 & (1 << CR2_TXEIE);
	if(temp1 && temp2){
		spi_txe_interrup_handle(pSPIHandle);
	}

	// RXNE
	temp1 = pSPIHandle->pSPI->SR & (1 << SR_RXNE);
	temp2 = pSPIHandle->pSPI->CR2 & (1 << CR2_RXNEIE);
	if(temp1 && temp2){
		spi_rxne_interrup_handle(pSPIHandle);
	}

	// OVER Flag
	temp1 = pSPIHandle->pSPI->SR & (1 << SR_OVR);
	temp2 = pSPIHandle->pSPI->CR2 & (1 << CR2_ERRIE);
	if(temp1 && temp2){
		spi_ovr_err_interrup_handle(pSPIHandle);
	}
}


static void spi_txe_interrup_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPI->CR1 & (1 << CR1_DFF)){
		// 16-bit
		pSPIHandle->pSPI->DR = *(uint16_t *)(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer +=2;
		pSPIHandle->TxLen -=2;
	}
	else {
		// 8-bit
		pSPIHandle->pSPI->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer ++;
		pSPIHandle->TxLen --;
	}
	if(! pSPIHandle->TxLen){
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPI->CR1 &= ~(1 << CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

static void spi_rxne_interrup_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPI->DR & (1 << CR1_DFF)){
		//16-bit data
		*(uint16_t *)pSPIHandle->pRxBuffer = pSPIHandle->pSPI->DR;
		pSPIHandle->pRxBuffer +=2;
		pSPIHandle->RxLen -=2;
	}
	else {
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPI->DR;
		pSPIHandle->pRxBuffer ++;
		pSPIHandle->RxLen ++;
	}
	if(!pSPIHandle->RxLen){
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPI->CR2 &= ~(1 << CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
static void spi_ovr_err_interrup_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPI->DR;
		temp = pSPIHandle->pSPI->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t event){
}

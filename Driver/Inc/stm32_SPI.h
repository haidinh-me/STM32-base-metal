/*
 * stm32_SPI.h
 *
 *  Created on: Aug 9, 2025
 *      Author: DINH
 */

#ifndef INC_STM32_SPI_H_
#define INC_STM32_SPI_H_

#include "stm32f4xx.h"
#include "stddef.h"

// Master selection
#define DEVICE_MODE_MASTER				1
#define DEVICE_MODE_SLAVE				0

// Bidirectional data mode enable
#define BUS_CONFIG_FD					1
#define BUS_CONFIG_HD					2
#define BUS_CONFIG_SIMPLEX_RXONLY		3

#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4

// Baud rate control
#define SCLK_SPEED_DIV2					0
#define SCLK_SPEED_DIV4					1
#define SCLK_SPEED_DIV8					2
#define SCLK_SPEED_DIV16				3
#define SCLK_SPEED_DIV32				4
#define SCLK_SPEED_DIV64				5
#define SCLK_SPEED_DIV128				6
#define SCLK_SPEED_DIV256				7

// DFF : Data frame format
#define DFF_8BIT						0
#define DFF_16BIT						1

// CPOL, CPHA
#define CPOL_LOW						0
#define CPOL_HIGH						1
#define CPHA_LOW						0
#define CPHA_HIGH						1

// SSM: Software slave management
#define SSM_ENABLE						1
#define SSM_DISABLE						0

#define CR1_CPHA						0
#define CR1_CPOL						1
#define CR1_MSTR						2
#define CR1_BR							3
#define CR1_SPE							6
#define CR1_LSBFIRST					7
#define CR1_SSI							8
#define CR1_SSM							9
#define CR1_RXONLY						10
#define CR1_DFF							11
#define CR1_CRCNEXT						12
#define CR1_CRCEN						13
#define CR1_BIDIOE						14
#define CR1_BIDIMODE					15

#define CR2_RXDMAEN						0
#define CR2_TXDMAEN						1
#define CR2_SSOE						2
#define CR2_FRF							4
#define CR2_ERRIE						5
#define CR2_RXNEIE						6
#define CR2_TXEIE						7


#define SR_RXNE							0		//RXNE: Receive buffer not empty (0: Rx buffer empty; 1: Rx buffer not empty)
#define SR_TXE							1		//Transmit buffer empty (0: Tx buffer not empty; 1: Tx buffer empty)
#define SR_CHS_IDE						2
#define SR_UDR							3
#define SR_CRCERR						4
#define SR_MODF							5
#define SR_OVR							6
#define SR_BSY							7		//Busy flag (0: SPI(or I2S) not busy; 1: SPI (or I2S)is busy in communication or Tx buffer is not empty)
#define SR_FRE							8		//FRE: Frame format error (0: No frame format error; 1: A frame format error occurred)


typedef struct{
	uint8_t DeviceMode;
	uint8_t BusConfig;
	uint8_t SclkSpeed;
	uint8_t DFF;
	uint8_t CPOL;
	uint8_t CPHA;
	uint8_t SSM;
}SPI_Config_t;

typedef struct{
	SPI_Register_t *pSPI;
	SPI_Config_t SPIConfig;
	uint8_t		*pTxBuffer;
	uint8_t		*pRxBuffer;
	uint32_t	TxLen;
	uint32_t	RxLen;
	uint8_t		TxState;
	uint8_t		RxState;
}SPI_Handle_t;

void SPI_PriClockControl(SPI_Handle_t *pSPIx, uint8_t EnorDi);
void SPI_Init(SPI_Handle_t *pSPIx);
void SPI_DeInit(SPI_Register_t *pSPIx);

void SPI_PripheralControl(SPI_Register_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_Register_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_Register_t *pSPIx, uint8_t EnorDi);

void SPI_SendData(SPI_Register_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReciveData(SPI_Register_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReciveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t event);

void SPI_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPrority);
void SPI_IRQHandling(SPI_Handle_t *SPIHandle);

#endif /* INC_STM32_SPI_H_ */

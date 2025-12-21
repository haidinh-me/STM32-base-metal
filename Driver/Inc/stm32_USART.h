/*
 * stm32_USART.h
 *
 *  Created on: Aug 17, 2025
 *      Author: DINH
 */

#include "stm32f4xx.h"

#ifndef INC_STM32_USART_H_
#define INC_STM32_USART_H_

// USART_SR (Status Register)
#define USART_SR_PE				0	//Parity error
#define USART_SR_FE				1	//Framing error
#define USART_SR_NF				2	//Noise detected flag
#define USART_SR_ORE			3	//Overrun error
#define USART_SR_IDLE			4	//IDLE line detected
#define USART_SR_RXNE			5	//Read data register not empty
#define USART_SR_TC				6	//Transmission complete
#define USART_SR_TXE			7	//Transmit data register empty
#define USART_SR_LBD			8	//LIN break detection flag
#define USART_SR_CTS			9	//CTS flag

// USART_CR1 (Control Register)
#define USART_CR1_SBK			0	//Send break
#define USART_CR1_RWU			1	//Receiver wake-up
#define USART_CR1_RE			2	//Receiver enable
#define USART_CR1_TE			3	//Transmitter enable
#define USART_CR1_IDLEIE		4	//IDLE interrupt enable
#define USART_CR1_RXNEIE		5	//RXNE interrupt enable
#define USART_CR1_TCIE			6	//Transmission complete interrupt enable
#define USART_CR1_TXEIE			7	//TXE interrupt enable
#define USART_CR1_PEIE			8	//PE interrupt enable
#define USART_CR1_PS			9	//Parity selection
#define USART_CR1_PCE			10	//Parity control enable
#define USART_CR1_WAKE			11	//Wake-up method
#define USART_CR1_M				12	//Word length
#define USART_CR1_UE			13	//USART enable
#define USART_CR1_OVER8			15	//Oversampling mode

// USART_CR2
#define USART_CR2_ADD			0	//Address of the USART node
#define USART_CR2_LBDL			5	//lin break detection length
#define USART_CR2_LBDIE			6	//LIN break detection interrupt enable
#define USART_CR2_LBCL			8	//Last bit clock pulse
#define USART_CR2_CPHA			9	//Clock phase
#define USART_CR2_CPOL			10	//Clock polarity
#define USART_CR2_CLKEN			11	//Clock enable
#define USART_CR2_STOP			12	//STOP bits	(00: 1 Stop bit) (01: 0.5 Stop bit) (10: 2 Stop bits) (Note: 11: 1.5 Stop bit)
#define USART_CR2_LINEN			14	//LIN mode enable

// USART_CR3
#define USART_CR3_EIE			0	//Error interrupt enable
#define USART_CR3_IREN			1	//IrDA mode enable
#define USART_CR3_IRLP			2	//IrDA low-power
#define USART_CR3_HDSEL			3	//Half-duplex selection
#define USART_CR3_NACK			4	//Smartcard NACK enable
#define USART_CR3_SCEN			5	//Smartcard mode enable
#define USART_CR3_DMAR			6	//DMA enable receiver
#define USART_CR3_DMAT			7	//DMA enable transmitter
#define USART_CR3_RTSE			8	//RTS enable
#define USART_CR3_CTSE			9	//CTS enable
#define USART_CR3_CTSIE			10	//CTS interrupt enable
#define USART_CR3_ONEBIT		11	//One sample bit method enable

typedef struct {
	uint8_t Mode;
	uint32_t BaudRate;
	uint8_t StopBit;
	uint8_t WordLenth;
	uint8_t Parity;
	uint8_t HWFlow;
}USART_Config_t;

typedef struct {
	USART_Register_t *pUSARTx;
	USART_Config_t Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;

//USART mode
#define USART_MODE_TX			0
#define USART_MODE_RX			1
#define USART_MODE_BOTH			2

//USATR baud rate
#define USART_BAUDRATE_1200		1200
#define USART_BAUDRATE_2400		2400
#define USART_BAUDRATE_9600		9800
#define USART_BAUDRATE_19200	19200
#define USART_BAUDRATE_38400	38400
#define USART_BAUDRATE_57600	57600
#define USART_BAUDRATE_115200	115200
#define USART_BAUDRATE_230400	230400
#define USART_BAUDRATE_460800	460800
#define USART_BAUDRATE_921600	921600
#define USART_BAUDRATE_2000000	2000000
#define USART_BAUDRATE_3000000	3000000

//USART Parity Control
#define USART_PARITY_EVEN		2
#define USART_PARITY_ODD		1
#define USART_PARITY_DIS		0

//USART Word Length
#define USART_WORDLENTH_8BIT	0
#define USART_WORDLENTH_9BIT	1

//USART No StopBit
#define USART_STOPBIT_1			0
#define USART_STOPBIT_0_5		1
#define USART_STOPBIT_2			2
#define USART_STOPBIT_1_5		3

//USART HWFLowControl
#define USART_HWFLOW_NONE		0
#define USART_HWFLOW_CTS		1
#define USART_HWFLOW_RTS		2
#define USART_HWFLOW_CTS_RTS	3

//USART Flag
#define USART_FLAG_TXE			(1 << USART_SR_TXE)
#define USART_FLAG_RNXE			(1 << USART_SR_RXNE)
#define USART_FLAG_TC			(1 << USART_SR_TC)

//USART Aplication satate
#define USART_BUSY_IN_TX		0
#define USART_BUSY_IN_RX		1
#define USART_BUSY_READY		3

#define USART_EVEN_TX_CMPLT		0
#define USART_EVEN_RX_CMPLT		1
#define USART_EVEN_IDLE			2
#define USART_EVEN_CTS			3
#define USART_EVEN_PE			4
#define USART_EVEN_LBD			5
#define USART_ERR_FE			6
#define USART_ERR_NF			7
#define USART_ERR_ORE			8


void USART_PriClockControl(USART_Register_t *pUSART,uint8_t EnorDi);

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);

void USART_SentData(USART_Handle_t *pUSARTHandle, uint8_t *TxBuffer, uint32_t Len);
void USART_ReciveData(USART_Handle_t *pUSARTHandle, uint8_t *RxBuffer, uint32_t Len);

uint8_t USART_SentDataIT(USART_Handle_t *pUSARTHandle, uint8_t *TxBuffer, uint32_t Len);
uint8_t USART_ReciveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *RxBuffer, uint32_t Len);

void USART_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

uint8_t USART_GetFlagStatus(USART_Register_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_Register_t *pUSARTx, uint8_t StatusFlagName);
void USART_PripheralControl(USART_Register_t *pUSARTx, uint8_t EnorDi);
void USART_SetBaudRate(USART_Register_t *pUSARTx, uint32_t BaudRate);

void USART_ApplicationEventCallBack(USART_Handle_t *pUSART, uint8_t AppEv);

#endif /* INC_STM32_USART_H_ */

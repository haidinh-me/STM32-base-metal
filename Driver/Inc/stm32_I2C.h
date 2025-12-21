/*
 * stm32_I2C.h
 *
 *  Created on: Sep 25, 2025
 *      Author: DINH
 */

#ifndef INC_STM32_I2C_H_
#define INC_STM32_I2C_H_

#include "stm32f4xx.h"
#include "stddef.h"


//I2C CR1
#define I2C_CR1_PE			0		//Bit 0 PE: Peripheral enable
#define I2C_CR1_SMBUS		1		//Bit 1 SMBUS: SMBus mode
#define I2C_CR1_SMBTYPE		3		//Bit 3 SMBTYPE: SMBus type
#define I2C_CR1_ENARP		4		//Bit 4 ENARP: ARP enable
#define I2C_CR1_ENPEC		5		//Bit 5 ENPEC: PEC enable
#define I2C_CR1_ENGC		6		//Bit 6 ENGC: General call enable
#define I2C_CR1_NOSTRETCH	7		//Bit 7 NOSTRETCH: Clock stretching disable (Target mode)
#define I2C_CR1_START		8		//Bit 8 START: Start generation
#define I2C_CR1_STOP		9		//Bit 9 STOP: Stop generation
#define I2C_CR1_ACK			10		//Bit 10 ACK: Acknowledge enable
#define I2C_CR1_POS			11		//Bit 11 POS: Acknowledge/PEC Position (for data reception)
#define I2C_CR1_PEC			12		//Bit 12 PEC: Packet error checking
#define I2C_CR1_ALERT		13		//Bit 13 ALERT: SMBus alert
#define I2C_CR1_SWRTS		15		//Bit 15 SWRST: Software reset

//I2C CR2
#define I2C_CR2_FREQ		0		//Bits 5:0 FREQ[5:0]: Peripheral clock frequency
#define I2C_CR2_ITERREN		8		//Bit 8 ITERREN: Error interrupt enable
#define I2C_CR2_ITEVTEN		9		//Bit 9 ITEVTEN: Event interrupt enable
#define I2C_CR2_ITBUFEN		10		//Bit 10 ITBUFEN: Buffer interrupt enable
#define I2C_CR2_DMAEN		11		//Bit 11 DMAEN: DMA requests enable
#define I2C_CR2_LAST		12		//Bit 12 LAST: DMA last transfer

//I2C OAR1
#define I2C_OAR1_ADD0		0		//Bit 0 ADD0: Interface address
#define I2C_OAR1_ADD		1		//Bits 7:1 ADD[7:1]: Interface address Bits 9:8 ADD[9:8]: Interface address
#define I2C_OAR1_ADDMODE	15		//Bit 15 ADDMODE Addressing mode (target mode)

//I2C OAR2
#define I2C_OAR2_ENDUAL		0		//Bit 0 ENDUAL: Dual addressing mode enable
#define I2C_OAR2_ADD2		1		//Bits 7:1 ADD2[7:1]: Interface address

//I2C SR1
#define I2C_SR1_SB			0		//Bit 0 SB: Start bit (Controller mode)
#define I2C_SR1_ADDR		1		//Bit 1 ADDR: Address sent (controller mode)/matched (target mode)
#define I2C_SR1_BTF			2		//Bit 2 BTF: Byte transfer finished
#define I2C_SR1_ADD10		3		//Bit 3 ADD10: 10-bit header sent (Controller mode)
#define I2C_SR1_STOPF		4		//Bit 4 STOPF: Stop detection (target mode)
#define I2C_SR1_RXNE		6		//Bit 6 RxNE: Data register not empty (receivers)
#define I2C_SR1_TXE			7		//Bit 7 TxE: Data register empty (transmitters)
#define I2C_SR1_BERR		8		//Bit 8 BERR: Bus error
#define I2C_SR1_ARLO		9		//Bit 9 ARLO: Arbitration lost (controller mode)
#define I2C_SR1_AF			10		//Bit 10 AF: Acknowledge failure
#define I2C_SR1_OVR			11		//Bit 11 OVR: Overrun/Underrun
#define I2C_SR1_PECERR		12		//Bit 12 PECERR: PEC Error in reception
#define I2C_SR1_TIMEOUT		14		//Bit 14 TIMEOUT: Timeout or Tlow error
#define I2C_SR1_SMBALERT	15		//Bit 15 SMBALERT: SMBus alert

//I2C SR2
#define I2C_SR2_MSL			0		//Bit 0 MSL: Controller/target
#define I2C_SR2_BUSY		1		//Bit 1 BUSY: Bus busy
#define I2C_SR2_TRA			2		//Bit 2 TRA: Transmitter/receiver
#define I2C_SR2_GENCALL		4		//Bit 4 GENCALL: General call address (Target mode)
#define I2C_SR2_SMBDEFAULT	5		//Bit 5 SMBDEFAULT: SMBus device default address (Target mode)
#define I2C_SR2_SMBHOST		6		//Bit 6 SMBHOST: SMBus host header (Target mode)
#define I2C_SR2_DUALF		7		//Bit 7 DUALF: Dual flag (Target mode)
#define I2C_SR2_PEC			8		//Bits 15:8 PEC[7:0] Packet error checking register

//I2C CCR
#define I2C_CCR_CCR			0		//Bits 11:0 CCR[11:0]: Clock control register in Fm/Sm mode (Controller mode)
#define I2C_CCR_DUTY		14		//Bit 14 DUTY: Fm mode duty cycle
#define I2C_CCR_FS			15		//Bit 15 F/S: I2C controller mode selection

//I2C FLTR
#define I2C_FLTR_DNF		0		//Bits 3:0 DNF[3:0]: Digital noise filter
#define I2C_FLTR_ANOFF		4		//Bit 4 ANOFF: Analog noise filter OFF

typedef struct{
	uint32_t SCLSpeed;
	uint8_t DeviceAddress;
	uint8_t AckControl;
	uint8_t FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_Register_t *pI2Cx;
	I2C_Config_t Config;

	uint32_t TxLen;
	uint32_t RxLen;
	uint32_t RxSize;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint8_t Sr;
}I2C_Handle_t;

#define I2C_BUSY_IN_TX 		0
#define I2C_BUSY_IN_RX		1
#define I2C_READY			2

#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM	400000

#define I2C_ACK_CONTROL_EN	1
#define I2C_ACK_CONTROL_DI	0

#define I2C_FM_DUTY_CYCLE_2	0
#define I2C_FM_DUTY_CYCLE_169	1

//Flags
#define I2C_FLAG_SB 		(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR 		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF 		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR10 	(1 << I2C_SR1_ADDR10)
#define I2C_FLAG_STOPF 		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE 		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE 		(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR 		(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO 		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF 		(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR 		(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR 	(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT 	(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT	(1 << I2C_SR1_SMBALERT)

#define I2C_DISABLE_SR		DISABLE
#define I2C_ENABLE_SR		ENABLE

//Application event
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_EV_DATA_REQ		3
#define I2C_EV_DATA_RCV		4
#define I2C_ERROR_BERR		5
#define I2C_ERROR_ARLO		6
#define I2C_ERROR_AF		7
#define I2C_ERROR_OVR		8
#define I2C_ERROR_PECERR	9
#define I2C_ERROR_TIMEOUT	10
#define	I2C_ERROR_SMBALERT	11

void I2C_PeriClockControl(I2C_Register_t * pI2Cx, uint8_t EnorDi);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_Register_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_Register_t *pI2Cx);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_PeripheralControl(I2C_Register_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_Register_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_Register_t *pI2Cx, uint8_t EnorDi);
void I2C_GenarateStopCondition(I2C_Register_t *pI2Cx);

void I2C_SlaveEnDiCallBack(I2C_Register_t *pI2Cx, uint8_t EnorDi);

void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32_I2C_H_ */

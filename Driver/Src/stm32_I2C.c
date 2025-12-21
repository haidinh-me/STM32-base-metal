/*
 * stm32_I2C.c
 *
 *  Created on: Sep 25, 2025
 *      Author: DINH
 */

#include "stm32_I2C.h"
#include "stm32_RCC.h"

static void I2C_GenarationStartCondition(I2C_Register_t *pI2Cx);
static void I2C_ExcuteAddressPhaseWrite(I2C_Register_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenarationStartCondition(I2C_Register_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}
static void I2C_ExcuteAddressPhaseWrite(I2C_Register_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read;
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle->RxSize == 1){
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else {
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else{
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;

	}
}
static void I2C_ExcuteAddressPhaseRead(I2C_Register_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TxLen > 0){
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->pTxBuffer ++;
		pI2CHandle->TxLen --;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->RxSize == 1){
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen --;
	}

	if(pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxLen == 2){
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		*(pI2CHandle->pTxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pTxBuffer ++;
		pI2CHandle->TxLen --;
	}

	if(pI2CHandle->RxLen == 0){
		if(pI2CHandle->Sr == I2C_DISABLE_SR){
			I2C_GenarateStopCondition(pI2CHandle->pI2Cx);
		}

		I2C_CloseReceiveData(pI2CHandle);

		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_PeripheralControl(I2C_Register_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi)	pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	else 		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
}

void I2C_PeriClockControl(I2C_Register_t * pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1)		I2C1_PCLK_EN();
		else if(pI2Cx == I2C2)	I2C2_PCLK_EN();
		else if(pI2Cx == I2C3) 	I2C3_PCLK_EN();
	}
	else{
		if(pI2Cx == I2C1)		I2C1_PCLK_DIS();
		else if(pI2Cx == I2C2)	I2C2_PCLK_DIS();
		else if(pI2Cx == I2C3) 	I2C3_PCLK_DIS();
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t temp = 0;

	//Enable peripheral for I2C
	I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);

	//Enable clock for I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// bit ACK
	temp = pI2CHandle->Config.AckControl |= (1 << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = temp;

	//FREQ CR2
	temp = 0;
	temp = RCC_GetPCLK1Value() /1000000U;
	pI2CHandle->pI2Cx->CR2 = (temp & 0x3F);

	//OAR1
	temp = 0;
	temp = pI2CHandle->Config.DeviceAddress;
	if(temp <= 127){ // 7bit address
		temp |= temp << I2C_OAR1_ADD ;
	}
	else {// 10bit address
		temp |= temp << I2C_OAR1_ADD0;
	}
	temp |= 1 << 14;	//Bit 14 Should always be kept at 1 by software(following rm)
	pI2CHandle->pI2Cx->OAR1 = temp;

	//CCR
	temp = 0;
	if(pI2CHandle->Config.SCLSpeed <= I2C_SCL_SPEED_SM){	//Standard Mode
		temp |= (RCC_GetPCLK1Value() / (2*pI2CHandle->Config.SCLSpeed) & 0x7FF);
	}
	else {								//Fast Mode
		temp |=  (1 << I2C_CCR_FS);
		if(pI2CHandle->Config.FMDutyCycle == I2C_FM_DUTY_CYCLE_2){
			temp |= (RCC_GetPCLK1Value() / (3*pI2CHandle->Config.SCLSpeed) & 0x7FF);
		}
		else{
			temp |= (1 << I2C_CCR_DUTY);
			temp |= (RCC_GetPCLK1Value() / (25*pI2CHandle->Config.SCLSpeed) & 0x7FF);
		}
	}
	pI2CHandle->pI2Cx->CCR = temp;

	//TRISE
	temp = 0;
	if(pI2CHandle->Config.SCLSpeed <= I2C_SCL_SPEED_SM){
		temp = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else{
		temp = (RCC_GetPCLK1Value() * 3 / 1000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = temp & 0x3F;
}

void I2C_DeInit(I2C_Handle_t *pI2CHandle){

}

uint8_t I2C_GetFlagStatus(I2C_Register_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return SET;
	}
	return RESET;
}

void I2C_GenarateStopCondition(I2C_Register_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	I2C_GenarationStartCondition(pI2CHandle->pI2Cx);

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	I2C_ExcuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	I2C_ClearADDRFlag(pI2CHandle);

	while(Len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer ++;
		Len --;
	}

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	if(Sr == I2C_DISABLE_SR)
		I2C_GenarateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_ManageAcking(I2C_Register_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi){
		//enable the ack
		pI2Cx->CR1 |= 1 << I2C_CR1_ACK;
	}
	else{
		//disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	I2C_GenarationStartCondition(pI2CHandle->pI2Cx);

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	I2C_ExcuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//read only 1 byte from slave
	if(Len == 1){
		//Disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait untill RXNE is set
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//genarate stop condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenarateStopCondition(pI2CHandle->pI2Cx);

		//read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//read alot of byte
	if(Len > 1){
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read data untill Len becomes zero
		for(uint32_t i=0; i > Len; i--){
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2){
				//Disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//genarate stop condition
				if(Sr == I2C_DISABLE_SR)
					I2C_GenarateStopCondition(pI2CHandle->pI2Cx);
			}
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
		}
	}

	if(pI2CHandle->Config.AckControl == I2C_ACK_CONTROL_EN){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;

	*(NVIC_IPR_BASEADDR + ipr) |= (IRQPriority << (irq*8));
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
	uint8_t busystatus = pI2CHandle->TxRxState;

	if(busystatus != I2C_BUSY_IN_TX && (busystatus != I2C_BUSY_IN_RX)){
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		I2C_GenarationStartCondition(pI2CHandle->pI2Cx);

		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
	}
	return busystatus;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
	uint8_t busystatus = pI2CHandle->TxRxState;

	if(busystatus != I2C_BUSY_IN_TX && (busystatus != I2C_BUSY_IN_RX)){
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RxSize = Len;
		pI2CHandle->Sr = Sr;

		I2C_GenarationStartCondition(pI2CHandle->pI2Cx);

		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
	}
	return busystatus;
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	pI2CHandle->TxRxState = I2C_READY;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->TxRxState = I2C_READY;

	if(pI2CHandle->Config.AckControl == I2C_ACK_CONTROL_EN){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1,temp2,temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//SB flag
	if(temp1 && temp3){
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			I2C_ExcuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			I2C_ExcuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
	}

	//ADDR flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3){
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//ADDR10 flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADD10);

	//STOPF flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3){
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);
	}

	//BTF flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3){
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
				if(pI2CHandle->TxLen == 0){
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenarateStopCondition(pI2CHandle->pI2Cx);

					I2C_CloseSendData(pI2CHandle);

					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

		}
	}

	//RXNE flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3){
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
		}
		else{
			if(! (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) )
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}

	//TXE flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3){
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
		else{
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp3,temp2;

	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);

	//BERR flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);
	if(temp2 && temp3){
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_BERR);
	}

	//ARLO flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);
	if(temp2 && temp3){
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_ARLO);
	}

	//AF flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);
	if(temp2 && temp3){
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_AF);
	}

	//OVR flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);
	if(temp2 && temp3){
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_OVR);
	}

	//PECERR flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_PECERR);
	if(temp2 && temp3){
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_PECERR);
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_PECERR);
	}

	//TIMEOUT flag
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);
	if(temp2 && temp3){
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

	//SMBALERT
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SMBALERT);
	if(temp2 && temp3){
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_SMBALERT);
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_SMBALERT);
	}
}

void I2C_SlaveSendData(I2C_Register_t *pI2Cx, uint8_t data){
	pI2Cx->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_Register_t *pI2Cx){
	return (uint8_t) pI2Cx->DR;
}

void I2C_SlaveEnDiCallBack(I2C_Register_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi){
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
	}
	else{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	}
}

__attribute__((weak)) void I2C_ApplicationEventCallBack(I2C_Handle_t *pSPIHandle,uint8_t AppEv){
}

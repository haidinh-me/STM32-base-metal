/*
 * stm32_USART.c
 *
 *  Created on: Aug 17, 2025
 *      Author: DINH
 */

#include "stm32_USART.h"
#include "math.h"
#include "stm32_RCC.h"

void USART_PriClockControl(USART_Register_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}
}

void USART_Init(USART_Handle_t *pUSARTHandle){
	uint32_t tempreg = 0;

	USART_PriClockControl(pUSARTHandle->pUSARTx,ENABLE);
	USART_PripheralControl(pUSARTHandle->pUSARTx,ENABLE);

	//CR1
	//mode
	if(pUSARTHandle->Config.Mode == USART_MODE_TX) tempreg |= (1 << USART_CR1_TE);
	else if(pUSARTHandle->Config.Mode == USART_MODE_RX) tempreg |= (1 << USART_CR1_RE);
	else if(pUSARTHandle->Config.Mode == USART_MODE_BOTH) tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));

	//worth length
	tempreg |= pUSARTHandle->Config.WordLenth << USART_CR1_M;

	//parity
	tempreg &= ~((1 << USART_CR1_PCE) | (1 << USART_CR1_PS));
	if(pUSARTHandle->Config.Parity == USART_PARITY_ODD)	{
		tempreg |= 1 << USART_CR1_PCE;
		tempreg |= 1 << USART_CR1_PS;
	}
	else if(pUSARTHandle->Config.Parity == USART_PARITY_EVEN){
		tempreg |= 1 << USART_CR1_PCE;
	}
	else if(pUSARTHandle->Config.Parity == USART_PARITY_DIS) {}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	//CR2
	tempreg = 0;
	//StopBit
	tempreg |= pUSARTHandle->Config.StopBit << USART_CR2_STOP;
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	//CR3
	tempreg = 0;
	//HWF
	if(pUSARTHandle->Config.HWFlow == USART_HWFLOW_CTS) tempreg |= (1 << USART_CR3_CTSE);
	else if(pUSARTHandle->Config.HWFlow == USART_HWFLOW_RTS) tempreg |= (1 << USART_CR3_RTSE);
	else if(pUSARTHandle->Config.HWFlow == USART_HWFLOW_CTS_RTS) tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->Config.BaudRate);
}

void USART_SetBaudRate(USART_Register_t *pUSARTx, uint32_t BaudRate){
	uint32_t Fclk = 0;
	float usartdiv;
	uint32_t matissa, fraction;

	if(pUSARTx == USART1 || pUSARTx == USART6){
		Fclk = RCC_GetPCLK2Value();
	}
	else Fclk = RCC_GetPCLK1Value();

	if(pUSARTx->CR1  &  (1 << USART_CR1_OVER8)){
		usartdiv = (float)Fclk/(8.0f * (float)BaudRate);
		matissa = (uint32_t)usartdiv;
		fraction = (uint32_t)roundf((usartdiv - (float)matissa) * 8.0f);
		if(fraction == 8){
			matissa +=1;
			fraction = 0;
		}

		pUSARTx->BRR = (matissa << 3) | (fraction & 0x07);
	}
	else {
		usartdiv = (float)Fclk/(16.0f * (float)BaudRate);
		matissa = (uint32_t)usartdiv;
		fraction = (uint32_t)roundf((usartdiv - (float)matissa) * 16.0f);
		if(fraction == 16){
			matissa +=1;
			fraction = 0;
		}

		pUSARTx->BRR = (matissa << 4) | (fraction & 0x0F);
	}
}

void USART_PripheralControl(USART_Register_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi) pUSARTx->CR1 |= (1 << 13);
	else pUSARTx->CR1 &= ~(1 << 13);
}

uint8_t USART_GetFlagStatus(USART_Register_t *pUSARTx, uint8_t StatusFlagName){
	uint8_t status = DISABLE;
	if(pUSARTx->SR &  StatusFlagName)
		return ENABLE;
	return status;
}

void USART_SentData(USART_Handle_t *pUSARTHandle, uint8_t *TxBuffer, uint32_t Len){
	uint16_t *temp;

	for(int i=0; i<Len; i++){
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		if(pUSARTHandle->Config.WordLenth == USART_WORDLENTH_9BIT){ //9bit data
			temp = (uint16_t*)TxBuffer;
			pUSARTHandle->pUSARTx->DR = (*temp & (uint16_t)0x1FF);

			if(pUSARTHandle->Config.Parity == USART_PARITY_DIS)
				TxBuffer +=2;
			else TxBuffer ++;
		}

		else {//8bit data
			pUSARTHandle->pUSARTx->DR = (*TxBuffer & (uint8_t)0xFF);
			TxBuffer ++;
		}
	}

	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

void USART_ReciveData(USART_Handle_t *pUSARTHandle, uint8_t *RxBuffer, uint32_t Len){
	for(int i=0; i< Len; i++){
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RNXE));

		if(pUSARTHandle->Config.WordLenth == USART_WORDLENTH_9BIT){//9bit data
			if(pUSARTHandle->Config.Parity == USART_PARITY_DIS){
				*((uint16_t*)RxBuffer) = pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF;
				RxBuffer +=2;
			}
			else{
				*RxBuffer = pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF;
				RxBuffer ++;
			}
		}

		else {//8bit data
			if(pUSARTHandle->Config.Parity == USART_PARITY_DIS )
				*RxBuffer = pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF;

			else
				*RxBuffer = pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F;

			RxBuffer ++;
		}
	}
}

void USART_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;

	*(NVIC_IPR_BASEADDR + ipr) |= (IRQPriority << (irq*8));
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle){
	uint32_t temp1,temp2;
	uint16_t *pdata;

	//TCIE flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if( temp1 && temp2){
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){
			if(! pUSARTHandle->TxLen){
				pUSARTHandle->TxBusyState = USART_BUSY_READY;
				pUSARTHandle->TxLen = 0;
				pUSARTHandle->pTxBuffer = NULL;

				//clear bit by a software sequence
				volatile uint32_t temp;
				temp = pUSARTHandle->pUSARTx->SR;
				temp = pUSARTHandle->pUSARTx->DR;
				(void)temp;

				// call application

				USART_ApplicationEventCallBack(pUSARTHandle,USART_EVEN_TX_CMPLT);
			}
		}
	}

	//TXEIE flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2){
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){
			if(pUSARTHandle->TxLen > 0){

				//check the wordlength
				if(pUSARTHandle->Config.WordLenth == USART_WORDLENTH_9BIT){//Is 9bit Wordlenght
					pdata = (uint16_t *)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = *pdata & (uint16_t)0x1FF;

					//check the parity bit
					if(pUSARTHandle->Config.Parity == USART_PARITY_DIS){
						pUSARTHandle->pTxBuffer +=2;
						pUSARTHandle->TxLen -=2;
					}

					else{
						pUSARTHandle->pTxBuffer ++;
						pUSARTHandle->TxLen --;
					}

				}

				else{//Is 8bit Wordlenght
					pUSARTHandle->pUSARTx->DR = *pUSARTHandle->pTxBuffer & (uint8_t)0xFF;
					pUSARTHandle->pTxBuffer ++;
					pUSARTHandle->TxLen --;

				}

			}

			if(pUSARTHandle->TxLen == 0)
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
		}
	}

	//RNEIE flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX){
			if(pUSARTHandle->RxLen > 0){

				//check wordlength
				if(pUSARTHandle->Config.WordLenth == USART_WORDLENTH_9BIT){//9bit wordlenth

					//check parity
					if(pUSARTHandle->Config.Parity == USART_PARITY_DIS){//parity disable
						*((uint16_t *)pUSARTHandle->pRxBuffer) = pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF;
						pUSARTHandle->pRxBuffer +=2;
						pUSARTHandle->RxLen -=2;
					}

					else {//parity enable
						*pUSARTHandle->pRxBuffer = pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF;
						pUSARTHandle->pRxBuffer ++;
						pUSARTHandle->RxLen --;
					}

				}
				else{//8bit wordlenth

					//check parity
					if(pUSARTHandle->Config.Parity == USART_PARITY_DIS){//parity disable
						*pUSARTHandle->pRxBuffer = pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF;
					}

					else{//parity enable
						*pUSARTHandle->pRxBuffer = pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F;
					}

					pUSARTHandle->pRxBuffer ++;
					pUSARTHandle->RxLen --;
				}
			}

			if(! pUSARTHandle->RxLen){
				pUSARTHandle->RxBusyState = USART_BUSY_READY;

				//clear bit by a software sequence
				volatile uint32_t temp;
				temp = pUSARTHandle->pUSARTx->SR;
				temp = pUSARTHandle->pUSARTx->DR;
				(void)temp;

				// call application
				USART_ApplicationEventCallBack(pUSARTHandle,USART_EVEN_RX_CMPLT);
			}
		}
	}

	// CTS flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR3_CTSE);
	if(temp1 && temp2){
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);
		USART_ApplicationEventCallBack(pUSARTHandle,USART_EVEN_CTS);
	}

	// Overrun Error Detected (ORE)
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
	if(temp1 && temp2){

		//clear bit by a software sequence
		volatile uint32_t temp;
		temp = pUSARTHandle->pUSARTx->SR;
		temp = pUSARTHandle->pUSARTx->DR;
		(void)temp;

		// call application
		USART_ApplicationEventCallBack(pUSARTHandle,USART_ERR_ORE);
	}

	// Idle Line Detected (IDLE)
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);
	if(temp1 && temp2){

		//clear bit by a software sequence
		volatile uint32_t temp;
		temp = pUSARTHandle->pUSARTx->SR;
		temp = pUSARTHandle->pUSARTx->DR;
		(void)temp;

		// call application
		USART_ApplicationEventCallBack(pUSARTHandle,USART_EVEN_IDLE);
	}

	// Parity Error (PE)
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_PE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_PEIE);
	if(temp1 && temp2){

		//clear bit by a software sequence
		volatile uint32_t temp;
		temp = pUSARTHandle->pUSARTx->SR;
		temp = pUSARTHandle->pUSARTx->DR;
		(void)temp;

		// call application
		USART_ApplicationEventCallBack(pUSARTHandle,USART_EVEN_PE);
	}

	// Break Flag (LBD)
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_LBD);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR2_LBDIE);
	if(temp1 && temp2){
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_LBD);
		USART_ApplicationEventCallBack(pUSARTHandle,USART_EVEN_LBD);
	}

	// Noise Flag, Overrun error and Framing Error in multibuffer communication (NF or ORE or FE)
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR3_EIE);
	if(temp2){
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & (1 << USART_SR_NF) ){

			//clear bit by a software sequence
			volatile uint32_t temp;
			temp = pUSARTHandle->pUSARTx->SR;
			temp = pUSARTHandle->pUSARTx->DR;
			(void)temp;

			// call application
			USART_ApplicationEventCallBack(pUSARTHandle,USART_ERR_NF);
		}
		if(temp1 & (1 << USART_SR_ORE) ){

			//clear bit by a software sequence
			volatile uint32_t temp;
			temp = pUSARTHandle->pUSARTx->SR;
			temp = pUSARTHandle->pUSARTx->DR;
			(void)temp;

			// call application
			USART_ApplicationEventCallBack(pUSARTHandle,USART_ERR_ORE);
		}
		if(temp1 & (1 << USART_SR_FE) ){

			//clear bit by a software sequence
			volatile uint32_t temp;
			temp = pUSARTHandle->pUSARTx->SR;
			temp = pUSARTHandle->pUSARTx->DR;
			(void)temp;

			// call application
			USART_ApplicationEventCallBack(pUSARTHandle,USART_ERR_FE);
		}
	}
}

uint8_t USART_SentDataIT(USART_Handle_t *pUSARTHandle, uint8_t *TxBuffer, uint32_t Len){
	uint8_t txstate = pUSARTHandle->TxBusyState;
	if(txstate != USART_BUSY_IN_TX){
		pUSARTHandle->TxLen =Len;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;
		pUSARTHandle->pTxBuffer = TxBuffer;

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}
	return txstate;
}

uint8_t USART_ReciveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *RxBuffer, uint32_t Len){
	uint8_t rxstate = pUSARTHandle->RxBusyState;
	if(rxstate != USART_BUSY_IN_RX){
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		pUSARTHandle->pRxBuffer = RxBuffer;

		(void)pUSARTHandle->pUSARTx->DR;

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}

__attribute__((weak)) void USART_ApplicationEventCallBack(USART_Handle_t *pSPIHandle,uint8_t event){
}

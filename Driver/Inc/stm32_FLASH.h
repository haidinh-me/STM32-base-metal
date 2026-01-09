/*
 * stm32_FLASH.h
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */

#ifndef INC_STM32_FLASH_H_
#define INC_STM32_FLASH_H_

#include <stm32f4xx.h>

#define FLASH_CR_PG		0
#define FLASH_CR_SER	1
#define FLASH_CR_MER	2
#define FLASH_CR_SNB	3
#define FLASH_CR_PSIZE	8
#define FLASH_CR_STRT	16
#define FLASH_CR_EOPIE	24
#define FLASH_CR_ERRIE	25
#define FLASH_CR_LOCK	31

#define FLASH_SR_EOP	0
#define FLASH_SR_OPERR	1
#define FLASH_SR_WRPERR	4
#define FLASH_SR_PGAERR	5
#define FLASH_SR_PGPERR	6
#define FLASH_SR_PGSERR	7
#define FLASH_SR_RDERR	8
#define FLASH_SR_BSY	16

#define FLASH_ACR_LATENCY	0
#define FLASH_ACR_PRFTEN	8
#define FLASH_ICEN			9
#define FLASH_DCEN			10
#define FLASH_ICSRT			11
#define FLASH_DCSRT			12

void Flash_Unlock(void);
void Flash_Lock(void);
void Flash_EraseSector(uint32_t Sector);
void Flash_EraseMass(void);
void ProgramWord(uint32_t Address, uint32_t Data);

#endif /* INC_STM32_FLASH_H_ */

/*
 * stm32_FLASH.c
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */

#ifndef SRC_STM32_FLASH_C_
#define SRC_STM32_FLASH_C_

#include <stm32_FLASH.h>

void Flash_Unlock(void)
{
	if(FLASH->CR & (1 << FLASH_CR_LOCK))
	{
		FLASH->OPTKEYR = 0x08192A3B;
		FLASH->OPTKEYR = 0x4C5D6E7F;
	}
}

void Flash_Lock(void)
{
	FLASH->CR |= (1 << FLASH_CR_LOCK);
}

void Flash_EraseSector(uint32_t Sector)
{
	Flash_Unlock();

	while(FLASH->SR & (1 << FLASH_SR_BSY));

	FLASH->CR |= (1 << FLASH_CR_SER);

	FLASH->CR |= (Sector << FLASH_CR_SNB);

	FLASH->CR |= (1 << FLASH_CR_STRT);

	while(FLASH->SR & (1 << FLASH_SR_BSY));

	FLASH->CR &= ~(1 << FLASH_CR_SER);

	Flash_Lock();
}

void Flash_EraseMass(void)
{
	Flash_Unlock();

	while(FLASH->SR & (1 <<FLASH_SR_BSY));

	FLASH->CR |= (1 << FLASH_CR_MER);

	FLASH->CR |= (1 << FLASH_CR_STRT);

	while(FLASH->SR & (1 << FLASH_SR_BSY));

	FLASH->CR &= ~(1 << FLASH_CR_MER);

	Flash_Lock();
}

void ProgramWord(uint32_t Address, uint32_t Data)
{
	Flash_Unlock();

	while(FLASH->SR & (1 << FLASH_SR_BSY));

	FLASH->CR |= (1 << FLASH_CR_PG);

	FLASH->CR |= (2 << FLASH_CR_PSIZE); // Word is 32 bit (2byte)

	*(uint32_t *) Address = Data;		// Wrting data into address

	while(FLASH->SR &(1 << FLASH_SR_BSY));

	Flash_Lock();
}

#endif /* SRC_STM32_FLASH_C_ */

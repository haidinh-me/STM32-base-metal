/*
 * stm32f4.h
 *
 *  Created on: Aug 6, 2025
 *      Author: DINH
 */

#ifndef STM32F4_H
#define STM32F4_H

#include <stdint.h>

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

// SYSCFG_R
#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x20001C00U		//SRAM2=SRAM2_BASEADDR + size(SRAM2_BASEADDR);
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR
#define MPU_BASEADDR			0xE000ED90U
#define SCB_BASEADDR			0xE000ED00U

// NVIC
#define NVIC_ISER_BASEADDR      (volatile uint32_t *)0xE000E100U
#define NVIC_ISER(x)			(volatile uint32_t *)(NVIC_ISER_BASEADDR + 4*x)

#define NVIC_ICER_BASEADDR      (volatile uint32_t *)0xE000E180U
#define NVIC_ICER(x)			(volatile uint32_t *)(NVIC_ICER_BASEADDR + 4*x)

#define NVIC_ISPR_BASEADDR      (volatile uint32_t *)0xE000E200U
#define NVIC_IABR_BASEADDR      (volatile uint32_t *)0xE000E300U
#define NVIC_IPR_BASEADDR       (volatile uint32_t *)0xE000E400U


// AHB Peripheral, APB Peripheral configure
#define APB1PERIPHERAL_BASEADDR 0x40000000U
#define APB2PERIPHERAL_BASEADDR 0x40010000U
#define AHB1PERIPHERAL_BASEADDR 0x40020000U
#define AHB2PERIPHERAL_BASEADDR 0x50000000U

// AHB1 PRIPHERAL
#define GPIOA_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x1000U)
#define GPIOH_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x1C00U)
#define RCC_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x3800U)
#define DMA1_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x6000U)
#define DMA2_BASEADDR			(AHB1PERIPHERAL_BASEADDR + 0x6400U)

// APB1 PERIPHERAL
#define I2C1_BASEADDR			(APB1PERIPHERAL_BASEADDR + 0x5400U)
#define I2C2_BASEADDR			(APB1PERIPHERAL_BASEADDR + 0x5800U)
#define I2C3_BASEADDR			(APB1PERIPHERAL_BASEADDR + 0x5C00U)
#define SPI2_BASEADDR			(APB1PERIPHERAL_BASEADDR + 0x3800U)
#define SPI3_BASEADDR			(APB1PERIPHERAL_BASEADDR + 0x3C00U)
#define USART2_BASEADDR			(APB1PERIPHERAL_BASEADDR + 0x4400U)

// APB2 PERIPHERAL
#define USART1_BASEADDR			(APB2PERIPHERAL_BASEADDR + 0x1000U)
#define USART6_BASEADDR			(APB2PERIPHERAL_BASEADDR + 0x1400U)
#define SPI1_BASEADDR			(APB2PERIPHERAL_BASEADDR + 0x3000U)
#define SPI4_BASEADDR			(APB2PERIPHERAL_BASEADDR + 0x3400U)
#define EXTI_BASEADDR			(APB2PERIPHERAL_BASEADDR + 0x3C00U)
#define SYSCFG_BASEADDR			(APB2PERIPHERAL_BASEADDR + 0x3800U)

// Configuration DMA Structure
typedef struct{
	volatile uint32_t LISR;					//low interrupt status register
	volatile uint32_t HISR;					//high interrupt status register
	volatile uint32_t LIFCR;				//low interrupt flag clear register
	volatile uint32_t HIFCR;				//high interrupt flag clear register
	struct{
		volatile uint32_t SCR;				//stream xconfiguration register
		volatile uint32_t SNDTR;			//stream x number of data register
		volatile uint32_t SPAR;				//stream x peripheral address register
		volatile uint32_t SMxAR[2];			//stream x memory x address register
		volatile uint32_t SFCR;				//stream x FIFO control register
	}STREAM[8];								//stream x
}DMA_Register_t;
#define DMA1	((DMA_Register_t *)DMA1_BASEADDR)
#define DMA2	((DMA_Register_t *)DMA2_BASEADDR)

//Configuration SBS Structure

typedef struct{
	__IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
	__IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
	__IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
	__IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
	__IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
	__IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
	__IOM uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
	__IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
	__IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
	__IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
	__IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
	__IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
	__IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
	__IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
	__IM  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
	__IM  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
	__IM  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
	__IM  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
	__IM  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
	uint32_t RESERVED0[5U];
	__IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
}SCB_Register_t;
#define SCB	((SCB_Register_t*) SCB_BASEADDR)

//Configuration FLASH Structure
typedef struct{
	volatile uint32_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPTKEYR;
	volatile uint32_t SR;
	volatile uint32_t CR;
	volatile uint32_t OPTCR;
}FLASH_Register_t;
#define FLASH ((FLASH_Register_t *) FLASH_BASEADDR)

//Configuration MPU Structure
typedef struct{
	__IM  uint32_t TYPE;                   /*!< Offset: 0x000 (R/ )  MPU Type Register */
	__IOM uint32_t CTRL;                   /*!< Offset: 0x004 (R/W)  MPU Control Register */
	__IOM uint32_t RNR;                    /*!< Offset: 0x008 (R/W)  MPU Region RNRber Register */
	__IOM uint32_t RBAR;                   /*!< Offset: 0x00C (R/W)  MPU Region Base Address Register */
	__IOM uint32_t RASR;                   /*!< Offset: 0x010 (R/W)  MPU Region Attribute and Size Register */
	__IOM uint32_t RBAR_A1;                /*!< Offset: 0x014 (R/W)  MPU Alias 1 Region Base Address Register */
	__IOM uint32_t RASR_A1;                /*!< Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size Register */
	__IOM uint32_t RBAR_A2;                /*!< Offset: 0x01C (R/W)  MPU Alias 2 Region Base Address Register */
	__IOM uint32_t RASR_A2;                /*!< Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size Register */
	__IOM uint32_t RBAR_A3;                /*!< Offset: 0x024 (R/W)  MPU Alias 3 Region Base Address Register */
	__IOM uint32_t RASR_A3;                /*!< Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size Register */
}MPU_Register_t;
#define MPU	((MPU_Register_t* )MPU_BASEADDR)

// Configuration GPIO Structure
typedef struct{
	volatile uint32_t MODER;				//Adder offset: 0x00
	volatile uint32_t OTYPER;				//Adder offset: 0x04
	volatile uint32_t OSPEDDR;				//Adder offset: 0x08
	volatile uint32_t PUPDR;				//Adder offset: 0x0C
	volatile uint32_t IDR;					//Adder offset: 0x10
	volatile uint32_t ODR;					//Adder offset: 0x14
	volatile uint32_t BSRR;					//Adder offset: 0x18
	volatile uint32_t LCKR;					//Adder offset: 0x1C
	volatile uint32_t AFR[2];				//Adder offset: AFRL: 0x20	AFRH: 0x24
}GPIO_Register_t;
#define GPIOA ((GPIO_Register_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_Register_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_Register_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_Register_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_Register_t *)GPIOE_BASEADDR)
#define GPIOH ((GPIO_Register_t *)GPIOH_BASEADDR)

// Configuration RCC Structure
typedef struct{
	volatile uint32_t CR;					//Adder offset: 0x00
	volatile uint32_t PLLCFGR; 				//Adder offset: 0x04
	volatile uint32_t CFGR;					//Adder offset: 0x08
	volatile uint32_t AHB1RSTR;				//Adder offset: 0x0C
	volatile uint32_t AHB2RSTR;				//Adder offset: 0x14
	volatile uint32_t Reserved0[2];			//Adder offset: 0x18	0x1C
	volatile uint32_t APB1RSTR;				//Adder offset: 0x20
	volatile uint32_t APB2RSTR;				//Adder offset: 0x24
	volatile uint32_t Reserved1[2];			//Adder offset: 0x28	0x2C
	volatile uint32_t AHB1ENR;				//Adder offset: 0x30
	volatile uint32_t AHB2ENR;				//Adder offset: 0x34
	volatile uint32_t Reserved2[2];			//Adder offset: 0x38	0x3C
	volatile uint32_t APB1ENR;				//Adder offset: 0x40
	volatile uint32_t APB2ENR;				//Adder offset: 0x44
	volatile uint32_t Reserved3[2];			//Adder offset: 0x48	0x4C
	volatile uint32_t AHB1LPENR;			//Adder offset: 0x50
	volatile uint32_t AHB2LPENR;			//Adder offset: 0x54
	volatile uint32_t Reserved4[2];			//Adder offset: 0x58	0x5C
	volatile uint32_t APB1LPENR;			//Adder offset: 0x60
	volatile uint32_t APB2LPENR;			//Adder offset: 0x64
	volatile uint32_t Reserved5[2];			//Adder offset: 0x68	0x6C
	volatile uint32_t BDCR;					//Adder offset: 0x70
	volatile uint32_t CSR;					//Adder offset: 0x74
	volatile uint32_t Reserved6[2];			//Adder offset: 0x78	0x7C
	volatile uint32_t SSCGR;				//Adder offset: 0x80
	volatile uint32_t PLLI2SCFGR;			//Adder offset: 0x84
	volatile uint32_t Reserved7;			//Adder offset: 0x88
	volatile uint32_t DCKCFGR;				//Adder offset: 0x8C
} RCC_Register_t;
#define RCC ((RCC_Register_t *)RCC_BASEADDR)

// Configuration SYSCFG_R (System Configure Register) Structure
typedef struct{
	volatile uint32_t MEMRMP;				//Adder offset: 0x00
	volatile uint32_t PMC;					//Adder offset: 0x04
	volatile uint32_t EXTICR[4];			//Adder offset: 0x08	0x0C	0x10	0x14
	volatile uint32_t Reserved[2];			//Adder offset: 0x18	0x1C
	volatile uint32_t CMPCR;				//Adder offset: 0x20
}SYSCFG_Register_t;
#define SYSCFG ((SYSCFG_Register_t* )SYSCFG_BASEADDR)

// Configuration External Interrupt Register Structure (EXTIR)
typedef struct{
	volatile uint32_t IMR; 					//Adder offset: 0x00
	volatile uint32_t EMR; 					//Adder offset: 0x04
	volatile uint32_t RTSR; 				//Adder offset: 0x08
	volatile uint32_t FTSR; 				//Adder offset: 0x0C
	volatile uint32_t SWIER; 				//Adder offset: 0x10
	volatile uint32_t PR; 					//Adder offset: 0x14
}EXTI_Register_t;
#define EXTI ((EXTI_Register_t *)EXTI_BASEADDR)

// Configure I2C Structure
typedef struct{
	volatile uint32_t CR1; 					//Adder offset: 0x00
	volatile uint32_t CR2; 					//Adder offset: 0x04
	volatile uint32_t OAR1; 				//Adder offset: 0x08
	volatile uint32_t OAR2; 				//Adder offset: 0x0C
	volatile uint32_t DR; 					//Adder offset: 0x10
	volatile uint32_t SR1; 					//Adder offset: 0x14
	volatile uint32_t SR2; 					//Adder offset: 0x18
	volatile uint32_t CCR; 					//Adder offset: 0x1C
	volatile uint32_t TRISE; 				//Adder offset: 0x20
	volatile uint32_t FLTR; 				//Adder offset: 0x24
}I2C_Register_t;
#define I2C1 ((I2C_Register_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_Register_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_Register_t *)I2C3_BASEADDR)

// Configure USART Structure
typedef struct{
	volatile uint32_t SR;					//Adder offset: 0x00
	volatile uint32_t DR;					//Adder offset: 0x04
	volatile uint32_t BRR;					//Adder offset: 0x08
	volatile uint32_t CR1;					//Adder offset: 0x0C
	volatile uint32_t CR2;					//Adder offset: 0x10
	volatile uint32_t CR3;					//Adder offset: 0x14
	volatile uint32_t GTPR;					//Adder offset: 0x18
}USART_Register_t;
#define USART1 ((USART_Register_t *)USART1_BASEADDR)
#define USART2 ((USART_Register_t *)USART2_BASEADDR)
#define USART6 ((USART_Register_t *)USART6_BASEADDR)

// Configuration SPI Structure
typedef struct{
	volatile uint32_t CR1; 					//Adder offset: 0x00
	volatile uint32_t CR2;					//Adder offset: 0x04
	volatile uint32_t SR; 					//Adder offset: 0x08
	volatile uint32_t DR; 					//Adder offset: 0x0C
	volatile uint32_t CRCPR; 				//Adder offset: 0x10
	volatile uint32_t RXCRCR; 				//Adder offset: 0x14
	volatile uint32_t TXCRCR; 				//Adder offset: 0x18
	volatile uint32_t I2SCFGR; 				//Adder offset: 0x1C
	volatile uint32_t I2SPR; 				//Adder offset: 0x20
}SPI_Register_t;
#define SPI1 ((SPI_Register_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_Register_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_Register_t *)SPI3_BASEADDR)
#define SPI4 ((SPI_Register_t *)SPI4_BASEADDR)

/*
 *	Clock enable Macros for SystemConfig
 */
#define SYSCFG_CLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * 	Clock enable Macros for DMA
 */
#define DMA1EN_MASK		21
#define DMA2EN_MASK		22
#define DMA1_PCLK_EN()	(RCC->AHB1ENR |= (1 << DMA1EN_MASK))
#define DMA2_PCLK_EN()	(RCC->AHB1ENR |= (1 << DMA2EN_MASK))

/*
 *	Clock enable Macros for GPIOx peripheral
 */
#define GPIO_PCLK_EN(x) (RCC->AHB1ENR |= (1 << GPIO_OFFSET(x)))

/*
 *	Clock enable Macros for I2Cx peripheral
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 *	Clock enable Macros for USARTx peripheral
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

/*
 *	Clock enable Macros for SPIx peripheral
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

/*
 *	Clock disable Macros for SystemConfig
 */
#define SYSCFG_CLK_DIS() (RCC->APB2ENR &= ~(1 << 14))

/*
 *	Clock disable Macros for GPIOx peripheral
 */
#define GPIO_PCLK_DIS(x) (RCC->AHB1ENR &= ~(1 << GPIO_OFFSET(x)))

/*
 *	Clock disable Macros for I2Cx peripheral
 */
#define I2C1_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 23))

/*
 *	Clock disable Macros for USARTx peripheral
 */
#define USART1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 5))

/*
 *	Clock disable Macros for SPIx peripheral
 */
#define SPI1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 15))
#define SPI4_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 13))

#define ENABLE			1
#define DISABLE			0
#define HIGH			ENABLE
#define LOW				DISABLE
#define SET				ENABLE
#define RESET			DISABLE

#define EXTI0			6
#define EXTI1			7
#define EXTI2			8
#define EXTI3			9
#define EXTI4			10
#define EXTI9_5			23
#define EXTI15_10		40
#define EXTI17			41
#define EXTI18			42

#define IRQ_SPI1		35
#define IRQ_SPI2		36
#define IRQ_SPI3		51
#define IRQ_SPI4		84

#define IRQ_USART1		37
#define IRQ_USART2		38
#define IRQ_USART6		71

#define IRQ_I2C1_EV		31
#define IRQ_I2C1_ER		32
#define IRQ_I2C2_EV		33
#define IRQ_I2C2_ER		34
#define IRQ_I2C3_EV		72
#define IRQ_I2C3_ER		73


#define GPIO_OFFSET(x)	( (x == GPIOA) ? 0 \
						: (x == GPIOB) ? 1 \
						: (x == GPIOC) ? 2 \
						: (x == GPIOD) ? 3 \
						: (x == GPIOE) ? 4 \
						: (x == GPIOH) ? 5 : 0)

#define GPIO_RCC_RESET(x)		do{(RCC->AHB1RSTR |= (1 << x)); (RCC->AHB1RSTR &= ~(1 << x));} while(0)



#endif

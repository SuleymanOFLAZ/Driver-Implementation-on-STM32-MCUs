/*
 * stm32f407xx.h
 *
 *  Created on: Apr 22, 2021
 *      Author: suleyman
 */
#include "stm32f407xx_gpio_driver.h"
#include "spi_driver.h"
#include <stdint.h>
#include <stddef.h>


#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

 /* Processor Specific */
#define NVIC_ISER 		((__vo uint32_t *)0xE000E100)
#define NVIC_ICER  		((__vo uint32_t *)0XE000E180)
#define NVIC_IPR  		((__vo uint32_t *)0xE000E400)

#define __vo volatile

 /* Some Generic Macros */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_BASEADDR_TO_CODE(x)	( (x==GPIOA) ? 0x0 :\
									  (x==GPIOB) ? 0x1 :\
									  (x==GPIOC) ? 0x2 :\
									  (x==GPIOD) ? 0x3 :\
									  (x==GPIOE) ? 0x4 :\
									  (x==GPIOF) ? 0x5 :\
									  (x==GPIOG) ? 0x6 :\
									  (x==GPIOH) ? 0x7 :\
									  (x==GPIOI) ? 0x8 :0 )

 /* IRQ numbers for EXTI */
#define IRQ_EXTI0			6
#define IRQ_EXTI1			7
#define IRQ_EXTI2			8
#define IRQ_EXTI3			9
#define IRQ_EXTI9_5			23
#define IRQ_EXTI15_10		40

 /* IRQ numbers for SPIs */
#define IRQ_SPI1			35
#define IRQ_SPI2			36
#define IRQ_SPI3			51

 /* Base addresses of FLASH and SRAM Memories */
#define FLASH_BASEADDR				0x08000000U  /* Base address of Main memory (flash) */
#define SRAM1_BASEADDR				0x20000000U  /* Base address of SRAM1 */
#define SRAM2_BASEADDR				0x2001C000U  /* Base address of SRAM1 + 112Kb */
#define ROM_BASEADDR				0x1FFF0000U  /* Base address of System memory */
#define OTPAREA_BASEADDR			0x1FFF7800U  /* Base address of OTP area */
#define SRAM SRAM1_BASEADDR

 /* Base addresses of bus domains */
#define PERIPH_BASE					0x40000000U  /* Base address of PERIPHERALS */
#define APB1PERIPH_BASEADDR			0x40000000U  /* Base address of APB1 Bus */
#define APB2PERIPH_BASEADDR			0x40010000U  /* Base address of APB2 Bus */
#define AHB1PERIPH_BASEADDR			0x40020000U  /* Base address of AHB1 Bus */
#define AHB2PERIPH_BASEADDR			0x50000000U  /* Base address of AHB2 Bus */

 /* Base address of AHB1 PERIPHERALS */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000U)  /* Base address of GPIOA Port */
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400U)  /* Base address of GPIOB Port */
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800U)  /* Base address of GPIOC Port */
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00U)  /* Base address of GPIOD Port */
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000U)  /* Base address of GPIOE Port */
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400U)  /* Base address of GPIOF Port */
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800U)  /* Base address of GPIOG Port */
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00U)  /* Base address of GPIOH Port */
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000U)  /* Base address of GPIOI Port */
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800U)  /* Base address of RCC Registers */

 /* Base address of APB1 PERIPHERALS */
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400U)  /* Base address of I2C1 */
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800U)  /* Base address of I2C2 */
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00U)  /* Base address of I2C3 */
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800U)  /* Base address of SPI2 */
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00U)  /* Base address of SPI3 */
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400U)  /* Base address of USART2 */
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800U)  /* Base address of USART3 */
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00U)  /* Base address of UART4 */
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000U)  /* Base address of UART5 */

 /* Base address of APB2 PERIPHERALS */
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000U)  /* Base address of SPI1 */
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000U)  /* Base address of USART1 */
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400U)  /* Base address of USART6 */
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800U)  /* Base address of SYSCFG */
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00U)  /* Base address of EXTI */

 /*
  * GPIO Registers definition
  */
typedef struct
{
	__vo uint32_t MODER;	/* GPIO port mode register 					- offset: 0x00 */
	__vo uint32_t OTYPER;	/* GPIO port output type register 			- offset: 0x04 */
	__vo uint32_t OSPEEDR;	/* GPIO port output speed register 			- offset: 0x08 */
	__vo uint32_t PUPDR;	/* GPIO port pull-up/pull-down register 	- offset: 0x0C */
	__vo uint32_t IDR;		/* GPIO port input data register 			- offset: 0x10 */
	__vo uint32_t ORD;		/* GPIO port output data register 			- offset: 0x14 */
	__vo uint32_t BSSR;		/* GPIO port bit set/reset register 		- offset: 0x18 */
	__vo uint32_t LCKR;		/* GPIO port configuration lock register 	- offset: 0x1C */
	__vo uint32_t AFRL;		/* GPIO alternate function low register 	- offset: 0x20 */
	__vo uint32_t AFRH;		/* GPIO alternate function high register 	- offset: 0x24 */
}GPIO_RegDef_t;

 /* GPIOAx Peripheral definition macros */
#define GPIOA		((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t *)GPIOI_BASEADDR)

/*
 * SPI Register Definition
 */
typedef struct
{
	__vo uint32_t SPI_CR1;				/* SPI control register 1				offset: 0x00 */
	__vo uint32_t SPI_CR2;				/* SPI control register 2				offset: 0x04 */
	__vo uint32_t SPI_SR;				/* SPI status register					offset: 0x08 */
	__vo uint32_t SPI_DR;				/* SPI data register					offset: 0x0C */
	__vo uint32_t SPI_CRCPR;			/* SPI CRC polynomial register			offset: 0x10 */
	__vo uint32_t SPI_RXCRCR;			/* SPI RX CRC register					offset: 0x14 */
	__vo uint32_t SPI_TXCRCR;			/* SPI TX CRC register					offset: 0x18 */
	__vo uint32_t SPI_I2SCFGR;			/* SPI_I2S configuration register 		offset: 0x1C */
	__vo uint32_t SPI_I2SPR;			/* SPI_I2S prescaler register			offset: 0x20 */
}SPI_RegDef_t;

#define SPI1		((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t *)SPI3_BASEADDR)

#define SPI1_RESET()	do{pRCC->RCC_APB2RSTR |= (1 << 12); pRCC->RCC_APB2RSTR &= ~(1 << 12);}while(0)
#define SPI2_RESET()	do{pRCC->RCC_APB1RSTR |= (1 << 14); pRCC->RCC_APB1RSTR &= ~(1 << 14);}while(0)
#define SPI3_RESET()	do{pRCC->RCC_APB1RSTR |= (1 << 15); pRCC->RCC_APB1RSTR &= ~(1 << 15);}while(0)


/*
 * EXTI Registers definition
 */

typedef struct
{
	__vo uint32_t IMR;		/* Interrupt mask register					- offset: 0x00 */
	__vo uint32_t EMR;		/* Event mask register						- offset: 0x04 */
	__vo uint32_t RTSR;		/* Rising trigger selection register 		- offset: 0x08 */
	__vo uint32_t FTSR;		/* Falling trigger selection register		- offset: 0x0C */
	__vo uint32_t SWIER;	/* Software interrupt event register		- offset: 0x10 */
	__vo uint32_t PR;		/* Pending register							- offset: 0x14 */
}EXTI_RegDef_t;

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * SYSCFG Registers definition
 */
typedef struct
{
	__vo uint32_t MEMRMP;		/* SYSCFG memory remap register							- offset: 0x00 */
	__vo uint32_t PMC;			/* SYSCFG peripheral mode configuration register		- offset: 0x04 */
	__vo uint32_t EXTICR[4];	/* SYSCFG external interrupt configuration register 1	- offset: 0x08 */
								/* SYSCFG external interrupt configuration register 2	- offset: 0x0C */
								/* SYSCFG external interrupt configuration register 3	- offset: 0x10 */
								/* SYSCFG external interrupt configuration register 4	- offset: 0x14 */
	__vo uint32_t RESERVED[2];
	__vo uint32_t CMPCR;		/* Compensation cell control register					- offset: 0x20 */
}SYSCFG_RegDef_t;

#define SYSCFG		((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

 /*
  * RCC Registers definition
  */
typedef struct
{
	__vo uint32_t RCC_CR;			/* RCC clock control register									- offset: 0x00 */
	__vo uint32_t RCC_PLLCFGR;		/* RCC PLL configuration register								- offset: 0x04 */
	__vo uint32_t RCC_CFGR;			/* RCC clock configuration register								- offset: 0x08 */
	__vo uint32_t RCC_CIR;			/* RCC clock interrupt register									- offset: 0x0C */
	__vo uint32_t RCC_AHB1RSTR;		/* RCC AHB1 peripheral reset register							- offset: 0x10 */
	__vo uint32_t RCC_AHB2RSTR;		/* RCC AHB2 peripheral reset register							- offset: 0x14 */
	__vo uint32_t RCC_AHB3RSTR;		/* RCC AHB3 peripheral reset register							- offset: 0x18 */
	__vo uint32_t RESERVED_1;		/* Reserved 0x1C */
	__vo uint32_t RCC_APB1RSTR;		/* RCC APB1 peripheral reset register							- offset: 0x20 */
	__vo uint32_t RCC_APB2RSTR;		/* RCC APB2 peripheral reset register							- offset: 0x24 */
	__vo uint32_t RESERVED_2;		/* Reserved 0x28 */
	__vo uint32_t RESERVED_3;		/* Reserved 0x2C */
	__vo uint32_t RCC_AHB1ENR;		/* RCC AHB1 peripheral clock enable register					- offset: 0x30 */
	__vo uint32_t RCC_AHB2ENR;		/* RCC AHB2 peripheral clock enable register					- offset: 0x34 */
	__vo uint32_t RCC_AHB3ENR;		/* RCC AHB3 peripheral clock enable register					- offset: 0x38 */
	__vo uint32_t RESERVED_4;		/* Reserved 0x3C */
	__vo uint32_t RCC_APB1ENR;		/* RCC APB1 peripheral clock enable register					- offset: 0x40 */
	__vo uint32_t RCC_APB2ENR;		/* RCC APB2 peripheral clock enable register					- offset: 0x44 */
	__vo uint32_t RESERVED_5;		/* Reserved 0x48 */
	__vo uint32_t RESERVED_6;		/* Reserved 0x4C */
	__vo uint32_t RCC_AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register	- offset: 0x50 */
	__vo uint32_t RCC_AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register	- offset: 0x54 */
	__vo uint32_t RCC_AHB3LPENR;	/* RCC AHB3 peripheral clock enable in low power mode register	- offset: 0x58 */
	__vo uint32_t RESERVED_7;		/* Reserved 0x5C */
	__vo uint32_t RCC_APB1LPENR;	/* RCC APB1 peripheral clock enable in low power mode register	- offset: 0x60 */
	__vo uint32_t RCC_APB2LPENR;	/* RCC APB2 peripheral clock enable in low power mode register	- offset: 0x64 */
	__vo uint32_t RESERVED_8;		/* Reserved 0x68 */
	__vo uint32_t RESERVED_9;		/* Reserved 0x6C */
	__vo uint32_t RCC_BDCR;			/* RCC Backup domain control register							- offset: 0x70 */
	__vo uint32_t RCC_CSR;			/* RCC clock control & status register							- offest: 0x74 */
	__vo uint32_t RESERVED_10;		/* Reserved 0x78 */
	__vo uint32_t RESERVED_11;		/* Reserved 0x7C */
	__vo uint32_t RCC_SSCGR;		/* RCC spread spectrum clock generation register				- offset: 0x80 */
	__vo uint32_t RCC_PLLI2SCFGR;	/* RCC PLLI2S configuration register							- offset: 0x84 */
}RCC_RegDef_t;

 /* RCC Peripheral definition macro */
#define pRCC			((RCC_RegDef_t *)RCC_BASEADDR)

 /* Clock Enable Macros for GPIOx Peripherals */
#define GPIOA_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (pRCC->RCC_AHB1ENR |= (1 << 8))

 /* Clock Enable Macros for I2Cx Peripherals */
#define I2C1_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 23))

 /* Clock Enable Macros for SPIx Peripherals */
#define SPI1_PCLK_EN() (pRCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 15))

 /* Clock Enable Macros for USARTx/UARTx Peripherals */
#define USART2_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (pRCC->RCC_APB1ENR |= (1 << 20))
#define USART1_PCLK_EN() (pRCC->RCC_APB2ENR |= (1 << 4))
#define USART6_PCLK_EN() (pRCC->RCC_APB2ENR |= (1 << 5))

 /* Clock Enable Macros for SYSCFG Peripherals */
#define SYSCFG_PCLK_EN() (pRCC->RCC_APB2ENR |= (1 << 14))

/* Clock Disable Macros for GPIOx Peripherals */
#define GPIOA_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (pRCC->RCC_AHB1ENR &= ~(1 << 8))

 /* Clock Disable Macros for I2Cx Peripherals */
#define I2C1_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 23))

 /* Clock Disable Macros for SPIx Peripherals */
#define SPI1_PCLK_DI() (pRCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 15))

 /* Clock Disable Macros for USARTx/UARTx Peripherals */
#define USART2_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (pRCC->RCC_APB1ENR &= ~(1 << 20))
#define USART1_PCLK_DI() (pRCC->RCC_APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI() (pRCC->RCC_APB2ENR &= ~(1 << 5))

 /* Clock Disable Macros for SYSCFG Peripherals */
#define SYSCFG_PCLK_DI() (pRCC->RCC_APB2ENR &= ~(1 << 14))

 /* Reset Peripheral Macros */
#define GPIOA_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 0)); (pRCC->RCC_AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 1)); (pRCC->RCC_AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 2)); (pRCC->RCC_AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 3)); (pRCC->RCC_AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 4)); (pRCC->RCC_AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 5)); (pRCC->RCC_AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 6)); (pRCC->RCC_AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 7)); (pRCC->RCC_AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REST() do{(pRCC->RCC_AHB1RSTR |= (1 << 8)); (pRCC->RCC_AHB1RSTR &= ~(1 << 8));}while(0)

#endif /* INC_STM32F407XX_H_ */

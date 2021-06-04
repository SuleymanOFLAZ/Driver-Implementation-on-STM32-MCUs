/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 23, 2021
 *      Author: suleyman
 */


#include "stm32f407xx.h"

/*
 * Peripheral Clock Setup
 */

/*****************************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function Enables or Disables GPIO Clock for given port
 *
 * @param[in]	- pGPIOx: Base Address of GPIOx Port
 * @param[in]	- EnorDi: Indicates Enabling or Disabling (macros)
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}

	}
}

/*
 * Init and De-init
 */

/*****************************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function initializes the GPIO for given port
 *
 * @param[in]	- pGPIOHandle: Includes GPIO base address and configure settings
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Setting Pin Mode
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	}
	else
	{
		// Interrupt Mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RE)
		{
			// 1. Configure the RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FE)
		{
			// 1. Configure the FTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFE)
		{
			// 1. Configure both RTSR and FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] &= ~(0xF << (temp2*4));
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2*4));


		// 3. Enable the EXTI Interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// Setting Output Type
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	// Setting Speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	//Setting Pull UP/DOWN Configuration
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;

	// Setting Alternate Function
	if(temp2 == 0)
	{
		pGPIOHandle->pGPIOx->AFRL &= ~(15 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	}
	else
	{
		pGPIOHandle->pGPIOx->AFRH &= ~(15 << (4 * temp1));
		pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp1));
	}
}

/*****************************************************************************
 * @fn			- GPIO_DeInit
 *
 * @brief		- This function de-initializes the GPIO for given port
 *
 * @param[in]	- pGPIOx: Base Address of GPIOx Port
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REST();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REST();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REST();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REST();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REST();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REST();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOF_REST();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REST();;
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REST();
	}

}

/*
 * Data read and write
 */

/*****************************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function reads value from given GPIO's pin
 *
 * @param[in]	- pGPIOx: Base Address of GPIOx Port
 * @param[in]	- PinNumber: Indicates pin number
 * @param[in]	-
 *
 * @return		- value that reads from pin
 *
 * @Note		- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return ( (pGPIOx->IDR >> PinNumber) & 0x00000001 );
}

/*****************************************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- This function reads values from given all GPIO port pins
 *
 * @param[in]	- pGPIOx: Base Address of GPIOx Port
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- values that read from pins (16 bit)
 *
 * @Note		- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return ( pGPIOx->IDR );
}

/*****************************************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- This function writes value from given GPIO's pin
 *
 * @param[in]	- pGPIOx: Base Address of GPIOx Port
 * @param[in]	- PinNumber: Indicates pin number
 * @param[in]	- Value: Value that want to write to the pin
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
		pGPIOx->ORD |= (Value << PinNumber);
	else
		pGPIOx->ORD &= ~(1 << PinNumber);
}

/*****************************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- This function writes value from given all GPIO port pins
 *
 * @param[in]	- pGPIOx: Base Address of GPIOx Port
 * @param[in]	- Value: Value that want to write to the pins (16 bit)
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ORD = Value;
}

/*****************************************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- This function toggles bit from given GPIO pin
 *
 * @param[in]	- pGPIOx: Base Address of GPIOx Port
 * @param[in]	- PinNumber: Indicates pin number
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ORD ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */

/*****************************************************************************
 * @fn			- GPIO_IRQConfig
 *
 * @brief		- This function configure IRQ for given GPIO Port
 *
 * @param[in]	- IRQNumber: Indicates IRQ Number
 * @param[in]	- IRQPriority: Indicates IRQ Priority
 * @param[in]	- EnorDi: Indicated enabling or disabling
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	volatile uint8_t temp1 = IRQNumber / 32;
	volatile uint8_t temp2 = IRQNumber % 32;

	if(EnorDi==ENABLE)
	{
		NVIC_ISER[temp1] |= (1 << temp2);
	}
	else
	{
		NVIC_ICER[temp1] |= (1 << temp2);
	}
}

/*****************************************************************************
 * @fn			- GPIO_IRQPriorityConfig
 *
 * @brief		- TThis function configure GPIO interrupt priority
 *
 * @param[in]	- RQNumber: Indicates IRQ number of interrupt
 * @param[in]	- IRQPriority: IRQ priority value
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	volatile uint8_t temp1 = IRQNumber / 4;
	volatile uint8_t temp2 = IRQNumber % 4;
	volatile uint8_t shift_amount;

	shift_amount = (temp2 * 8) + (8 - 4);  // This is MCU Specific calculation. In STM32F4xx MCU there is only 4 implemented priority bits.

	NVIC_IPR[temp1] &= ~(0xFF << (temp2 * 8));
	NVIC_IPR[temp1] |= (IRQPriority << shift_amount);
}

/*****************************************************************************
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- This function manages Interrupt operation from given GPIO pin
 *
 * @param[in]	- PinNumber: Indicates pin number
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Check the IRQ Pending bit set or reset
	if(EXTI->PR & (1 << PinNumber))
	{
		// set the Pending Bit for clearing it
		EXTI->PR = (1 << PinNumber);
	}
}

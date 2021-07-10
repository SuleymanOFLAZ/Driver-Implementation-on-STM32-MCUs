/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 8 Tem 2021
 *      Author: suley
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PrescalerDivFactors[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PrescalerDivFactors[4] = {2, 4, 8, 16};

/*****************************************************************************
 * @fn			- RCC_GetPCLK1Value
 *
 * @brief		- This function returns PCLK1 Clock Value
 *
 * @param[in]	- none
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- PCLK1 Clock Value
 *
 * @Note		- none
 */
uint32_t RCC_GetPLLClockValue(void)
{
	// Implement if it is neccesary
	return 0;
}

/*****************************************************************************
 * @fn			- RCC_GetPCLK1Value
 *
 * @brief		- This function returns PCLK1 Clock Value
 *
 * @param[in]	- none
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- PCLK1 Clock Value
 *
 * @Note		- none
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint8_t clksrc, temp;
	uint16_t ahbpre, apb1pre;
	uint32_t pclk1, SystemClk;

	clksrc = (pRCC->RCC_CFGR >> 2) & 3;
	if(clksrc == 0) // If System Clock is HSI
		SystemClk = 16000000; 	// Specific to STM32F407xx MCUs
	else if(clksrc == 1) // If System Clock is HSE
		SystemClk = 8000000; 	// Specific to STM32 Discovery Board
	else if(clksrc == 3) // If System Clock is PLL
		SystemClk = RCC_GetPLLClockValue();

	// Get AHB Prescaler Value
	temp = (pRCC->RCC_CFGR >> 4) & 0xF;
	if(temp < 8)
		ahbpre = 1;
	else
		ahbpre = AHB_PrescalerDivFactors[temp - 8];

	// Get APB1 Prescaler Value
	temp = (pRCC->RCC_CFGR >> 10) & 0x7;
	if(temp < 4)
		apb1pre = 1;
	else
		apb1pre = APB1_PrescalerDivFactors[temp - 4];

	pclk1 = (SystemClk / ahbpre) / apb1pre;

	return pclk1;
}

/*****************************************************************************
 * @fn			- RCC_GetPCLK2Value
 *
 * @brief		- This function returns PCLK2 Clock Value
 *
 * @param[in]	- none
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- PCLK2 Clock Value
 *
 * @Note		- none
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint8_t clksrc, temp;
	uint16_t ahbpre, apb2pre;
	uint32_t pclk2, SystemClk;

	clksrc = (pRCC->RCC_CFGR >> 2) & 3;
	if(clksrc == 0) // If System Clock is HSI
		SystemClk = 16000000; 	// Specific to STM32F407xx MCUs
	else if(clksrc == 1) // If System Clock is HSE
		SystemClk = 8000000; 	// Specific to STM32 Discovery Board
	else if(clksrc == 3) // If System Clock is PLL
		SystemClk = RCC_GetPLLClockValue();

	// Get AHB Prescaler Value
	temp = (pRCC->RCC_CFGR >> 4) & 0xF;
	if(temp < 8)
		ahbpre = 1;
	else
		ahbpre = AHB_PrescalerDivFactors[temp - 8];

	// Get APB2 Prescaler Value
	temp = (pRCC->RCC_CFGR >> 13) & 0x7;
	if(temp < 4)
		apb2pre = 1;
	else
		apb2pre = APB1_PrescalerDivFactors[temp - 4];

	pclk2 = (SystemClk / ahbpre) / apb2pre;

	return pclk2;
}

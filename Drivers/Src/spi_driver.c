/*
 * spi_driver.c
 *
 *  Created on: May 1, 2021
 *      Author: suleyman
 */
#include "stm32f407xx.h"

/*
 * Some interrupt handling helper functions
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*****************************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function controls the SPI peripheral clock
 *
 * @param[in]	- pSPIx: Base address of SPI Peripheral
 * @param[in]	- EnorDi: Indicates operation (enable or disable)
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}

	else if(EnorDi == DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*****************************************************************************
 * @fn			- SPI_Init
 *
 * @brief		- This function Initializes SPI Peripheral
 *
 * @param[in]	- pSPIHandle: A Structure that included base address of SPI Peripheral and Peripheral settings
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/* Enabling SPI Peripheral */
	//pSPIHandle->pSPIx->SPI_CR1 |= (1 << 6);

	/* SPI MODE Configure */
	if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_MODE_SLAVE)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 2);
	}
	else if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_MODE_MASTER)
	{
		pSPIHandle->pSPIx->SPI_CR1 |= (1 << 2);
	}

	/* SPI BUS Configure */
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FULLDUBLEX)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 15);
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 10);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HALFDUBLEX)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 10);
		pSPIHandle->pSPIx->SPI_CR1 |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 15);
		pSPIHandle->pSPIx->SPI_CR1 |= (1 << 10);
	}

	/* SPI SCLK Speed Configure */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(7 << 3);
	pSPIHandle->pSPIx->SPI_CR1 |= ( (pSPIHandle->SPIConfig.SPI_SclkSpeed & 7) << 3);

	/* SPI DFF Configure */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 11);
	pSPIHandle->pSPIx->SPI_CR1 |= ( (pSPIHandle->SPIConfig.SPI_DFF & 1) << 11);

	/* SPI CPOL Configure */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 1);
	pSPIHandle->pSPIx->SPI_CR1 |= ( (pSPIHandle->SPIConfig.SPI_CPOL & 1) << 1);

	/* SPI CPHA Configure */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 0);
	pSPIHandle->pSPIx->SPI_CR1 |= ( (pSPIHandle->SPIConfig.SPI_CPHA & 1) << 0);

	/* SPI SSM Configure */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(1 << 9);
	pSPIHandle->pSPIx->SPI_CR1 |= ( (pSPIHandle->SPIConfig.SPI_SSM & 1) << 9);

}

/*****************************************************************************
 * @fn			- SPI_DeInit
 *
 * @brief		- This function De-Initializes SPI Peripheral
 *
 * @param[in]	- pSPIx: Base address of SPI Peripheral
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_RESET();
	}
}

/*****************************************************************************
 * @fn			- SPI_SendData
 *
 * @brief		- This function sends SPI data
 *
 * @param[in]	- pSPIx: Base address of SPI Peripheral
 * @param[in]	- pTxBuffer: Address of Transmit buffer
 * @param[in]	- Len: Length of data that will send
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while( (SPI_GetFlag(SPI2, SPI_FLAG_TXE)) == 0 );
		if( !((pSPIx->SPI_CR1 >> 11) & 1)  )		// IF DFF is 8-bit
		{
			pSPIx->SPI_DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}
		else										// IF DFF is 16-bit
		{
			pSPIx->SPI_DR = *(uint16_t *)pTxBuffer;
			(uint16_t *)pTxBuffer++;
			Len -= 2;
		}
	}
}

/*****************************************************************************
 * @fn			- SPI_ReceiveData
 *
 * @brief		- This function receives SPI data
 *
 * @param[in]	- pSPIx: Base address of SPI Peripheral
 * @param[in]	- pRxBuffer: Address of Receive buffer
 * @param[in]	- Len: Length of data that will receive
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	if(Len > 0)
	{
		while( !(SPI_GetFlag(SPI2, SPI_FLAG_RXNE)) );
		if( !((pSPIx->SPI_CR1 >> 11) & 1) )
		{
			// DF=8-bit
			*pRxBuffer = pSPIx->SPI_DR;
			pRxBuffer++;
			Len--;
		}
		else
		{
			// DF=16-bit
			*(uint16_t *)pRxBuffer = pSPIx->SPI_DR;
			(uint16_t *)pRxBuffer++;
			Len -= 2;
		}
	}
}

/*****************************************************************************
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		- This function initializes SPI interrupt
 *
 * @param[in]	- IRQNumber: Indicates IRQ number of interrupt
 * @param[in]	- EnorDi: Indicates operation (enable or disable)
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief		- TThis function configure SPI interrupt priority
 *
 * @param[in]	- RQNumber: Indicates IRQ number of interrupt
 * @param[in]	- IRQPriority: IRQ priority value
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	volatile uint8_t temp1 = IRQNumber / 4;
	volatile uint8_t temp2 = IRQNumber % 4;
	volatile uint8_t shift_amount;

	shift_amount = (temp2 * 8) + (8 - 4);  // This is MCU Specific calculation. In STM32F4xx MCU there is only 4 implemented priority bits.

	NVIC_IPR[temp1] &= ~(0xFF << (temp2 * 8));
	NVIC_IPR[temp1] |= (IRQPriority << shift_amount);
}

/*****************************************************************************
 * @fn			- SPI_IRQHandling
 *
 * @brief		- This function reset Interrupt pending bit of SPI
 *
 * @param[in]	- pHandle: A Structure that included base address of SPI Peripheral and Peripheral settings
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	temp1 = ((pHandle->pSPIx->SPI_SR >> 1) & 1 );	// Check TXE
	temp2 = ((pHandle->pSPIx->SPI_CR2 >> 7) & 1 );	// Check TXEIE

	if( temp1 && temp2)
	{
		// Handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	temp1 = ((pHandle->pSPIx->SPI_SR >> 0) & 1 );  // Check RXNE
	temp2 = ((pHandle->pSPIx->SPI_CR2 >> 6) & 1 ); // Check RXNEIE

	if( temp1 && temp2)
	{
		// Handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	temp1 = ((pHandle->pSPIx->SPI_SR >> 6) & 1 );  // Check OVR (Overrun) flag
	temp2 = ((pHandle->pSPIx->SPI_CR2 >> 5) & 1 ); // Check ERRIE

	if( temp1 && temp2)
	{
		// Handle overrun error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}


/*****************************************************************************
 * @fn			- SPI_SendDataIT
 *
 * @brief		- This function sends SPI data with non-blocking type (with SPI Interrupt)
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures
 * @param[in]	- pTxBuffer: Address of Transmit buffer
 * @param[in]	- Len: Length of data that will send
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	volatile uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		// 2. Mark SPI state as bussy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 7);
	}
	// 4. Data Transmission will be handled by ISR code

	return state;
}

/*****************************************************************************
 * @fn			- SPI_ReceiveDataIT
 *
 * @brief		- This function receives SPI data with non-blocking type (with SPI Interrupt)
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures
 * @param[in]	- pTxBuffer: Address of Transmit buffer
 * @param[in]	- Len: Length of data that will send
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	volatile uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		// 2. Mark SPI state as bussy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 6);
	}
	// 4. Data Transmission will be handled by ISR code

	return state;
}

/*
 * Other Peripheral Control APIs
 */

/*****************************************************************************
 * @fn			- SPI_PeripheralControl
 *
 * @brief		- This function enables or disables given SPI peripheral via SPI enable bit of CR1 register
 *
 * @param[in]	- pSPIx: A structure that includes base address of SPI Peripheral
 * @param[in]	- EnorDi: Indicates enable or disable options
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << 6);  // Set SPI enable bit
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << 6);  // Reset SPI enable bit
	}
}

/*****************************************************************************
 * @fn			- SPI_SSIConfig
 *
 * @brief		- This function enables or disables "Internal Slave Select" option of given SPI peripheral via SSM (Software Slave Management) bit of CR1 Register
 *
 * @param[in]	- pSPIx: A structure that includes base address of SPI Peripheral
 * @param[in]	- EnorDi: Indicates enable or disable options
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << 8);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << 8);
	}
}

/*****************************************************************************
 * @fn			- SPI_GetFlag
 *
 * @brief		- This function returns given SPI Flag
 *
 * @param[in]	- pSPIx: A structure that includes base address of SPI Peripheral
 * @param[in]	- flag: Indicates flags  --  check for options: @spi_flags   in SPI driver header file
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t SPI_GetFlag(SPI_RegDef_t *pSPIx, uint8_t flag)
{
	return (uint8_t)((pSPIx->SPI_SR >> flag) & 1); // check for options: @spi_flags   in SPI driver header file
}

/*****************************************************************************
 * @fn			- SPI_SSOEConfig
 *
 * @brief		- This function enables or disables "Slave Select Output Enable" for given SPI peripheral via SSOE bit of CR2 Regiser
 *
 * @param[in]	- pSPIx: A structure that includes base address of SPI Peripheral
 * @param[in]	- EnorDi: Indicates enable or disable options
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << 2);
	}
	else
	{
		pSPIx->SPI_CR2 &= ~(1 << 2);
	}
}

/*****************************************************************************
 * @fn			- spi_txe_interrupt_handle
 *
 * @brief		- This function runs SPI transmit steps when TXE happens
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures and non-blocking(Interrupt) mode global variables
 * @param[in]	- none
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( !((pSPIHandle->pSPIx->SPI_CR1 >> 11) & 1) ) // Check DFF
	{
		// If DFF is 8-bit
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}
	else
	{
		// If DFF is 16-bit
		pSPIHandle->pSPIx->SPI_DR = *(uint16_t*)pSPIHandle->pTxBuffer;
		(uint16_t *)pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen-=2;
	}
	if(pSPIHandle->TxLen == 0)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/*****************************************************************************
 * @fn			- spi_rxne_interrupt_handle
 *
 * @brief		-  This function runs SPI receive steps when RXE happens
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures and non-blocking(Interrupt) mode global variables
 * @param[in]	- none
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( !((pSPIHandle->pSPIx->SPI_CR1 >> 11) & 1) ) // Check DFF
	{
		// If DFF is 8-bit
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}
	else
	{
		// If DFF is 16-bit
		*(uint16_t *)pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->SPI_DR;
		(uint16_t *)pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen-=2;
	}
	if(pSPIHandle->RxLen == 0)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/*****************************************************************************
 * @fn			- spi_ovr_err_interrupt_handle
 *
 * @brief		-  This function runs SPI handling steps when spi overrun error happens
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures and non-blocking(Interrupt) mode global variables
 * @param[in]	- none
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	// 1. Clean the ovr flag -- (This steps describes in MCU's relevant reference manual document)
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX) // IS SPI is not busy with transmit data
	{
		temp = pSPIHandle->pSPIx->SPI_DR; // Read access to DR register
		temp = pSPIHandle->pSPIx->SPI_SR; // Read access to SR register
	}
	(void)temp;
	// 2. Inform application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/*****************************************************************************
 * @fn			- SPI_ClearOVRFlag
 *
 * @brief		-  This function runs SPI handling steps when spi overrun error happens
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures and non-blocking(Interrupt) mode global variables
 * @param[in]	- none
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR; // Read access to DR register
	temp = pSPIx->SPI_SR; // Read access to SR register
	(void)temp; // For prevent unused variable warning
}

/*****************************************************************************
 * @fn			- SPI_CloseTransmission
 *
 * @brief		-  This function runs SPI handling steps when spi overrun error happens
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures and non-blocking(Interrupt) mode global variables
 * @param[in]	- none
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << 7); 	// Clear the TXEIE bit
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/*****************************************************************************
 * @fn			- SPI_CloseReception
 *
 * @brief		-  This function runs SPI handling steps when spi overrun error happens
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures and non-blocking(Interrupt) mode global variables
 * @param[in]	- none
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << 6); 	// Clear the RXNEIE bit
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


/*****************************************************************************
 * @fn			- SPI_ApplicationEventCallback
 *
 * @brief		-  This is weak implementation. The application may override this function.
 *
 * @param[in]	- pSPIHandle: A structure that includes base address of SPI Peripheral and config structures and non-blocking(Interrupt) mode global variables
 * @param[in]	- AppEv: Indicates Event Type
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- none
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// This is weak implementation. The application may override this function.
}

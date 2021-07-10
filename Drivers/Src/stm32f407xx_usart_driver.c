/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: 4 Tem 2021
 *      Author: suley
 */

#include "stm32f407xx.h"

static void USART_ConfigureBaudRate(USART_Handle_t *pHandle);
static void USART_CloseCommunication(USART_Handle_t *pHandle, uint8_t ClsSel);

/*****************************************************************************
 * @fn			- USART_IRQPriorityConfig
 *
 * @brief		- This function configures the priority of USART interrupts
 *
 * @param[in]	- IRQNumber: Indicates IRQ Number
 * @param[in]	- IRQPriority: Indicates interrpt priority level
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		-
 */
static void USART_CloseCommunication(USART_Handle_t *pHandle, uint8_t Ev)
{
	// First disable the USART interrupts

	// Clear the global USART data
	if(Ev == USART_TX)
	{
		pHandle->TxLen = 0;
		pHandle->pTxBuffer = NULL;
		pHandle->USART_TxStatus = USART_STATUS_TX_READY;

		//pHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TXEIE);
		pHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TCIE);
	}
	if(Ev == USART_RX)
	{
		pHandle->RxLen = 0;
		pHandle->pTxBuffer = NULL;
		pHandle->USART_RxStatus = USART_STATUS_RX_READY;

		// pHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_RXNEIE);
		pHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_PEIE);
	}

	if((pHandle->USART_TxStatus == USART_STATUS_TX_READY) && (pHandle->USART_TxStatus == USART_STATUS_RX_READY))
	{
		// That mean both of Tx end Rx not active, so cloes all interrupts
		pHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_IDLEIE);
		pHandle->pUSARTx->USART_CR2 |= (1 << USART_CR2_LBDIE);
		pHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_EIE);
		pHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PEIE);
		pHandle->pUSARTx->USART_CR3 &= ~(1 << USART_CR3_CTSIE);
	}
}

/*
 * Peripheral Clock Setup
 */

/*****************************************************************************
 * @fn			- USART_PeriClockControl
 *
 * @brief		- This function Enables or Disables USART/UART Clock for given port
 *
 * @param[in]	- pI2Cx: Base Address of USARTx Port
 * @param[in]	- EnorDi: Indicates Enabling or Disabling (macros)
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/*****************************************************************************
 * @fn			- USART_Init
 *
 * @brief		- This function initializes the USART for given port
 *
 * @param[in]	- pUSARTHandle: Includes Register Definition Structures and Configuration Structures of USART Unit
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// 1. Configure USART Mode
	if( (pUSARTHandle->USART_Config.USART_Mode == USART_CNFG_MODE_TX) || (pUSARTHandle->USART_Config.USART_Mode == USART_CNFG_MODE_TXRX))
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TE);
	if( (pUSARTHandle->USART_Config.USART_Mode == USART_CNFG_MODE_RX) || (pUSARTHandle->USART_Config.USART_Mode == USART_CNFG_MODE_TXRX))
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RE);

	// 2. Configure Word Length
	if(pUSARTHandle->USART_Config.USART_WordLen == USART_CNFG_WORDLEN_8)
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_M);
	else if(pUSARTHandle->USART_Config.USART_WordLen == USART_CNFG_WORDLEN_9)
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_M);

	// 3. Configure Baud Rate
	USART_ConfigureBaudRate(pUSARTHandle);

	// 4. Configure Stop Bits
	if(pUSARTHandle->USART_Config.USART_StopLen == USART_CNFG_STOPBITLEN0_5)
		pUSARTHandle->pUSARTx->USART_CR2 |= (1 << USART_CR2_STOP);
	else if(pUSARTHandle->USART_Config.USART_StopLen == USART_CNFG_STOPBITLEN_1)
		pUSARTHandle->pUSARTx->USART_CR2 &= ~(3 << USART_CR2_STOP);
	else if(pUSARTHandle->USART_Config.USART_StopLen == USART_CNFG_STOPBITLEN1_5)
		pUSARTHandle->pUSARTx->USART_CR2 |= (3 << USART_CR2_STOP);
	else if(pUSARTHandle->USART_Config.USART_StopLen == USART_CNFG_STOPBITLEN_2)
		pUSARTHandle->pUSARTx->USART_CR2 |= (2 << USART_CR2_STOP);

	// 5. Configure Parity Control
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_DI)
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_PCE);
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_EN_EVEN)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_PS);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_EN_ODD)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PS);
	}

	// 6. Configure Hardware Flow Control
	if((pUSARTHandle->USART_Config.USART_HWFlowControl == USART_CNFG_HWFLOWCONTROL_CTS) || (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_CNFG_HWFLOWCONTROL_CTSRTS))
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_CTSE);
	if((pUSARTHandle->USART_Config.USART_HWFlowControl == USART_CNFG_HWFLOWCONTROL_RTS) || (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_CNFG_HWFLOWCONTROL_CTSRTS))
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_RTSE);

	// 7. Configure OVER 8
	if(pUSARTHandle->USART_Config.USART_OVER8Sel == USART_CNFG_OVER8_SET)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_OVER8);
	}
}

/*****************************************************************************
 * @fn			- USART_DeInit
 *
 * @brief		- This function de-initializes the USART for given port
 *
 * @param[in]	- pUSARTx: Base Address of USARTx Port
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_RESET();
	}
}

/*
 * Data Send and Receive
 */

/*****************************************************************************
 * @fn			- USART_SendData
 *
 * @brief		- This function sends data with USART/UART Communication
 *
 * @param[in]	- pUSARTx: Base address of USART Peripheral
 * @param[in]	- pTxBuffer	: Transmission buffer
 * @param[in]	- Len		: Length info of data that will transmit
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_SendData(USART_Handle_t *pHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pData;

	// Loop over untill "Len" number of bytes are transferred
	for(uint32_t i=0; i<Len; i++)
	{
		// Wait for TXE is empty
		while(!(pHandle->pUSARTx->USART_SR & (1 << USART_SR_TXE)));

		// Check the USART_WordLen item for 9-bit or 8- bit in a frame
		if(pHandle->USART_Config.USART_WordLen == USART_CNFG_WORDLEN_9)
		{
			// 9-bit word length is using.
			pData = (uint16_t *)pTxBuffer;
			pHandle->pUSARTx->USART_DR = (*pData & (uint16_t)0x01FF);

			// Check for USART_Parity Control
			if(pHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_DI)
			{
				// No parity is used in this transfer, so 9 bits of user data will be sent
				// increment the pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// Parity bit is used in this transfer, so 8 bits of user data will be sent
				// The 9th bit will be replaced by parity bit by hardware
				pTxBuffer++;
			}
		}
		else
		{
			// This is 8 bits data transfer
			pHandle->pUSARTx->USART_DR = (*pTxBuffer & (uint8_t)0xFF);

			pTxBuffer++;
		}
	}

	// Wait until TC is set (Transmission Complated)
	while(!(USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_TC)));
}

/*****************************************************************************
 * @fn			- USART_ReceiveData
 *
 * @brief		- This function receives serial data with USART/UART Communication
 *
 * @param[in]	- pUSARTx: Base address of USART Peripheral
 * @param[in]	- pRxBuffer	: Receive buffer
 * @param[in]	- Len		: Length info of data that will transmit
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_ReceiveData(USART_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i=0; i<Len; i++)
	{
		// Wait for RXNE is empty
		while(!(pHandle->pUSARTx->USART_SR & (1 << USART_SR_RXNE)));

		// Check the USART_WordLen item for 9-bit or 8- bit in a frame
		if(pHandle->USART_Config.USART_WordLen == USART_CNFG_WORDLEN_9)
		{
			// 9-bit word length is using.

			// Check for USART_Parity Control
			if(pHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_DI)
			{
				// No parity is used in this transfer, so 9 bits of user data will be receive
				// increment the pRxBuffer twice
				*((uint16_t *)pRxBuffer) = ((uint16_t )pHandle->pUSARTx->USART_DR & ((uint16_t)0x01FF));
				(uint16_t *)pRxBuffer++;
			}
			else
			{
				// Parity bit is used in this transfer, so 8 bits of user data will be receive, and 1 parity bit receive
				*pRxBuffer = pHandle->pUSARTx->USART_DR & (uint8_t)0xFF;
				pRxBuffer++;
			}
		}
		else
		{
			// This is 8 bits data transfer

			// check are we using USART_Prity or not
			if(pHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_DI)
			{
				// No parity is used in this transfer, so all 8 bits of user data will be receive
				// increment the pRxBuffer twice
				*pRxBuffer = pHandle->pUSARTx->USART_DR & (uint8_t)0xFF;
			}
			else
			{
				// Parity is used, 7 bits are user data and 1 bit is parity bit, mask with 0x7F
				*pRxBuffer = pHandle->pUSARTx->USART_DR & (uint8_t)0x7F;
			}

			// Increment the buffer
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - USART data send function with using USART interrupts
 *
 * @param[in]         - *pUSARTHandle	: pointer to USART Handle Structure
 * @param[in]         -	*pTxBuffer	: pointer to Transmission Buffer
 * @param[in]         -	Len			: Length information of data that will send
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - Current USART state
 *
 * @Note              -

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t status = pUSARTHandle->USART_TxStatus;

	if(status == USART_STATUS_TX_READY)
	{
		pUSARTHandle->USART_TxStatus = USART_STATUS_TX_BUSY;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = Len;

		// Enable Interrupts
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TXEIE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TCIE);
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_CTSIE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_IDLEIE);
		pUSARTHandle->pUSARTx->USART_CR2 |= (1 << USART_CR2_LBDIE);
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_EIE);
	}

	return status;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - USART receive send function with using USART interrupts
 *
 * @param[in]         - *pUSARTHandle	: pointer to USART Handle Structure
 * @param[in]         -	*pRxBuffer	: pointer to Receive Buffer
 * @param[in]         -	Len			: Length information of data that will receive
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - Current USART state
 *
 * @Note              -

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t status = pUSARTHandle->USART_RxStatus;

	if(status == USART_STATUS_RX_READY)
	{
		pUSARTHandle->USART_RxStatus = USART_STATUS_RX_BUSY;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxLen = Len;

		// Enable Interrupts
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RXNEIE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_IDLEIE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PEIE);
		pUSARTHandle->pUSARTx->USART_CR2 |= (1 << USART_CR2_LBDIE);
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_EIE);

		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RE);
	}

	return status;
}

/*
 * IRQ Configuration and ISR handling
 */

/*****************************************************************************
 * @fn			- USART_IRQInterruptConfig
 *
 * @brief		- This function enables or disables interrupt for USART peripheral
 *
 * @param[in]	- IRQNumber: Indicates IRQ Number
 * @param[in]	- EnorDi: indicates Enable or Disable options
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note
 * */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	uint16_t temp_div, temp_mod;
	temp_div = IRQNumber / 32;
	temp_mod = IRQNumber % 32;

	if(EnorDi == ENABLE)
	{
		NVIC_ISER[temp_div] |= (1 << temp_mod);
	}
	else if(EnorDi == DISABLE)
	{
		NVIC_ICER[temp_div] |= (1 << temp_mod);
	}
}

/*****************************************************************************
 * @fn			- USART_IRQPriorityConfig
 *
 * @brief		- This function configures the priority of USART interrupts
 *
 * @param[in]	- IRQNumber: Indicates IRQ Number
 * @param[in]	- IRQPriority: Indicates interrpt priority level
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		-
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp_div, temp_mod;
	temp_div = IRQNumber / 4;
	temp_mod = IRQNumber % 4;

	IRQPriority = IRQPriority << 4;  // This is special for STM32f407xx MCUs, First 4 MSB bits are available for priority configuration.

	NVIC_IPR[temp_div] &= ~(0xFF << (temp_mod * 8));  // Each Interrupt Priority Configuration field is 8 bits in CortexM4 MPUs
	NVIC_IPR[temp_div] |= (IRQPriority << (temp_mod * 8));

}

/*********************************************************************
 * @fn      		  - USART_IRQHandling
 *
 * @brief             - Interrupt Service Routine for USART Event Interrupts (ISR)
 *
 * @param[in]         - *pHandle	: pointer to USART Handle Structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -

 */
void USART_IRQHandling(USART_Handle_t *pHandle)
{
	__vo uint32_t temp;

	// First we must check reason of interrupt
	if( USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_TXE) && (pHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_TXEIE)))
	{
		// TxE Interrupt Happened
		if(pHandle->USART_TxStatus == USART_STATUS_TX_BUSY)
		{
			// TxE Interrupt Happened
			if(pHandle->TxLen > 0)
			{
				if(pHandle->USART_Config.USART_WordLen == USART_CNFG_WORDLEN_9)
				{
					// 9-bit word length is using.
					pHandle->pUSARTx->USART_DR = ( *(uint16_t *)pHandle->pTxBuffer & (uint16_t)0x01FF);

					// Check for USART_Parity Control
					if(pHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_DI)
					{
						// No parity is used in this transfer, so 9 bits of user data will be sent
						// increment the pTxBuffer twice
						pHandle->pTxBuffer++;
						pHandle->pTxBuffer++;
						pHandle->TxLen--;
					}
					else
					{
						// Parity bit is used in this transfer, so 8 bits of user data will be sent
						// The 9th bit will be replaced by parity bit by hardware
						pHandle->pTxBuffer++;
						pHandle->TxLen--;
					}
				}
				else
				{
					// This is 8 bits data transfer
					pHandle->pUSARTx->USART_DR = (*(uint16_t *)pHandle->pTxBuffer & (uint8_t)0xFF);

					pHandle->pTxBuffer++;
					pHandle->TxLen--;
				}

				if(pHandle->TxLen == 0)
				{
					// Clear the TXEIE bit
					pHandle->pUSARTx->USART_CR1 &= ~(USART_CR1_TXEIE);
				}
			}

		}
	}

	if( USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_RXNE) && (pHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_RXNEIE)))
	{
		// RxNE interrupt happened
		if(pHandle->USART_RxStatus == USART_STATUS_RX_BUSY)
		{
			if(pHandle->RxLen > 0)
			{
				// Check the USART_WordLen item for 9-bit or 8- bit in a frame
				if(pHandle->USART_Config.USART_WordLen == USART_CNFG_WORDLEN_9)
				{
					// 9-bit word length is using.
					// Check for USART_Parity Control
					if(pHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_DI)
					{
						// No parity is used in this transfer, so 9 bits of user data will be receive
						// increment the pRxBuffer twice
						*((uint16_t *)pHandle->pRxBuffer) = ((uint16_t )pHandle->pUSARTx->USART_DR & ((uint16_t)0x01FF));
						(uint16_t *)pHandle->pRxBuffer++;
						pHandle->RxLen--;
					}
					else
					{
					// Parity bit is used in this transfer, so 8 bits of user data will be receive, and 1 parity bit receive
					*pHandle->pRxBuffer = pHandle->pUSARTx->USART_DR & (uint8_t)0xFF;
					pHandle->pRxBuffer++;
					pHandle->RxLen--;
					}
				}
				else
				{
					// This is 8 bits data transfer

					// check are we using USART_Prity or not
					if(pHandle->USART_Config.USART_ParityControl == USART_CNFG_PARITY_DI)
					{
						// No parity is used in this transfer, so all 8 bits of user data will be receive
						// increment the pRxBuffer twice
						*pHandle->pRxBuffer = pHandle->pUSARTx->USART_DR & (uint8_t)0xFF;
					}
					else
					{
						// Parity is used, 7 bits are user data and 1 bit is parity bit, mask with 0x7F
						*pHandle->pRxBuffer = pHandle->pUSARTx->USART_DR & (uint8_t)0x7F;
					}

					// Increment the buffer
					pHandle->pRxBuffer++;
					pHandle->RxLen--;
				}
			}
			if(!pHandle->RxLen)
			{
				// Clear RXNEIE bit
				pHandle->pUSARTx->USART_CR1 &= ~(USART_CR1_RXNEIE);

				USART_CloseCommunication(pHandle, USART_RX);

				USART_ApplicationEventCallback(pHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	if(USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_TC) && (pHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_TCIE)))
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pHandle->USART_TxStatus == USART_STATUS_TX_BUSY)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pHandle->TxLen)
			{
				//Implement the code to clear the TC flag
				pHandle->pUSARTx->USART_SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_TCIE);

				//Reset the application state
				USART_CloseCommunication(pHandle, USART_TX);

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	if( (USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_CTS)) && (pHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_CTSE)) && (pHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_CTSIE)) )
	{
		// This interrupt happened because of CTS
		// Note : This is not applicable for UART5 and UART5 (Only applicaple in USART)

		// Clear the CTS FLAG
		pHandle->pUSARTx->USART_SR &= ~(1 << USART_SR_CTS);

		// Notify the user application
		USART_ApplicationEventCallback(pHandle, USART_EVENT_CTS);
	}

	if( (USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_IDLE)) & (pHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_IDLEIE)) )
	{
		// This interrupt happened because of IDLE

		// Clear IDLE Flag - Read SR and DR
		temp = pHandle->pUSARTx->USART_SR;
		temp = pHandle->pUSARTx->USART_DR;

		// Notify the user application
		USART_ApplicationEventCallback(pHandle, USART_EVENT_IDLE);
	}

	if( (USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_ORE)) & (pHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_RXNEIE)) )
	{
		// This interrupt happened because of Overrun ERROR

		// Clear the ORE bit - Read SR and DR
		temp = pHandle->pUSARTx->USART_SR;
		temp = pHandle->pUSARTx->USART_DR;

		// Notify the user application
		USART_ApplicationEventCallback(pHandle, USART_ERROR_ORE);
	}

	if(pHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_EIE))
	{
		// Error flag is set. This is happened because of Noise flag, overrun error and farming error in multibuffer communication.
		// The blow code will executed in only if multibuffer mode is used. Please refer to Reference Manual.
		if(USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_FE))
		{
			// Farming error detected
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			temp = pHandle->pUSARTx->USART_SR;
			temp = pHandle->pUSARTx->USART_DR;
			USART_ApplicationEventCallback(pHandle, USART_ERROR_FE);
		}
		if(USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_NF))
		{
			// Noise error detected
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			temp = pHandle->pUSARTx->USART_SR;
			temp = pHandle->pUSARTx->USART_DR;
			USART_ApplicationEventCallback(pHandle, USART_ERROR_NF);
		}
		if(USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_ORE))
		{
			// Overrun error detected
			/*
			 	 This bit is set by hardware when the word currently being received in the shift register is
				ready to be transferred into the RDR register while RXNE=1. An interrupt is generated if
				RXNEIE=1 in the USART_CR1 register. It is cleared by a software sequence (an read to the
				USART_SR register followed by a read to the USART_DR register).
			 */
			temp = pHandle->pUSARTx->USART_SR;
			temp = pHandle->pUSARTx->USART_DR;
			USART_ApplicationEventCallback(pHandle, USART_ERROR_ORE);
		}
	}

	if( (USART_GetFlagStatus(pHandle->pUSARTx, USART_SR_PE)) & (pHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_PEIE)) )
	{
		// Interrupt happened because of Parity Error

		// Clear the PE bit

		// Notify the use application
		USART_ApplicationEventCallback(pHandle, USART_ERROR_PE);
	}

	if( (USART_GetFlagStatus(pHandle->pUSARTx, USART_SR_LBD)) & (pHandle->pUSARTx->USART_CR2 & (1 << USART_CR2_LBDIE)) )
	{
		// Interrupt happened because of LIN break detection

		// Clear the PE bit
		pHandle->pUSARTx->USART_SR &= ~(1 << USART_SR_LBD);

		// Notify the use application
		USART_ApplicationEventCallback(pHandle, USART_ERROR_LBD);
	}

	(void)temp;
}

/*
 * Other Peripheral Control APIs
 */

/*****************************************************************************
 * @fn			- USART_PeripheralControl
 *
 * @brief		- This function enables or disables a given USART Peripheral
 *
 * @param[in]	- *pUSARTx: Base Address of USARTx Port
 * @param[in]	- EnorDi: indicates Enable or Disable options
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		-
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);
	}
	else if(EnorDi == DISABLE)
	{
		pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*****************************************************************************
 * @fn			- I2C_GetStatusFlag
 *
 * @brief		- This function returns status of a given USART flag
 *
 * @param[in]	- *pUSARTx: Base Address of USARTx Port
 * @param[in]	- flag: indicates a USART flag
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- flag status
 *
 * @Note		-
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	return ((pUSARTx->USART_SR >> FlagName) & 1);
}

/*****************************************************************************
 * @fn			- USART_ClearFlag
 *
 * @brief		- This function clear a given USART flag of a given USART Peripheral
 *
 * @param[in]	- *pUSARTx: Base Address of USARTx Port
 * @param[in]	- flag: indicates flags
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		-
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->USART_SR &= ~(1 << StatusFlagName);
}

/*
 * Application callback
 */

/*****************************************************************************
 * @fn			- USART_ApplicationEventCallback
 *
 * @brief		- This is a weak function implementation for Application events
 *
 * @param[in]	- *pUSARTHandle: pointer to USART Handle structure
 * @param[in]	- AppEv : Indicates event type that happened
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- this is a weak implementation. User should re-implement it depends its application
 */
__attribute((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{

}

static void USART_ConfigureBaudRate(USART_Handle_t *pHandle)
{
	uint32_t PclkValue;
	float USARTDIV;
	uint8_t DIV_Fraction;
	uint16_t DIV_Mantissa;

	// Check that the which bus the current USART unit hanging on
	if(pHandle->pUSARTx == USART2 || pHandle->pUSARTx == USART3 || pHandle->pUSARTx == UART4 || pHandle->pUSARTx == UART5)
	{
		// Hanging on APB2 bus, so get pclk2
		PclkValue = RCC_GetPCLK1Value();
	}
	else if( pHandle->pUSARTx == USART1 || pHandle->pUSARTx == USART6 )
	{
		// Hanging on APB1 bus, so get pclk1
		PclkValue = RCC_GetPCLK2Value();
	}

	// Check OVER8 value for USARDIV calculation
	if(pHandle->USART_Config.USART_OVER8Sel == USART_CNFG_OVER8_SET)
	{
		// Oversampling by 8 is using
		USARTDIV = (float)PclkValue / (float)(8*pHandle->USART_Config.USART_BaudRate);
	}
	else
	{
		// Oversampling by 16 is using
		USARTDIV = (float)PclkValue / (float)(16*pHandle->USART_Config.USART_BaudRate);
	}

	USARTDIV *= 100;

	DIV_Mantissa = USARTDIV / 100;

	DIV_Fraction = USARTDIV - (DIV_Mantissa * 100);

	// Check OVER8 value for writing USARDIV value to BRR
	if(pHandle->USART_Config.USART_OVER8Sel == USART_CNFG_OVER8_SET)
	{
		// 3 bits of DIV_Fravrion area is available for Fraction part of BRR, 4. bit must be clear
		DIV_Fraction = ((DIV_Fraction * 8) + 50) / 100;
		pHandle->pUSARTx->USART_BRR |= (DIV_Fraction & 0x07);
	}
	else
	{
		// 4 bits of DIV_Fravrion area is available for Fraction part of BRR
		DIV_Fraction = ((DIV_Fraction * 16) + 50) / 100;
		pHandle->pUSARTx->USART_BRR |= (DIV_Fraction & 0xF);
	}
	pHandle->pUSARTx->USART_BRR |= (DIV_Mantissa << 4);
}

/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 10 Haz 2021
 *      Author: suley
 */

#include "stm32f407xx_i2c_driver.h"

/*****************************************************************************
 * @fn			- I2C_ApplicationEventCallback
 *
 * @brief		- This is a weak function implementation for Application events
 *
 * @param[in]	- pI2CHandle: pointer to I2C Handle structure
 * @param[in]	- EvType : Indicates event type that happened
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- this is a weak implementation. User should re-implement it depends its application
 */
__attribute((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t EvType)
{

}

/*****************************************************************************
 * @fn			- GenerateStartCondition
 *
 * @brief		- This function generates the START Condition by manipulating START bit (7th bit) of CR1 Register for given I2C Peripheral
 *
 * @param[in]	- pI2Cx: Base Address of I2Cx Port
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- static to this file
 */

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

/*****************************************************************************
 * @fn			- GenerateStopCondition
 *
 * @brief		- This function generates the STOP Condition by manipulating START bit (7th bit) of CR1 Register for given I2C Peripheral
 *
 * @param[in]	- pI2Cx: Base Address of I2Cx Port
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- static to this file
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}


/*****************************************************************************
 * @fn			- I2C_ExecuteAddressPhase
 *
 * @brief		- This function executes the address phase which is a part of I2C Communication
 *
 * @param[in]	- pI2Cx: Base Address of I2Cx Port
 * @param[in]	- SlaveAddr: I2C Slave Device Address
 * @param[in]	- RorW: Indicates Read/Write bit of Address Phase (LSB)
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- static to this file
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, _Bool RorW)
{
	SlaveAddr = (SlaveAddr << 1);
	if(RorW == I2C_TRANSMIT)
		SlaveAddr &= ~1;
	else if(RorW == I2C_RECEIVE)
		SlaveAddr |= 1;
	pI2Cx->I2C_DR = SlaveAddr;
}

/*****************************************************************************
 * @fn			- I2C_ClearADDRFlag
 *
 * @brief		- This function clears the ADDR flag for given I2C Peripheral
 *
 * @param[in]	- pI2Cx: Base Address of I2Cx Port
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- static to this file
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	volatile uint32_t dummyread;
	dummyread = pI2Cx->I2C_SR1;
	dummyread = pI2Cx->I2C_SR2;
	(void)dummyread;
}

/*
 * Peripheral Clock Setup
 */

/*****************************************************************************
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- This function Enables or Disables I2C Clock for given port
 *
 * @param[in]	- pI2Cx: Base Address of I2Cx Port
 * @param[in]	- EnorDi: Indicates Enabling or Disabling (macros)
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

/*****************************************************************************
 * @fn			- I2C_DeInit
 *
 * @brief		- This function de-initializes the I2C for given port
 *
 * @param[in]	- pI2Cx: Base Address of I2Cx Port
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_RESET();
	}
}

/*****************************************************************************
 * @fn			- I2C_Init
 *
 * @brief		- This function initializes the I2C for given port
 *
 * @param[in]	- pI2CHandle: Includes Register Definition Structures and Configuration Structures of I2C Unit
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	volatile uint32_t tempreg = 0;

	// ACK Control Bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2C->I2C_CR1 |= tempreg;

	// Configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2C->I2C_CR2 |= (tempreg & 0x3F);

	// Program the device own address
	tempreg = 0;
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14); // 14th bit must be set according to reference manual
	pI2CHandle->pI2C->I2C_OAR1 |= tempreg;

	// CCR Configuration
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCLSPEED_SM)
	{
		// Mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg = (ccr_value & 0xFFF);
	}
	else
	{
		// Mode is fast mode
		tempreg |= (1 << 15); // Select FM
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2C->I2C_CCR = tempreg;

	// TRISE Configuration
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCLSPEED_SM)
	{
		// Mode is standart mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// Mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2C->I2C_TRISE = (tempreg & 0x3F);
}

/*****************************************************************************
 * @fn			- I2C_MasterSendData
 *
 * @brief		- This function handles sending data in master mode
 *
 * @param[in]	- pI2CHandle: Includes Register Definition Structures and Configuration Structures of I2C Unit
 * @param[in]	- pTxbuffer	: Transmission buffer
 * @param[in]	- Len		: Bytes of data that will transmit
 * @param[in]	- SlaveAddr	: Address of I2C Slave Device
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, _Bool Sr)
{
	// 1. Generate the START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2C);

	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetStatusFlag(pI2CHandle->pI2C, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2C, SlaveAddr, I2C_TRANSMIT);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetStatusFlag(pI2CHandle->pI2C, I2C_FLAG_ADDR));

	// 5. Clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2C);

	// 6. Send the data until Len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetStatusFlag(pI2CHandle->pI2C, I2C_FLAG_TXE)); // Wait till TXE set
		pI2CHandle->pI2C->I2C_DR = *pTxbuffer;
		pTxbuffer ++;
		Len --;
	}

	// 7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP Condition
	// Note: TxE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to low)
	while(! I2C_GetStatusFlag(pI2CHandle->pI2C, I2C_FLAG_TXE));
	while(! I2C_GetStatusFlag(pI2CHandle->pI2C, I2C_FLAG_BTF));

	// 8. Generate STOP condition and master need to wait for the completion of stop condition.
	// Note: Generating STOP, automatically clears the BTF
	if(Sr == I2C_SR_NO)
		I2C_GenerateStopCondition(pI2CHandle->pI2C);
}

/*****************************************************************************
 * @fn			- I2C_MasterReceiveData
 *
 * @brief		- This function handles reveiving data in master mode
 *
 * @param[in]	- pI2CHandle: Includes Register Definition Structures and Configuration Structures of I2C Unit
 * @param[in]	- pRxbuffer	: Receive buffer
 * @param[in]	- Len		: Bytes of data that will transmit
 * @param[in]	- SlaveAddr	: Address of I2C Slave Device
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_MasterReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, _Bool Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in SR1
	while(! I2C_GetStatusFlag(pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of slave with r/w bit set to r(1) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2Cx, SlaveAddr, I2C_RECEIVE);

	// 4. Wait until address phase is completed by checking ADDR flag in the SR1
	while(! I2C_GetStatusFlag(pI2Cx, I2C_FLAG_ADDR));

	// procedure to read only 1 byte from slave
	if(Len == 1)
	{
		// Disable ACKing
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);

		// Clear ADDR Flag
		I2C_ClearADDRFlag(pI2Cx);

		// Wait for RXNE set
		while(! I2C_GetStatusFlag(pI2Cx, I2C_FLAG_RXNE));

		// Generate STOP Condition
		if(Sr == I2C_SR_NO)
			I2C_GenerateStopCondition(pI2Cx);

		// Read data in to buffer
		*pRxbuffer = (uint8_t)(pI2Cx->I2C_DR & 0xFF);
	}

	// procedure to read data from slave when Len>1
	if(Len > 1)
	{
		// Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2Cx);

		// Read the data until Len becomes zero
		for(uint32_t i=Len ; i > 0 ; i--)
		{
			// wait until RxNE becomes 1
			while(! I2C_GetStatusFlag(pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) // If last 2 are reaming
			{
				// Clear the ACK bit
				pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);

				// Generate STOP condition
				if(Sr == I2C_SR_NO)
					I2C_GenerateStopCondition(pI2Cx);
			}

			// Read the data from data register in to buffer
			*pRxbuffer = (uint8_t)(pI2Cx->I2C_DR & 0xFF);

			// Increment the buffer address
			pRxbuffer ++;
		}
	}

	// Re-Enable ACKing
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
}



/*****************************************************************************
 * @fn			- I2C_GetStatusFlag
 *
 * @brief		- This function returns status of a given I2C flag
 *
 * @param[in]	- pI2Cx: Base Address of I2Cx Port
 * @param[in]	- flag: indicates flags
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- flag status
 *
 * @Note		-
 */
_Bool I2C_GetStatusFlag(I2C_RegDef_t *pI2Cx, uint8_t flag)
{
	_Bool status;

	if(flag < 16)
		status = (pI2Cx->I2C_SR1 >> flag) & 1;
	else
		status = (pI2Cx->I2C_SR2 >> (flag-16)) & 1;

	return status;
}

/*****************************************************************************
 * @fn			- I2C_PeripheralControl
 *
 * @brief		- This function enables or disables a given I2C Peripheral
 *
 * @param[in]	- pI2Cx: Base Address of I2Cx Port
 * @param[in]	- EnorDi: indicates Enable or Disable options
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		-
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, _Bool EorD)
{
	if(EorD == ENABLE)
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	else
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
}

/*****************************************************************************
 * @fn			- IRQInterruptConfig
 *
 * @brief		- This function makes interrupt configuration for i2c peripheral
 *
 * @param[in]	- IRQNumber: Indicates IRQ Number
 * @param[in]	- EnorDi: indicates Enable or Disable options
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		-
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	volatile uint8_t temp_div, temp_mod;
	temp_div = IRQNumber / 32;
	temp_mod = IRQNumber % 32;

	if(EnorDi == ENABLE)
		NVIC_ISER[temp_div] |= (1 << temp_mod);
	else if(EnorDi == DISABLE)
		NVIC_ICER[temp_div] |= (1 << temp_mod);
}

/*****************************************************************************
 * @fn			- I2C_IRQPriorityConfig
 *
 * @brief		- This function configures the priority a interrupt
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	volatile uint8_t temp_div, temp_mod;
	temp_div = IRQNumber / 4;
	temp_mod = IRQNumber % 4;

	IRQPriority = (IRQPriority << 4) & 0xF0; // This is MCU Specific calculation. In STM32F4xx MCU there is only 4 implemented priority bits.

	NVIC_IPR[temp_div] &= ~(0xFF << (temp_mod * 8));
	NVIC_IPR[temp_div] |= ((uint8_t)IRQPriority << (temp_mod * 8));

}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - I2C data send function with using i2c interrupts
 *
 * @param[in]         - *pI2CHandle	: pointer to I2C Handle Structure
 * @param[in]         -	*pTxBuffer	: pointer to Transmission Buffer
 * @param[in]         -	Len			: Length information of data that will send
 * @param[in]         -	SlaveAddr	: I2C slave device address
 * @param[in]         -	Sr			: Indicates that repeated start active or not
 *
 * @return            - Current I2C state
 *
 * @Note              -

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_STATE_TX) && (busystate != I2C_STATE_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_STATE_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2C);


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2C->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2C->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);


		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2C->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             - I2C data receive function with using i2c interrupts
 *
 * @param[in]         - *pI2CHandle	: pointer to I2C Handle Structure
 * @param[in]         -	*pRxBuffer	: pointer to Receive Buffer
 * @param[in]         -	Len			: Length information of data that will receive
 * @param[in]         -	SlaveAddr	: I2C slave device address
 * @param[in]         -	Sr			: Indicates that repeated start active or not
 *
 * @return            - Current I2C state
 *
 * @Note              -

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_STATE_TX) && (busystate != I2C_STATE_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_STATE_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2C);


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2C->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);


		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2C->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);


		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2C->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

void I2C_CloseTransfer(I2C_Handle_t *pI2CHandle, _Bool TxorRx)
{
	// First we must make sure that interrupts are disabled
	pI2CHandle->pI2C->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2C->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	if(TxorRx == I2C_TRANSMIT)
	{
		// Close operation for Transmit
		pI2CHandle->TxRxState = I2C_STATE_READY;
		pI2CHandle->pTxBuffer = NULL;
		pI2CHandle->TxLen = 0;
	}
	else
	{
		// Close operation for Receive
		pI2CHandle->TxRxState = I2C_STATE_READY;
		pI2CHandle->pRxBuffer = NULL;
		pI2CHandle->RxLen = 0;
		pI2CHandle->RxSize = 0;

		// Enable ACKing (Because we disabled it when communication was ending)
		if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN)
			pI2CHandle->pI2C->I2C_CR1 |= (1 << I2C_CR1_ACK);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterHandleRXNEInterrupt
 *
 * @brief             - This function handles RxNE Interruptd for I2C Master Mode
 *
 * @param[in]         - *pI2CHandle	: pointer to I2C Handle Structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - Subfunction of I2C_EV_IRQHandling

 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		// Read to Rx Buffer
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2C->I2C_DR;
		pI2CHandle->RxLen--;
	}
	else if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			// Disable ACKing
			pI2CHandle->pI2C->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
		}
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2C->I2C_DR;
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
	}

	if(pI2CHandle->RxLen == 0)
	{
		// Close the I2C data reception and notify the application
		// 1. Generate Stop Condition
		if(pI2CHandle->Sr == I2C_SR_NO)
			I2C_GenerateStopCondition(pI2CHandle->pI2C);

		// 2. Close the I2C Rx
		I2C_CloseTransfer(pI2CHandle, I2C_RECEIVE);

		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterHandleTXEInterrupt
 *
 * @brief             - This function handles TXE Interruptd for I2C Master Mode
 *
 * @param[in]         - *pI2CHandle	: pointer to I2C Handle Structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - Subfunction of I2C_EV_IRQHandling

 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		// Load the data into DR
		pI2CHandle->pI2C->I2C_DR = *pI2CHandle->pTxBuffer;

		// Decrement the Tx Length
		pI2CHandle->TxLen--;

		// Increment the Tx Buffer
		pI2CHandle->pTxBuffer++;
	}
}

/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             - Interrupt Service Routine for I2C Event Interrupts (ISR)
 *
 * @param[in]         - *pI2CHandle	: pointer to I2C Handle Structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - Interrupt handling for different I2C events (refer SR1)

 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device
	uint8_t temp1, temp2, temp3;

	temp1 = (pI2CHandle->pI2C->I2C_CR2 >> I2C_CR2_ITEVTEN) & 1;
	temp2 = (pI2CHandle->pI2C->I2C_CR2 >> I2C_CR2_ITBUFEN) & 1;

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	temp3 = (pI2CHandle->pI2C->I2C_SR1 >> I2C_SR1_SB) & 1;
	if(temp1 && temp3)
	{
		// The interrupt generated because of SB event
		// This block is never executed in slave mode because of SB is always zero in Slave Mode
		// Next Step is executingt the Address Phase
		if(pI2CHandle->TxRxState == I2C_STATE_TX)
			I2C_ExecuteAddressPhase(pI2CHandle->pI2C, pI2CHandle->DevAddr, I2C_TRANSMIT);
		else if(pI2CHandle->TxRxState == I2C_STATE_RX)
			I2C_ExecuteAddressPhase(pI2CHandle->pI2C, pI2CHandle->DevAddr, I2C_RECEIVE);
	}

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = (pI2CHandle->pI2C->I2C_SR1 >> I2C_SR1_ADDR) & 1;
	if(temp1 && temp3)
	{
		// The interrupt generated because of ADDR event that mean address phase successfully ACKed
		// Then Clear ADDR
		if((pI2CHandle->TxRxState == I2C_STATE_RX) && (pI2CHandle->RxSize == 1))
			{
				// Disable ACKing
				pI2CHandle->pI2C->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
				I2C_ClearADDRFlag(pI2CHandle->pI2C);
			}
		else
			I2C_ClearADDRFlag(pI2CHandle->pI2C);
	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = (pI2CHandle->pI2C->I2C_SR1 >> I2C_SR1_BTF) & 1;
	if(temp1 && temp3)
	{
		// BTF
		// Generate Stop Condition
		if(pI2CHandle->TxRxState == I2C_STATE_TX)
		{
			// Make sure that TXE is also set.
			if(pI2CHandle->pI2C->I2C_SR1 & (1 << I2C_SR1_TxE))
			{
				// BTS=1, TxE=1

				if(pI2CHandle->TxLen == 0)
				{
					// 1. Generate STOP Condition
					if(pI2CHandle->Sr == I2C_SR_NO)
						I2C_GenerateStopCondition(pI2CHandle->pI2C);

					// 2. Reset all the memeber element of the handle structure
					I2C_CloseTransfer(pI2CHandle, I2C_TRANSMIT);

					// 3. Notify the application about transmission completed
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}

	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = (pI2CHandle->pI2C->I2C_SR1 >> I2C_SR1_STOPF) & 1;
	if(temp1 && temp3)
	{
		// STOPF flag is set
		// Clear the STOPF Flag (1-Read from SR1 (already done) 2-Write to CR1)
		pI2CHandle->pI2C->I2C_CR1 |= 0x0000; // This not affect the CR1, just dummy read

		// Notify the Application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle For interrupt generated by TXE event
	temp3 = (pI2CHandle->pI2C->I2C_SR1 >> I2C_SR1_TxE) & 1;
	if(temp1 && temp2 && temp3)
	{
		// TXE is set

		// First we must confirm that the device is master or slave
		if(pI2CHandle->pI2C->I2C_SR2 & (1 << I2C_SR2_MSL)) // If Masterif(pI2CHandle->TxRxState == I2C_STATE_TX)
		{
			if(pI2CHandle->TxRxState == I2C_STATE_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// Slave Mode
			// First make sure that it is in the transmit mode
			if(pI2CHandle->pI2C->I2C_SR2 & (1 << I2C_SR2_TRA))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		}
	}

	//6. Handle For interrupt generated by RXNE event
	temp3 = (pI2CHandle->pI2C->I2C_SR1 >> I2C_SR1_RxNE) & 1;
	if(temp1 && temp2 && temp3)
	{
		// RXNE is set

		// First check device mode
		if(pI2CHandle->pI2C->I2C_SR2 & (1 << I2C_SR2_MSL)) // If device is master
		{
			if(pI2CHandle->TxRxState == I2C_STATE_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// Slave Mode
			// First make sure that it is in the receive mode
			if(!(pI2CHandle->pI2C->I2C_SR2 & (1 << I2C_SR2_TRA)))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}


}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             - Interrupt Service Routine for I2C Error Interrupts (ISR)
 *
 * @param[in]         - *pI2CHandle	: pointer to I2C Handle Structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -

 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2C->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2C->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2C->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	    I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2C->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2C->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2C->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2C->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2C->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2C->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2C->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2C->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

/*********************************************************************
 * @fn      		  - I2C_SlaveSendData
 *
 * @brief             - This functions sends 1 byte data via I2C peripheral in Slave mode
 *
 * @param[in]         - *pI2Cx	: Base address of an I2C peripheral
 * @param[in]         - data : 1 byte data that must transmit
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -

 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->I2C_DR = data;
}

/*********************************************************************
 * @fn      		  - I2C_SlaveReceiveData
 *
 * @brief             - This functions recieves 1 byte data via I2C peripheral in Slave mode
 *
 * @param[in]         - *pI2Cx	: Base address of an I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - 1 byte data that must receive
 *
 * @Note              -

 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->I2C_DR;
}

/*********************************************************************
 * @fn      		  - I2C_SlaveCallbackEventsControl
 *
 * @brief             - This function enables I2C interrupts for slave mode
 *
 * @param[in]         - *pI2Cx	: Base address of an I2C peripheral
 * @param[in]         - EnorDi : Indicates enable or diable
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -

 */
void I2C_SlaveCallbackEventsControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}

/*****************************************************************************
 * @fn			- I2C_ACKControl
 *
 * @brief		- This function controls ACKing of given I2C peripheral
 *
 * @param[in]	- *pI2Cx	: Base address of an I2C peripheral
 * @param[in]	- EnorDi : Indicates enable or diable
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

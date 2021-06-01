/*
 * spi_driver.h
 *
 *  Created on: May 1, 2021
 *      Author: suleyman
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;		/* Check for options: @DeviceMode */
	uint8_t SPI_BusConfig;		/* Check for options: @BusConfig */
	uint8_t SPI_SclkSpeed;		/* Check for options: @SclkSpeed */
	uint8_t SPI_DFF;			/* Check for options: @DFF */
	uint8_t SPI_CPOL;			/* Check for options: @CPOL */
	uint8_t SPI_CPHA;			/* Check for options: @CPHA */
	uint8_t SPI_SSM;			/* Check for options: @SSM */
}SPI_Config_t;

/* Options for DeviceMode
 * @DeviceMode
 */
#define SPI_MODE_SLAVE			0
#define SPI_MODE_MASTER			1

/* Options for BusConfig
 * @BusConfig
 */
#define SPI_BUS_FULLDUBLEX				0
#define SPI_BUS_HALFDUBLEX				1
#define SPI_BUS_SIMPLEX_TXONLY			SPI_BUS_FULLDUBLEX	/* Same settings with FD */
#define SPI_BUS_SIMPLEX_RXONLY			2

/* Options for SclkSpeed
 * @SclkSpeed
 */
#define SPI_SCLKSPEED_DIV2		0
#define SPI_SCLKSPEED_DIV4		1
#define SPI_SCLKSPEED_DIV8		2
#define SPI_SCLKSPEED_DIV16		3
#define SPI_SCLKSPEED_DIV32		4
#define SPI_SCLKSPEED_DIV64		5
#define SPI_SCLKSPEED_DIV128	6
#define SPI_SCLKSPEED_DIV256	7

/* Options for DFF
 * @DFF
 */
#define SPI_DFF_8BIT	0
#define SPI_DFF_16BIT	1

/* Options for CPOL
 * @CPOL
 */
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1

/* Options for CPHA
 * @CPHA
 */
#define SPI_CPHA_LOW	0
#define SPI_CPHA_HIGH	1

/* Options for CPHA
 * @CPHA
 */
#define SPI_SSM_SOFTWARE_DI		0
#define SPI_SSM_SOFTWARE_EN		1

typedef struct
{
	SPI_RegDef_t *pSPIx;		/* This hold the base address of SPIx(x:1,2,3) peripheral */
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;			/* To store the app. Tx buffer address */
	uint8_t *pRxBuffer;			/* To store the app. Rx buffer address */
	uint32_t TxLen;				/* To store Tx len */
	uint32_t RxLen;				/* To store Rx len */
	uint8_t TxState;			/* To store Tx State */
	uint8_t RxState;			/* To store Rx State */
}SPI_Handle_t;

/*********************************************************************************
 * 								APIs supported by this driver
 * 		For more information about APIs check the function definitions
 **********************************************************************************/
/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 *  Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlag(SPI_RegDef_t *pSPIx, uint8_t flag);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * AplicationEventCallback Function
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);


/* SPI SR FLAGS */   // @spi_flags
#define SPI_FLAG_RXNE		0
#define SPI_FLAG_TXE		1
#define SPI_FLAG_CHSIDE		2
#define SPI_FLAG_UDR		3
#define SPI_FLAG_CRCERR		4
#define SPI_FLAG_MODF		5
#define SPI_FLAG_OVR		6
#define SPI_FLAG_BSY		7
#define SPI_FLAG_FRE		8

 /* SPI Application States for Interrupt based APIs */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

 /* Possible SPI Application Events */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

#endif /* INC_SPI_DRIVER_H_ */

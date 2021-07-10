/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: 4 Tem 2021
 *      Author: suley
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

typedef struct{
	uint8_t USART_Mode;				/* Configures USART Mode - check for options: @usart_mode  */
	uint8_t USART_WordLen;			/* One packet data len that used in usart communication -in bits- - check for options: @usart_data_len */
	uint8_t USART_StopLen;			/* stop bit length in usart communication -  check for options: @usart_stopbit_len  */
	uint32_t USART_BaudRate;		/* Baud Rate Value -  check for options: @usart_baudrate */
	uint8_t USART_ParityControl;	/* Parity bit EN or DI -  check for options: @usart_parity  */
	uint8_t USART_HWFlowControl;	/* Hardware Flow Control EN or DI (not configurable for uart units) - check for options: @usart_hardwareflowcontrol */
	uint8_t USART_OVER8Sel;			/* OVER8 set or reset, (Decides oversampling rate) - check for options: @usar_over8sel */
}USART_Config_t;

typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t USART_TxStatus;
	uint8_t USART_RxStatus;
}USART_Handle_t;

#define USART_TX	0
#define USART_RX	1

/*
 * USART Configuration Macros
 */

#define USART_CNFG_MODE_TX		0	/* @usart_mode */
#define USART_CNFG_MODE_RX		1
#define USART_CNFG_MODE_TXRX		2

#define USART_CNFG_BAUD_1200		1200	/* @usart_baudrate */
#define USART_CNFG_BAUD_2400		2400
#define USART_CNFG_BAUD_9600		9600
#define USART_CNFG_BAUD_19200		19200
#define USART_CNFG_BAUD_38400		38400
#define USART_CNFG_BAUD_57600		57600
#define USART_CNFG_BAUD_115200		115200
#define USART_CNFG_BAUD_230400		230400
#define USART_CNFG_BAUD_460800		460800
#define USART_CNFG_BAUD_921600		921600
#define USART_CNFG_BAUD_2M			2000000
#define USART_CNFG_BAUD_3M			3000000

#define USART_CNFG_WORDLEN_8	0	/* @usart_data_len */
#define USART_CNFG_WORDLEN_9	1

#define USART_CNFG_STOPBITLEN_1		0	/* @usart_stopbit_len */
#define USART_CNFG_STOPBITLEN0_5	1
#define USART_CNFG_STOPBITLEN1_5	2
#define USART_CNFG_STOPBITLEN_2		3

#define USART_CNFG_PARITY_EN_ODD		0	/* @usart_parity */
#define USART_CNFG_PARITY_EN_EVEN		1
#define USART_CNFG_PARITY_DI			2

#define USART_CNFG_HWFLOWCONTROL_NONE		0	/* @usart_hardwareflowcontrol */
#define USART_CNFG_HWFLOWCONTROL_CTS		1
#define USART_CNFG_HWFLOWCONTROL_RTS		2
#define USART_CNFG_HWFLOWCONTROL_CTSRTS		3

#define USART_CNFG_OVER8_RESET	0	/* @usar_over8sel */
#define USART_CNFG_OVER8_SET	1

/*
 * USART State Macros for IT Apllication
 */
#define USART_STATUS_TX_READY		0
#define USART_STATUS_TX_BUSY		1
#define USART_STATUS_RX_READY		0
#define USART_STATUS_RX_BUSY		1

/*
 * USART Flag Macros
 */
#define USART_FLAG_PE		0
#define USART_FLAG_FE		1
#define USART_FLAG_NF		2
#define USART_FLAG_ORE		3
#define USART_FLAG_IDLE		4
#define USART_FLAG_RXNE		5
#define USART_FLAG_TC		6
#define USART_FLAG_TXE		7
#define USART_FLAG_LBD		8
#define USART_FLAG_CTS		9

/*
 * Callback events and errors
 */
#define USART_EVENT_TX_CMPLT	0
#define USART_EVENT_RX_CMPLT	1
#define USART_EVENT_CTS			2
#define USART_EVENT_IDLE		3
#define USART_ERROR_ORE			4
#define USART_ERROR_FE			5
#define USART_ERROR_NF			6
#define USART_ERROR_PE			7
#define USART_ERROR_LBD			8

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */

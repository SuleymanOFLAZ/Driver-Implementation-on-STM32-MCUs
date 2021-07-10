/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 10 Haz 2021
 *      Author: suleyman
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint32_t I2C_SCLSpeed;			/* I2C SCL Speed. For options check: @I2C_SCLSpeed */
	uint8_t I2C_DeviceAddress;		/* I2C Device Address */
	uint8_t I2C_ACKControl;			/* I2C ACK Control. For options check: @I2C_SCLSpeed */
	uint8_t I2C_FMDutyCycle;		/* I2C Fast Mode Duty Cycle. For options check: @I2C_FMDutyCycle */
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2C;				/* I2C Peripheral Base Address */
	I2C_Config_t I2C_Config;		/* I2C Configuration Structure */
	uint8_t		*pTxBuffer;			/* To store application Transmission Buffer Address */
	uint8_t		*pRxBuffer;			/* To store application Receive Buffer Address */
	uint32_t	TxLen;				/* To store Tx length */
	uint32_t	RxLen;				/* To store Rx length */
	uint8_t		TxRxState;			/* To store communication state, look for options: @i2c_application_states */
	uint8_t		DevAddr;			/* To store Slave/Device Address */
	uint32_t	RxSize;				/* To store Rx Size */
	uint8_t		Sr;					/* To store Repeated Start state */
}I2C_Handle_t;

 // I2C Application States @i2c_application_states
#define I2C_STATE_READY		0
#define I2C_STATE_TX		1
#define I2C_STATE_RX		2

#define I2C_SCLSPEED_SM 	100000  /* @I2C_SCLSpeed */
#define I2C_SCLSPEED_FM4K 	400000
#define I2C_SCLSPEED_FM2K 	200000

#define I2C_ACK_EN 0	/* @I2C_ACKControl */
#define I2C_ACK_DI 1

#define I2C_FM_DUTY_2		0	/* @I2C_FMDutyCycle */
#define I2C_FM_DUTY_16_9	1

#define I2C_TRANSMIT 0
#define I2C_RECEIVE  1

/*
 * I2C Flags
 */
#define I2C_FLAG_SB			0
#define I2C_FLAG_ADDR		1
#define I2C_FLAG_BTF		2
#define I2C_FLAG_ADD10		3
#define I2C_FLAG_STOPF		4
#define I2C_FLAG_RXNE		6
#define I2C_FLAG_TXE		7
#define I2C_FLAG_BERR		8
#define I2C_FLAG_ARLO		9
#define I2C_FLAG_AF			10
#define I2C_FLAG_OVR		11
#define I2C_FLAG_PECERR		12
#define I2C_FLAG_TIMEOUT	14
#define I2C_FLAG_SMBALERT	15
#define I2C_FLAG_MSL		16
#define I2C_FLAG_BUSY		17
#define I2C_FLAG_TRA		18
#define I2C_FLAG_GENCALL	20
#define I2C_FLAG_SMBDEFAULT	21
#define I2C_FLAG_SMBHOST	22
#define I2C_FLAG_DUALF		23

/*
 * other macros
 */
#define I2C_SR_YES	1
#define I2C_SR_NO	0

/*
 * I2C Application event and error macros
 */
#define I2C_EV_TX_CMPLT 0
#define I2C_EV_RX_CMPLT 1
#define I2C_EV_STOP		2
#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7
#define I2C_EV_DATA_REQ 8
#define I2C_EV_DATA_RCV 9

/*
 * API Prototypes
 */

 // Initialization - DeInitialization, Clock Control APIs

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

 // Send and Reveive Data
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, _Bool Sr);
void I2C_MasterReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, _Bool Sr);
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

 // IRQ Configuration and ISR Handling APIs
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_SlaveCallbackEventsControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

 // Other APIs
_Bool I2C_GetStatusFlag(I2C_RegDef_t *pI2Cx, uint8_t flag);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, _Bool EorD);
void I2C_CloseTransfer(I2C_Handle_t *pI2CHandle, _Bool TxorRx);
void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */

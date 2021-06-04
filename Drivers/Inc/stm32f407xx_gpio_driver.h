/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Apr 23, 2021
 *      Author: suley
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

 /* A Structure that holds pin configuration settings */
typedef struct
{
	uint8_t GPIO_PinNumber;				/* check for options: @Pin_Numbers */
	uint8_t GPIO_PinMode;				/* check for options: @Pin_Mode */
	uint8_t GPIO_PinSpeed;				/* check for options: @Pin_Speed */
	uint8_t GPIO_PinPuPdControl;		/* check for options: @Pin_PUPD */
	uint8_t GPIO_PinOPType;				/* check for options: @Pin_OType */
	uint8_t GPIO_PinAltFunMode;			/* check for options: @Pin_AltFunMode */
}GPIO_PinConfig_t;

 /* A Structure for Pin Configuration Operation */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/* Address of the GPIO port that pin is belong */
	GPIO_PinConfig_t GPIO_PinConfig;	/* Pin Configuration Settings */
}GPIO_Handle_t;

 /* PinNumber Options
  * @Pin_Numbers
  */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15

/* PinMode Options
 * @Pin_Mode
 */
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_RE			4
#define GPIO_MODE_IT_FE			5
#define GPIO_MODE_IT_RFE		6

/* PinSpeed Options
 * @Pin_Speed
 */
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_HIGH				2
#define GPIO_SPEED_WHIGH			3

/* PinPuPdControl Options
 * @Pin_PUPD
 */
#define GPIO_PUPD_NO		0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2

/* PinOPType Options
 * @Pin_OType
 */
#define GPIO_OTYPE_PUSHPULL		0
#define GPIO_OTYPE_OPENDRAIN		1

/* PinAltFunMode Options
 * @Pin_AltFunMode
 */
#define GPIO_ALTFUN_AF0			0
#define GPIO_ALTFUN_AF1			1
#define GPIO_ALTFUN_AF2			2
#define GPIO_ALTFUN_AF3			3
#define GPIO_ALTFUN_AF4			4
#define GPIO_ALTFUN_AF5			5
#define GPIO_ALTFUN_AF6			6
#define GPIO_ALTFUN_AF7			7
#define GPIO_ALTFUN_AF8			8
#define GPIO_ALTFUN_AF9			9
#define GPIO_ALTFUN_AF10		10
#define GPIO_ALTFUN_AF11		11
#define GPIO_ALTFUN_AF12		12
#define GPIO_ALTFUN_AF13		13
#define GPIO_ALTFUN_AF14		14
#define GPIO_ALTFUN_AF15		15

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * GPIO EXTI IRQ Numbers
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

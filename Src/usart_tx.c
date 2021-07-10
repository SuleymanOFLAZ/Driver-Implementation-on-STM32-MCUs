/*
 * i2c_mastersenddata.c
 *
 *  Created on: 21 Haz 2021
 *      Author: suley
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

// extern void initialise_monitor_handles();

#define I2C_MY_ADDR 0x61
#define SLAVE_ADDR  0x68

#define I2C_COMMAND_GETLENINFO 	0x51
#define I2C_COMMAND_FULLREAD	0x52

_Bool user_button = 0;

USART_Handle_t usart_unit;

//somedata
uint8_t message_buffer[] = "USART TX testing...\n";

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PA2 --> TX
 * PA3 --> RX
 * ALT function mode : 7
 * USART unit: 2
 * Baud rate: 115200bps
 */

void USART2_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOA;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&I2CPins);

	//SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&I2CPins);
}

void USART2_Inits(void)
{
	usart_unit.pUSARTx = USART2;
	usart_unit.USART_Config.USART_BaudRate = USART_CNFG_BAUD_115200;
	usart_unit.USART_Config.USART_HWFlowControl = USART_CNFG_HWFLOWCONTROL_NONE;
	usart_unit.USART_Config.USART_Mode = USART_CNFG_MODE_TX;
	usart_unit.USART_Config.USART_OVER8Sel = USART_CNFG_OVER8_RESET;
	usart_unit.USART_Config.USART_ParityControl = USART_CNFG_PARITY_DI;
	usart_unit.USART_Config.USART_StopLen = USART_CNFG_STOPBITLEN_1;
	usart_unit.USART_Config.USART_WordLen = USART_CNFG_WORDLEN_8;

	USART_Init(&usart_unit);
}



void GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t i2cIntPin;
	memset(&i2cIntPin,0,sizeof(i2cIntPin));

	//this is led gpio configuration
	i2cIntPin.pGPIOx = GPIOA;
	i2cIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	i2cIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RE;
	i2cIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	i2cIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_Init(&i2cIntPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

}

int main(void)
{
	// initialise_monitor_handles();
	// printf("Application is running\n");

	// Enable the Peripheral Clocks
	USART_PeriClockControl(USART2, ENABLE);
	// GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	// GPIO Inits
	USART2_GPIOInits();

	// USART Inits
	USART2_Inits();

	// Interrupt Button Init
	GPIO_InterruptPinInit();

	// Enable I2C Peripheral
	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		if(user_button == 1)
		{
			// Send data via UART
			while(USART_SendData(&usart_unit, message_buffer, sizeof(message_buffer)) == USART_STATUS_TX_BUSY);

			user_button = 0;

			delay();
			delay();
			delay();
		}
	}


	while(1);
	return 0;
}

/* Slave data available interrupt handler */
void EXTI0_IRQHandler(void)
{
	user_button = 1;
	GPIO_IRQHandling(GPIO_PIN_0);
}

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
char *message[3] = {"Interstellar", "Inception", "TENET"};
uint8_t rx_buffer[250];
uint8_t rxCmplt = 0;

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
	GPIO_Handle_t Pins;

	Pins.pGPIOx = GPIOA;
	Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCL
	Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&Pins);

	//SDA
    Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&Pins);
}

void USART2_Inits(void)
{
	usart_unit.pUSARTx = USART2;
	usart_unit.USART_Config.USART_BaudRate = USART_CNFG_BAUD_115200;
	usart_unit.USART_Config.USART_HWFlowControl = USART_CNFG_HWFLOWCONTROL_NONE;
	usart_unit.USART_Config.USART_Mode = USART_CNFG_MODE_TXRX;
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
	uint32_t cnt = 0;

	// IRQ Config
	USART_IRQPriorityConfig(IRQ_USART2, 4);
	USART_IRQInterruptConfig(IRQ_USART2, ENABLE);

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
			// Next message index ; make sure that cnt value doesn't cross 2
			cnt = cnt % 3;

			// First enable the reception in interrupt mode
			while(USART_ReceiveDataIT(&usart_unit, rx_buffer, strlen(message[cnt])) == USART_STATUS_TX_BUSY);

			// Sent the message
			//while(USART_SendDataIT(&usart_unit, (uint8_t *)message[cnt], strlen(message[cnt])) == USART_STATUS_TX_BUSY);
			USART_SendData(&usart_unit, (uint8_t *)message[cnt], strlen(message[cnt]));

			printf("Transmitted: %s\n", message[cnt]);

			// Wait until bites received from arduino
			while(rxCmplt != 1);

			// Make sure the last byte of the received message is null character
			rx_buffer[strlen(message[cnt])] = '\0';

			printf("Received: %s\n", rx_buffer);

			rxCmplt = 0;

			cnt++;

			user_button = 0;
		}
	}


	while(1);
	return 0;
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{
	if(AppEv == USART_EVENT_TX_CMPLT)
	{
		// printf("USART2 TX Completed.\n");
	}
	if(AppEv == USART_EVENT_RX_CMPLT)
	{
		// printf("USART2 RX Completed.\n");
		rxCmplt = 1;
	}
	if(AppEv == USART_EVENT_CTS)
	{
		printf("USART2 CTS.\n");
	}
	if(AppEv == USART_EVENT_IDLE)
	{
		printf("USART2 IDLE.\n");
	}
	if(AppEv == USART_ERROR_FE)
	{
		printf("USART2 ERROR: Farming Error.\n");
	}
	if(AppEv == USART_ERROR_LBD)
	{
		printf("USART2 ERROR: LBD.\n");
	}
	if(AppEv == USART_ERROR_NF)
	{
		printf("USART2 ERROR: Noise detected.\n");
	}
	if(AppEv == USART_ERROR_ORE)
	{
		printf("USART2 ERROR: Overrun Error.\n");
	}
	if(AppEv == USART_ERROR_PE)
	{
		printf("USART2 ERROR: Parity Error.\n");
	}
}

/* Slave data available interrupt handler */
void EXTI0_IRQHandler(void)
{
	user_button = 1;
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
}

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usart_unit);
}


void HardFault_Handler(void)
{
	printf("ERROR \n");
}
void MemManage_Handler(void)
{
	printf("ERROR \n");
}
void BusFault_Handler(void)
{
	printf("ERROR \n");
}
void UsageFault_Handler(void)
{
	printf("ERROR \n");
}

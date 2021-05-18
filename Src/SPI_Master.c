/*
 * SPI_Master.c
 *
 *  Created on: May 4, 2021
 *      Author: suleyman
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

/*
 * 		SPI2:
 * 			SCLK: PB13
 * 			MOSI: PB15
 * 			MISO: PB14
 * 			NSS:  PB12
 */

void InitGPIO(void);
void InitSPI(void);
void InitButton(void);
void SendMessage(void);
void delay(void);


#define NACK 0xA5
#define ACK 0xF5

//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

#define DPIN_9 9

_Bool ButtonInterrupt = 0;
uint8_t CMD_State= 0;
uint8_t CMD_Response = 0;
uint8_t cmd = COMMAND_LED_CTRL;
uint8_t dummy_write = 0xFF;
uint8_t dummy_read = 0xFF;
uint8_t args[2];
uint8_t SPI_RecievedData ;
uint8_t message[] = "Faster than light\0";
uint8_t id[10];

uint8_t ACKorNACK(uint8_t value)
{
	if(value == ACK)
		return 1;
	else
		return 0;
}

int main(void)
{
	/* Init Peripheral */
	InitGPIO();
	InitSPI();
	InitButton();

	while(1)
	{
		if(ButtonInterrupt)
		{
			switch(CMD_State)
			{
			case 1:
				SPI_PeripheralControl(SPI2, ENABLE);

				// send command
				cmd = COMMAND_LED_CTRL;
				SPI_SendData(SPI2, &cmd, 1);

				// dummy read for clearing RXNE bit
				SPI_ReceiveData(SPI2, &dummy_read, 1);

				// send dummy data for receive response
				SPI_SendData(SPI2, &dummy_write, 1);

				// Reveive the response
				SPI_ReceiveData(SPI2, &CMD_Response, 1);

				if(ACKorNACK(CMD_Response))
				{
					args[0] = DPIN_9;
					args[1] = LED_ON;
					SPI_SendData(SPI2, args, 2);
				}

				while( (SPI_GetFlag(SPI2, SPI_FLAG_BSY)) );
				SPI_PeripheralControl(SPI2, DISABLE);
				break;

			case 2:
				SPI_PeripheralControl(SPI2, ENABLE);

				// send command
				cmd = COMMAND_SENSOR_READ;
				SPI_SendData(SPI2, &cmd, 1);

				// dummy read for clearing RXNE bit
				SPI_ReceiveData(SPI2, &dummy_read, 1);

				// send dummy data for receive response
				SPI_SendData(SPI2, &dummy_write, 1);

				// Reveive the response
				SPI_ReceiveData(SPI2, &CMD_Response, 1);

				if(ACKorNACK(CMD_Response))
				{
					args[0] = ANALOG_PIN0;
					SPI_SendData(SPI2, args, 1);
					// dummy read for clearing RXNE bit
					SPI_ReceiveData(SPI2, &dummy_read, 1);

					delay();

					// send dummy data for receive response
					SPI_SendData(SPI2, &dummy_write, 1);

					SPI_ReceiveData(SPI2, &SPI_RecievedData, 1);

					while( (SPI_GetFlag(SPI2, SPI_FLAG_BSY)) );
					SPI_PeripheralControl(SPI2, DISABLE);
				}

				break;

			case 3:
				SPI_PeripheralControl(SPI2, ENABLE);

				// send command
				cmd = COMMAND_LED_READ;
				SPI_SendData(SPI2, &cmd, 1);

				// dummy read for clearing RXNE bit
				SPI_ReceiveData(SPI2, &dummy_read, 1);

				// send dummy data for receive response
				SPI_SendData(SPI2, &dummy_write, 1);

				// Reveive the response
				SPI_ReceiveData(SPI2, &CMD_Response, 1);

				if(ACKorNACK(CMD_Response))
				{
					args[0] = DPIN_9;
					SPI_SendData(SPI2, args, 1);
					// dummy read for clearing RXNE bit
					SPI_ReceiveData(SPI2, &dummy_read, 1);

					delay();

					// send dummy data for receive response
					SPI_SendData(SPI2, &dummy_write, 1);

					SPI_ReceiveData(SPI2, &SPI_RecievedData, 1);

					while( (SPI_GetFlag(SPI2, SPI_FLAG_BSY)) );
					SPI_PeripheralControl(SPI2, DISABLE);
				}
				break;

			case 4:
				SPI_PeripheralControl(SPI2, ENABLE);

				// send command
				cmd = COMMAND_PRINT ;
				SPI_SendData(SPI2, &cmd, 1);

				// dummy read for clearing RXNE bit
				SPI_ReceiveData(SPI2, &dummy_read, 1);

				// send dummy data for receive response
				SPI_SendData(SPI2, &dummy_write, 1);

				// Reveive the response
				SPI_ReceiveData(SPI2, &CMD_Response, 1);

				if(ACKorNACK(CMD_Response))
				{
					args[0] = 2;
					args[1] = 'S';
					args[2] = 'O';
					SPI_SendData(SPI2, args, 3);
					// dummy read for clearing RXNE bit
					SPI_ReceiveData(SPI2, &dummy_read, 1);

					while( (SPI_GetFlag(SPI2, SPI_FLAG_BSY)) );
					SPI_PeripheralControl(SPI2, DISABLE);

				}

				break;

			case 5:
				SPI_PeripheralControl(SPI2, ENABLE);

				// send command
				cmd = COMMAND_ID_READ ;
				SPI_SendData(SPI2, &cmd, 1);

				// dummy read for clearing RXNE bit
				SPI_ReceiveData(SPI2, &dummy_read, 1);

				// send dummy data for receive response
				SPI_SendData(SPI2, &dummy_write, 1);

				// Reveive the response
				SPI_ReceiveData(SPI2, &CMD_Response, 1);

				if(ACKorNACK(CMD_Response))
				{
					delay();

					for(int i=0; i<10 ; i++){
						// send dummy data for receive response
						SPI_SendData(SPI2, &dummy_write, 1);
						SPI_ReceiveData(SPI2, &id[i], 1);
					}

					while( (SPI_GetFlag(SPI2, SPI_FLAG_BSY)) );
					SPI_PeripheralControl(SPI2, DISABLE);
				}

				CMD_State = 0;
				break;
			default:
				break;
			}

			ButtonInterrupt = 0;
		}
	}
	return 0;
}

void InitGPIO(void)
{
	// Enable Peripheral Clock
	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Handle_t GPIO_Pins;
	GPIO_Pins.pGPIOx = GPIOB;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFUN_AF5;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Init GPIO SCK
	GPIO_Init(&GPIO_Pins);

	// Init GPIO MOSI
	GPIO_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&GPIO_Pins);

	// Init GPIO MISO
	GPIO_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&GPIO_Pins);

	// Init GPIO NSS
	GPIO_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&GPIO_Pins);
}

void InitSPI(void)
{
	// Enable Peripheral Clock
	SPI_PeriClockControl(SPI2, ENABLE);

	SPI_Handle_t SPI2_FD;
	SPI2_FD.pSPIx = SPI2;
	SPI2_FD.SPIConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2_FD.SPIConfig.SPI_BusConfig = SPI_BUS_FULLDUBLEX;
	SPI2_FD.SPIConfig.SPI_SclkSpeed = SPI_SCLKSPEED_DIV8;
	SPI2_FD.SPIConfig.SPI_SSM = SPI_SSM_SOFTWARE_DI;
	SPI2_FD.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2_FD.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_FD.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;

	// Init SPI
	SPI_Init(&SPI2_FD);

	SPI_SSOEConfig(SPI2, ENABLE);
}

void InitButton(void)
{
	// Enable Peripheral Clock
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Handle_t GPIO_Pins;
	GPIO_Pins.pGPIOx = GPIOA;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RE;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFUN_AF0;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Init Button
	GPIO_Init(&GPIO_Pins);

	GPIO_IRQConfig(6, 3, ENABLE);
}

void EXTI0_IRQHandler(void)
{
	ButtonInterrupt = 1;
	CMD_State += 1;
	while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
	delay();
	GPIO_IRQHandling(0);
}

void SendMessage(void)
{
	char message[] = "Faster Then LIGHT, One Man ARMY";
	uint8_t message_len = strlen(message);

	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, &message_len, 1);
	while( (SPI_GetFlag(SPI2, SPI_FLAG_BSY)) );
	SPI_SendData(SPI2, (uint8_t *)message, message_len);
	while( (SPI_GetFlag(SPI2, SPI_FLAG_BSY)) );

	SPI_PeripheralControl(SPI2, DISABLE);
}

void delay(void)
{
	for(uint32_t i=0; i< 10000; i++);
}

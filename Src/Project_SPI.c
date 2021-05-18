/*
 * Project_SPI.c
 *
 *  Created on: May 3, 2021
 *      Author: suleyman
 */
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "spi_driver.h"
#include <string.h>

void InitGPIO(void);
void InitSPI(void);

int main(void)
{
	InitGPIO();
	InitSPI();

	SPI_SSIConfig(SPI2, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);
	char message[] = "Hello World!";

	for(int i = 0; i<10; i++)
	SPI_SendData(SPI2, (uint8_t*)message, strlen(message));

	while( !SPI_GetFlag(SPI2, SPI_FLAG_TXE) );
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}

void InitGPIO(void)
{
	// Enable Peripheral Clock
	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Handle_t GPIO_Pins;
	GPIO_Pins.pGPIOx = GPIOB;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFUN_AF5;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	GPIO_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Init GPIO SCK
	GPIO_Init(&GPIO_Pins);

	// Init GPIO MOSI
	GPIO_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
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
	SPI2_FD.SPIConfig.SPI_SclkSpeed = SPI_SCLKSPEED_DIV4;
	SPI2_FD.SPIConfig.SPI_SSM = SPI_SSM_SOFTWARE_EN;
	SPI2_FD.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2_FD.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_FD.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;

	// Init SPI
	SPI_Init(&SPI2_FD);
}

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

//somedata
uint8_t message_buffer[250];

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB6 --> SCL
 * PB7 --> SDA
 * ALT function mode : 4
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_OPENDRAIN;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	//SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C_Handle_t I2CSettings;

	I2CSettings.pI2C = I2C1;
	I2CSettings.I2C_Config.I2C_ACKControl = I2C_ACK_EN;
	I2CSettings.I2C_Config.I2C_DeviceAddress = I2C_MY_ADDR;
	I2CSettings.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2CSettings.I2C_Config.I2C_SCLSpeed = I2C_SCLSPEED_SM;

	I2C_Init(&I2CSettings);
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
	printf("Application is running\n");

	uint8_t len_info = 0;
	uint8_t i2c_command;

	// Enable the Peripheral Clocks
	I2C_PeriClockControl(I2C1, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	// GPIO Inits
	I2C1_GPIOInits();

	// I2C Inits
	I2C1_Inits();

	// Interrupt Button Init
	GPIO_InterruptPinInit();

	// Enable I2C Peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_Handle_t I2C1Handle;
	I2C1Handle.pI2C = I2C1;

	while(1)
	{
		if(user_button == 1)
		{
			// Send Command to get length info
			i2c_command = I2C_COMMAND_GETLENINFO;
			I2C_MasterSendData(&I2C1Handle, &i2c_command, 1, SLAVE_ADDR, I2C_SR_YES);

			// Read length info
			I2C_MasterReceiveData(I2C1, &len_info, 1, SLAVE_ADDR, I2C_SR_YES);

			// Send data read coomand
			i2c_command = I2C_COMMAND_FULLREAD;
			I2C_MasterSendData(&I2C1Handle, &i2c_command, 1,SLAVE_ADDR , I2C_SR_YES);

			// Read Full data
			I2C_MasterReceiveData(I2C1, message_buffer, len_info, SLAVE_ADDR, I2C_SR_NO);

			// Display message
			message_buffer[len_info+1] = '\0';
			printf("Received message: %s", message_buffer);


			user_button = 0;
		}
		delay();
		delay();
		delay();
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


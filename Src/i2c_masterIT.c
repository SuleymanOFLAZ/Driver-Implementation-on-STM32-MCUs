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
_Bool Flag_MessageReceived = 0;

I2C_Handle_t I2C1Handle;

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
	I2C1Handle.pI2C = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = I2C_MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCLSPEED_SM;

	I2C_Init(&I2C1Handle);
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

	// I2C IRQ Configuration - priority doesn't matter in this application
	I2C_IRQInterruptConfig(IRQ_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_I2C1_ER, ENABLE);

	// Enable I2C Peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1)
	{
		if(user_button == 1)
		{
			// Send Command to get length info
			i2c_command = I2C_COMMAND_GETLENINFO;
			while(I2C_MasterSendDataIT(&I2C1Handle, &i2c_command, 1, SLAVE_ADDR, I2C_SR_YES) != I2C_STATE_READY);

			// Read length info
			while(I2C_MasterReceiveDataIT(&I2C1Handle, &len_info, 1, SLAVE_ADDR, I2C_SR_YES) != I2C_STATE_READY);

			// Send data read coomand
			i2c_command = I2C_COMMAND_FULLREAD;
			while(I2C_MasterSendDataIT(&I2C1Handle, &i2c_command, 1,SLAVE_ADDR , I2C_SR_YES) != I2C_STATE_READY);
			Flag_MessageReceived = 0;

			// Read Full data
			while(I2C_MasterReceiveDataIT(&I2C1Handle, message_buffer, len_info, SLAVE_ADDR, I2C_SR_NO) != I2C_STATE_READY);

			while(Flag_MessageReceived == 0);
			// Display message
			message_buffer[len_info+1] = '\0';
			printf("Received message: %s", message_buffer);
			Flag_MessageReceived = 0;


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

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t EvType)
{
	if(EvType == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed. \n");
	}
	else if(EvType == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed. \n");
		Flag_MessageReceived = 1;
	}
	else if(EvType == I2C_ERROR_AF)
	{
		printf("ERROR: ACK Failature. \n");

		// Close Communication
		I2C_CloseTransfer(pI2CHandle, I2C_TRANSMIT);

		// Generate STOP Condition to release bus
		pI2CHandle->pI2C->I2C_CR1 |= (1 << I2C_CR1_STOP);

		// Hang in infinite loop
		while(1);
	}
}

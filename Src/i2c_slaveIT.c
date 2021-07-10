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

#define SLAVE_ADDR  0x68
#define I2C_MY_ADDR SLAVE_ADDR


#define I2C_COMMAND_SENDLENINFO 	0x51
#define I2C_COMMAND_WRITEMESSAGE	0x52

_Bool user_button = 0;
_Bool Flag_MessageReceived = 0;

I2C_Handle_t I2C1Handle;

uint8_t message_buffer[] = "Look up to stars!\n";

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

	// Enanle I2C Interrupts
	I2C_SlaveCallbackEventsControl(I2C1, ENABLE);

	// Enable I2C Peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// Enable ACKing - We must enable it after PE
	I2C_ACKControl(I2C1Handle.pI2C, ENABLE);

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
	static uint8_t CommandCode = 0, Cnt = 0;

	if(EvType == I2C_EV_DATA_REQ)
	{
		// Master wants some data. Slave has to send it
		if(CommandCode == I2C_COMMAND_SENDLENINFO)
		{
			I2C_SlaveSendData(pI2CHandle->pI2C, sizeof(message_buffer));
		}
		else if(CommandCode == I2C_COMMAND_WRITEMESSAGE)
		{
			I2C_SlaveSendData(pI2CHandle->pI2C, message_buffer[Cnt++]);
		}
	}else if(EvType == I2C_EV_DATA_RCV)
	{
		// Data is waiting the slave to read. Slave has to read it.
		CommandCode = I2C_SlaveReceiveData(pI2CHandle->pI2C);
	}else if(EvType == I2C_ERROR_AF)
	{
		// This happens only during slave transmission.
		// Master has sand the NACK. So slave need understand that master doesn't need more data.
		CommandCode = 0xFF;
		Cnt = 0;
	}else if(EvType == I2C_EV_STOP)
	{
		// This happens only during slave reception.
		// Master has ended the communication with the slave.
	}
}

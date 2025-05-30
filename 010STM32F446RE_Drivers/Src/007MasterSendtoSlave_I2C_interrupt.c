/*
 * 006I2C_MASTERReadfromslave_SlaveArduino.c
 *
 *  Created on: May 10, 2025
 *      Author: eldon
 */


#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>


void delay(void){

	for(uint32_t i=0; i < 200000 ; i++);
}



I2C_Handle_t I2C_handle;
uint8_t msg[] = "Hi!";
uint8_t len=0;
uint8_t cmd1 = 0x51, cmd2 = 0x52;
uint8_t rxBuffer[1];


void gpio_init(void){
	GPIO_Handle_t Gpio_handle;

	//initialize PC13 user button
	Gpio_handle.pGPIOx = GPIOC;
	Gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	Gpio_handle.GPIO_PinConfig.GPIO_PinAFMode = NONE;
	Gpio_handle.GPIO_PinConfig.GPIO_PinOType = GPIO_OPTYPE_PUSHPULL;
	Gpio_handle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	Gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPD;
	GPIO_Init(&Gpio_handle);

}

void i2c_gpioinit(void){
	GPIO_Handle_t I2CGpio_handle;

	//initialize PB6 I2C SCL
	I2CGpio_handle.pGPIOx = GPIOB;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinAFMode = GPIO_AF4;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinOType = GPIO_OPTYPE_OPENDRAIN;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPD;

	I2CGpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6;
	GPIO_Init(&I2CGpio_handle);

	//initialize PB7 I2C SDA
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN7;
	GPIO_Init(&I2CGpio_handle);
}

void i2c_init(void){

	I2C_handle.pI2Cx = I2C1;
	I2C_handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C_handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C_handle.I2C_Config.I2C_FMDutyCycle	= I2C_FM_DUTY_2;
	I2C_handle.I2C_Config.I2C_SCLSpeed	= I2C_SCL_SPEED_STANDARD;
	I2C_Init(&I2C_handle);

}

int main(void){

	i2c_gpioinit();

	gpio_init();

	i2c_init();

	//IRQ Configurations

	//ENABLED IRQ_NUMBER_EXTI10_15 = IRQ 40, TO BE RECEIVED AND PROCESSED BY THE CPU
	GPIO_IRQInterruptConfig(IRQ_NUMBER_EXTI10_15, ENABLE);

	//SETS THE PRIORITY OF IRQ 40 TO HIGHEST PRIOTITY THE LESSER THE NUMBER THE HIGHER THE PRIORITY
	GPIO_IRQPriorityConfig(IRQ_NUMBER_EXTI10_15, 1);

	I2C_IRQInterruptConfig(IRQ_NUMBER_I2C1_EVENT, ENABLE);

	I2C_IRQInterruptConfig(IRQ_NUMBER_I2C1_ERR, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);


	while(1){


	}




	return 0;
}



void EXTI15_10_IRQHandler(){

	delay();


	//sends command to retrieve the data length
	I2C_MasterSendDataIT(&I2C_handle, &cmd1, 1, 0x68, I2C_REPEATED_START_EN);

	//receive data length from slave
	while(I2C_MasterReceiveDataIT(&I2C_handle, &len, 1, 0x68, I2C_REPEATED_START_EN) != I2C_READY);


	//send command to retrieve the whole length of data from slave
	while(I2C_MasterSendDataIT(&I2C_handle, &cmd2, 1, 0x68, I2C_REPEATED_START_EN) != I2C_READY);

	//rxBuffer[len];

	//receive whole data from slave
	while(I2C_MasterReceiveDataIT(&I2C_handle, rxBuffer, len, 0x68, I2C_REPEATED_START_DI) != I2C_READY);

	printf("\nReceived Data: ");
	for(uint32_t i=0; i<len; i++){

		printf("%c", rxBuffer[i]);

	}


	GPIO_IRQHandling(13);

}


void I2C1_EV_IRQHandler(){

	I2C_Event_IRQHandling(&I2C_handle);
}




void I2C1_ER_IRQHandler(){

	I2C_Err_IRQHandling(&I2C_handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	switch(event)
	{
	case I2C_EV_TX_CMPLT:
		printf("Data Transmission Completed\n");
		break;
	case I2C_EV_RX_CMPLT:
		printf("Data Reception Completed\n");
		break;
	case I2C_EV_STOP:
		printf("Stop Condition Detected by Slave\n");
		break;
	case I2C_ERROR_BERR:
		printf("Bus Error\n");
		break;
	case I2C_ERROR_ARLO:
		printf("Arbitration Lost Error\n");
		break;
	case I2C_ERROR_AF:
		printf("Acknowledge Failure Error\n");
		break;
	case I2C_ERROR_OVR:
		printf("Overrun/Underrun Error\n");
		break;
	case I2C_ERROR_TIMEOUT:
		printf("Timeout Error\n");
		break;
	default:
		printf("Unknown Error\n");
		break;
	}
}












/*
 * 005I2C_MasterRX_SlaveArduino.c
 *
 *  Created on: May 9, 2025
 *      Author: eldon
 */


#include "stm32f446xx.h"
#include <string.h>

uint8_t msg[] = "Hello World!";


void delay(void){

	for(uint32_t i=0; i < 200000 ; i++);
}


I2C_Handle_t I2C_handle;

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

	//initialize PC6 I2C SCL
	I2CGpio_handle.pGPIOx = GPIOB;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinAFMode = GPIO_AF4;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinOType = GPIO_OPTYPE_OPENDRAIN;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	I2CGpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPD;

	I2CGpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6;
	GPIO_Init(&I2CGpio_handle);

	//initialize PC7 I2C SDA
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

	I2C_PeripheralControl(I2C1, ENABLE);

	while(1){
/*
		while(GPIO_ReadInputPin(GPIOC, 13) == 1);

		delay();

		I2C_MasterSendData(&I2C_handle, msg, strlen((char*)msg), 0x68);
		*/

	}




	return 0;
}



void EXTI15_10_IRQHandler(){

	delay();

	I2C_MasterSendData(&I2C_handle, msg, strlen((char*)msg), 0x68);
	GPIO_IRQHandling(13);

}





/*
 * 008STM32_Slave_ArduinoMaster_i2cIT.c
 *
 *  Created on: May 16, 2025
 *      Author: eldon
 */


/*
 * 006I2C_MASTERReadfromslave_SlaveArduino.c
 *
 *  Created on: May 10, 2025
 *      Author: eldon
 */


#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR 0X68


void delay(void){

	for(uint32_t i=0; i < 200000 ; i++);
}



I2C_Handle_t I2C_handle;
uint8_t TxBuffer[] = "Your STM32 is in slave mode!\n";


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
	I2C_handle.I2C_Config.I2C_DeviceAddress = SLAVE_ADDR;
	I2C_handle.I2C_Config.I2C_FMDutyCycle	= I2C_FM_DUTY_2;
	I2C_handle.I2C_Config.I2C_SCLSpeed	= I2C_SCL_SPEED_STANDARD;
	I2C_Init(&I2C_handle);

}

int main(void){

	i2c_gpioinit();

	i2c_init();

	//IRQ Configurations
	I2C_IRQInterruptConfig(IRQ_NUMBER_I2C1_EVENT, ENABLE);

	I2C_IRQInterruptConfig(IRQ_NUMBER_I2C1_ERR, ENABLE);

	I2C_SlaveEn_CallbackEvents(I2C1, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);


	while(1){


	}



	return 0;
}


void I2C1_EV_IRQHandler(){

	I2C_Event_IRQHandling(&I2C_handle);
}


void I2C1_ER_IRQHandler(){

	I2C_Err_IRQHandling(&I2C_handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{

	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;

	if(AppEvent == I2C_EV_DATA_REQ){

		//master wants some data, so slave will send
		if(commandCode == 0x51){

			//master requesting data length, send data length to master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*) TxBuffer));

		}else if(commandCode == 0x52){
			//master is requesting the whole data length, send the whole data to master
				I2C_SlaveSendData(pI2CHandle->pI2Cx, TxBuffer[Cnt++]);
		}

	}

	else if(AppEvent == I2C_EV_DATA_RECV){
		//master will send some commands, so slave will receive data

		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}

	else if(AppEvent == I2C_ERROR_AF){

		//happens when master doesnt want anymore data from slave so it sends a NACK to let slave know to stop sending data
		commandCode = 0xFF;
		Cnt = 0;

	}

	else if(AppEvent == I2C_EV_STOP){

		//happens when slave is receiving data from master, when master sends all data it generates a stop condition.

	}
}












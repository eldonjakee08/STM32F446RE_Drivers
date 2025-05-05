/*
 * 004SPI_Receive.c
 *
 *  Created on: Apr 29, 2025
 *      Author: eldon
 */


#include <stdint.h>
#include "stm32f446xx.h"
#include <stdio.h>

#define COMMAND_LED_CTRL 		0X50
#define COMMAND_SENSOR_READ		0X51
#define COMMAND_LED_READ    	0X52
#define COMMAND_PRINT   		0X53
#define COMMAND_ID_READ 		0X54

#define LED_ON 					1
#define LED_OFF 				0

#define ANLOG_PIN0 				0
#define ANLOG_PIN1 				1
#define ANLOG_PIN2 				2
#define ANLOG_PIN3 				3
#define ANLOG_PIN4 				4

#define LED_PIN					9
/*
 * PB12 - NSS
 * PB13 - SCLK
 * PB14 - MISO
 * PB15 - MOSI
 *
 * AF5
 */

void delay(uint32_t count) {
	for (uint32_t i = 0; i < count; i++);
}


void GPIOPins_Init()
{
	GPIO_Handle_t GPIOHandle;

	//config PB12 NSS as Alternate function SPI pins
	GPIOHandle.pGPIOx = GPIOB;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOType = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAFMode = GPIO_AF5;

	//initialize PB12 config
	GPIO_Init(&GPIOHandle);


	//config PB13 SCLK as Alternate function SPI pins
	GPIOHandle.pGPIOx = GPIOB;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOType = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAFMode = GPIO_AF5;

	//initialize PB13 config
	GPIO_Init(&GPIOHandle);

	//config PB14 MISO as Alternate function SPI pins
	GPIOHandle.pGPIOx = GPIOB;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN14;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOType = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAFMode = GPIO_AF5;

	//initialize PB14 config
	GPIO_Init(&GPIOHandle);

	//config PB15 MOSI as Alternate function SPI pins
	GPIOHandle.pGPIOx = GPIOB;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN15;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOType = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAFMode = GPIO_AF5;

	//initialize PB15 config
	GPIO_Init(&GPIOHandle);
}

void GPIOButton_Init()
{
	GPIO_Handle_t GPIOHandle;

	//config PC13 as input pin for button
	GPIOHandle.pGPIOx = GPIOC;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOType	= NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = NONE;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAFMode = NONE;

	//initialize PC13 config
	GPIO_Init(&GPIOHandle);
}


void SPIInit()
{
	SPI_Handle_t SPI2Handle;

	//config settings for SPI2
	SPI2Handle.pSPIx 						= SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig 		= SPI_BUSCONF_FULLDUPLEX;
	SPI2Handle.SPIConfig.SPI_CPHA 			= SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL 			= SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_DFF 			= SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_DeviceMode		= SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM			= SPI_SSM_DI;
	SPI2Handle.SPIConfig.SPI_SclkSpeed		= SPI_SCLK_SPEED_DIV8;

	//initialize the SPI2 settings
	SPI_Init(&SPI2Handle);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte) {
	if (ackbyte == 0xA5) {
		return 1; //acknowledge byte received
	} else {
		return 0; //acknowledge byte not received
	}
}

int main(void){

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;


	GPIOPins_Init();
	printf("GPIO Pins Initialized\n");

	SPIInit();
	printf("SPI Initialized\n");

	GPIOButton_Init();
	printf("GPIO Button Initialized\n");

	while(1)
	{
		while(GPIO_ReadInputPin(GPIOC, GPIO_PIN13) == 1); //wait for button press

		delay(1000000); //debounce delay

		SPI_PeripheralClkCtrl(SPI2, ENABLE);

		uint8_t commcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command to slave
		SPI_SendData(SPI2, &commcode, 1);
		printf("Sent COMMAND_LED_CTRL to slave\n");

		//dummy read to clear RXNE register. Honestly the instructor could have programmed the slave to send
		//a acknowledge byte immediately after sending the command. IDK why? i'll explore more.
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send dummy byte to get the ack byte from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//receive ack byte from slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN; //turn on LED
			args[1] = LED_ON;

			//send the LED control arguments
			SPI_SendData(SPI2, args, 2);
		}

		while(GPIO_ReadInputPin(GPIOC, GPIO_PIN13) == 1); //wait for button press

		delay(1000000); //debounce delay

		commcode = COMMAND_SENSOR_READ;
		//send command to slave
		SPI_SendData(SPI2, &commcode, 1);

		//dummy read to clear RXNE register. Honestly the instructor could have programmed the slave to send
		//a acknowledge byte immediately after sending the command. IDK why? i'll explore more.
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send dummy byte to get the ack byte from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//receive ack byte from slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			uint8_t analog_pin = ANLOG_PIN1;

			//sends the analog pin to read
			SPI_SendData(SPI2, &analog_pin, 1);

			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//send dummy byte to get the sensor data from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t sensor_data=0;
			SPI_ReceiveData(SPI2, &sensor_data, 1);

		}


		while(GPIO_ReadInputPin(GPIOC, GPIO_PIN13) == 1); //wait for button press

		delay(1000000); //debounce delay

		commcode = COMMAND_LED_READ;
		//send command to slave
		SPI_SendData(SPI2, &commcode, 1);

		//dummy read to clear RXNE register. Honestly the instructor could have programmed the slave to send
		//a acknowledge byte immediately after sending the command. IDK why? i'll explore more.
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send dummy byte to get the ack byte from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//receive ack byte from slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			uint8_t led_pin = LED_PIN;

			//sends the analog pin to read
			SPI_SendData(SPI2, &led_pin, 1);

			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//send dummy byte to get the sensor data from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t sensor_data=0;
			SPI_ReceiveData(SPI2, &sensor_data, 1);

		}


	}


	return 0;

}

/*
 * 003SPI_tx.c
 *
 *  Created on: Apr 26, 2025
 *      Author: eldon
 */
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

/*
 * SPI2 PINS
 * PC1 - MOSI AF7
 * PC2 - MISO AF5
 * PB10 - SCK	AF5
 * PB12 - NSS	AF5
 */

SPI_Handle_t SPI_handle;

int main(void)
{
	GPIO_PeripheralClkCtrl(GPIOC, ENABLE);
	GPIO_PeripheralClkCtrl(GPIOB, ENABLE);

	//initialize pin PC1 as SPI2 MOSI NO MISO NEEDED SINCE WE ARE ONLY TX
	GPIO_Init2(GPIOC, GPIO_PIN1, GPIO_MODE_ALTFN, GPIO_NO_PULLUP_PULLDOWN, NONE, NONE, GPIO_AF7);

	//INTIALIZE PIN PB10 AS SPI2 SCK
	GPIO_Init2(GPIOB, GPIO_PIN10, GPIO_MODE_ALTFN, GPIO_NO_PULLUP_PULLDOWN, NONE, NONE, GPIO_AF5);

	//INITIALIZE PC2 MISO
	//GPIO_Init2(GPIOC, GPIO_PIN2, GPIO_MODE_ALTFN, GPIO_NO_PULLUP_PULLDOWN, NONE, NONE, GPIO_AF5);

	//INITIALIZE PB12 NSS PIN
	//GPIO_Init2(GPIOB, GPIO_PIN12, GPIO_MODE_ALTFN, GPIO_NO_PULLUP_PULLDOWN, NONE, NONE, GPIO_AF5);

	//ENABLE SPI2 CLOCK PERIPHERAL
	SPI_PeripheralClkCtrl(SPI2, ENABLE);


	//SPI2 INTIALIZATION PARAMETERS INPUT
	SPI_handle.pSPIx = SPI2;
	SPI_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI_handle.SPIConfig.SPI_BusConfig = SPI_SCLK_SPEED_DIV2;
	SPI_handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI_handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	//INITIALIZE SPI2 SETTINGS
	SPI_Init(&SPI_handle);

	//forces SSI to high to avoid MODF error
	SPI_SSI_EnDi(SPI2, ENABLE);

	//allows CPU to process SPI2 interrupts
	SPI_IRQInterruptConfig(IRQ_NUMBER_SPI2,ENABLE);
	SPI_IRQPriorityConfig(IRQ_NUMBER_SPI2, 0);

	//ENABLES SPI2 PERIPHERAL
	//Note: Only enable SPI peripheral when SPI config is set
	SPI_Periperal_EnDi(SPI2, ENABLE);

	//START SENDING DATA TO SPI2
	char msg[] = "Hello World!";
 	SPI_SendDataIT(&SPI_handle, (uint8_t*)msg, strlen(msg));


	for(;;){

	}


	return 0;
}


void SPI2_IRQHandler(){
	SPI_IRQHandling(&SPI_handle);
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	if((pSPIHandle->pSPIx == SPI2) && (AppEvent == SPI_EVENT_TX_CMPLT))
	{
		//Waits till SPI is not busy. SPI_SR_BSY = 1 means busy.
		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY) == FLAG_SET);

		//disables SPI communication
		SPI_Periperal_EnDi(SPI2, DISABLE);
	}
}



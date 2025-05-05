/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 23, 2025
 *      Author: eldon
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include <stdint.h>
#include "stm32f446xx.h"

typedef struct
{
	uint8_t 		SPI_DeviceMode;		//SPI Device mode @DEVICE_MODE
	uint8_t 		SPI_BusConfig;		//@SPI_BUS_CONFIG, determines the SPI communication set-up
	uint8_t 		SPI_SclkSpeed;		//@SPI_BAUDRATE, determines the data transmission speed.
	uint8_t 		SPI_DFF;			//@SPI_DFF, SPI Data Frame Format. Default 8 bits but can be set to 16bit here.
	uint8_t 		SPI_CPOL;			//@CPOL, Source CLK Polarity. 1 = high level idle state, 0 = low level idle state
	uint8_t 		SPI_CPHA;			//@CPHA, Source Clock Phase. 1 = data sample at trailing edge, 0 = data sample at leading edge
	uint8_t 		SPI_SSM;			//@SPI_SSM, Software Slave Management
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t 	*pSPIx;				//Base address of the SPIx peripheral
	SPI_Config_t	SPIConfig;			//SPI configuration settings
	uint8_t         *pTxBuffer;			//Pointer to hold the Tx buffer address
	uint8_t			*pRxBuffer;			//Pointer to hold the Rx buffer address
	uint32_t     	TxLen;				//Length of Tx data
	uint32_t     	RxLen;				//Length of Rx data
	uint8_t			TxState;			//Tx state
	uint8_t			RxState;			//Rx state
}SPI_Handle_t;


/*
 * @DEVICE_MODE SPI Device modes
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0


/*
 * @SPI_BUS_CONFIG
 */
#define SPI_BUSCONF_FULLDUPLEX			1
#define SPI_BUSCONF_HALFDUPLEX			2
#define SPI_BUSCONF_SIMPLEX_RXONLY		3


/*
 * @SPI_BAUDRATE
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7


/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1


/*
 * @CPOL
 */
#define SPI_CPOL_HIGH		1	//When High, Idle state is at High.
#define SPI_CPOL_LOW		0	//When Low, Idle state is at low.


#define SPI_CPHA_HIGH		1	//When High, The second clock transition is the  data capture edge
#define SPI_CPHA_LOW		0	//When Low, The first clock transition is the data capture edge.


/*
 * @SPI_SSM
 */
#define SPI_SSM_EN			1	//Software Slave Management enabled
#define SPI_SSM_DI			0	//Software Slave Management disabled

/*
 * @SPI_STATUS
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/***********************************************************************************************************************
 * 									APIs supported by this driver
 * 					for more information about the APIs check the function definitions
 ***********************************************************************************************************************/

/*
 * SPI Peripheral Clock set-up
 */
void SPI_PeripheralClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * SPI Initialization & De-initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeriperalCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSI_EnDi(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * SPI data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length_data);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length_data);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);



/*
 * Other peripheral APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * IRQ config and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Application event call back
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

/*
 * Other peripheral control APIs
 */



#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */

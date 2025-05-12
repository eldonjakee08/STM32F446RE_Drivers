/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: May 5, 2025
 *      Author: eldon
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"


typedef struct
{
	uint32_t 	I2C_SCLSpeed; 		//@I2C_SCLSpeed, Sets the clock speed of the I2C bus
	uint8_t 	I2C_DeviceAddress; 	//if MCU is acting as slave, user must specify the address
	uint8_t 	I2C_AckControl;		//@I2C_ACK_CONTROL, setting to enable the auto ACKING
	uint8_t 	I2C_FMDutyCycle;	//@I2C_FM_DUTY_CYCLE, sets the duty cycle of Clock when its in "fast mode"

}I2C_Config_t;


typedef struct
{
	I2C_RegDef_t 	*pI2Cx;				//base address of I2C peripheral, this could be called as an instance as well if referring to STM32 HAL
	I2C_Config_t 	I2C_Config;			//I2C configuration settings here
	uint8_t 		*pTxBuffer;			//To store the application Tx Buffer address
	uint8_t 		*pRxBuffer;			//To store the application Rx Buffer address
	uint32_t 		TxLen;				//to store the Tx Len
	uint32_t 		RxLen;				//to store the Rx Len
	uint8_t 		TxRxState;			//to store the communication state
	uint8_t 		DeviceAddr;			//to store the slave/device address
	uint32_t 		RxSize;				//to store Rx Size
	uint8_t 		Sr;					//to store repeated start value

}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_STANDARD        	100000
#define I2C_SCL_SPEED_FAST4k           	400000
#define I2C_SCL_SPEED_FAST2K           	200000


/*
 * @I2C_ACK_CONTROL
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0


/*
 * @I2C_FM_DUTY_CYCLE
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * Master read and write bits to slave macros
 */
#define I2C_MASTER_READ				1
#define I2C_MASTER_WRITE			2


/*
 * Repeated Start Enable Macros
 */
#define I2C_REPEATED_START_EN		1
#define I2C_REPEATED_START_DI		0

/*
 * I2C application states
 */
#define I2C_READY 				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2


/*
 * I2C application event/error macros
 */
#define I2C_EV_TX_CMPLT 		0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7



/***********************************************************************************************************************
 * 									APIs supported by this driver
 * 					for more information about the APIs check the function definitions
 ***********************************************************************************************************************/

/*
 * SPI Peripheral Clock set-up
 */
void I2C_PeripheralClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi); //DONE


/*
 * I2C Initialization & De-initialization
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx); //DONE


/*
 * I2C data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Repeated_Start_EN);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Repeated_Start_EN);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Repeated_Start_EN);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Repeated_Start_EN);

/*
 * Other peripheral APIs
 */
uint8_t I2C_GetSR1FlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);
uint8_t I2C_GetSR2FlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi); //DONE
uint32_t RCC_GetPCLK1Value(void);
uint8_t Get_TriseValue(I2C_Handle_t *pI2CHandle);



/*
 * IRQ config and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_Event_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_Err_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Application event call back
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */

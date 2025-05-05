/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: May 5, 2025
 *      Author: eldon
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include <stdint.h>
#include "stm32f446xx.h"


typedef struct
{
	uint8_t I2C_SCLSpeed; 		//@I2C_SCLSpeed, Sets the clock speed of the I2C bus
	uint8_t I2C_DeviceAddress; 	//if MCU is acting as slave, user must specify the
	uint8_t I2C_AckControl;		//@I2C_ACK_CONTROL, setting to enable the auto ACKING
	uint8_t I2C_FMDutyCycle;	//@I2C_FM_DUTY_CYCLE, sets the duty cycle of Clock when its in "fast mode"

}I2C_Config_t;


typedef struct
{
	I2C_RegDef_t *pI2Cx;			//base address of I2C peripheral, this could be called as an instance as well if referring to STM32 HAL
	I2C_Config_t I2C_Config;	//I2C configuration settings here
}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_STANDARD        	100e+3
#define I2C_SCL_SPEED_FAST4k           	400e+3
#define I2C_SCL_SPEED_FAST2K           	200e+3


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

/***********************************************************************************************************************
 * 									APIs supported by this driver
 * 					for more information about the APIs check the function definitions
 ***********************************************************************************************************************/

/*
 * SPI Peripheral Clock set-up
 */
void I2C_PeripheralClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi); //DONE


/*
 * SPI Initialization & De-initialization
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx); //DONE


/*
 * SPI data send and receive
 */



/*
 * Other peripheral APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi); //DONE



/*
 * IRQ config and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Application event call back
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */

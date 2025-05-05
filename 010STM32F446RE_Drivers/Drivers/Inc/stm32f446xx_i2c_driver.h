/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: May 5, 2025
 *      Author: eldon
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_



typedef struct
{
	uint8_t I2C_SCLSpeed; 		//Sets the clock speed of the I2C bus
	uint8_t I2C_DeviceAddress; 	//if MCU is acting as slave, user must specify the
	uint8_t I2C_AckControl;		//setting to enable the auto ACKING
	uint8_t I2C_FMDutyCycle;	//sets the duty cycle of Clock when its in "fast mode"

}I2C_Config_t;


typedef struct
{
	I2C_RegDef_t *I2Cx;			//base address of I2C peripheral, this could be called as an instance as well if referring to STM32 HAL
	I2C_Config_t I2C_Config;	//I2C configuration settings here
}I2C_Handle_t;


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */

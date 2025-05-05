/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: May 5, 2025
 *      Author: eldon
 */


#include <stdint.h>
#include "stm32f446xx.h"


/*********************************************************************************************
 * @fn			- I2C_PeripheralClkCtrl
 *
 * #brief		- enables or disables the peripheral clock for the given I2C port
 *
 * @param[in]	- I2C_RegDef_t *pI2Cx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t EnorDi, input peripheral clock state here either "ENABLE" or "DISABLE"
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_PeripheralClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	uint8_t temp1 = I2C_BASEADDR_TO_CODE(pI2Cx);

	if(EnorDi == ENABLE){
		switch(temp1){
		case 0:
			I2C1_PCLK_EN();
			break;
		case 1:
			I2C2_PCLK_EN();
			break;
		case 2:
			I2C3_PCLK_EN();
			break;
		default:
			break;
		}

	}
	else{
		switch(temp1){
		case 0:
			I2C1_PCLK_DI();
			break;
		case 1:
			I2C2_PCLK_DI();
			break;
		case 2:
			I2C3_PCLK_DI();
			break;
		default:
			break;
		}

	}
}


/*********************************************************************************************
 * @fn			- I2C_PeripheralControl
 *
 * #brief		- Enables or disables the I2C peripheral
 *
 * @param[in]	- I2C_RegDef_t *pI2Cx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t EnorDi, input peripheral clock state here either "ENABLE" or "DISABLE"
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE); //Enable the peripheral
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE); //Disable the peripheral
	}
}


/*********************************************************************************************
 * @fn			- I2C_DeInit
 *
 * #brief		- Resets the I2C peripheral settings and reverts it back to reset values.
 *
 * @param[in]	- I2C_RegDef_t *pI2Cx, pointer variable, input the address of the structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	uint8_t temp1 = I2C_BASEADDR_TO_CODE(pI2Cx);

	switch(temp1){
	case 0:
		I2C1_REG_RESET();
		break;
	case 1:
		I2C2_REG_RESET();
		break;
	case 2:
		I2C3_REG_RESET();
		break;
	default:
		break;
	}

}



/*********************************************************************************************
 * @fn			- I2C_Init
 *
 * #brief		- Initializes the I2C peripheral according to the user settings
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, pointer variable, input the address of the structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

}

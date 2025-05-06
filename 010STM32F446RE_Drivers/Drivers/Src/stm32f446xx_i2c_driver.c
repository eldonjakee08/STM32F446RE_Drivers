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
	uint32_t SCLSpeed = pI2CHandle->I2C_Config.I2C_SCLSpeed;
	uint32_t CCRVal;

	//for standard mode at 100Kbps
	if(SCLSpeed <= 100e+3)
	{
		//set the MCU into master standard mode
		pI2CHandle->pI2Cx->CCR &= ~(1 << I2C_CCR_FS);

		//fetches the current APB1 clock value
		uint32_t APB1Clock = RCC_GetPCLK1Value();

		//clears the I2C_CR2:FREQ5:0 bitfield just to be sure thats its clean before setting
		pI2CHandle->pI2Cx->CR2 &= ~(63 << I2C_CR2_FREQ5_0);

		//program the current APB1 clock freq into I2C_CR2:FREQ5:0
		//divided the APB1Clock value with 1e+6 only need the ten millions and millions digit
		pI2CHandle->pI2Cx->CR2 |= (APB1Clock/1000000) << I2C_CR2_FREQ5_0;

		//calculate the needed CCR value
		CCRVal = (1/SCLSpeed) / (2 * (1/APB1Clock) );

		//programs the calculated CCR value into the CCR register
		pI2CHandle->pI2Cx->CCR |= CCRVal << I2C_CCR_CCR11_0;
	}
	//for fast mode 200kbs and 400kbps
	else{
		switch(SCLSpeed){
		case I2C_SCL_SPEED_FAST2K:
			//set the MCU into master fast mode
			pI2CHandle->pI2Cx->CCR |= 1 << I2C_CCR_FS;

			//sets the DUTY = 0, tlow/thigh = 2.
			pI2CHandle->pI2Cx->CCR &= ~(1<<I2C_CCR_DUTY);

			//fetches the current APB1 clock value
			uint32_t APB1Clock = RCC_GetPCLK1Value();

			//clears the I2C_CR2:FREQ5:0 bitfield just to be sure thats its clean before setting
			pI2CHandle->pI2Cx->CR2 &= ~(63 << I2C_CR2_FREQ5_0);

			//program the current APB1 clock freq into I2C_CR2:FREQ5:0
			//divided the APB1Clock value with 1 million only need the ten millions and millions digit
			pI2CHandle->pI2Cx->CR2 |= (APB1Clock/1000000) << I2C_CR2_FREQ5_0;

			//calculates the needed CCR value
			CCRVal = (1/SCLSpeed) / (3 * (1/APB1Clock) );

			//programs the calculated CCR value into the CCR register
			pI2CHandle->pI2Cx->CCR |= CCRVal << I2C_CCR_CCR11_0;
			break;

		case I2C_SCL_SPEED_FAST4k:
			//set the MCU into master fast mode
			pI2CHandle->pI2Cx->CCR |= 1 << I2C_CCR_FS;

			//sets the DUTY = 1, tlow/thigh = 16/9.
			pI2CHandle->pI2Cx->CCR |= 1<<I2C_CCR_DUTY;

			//fetches the current APB1 clock value
			uint32_t APB1Clock = RCC_GetPCLK1Value();

			//clears the I2C_CR2:FREQ5:0 bitfield just to be sure thats its clean before setting
			pI2CHandle->pI2Cx->CR2 &= ~(63 << I2C_CR2_FREQ5_0);

			//program the current APB1 clock freq into I2C_CR2:FREQ5:0
			//divided the APB1Clock value with 1 million only need the ten millions and millions digit
			pI2CHandle->pI2Cx->CR2 |= (APB1Clock/1000000) << I2C_CR2_FREQ5_0;

			//calculates the needed CCR value
			CCRVal = (1/SCLSpeed) / (25 * (1/APB1Clock) );

			//programs the calculated CCR value into the CCR register
			pI2CHandle->pI2Cx->CCR |= CCRVal << I2C_CCR_CCR11_0;
			break;
		default:
			break;
		}


	}
}


/*********************************************************************************************
 * @fn			- RCC_GetPCLK1Value
 *
 * #brief		- fetches the current APB1 bus clock value
 *
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note 		-
 */
uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, system_clk;
	uint8_t clk_src, ahb_prescaler, apb1_prescaler;

	// Get the system clock source
	clk_src = (RCC->CFGR >> 2) & 0x3;

	if (clk_src == 0) {
		// HSI oscillator used as system clock
		system_clk = 16e+6; // HSI frequency is 16 MHz
	} else if (clk_src == 1) {
		// HSE oscillator used as system clock
		system_clk = 8e+6; // Assume HSE frequency is 8 MHz
	} else if (clk_src == 2) {
		// PLL used as system clock
		// Calculate PLL output frequency (not shown here for brevity)
		//too complex, will add soon once I know how to use PLL as clock src.
	}

	// Get AHB prescaler
	ahb_prescaler = (RCC->CFGR >> 4) & 0xF;
	if (ahb_prescaler < 8) {
		ahb_prescaler = 1; // No division
	} else {
		//this line is neat, this converts the binary value of the prescaler into its equivalent decimal value
		ahb_prescaler = 1 << (ahb_prescaler - 7); // 2, 4, 8, ..., 512
	}

	// Get APB1 prescaler
	apb1_prescaler = (RCC->CFGR >> 10) & 0x7;
	if (apb1_prescaler < 4) {
		apb1_prescaler = 1; // No division
	} else {
		apb1_prescaler = 1 << (apb1_prescaler - 3); // 2, 4, 8, 16
	}

	// Calculate PCLK1
	pclk1 = (system_clk / ahb_prescaler) / apb1_prescaler;

	return pclk1;
}




































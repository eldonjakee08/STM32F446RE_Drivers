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
	uint8_t temp = 0;

	//enables ACKing
	pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_ACK;

	//fetches the current APB1 clock value
	uint32_t APB1Clock = RCC_GetPCLK1Value();

	//clears the I2C_CR2:FREQ5:0 bitfield just to be sure thats its clean before setting
	pI2CHandle->pI2Cx->CR2 &= ~(63 << I2C_CR2_FREQ5_0);

	//program the current APB1 clock freq into I2C_CR2:FREQ5_0
	//divided the APB1Clock value with 1 million, only need the ten millions and millions digit
	pI2CHandle->pI2Cx->CR2 |= (APB1Clock/1000000) << I2C_CR2_FREQ5_0;

	//programs the user defined slave address to I2C_OAR1:ADD7_1 register
	pI2CHandle->pI2Cx->OAR1 |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD7_1;

	//set the 14th bit in OAR1 register to 1 as instructed by reference manual.
	pI2CHandle->pI2Cx->OAR1 |= 1 << I2C_OAR1_BIT14;

	//for standard mode at 100Kbps
	if(SCLSpeed <= I2C_SCL_SPEED_STANDARD)
	{
		//set the MCU into master standard mode
		temp &= ~(1 << I2C_CCR_FS);

		//calculate the needed CCR value
		CCRVal = (1/SCLSpeed) / (2 * (1/APB1Clock) );

		//set the CCR value
		temp |= CCRVal << I2C_CCR_CCR11_0;

		//programs the temp value into the CCR register
		pI2CHandle->pI2Cx->CCR = temp;
	}
	//for fast mode 200kbs and 400kbps
	else{

		//set the MCU into master fast mode
		temp |= 1 << I2C_CCR_FS;

		//set the Fast Mode duty cycle
		temp |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY;

		switch(SCLSpeed){

		case I2C_SCL_SPEED_FAST2K:
			//calculates the needed CCR value
			CCRVal = (1/SCLSpeed) / (3 * (1/APB1Clock) );

			//set the CCR value
			temp |= CCRVal << I2C_CCR_CCR11_0;

			//programs the temp value into the CCR register
			pI2CHandle->pI2Cx->CCR = temp;
			break;

		case I2C_SCL_SPEED_FAST4k:
			//calculates the needed CCR value
			CCRVal = (1/SCLSpeed) / (25 * (1/APB1Clock) );

			//set the CCR value
			temp |= CCRVal << I2C_CCR_CCR11_0;

			//programs the temp value into the CCR register
			pI2CHandle->pI2Cx->CCR = temp;
			break;

		default:
			break;
		}
	}

}


/*********************************************************************************************
 * @fn			- I2C_MasterSendData
 *
 * #brief		- Send data thru I2C peripheral in master mode
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, pointer variable, input the address of the structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr){

	uint16_t dummy_read __unused =0;

	//generate START condition
	pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_START;

	//wait for SB flag to be set which means start bit executed successfully
	while( I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB) == 0 );

	//sets the r/nw bit to write mode
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1 << 0);

	//write the slave address to be sent thru which also clears the SB flag.
	pI2CHandle->pI2Cx->DR |= SlaveAddr;

	//waits for ADDR flag to be set which means address bit sent successfully
	while( I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR) == 0 );

	//dummy read from SR2 to clear the ADDR flag.
	dummy_read = pI2CHandle->pI2Cx->SR2;

	//start sending thru polling method
	while(Len > 0){

		//check if shift register TX buffer is empty, if empty write the data into data register to be sent
		if( ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_TXE) & 0x1) == 1){

			pI2CHandle->pI2Cx->DR = *pTxBuffer;

			//decrements data length and increment pointer address to next byte is to be sent .
			Len--;
			pTxBuffer++;
		}
	}

	//wait for TXE = 1 & BTF = 1 which means data register and shift register is empty
	while( (I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE) & I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF)) == 0);

	//generate the stop condition & transmission complete
	pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_STOP;
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


/*********************************************************************************************
 * @fn			- I2C_GetSR1FlagStatus
 *
 * #brief		- fetches the current flag status of the SR1 register
 *
 * @param[in]	- I2C_RegDef_t *pI2Cx, pointer variable, input the address of the structure variable
 * @param[in]	- uint8_t FlagName, input the flag name to be checked
 *
 * @return		- uint8_t, returns the flag status
 *
 * @Note 		-
 */
uint8_t I2C_GetSR1FlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName){

	//gets the flag status from SR1
	uint8_t flag_status = (pI2Cx->SR1 >> FlagName) & 0x0001;

	//return the flag status
	return flag_status;
}


/*********************************************************************************************
 * @fn			- I2C_GetSR2FlagStatus
 *
 * #brief		- fetches the current flag status of SR2 register
 *
 * @param[in]	- I2C_RegDef_t *pI2Cx, pointer variable, input the address of the structure variable
 * @param[in]	- uint8_t FlagName, input the flag name to be checked
 *
 * @return		- uint8_t, returns the flag status
 *
 * @Note 		-
 */
uint8_t I2C_GetSR2FlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName){

	uint8_t flag_status=0;

	//gets the PEC 8 bit flag status from SR2
	if(FlagName == I2C_SR2_PEC15_8){
		//return the flag status
		flag_status =  ( (pI2Cx->SR2 >> FlagName) & 0x00FF );
	} else {
		//if not the PEC flag, then just get the 1 bit flag status from SR2
		flag_status = (pI2Cx->SR1 >> FlagName) & 0x0001;
	}

	return flag_status;
}




































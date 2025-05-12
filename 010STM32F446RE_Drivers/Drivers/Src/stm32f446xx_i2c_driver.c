/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: May 5, 2025
 *      Author: eldon
 */


#include <stdint.h>
#include "stm32f446xx.h"


static void i2c_startphase_addressphase(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t rw_bit);
static void I2C_MasterRxNEInterrupt_Handler(I2C_Handle_t *pI2CHandle);
static void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
static void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
__weak void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t event);


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

		//enables ACKing, can only be enabled when PE = 1
		pI2Cx->CR1 |= 1 << I2C_CR1_ACK;

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
	I2C_PeripheralClkCtrl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t SCLSpeed = pI2CHandle->I2C_Config.I2C_SCLSpeed;
	uint16_t CCRVal;
	uint8_t temp = 0;

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

		//calculate the needed CCR , for some reason it will skip this calculation and go to
		//infinite loop, idk why probably kay float ni sya?
		CCRVal = APB1Clock / (2 * SCLSpeed);

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
			CCRVal = APB1Clock / (3 * SCLSpeed);

			//set the CCR value
			temp |= CCRVal << I2C_CCR_CCR11_0;

			//programs the temp value into the CCR register
			pI2CHandle->pI2Cx->CCR = temp;
			break;

		case I2C_SCL_SPEED_FAST4k:
			//calculates the needed CCR value
			CCRVal = APB1Clock / (25 * SCLSpeed);

			//set the CCR value
			//temp |= CCRVal << I2C_CCR_CCR11_0;

			//programs the temp value into the CCR register
			pI2CHandle->pI2Cx->CCR = temp;
			break;

		default:
			break;
		}
	}

	//TRISE configuration
	uint8_t trise_val;
	trise_val = Get_TriseValue(pI2CHandle);

	//programs the calculated trise value into the Trise register
	//masked the first 6 bits only since TRISE register is only 6 bits long.
	pI2CHandle->pI2Cx->TRISE = (trise_val & 0x3f);
}


/*********************************************************************************************
 * @fn			- I2C_MasterSendData
 *
 * #brief		- Send data thru I2C peripheral in master mode
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, pointer variable, input the address of the structure variable
 * @param[in]	- uint8_t *pTxBuffer, pointer variable, input the address of the data to be sent
 * @param[in]	- uint32_t Len, input the length of the data to be sent
 * @param[in]	- uint8_t SlaveAddr, input the slave address that will receive the data
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Repeated_Start_EN)
{
	uint16_t dummy_read __unused =0;

	//start phase and address phase start
	i2c_startphase_addressphase(pI2CHandle, SlaveAddr, I2C_MASTER_WRITE);

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
	while( I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE) == 0);
	while( I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF) == 0);


	if(Repeated_Start_EN == I2C_REPEATED_START_DI)
		//generate the stop condition
		pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_STOP;

}


/*********************************************************************************************
 * @fn			- I2C_MasterSendDataIT
 *
 * #brief		- Send data thru I2C peripheral in master mode, non polling function
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, pointer variable, input the address of the structure variable
 * @param[in]	- uint8_t *pTxBuffer, pointer variable, input the address of the data to be sent
 * @param[in]	- uint32_t Len, input the length of the data to be sent
 * @param[in]	- uint8_t SlaveAddr, input the slave address that will receive the data
 *
 * @return		- none
 *
 * @Note 		-
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Repeated_Start_EN)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DeviceAddr = SlaveAddr;
		pI2CHandle->Sr = Repeated_Start_EN;

		//Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_START;

		//Implement the code to enable ITBUFEN Control Bit, this enables interrupt for when TXE = 1
		pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITBUFEN;

		//Implement the code to enable ITEVTEN Control Bit, this enables interrupt coming from event triggers like TXE = 1
		pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITEVTEN;

		//Implement the code to enable ITERREN Control Bit. this enables interrupt coming from errors during transmission
		pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITERREN;
	}

	return busystate;
}


/*********************************************************************************************
 * @fn			- I2C_MasterReceiveData
 *
 * #brief		- Receive data from an i2c slave
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, pointer variable, input the address of the structure variable
 * @param[in]	- uint8_t *pTxBuffer, pointer variable, input the address of the data to be sent
 * @param[in]	- uint32_t Len, input the length of the data to be sent
 * @param[in]	- uint8_t SlaveAddr, input the slave address that will receive the data
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Repeated_Start_EN)
{
	uint16_t dummy_read __unused =0;

	//1. Generate the START condition

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )

	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	//summed up in this one helper function i made
	i2c_startphase_addressphase(pI2CHandle, SlaveAddr, I2C_MASTER_READ);


	//procedure to read only 1 byte from slave, i think this will be used to receive data from slave
	//on the data length the master will need from the slave
	if(Len == 1)
	{
		//Disable Acking so master will send a NACK after receiving 1 byte of data. The NACK is to tell slave it doesnt want anymore data
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

		//to clear the ADDR flag
		dummy_read = pI2CHandle->pI2Cx->SR2;

		//wait until  RXNE becomes 1
		while(I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE) == 0);

		if(Repeated_Start_EN == I2C_REPEATED_START_DI)
			//generate the stop condition
			pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_STOP;

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

		if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
			//re-enable ACKing
			pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_ACK;

			return;
		}
	}


	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		dummy_read = pI2CHandle->pI2Cx->SR2;

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE) == 0);

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking so master will send NACK when receiving the last byte of data. The NACK is to tell the slave it doesn't need any more data
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				if(Repeated_Start_EN == I2C_REPEATED_START_DI)
					//generate the stop condition
					pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_STOP;
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

		if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
			//re-enable ACKing
			pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_ACK;

			return;
		}

	}



}


/*********************************************************************************************
 * @fn			- I2C_MasterReceiveDataIT
 *
 * #brief		- receive data thru I2C peripheral in master mode, non polling function
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, pointer variable, input the address of the structure variable
 * @param[in]	- uint8_t *pTxBuffer, pointer variable, input the address of the data to be sent
 * @param[in]	- uint32_t Len, input the length of the data to be sent
 * @param[in]	- uint8_t SlaveAddr, input the slave address that will receive the data
 *
 * @return		- none
 *
 * @Note 		-
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Repeated_Start_EN)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DeviceAddr = SlaveAddr;
		pI2CHandle->Sr = Repeated_Start_EN;

		//Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_START;

		//Implement the code to enable ITBUFEN Control Bit, this enables interrupt for when RxNE = 1
		pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITBUFEN;

		//Implement the code to enable ITEVTEN Control Bit, this enables interrupt coming from event triggers like RxNE = 1
		pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITEVTEN;

		//Implement the code to enable ITERREN Control Bit. this enables interrupt coming from errors during reception
		pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITERREN;

	}

	return busystate;
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
 * @fn			- Get_TriseValue
 *
 * #brief		- calculates the Trise value to be programmed into Trise register with the curent PCLK1
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, structure address of the i2c handler.
 *
 * @return		- uint8_t, returns the calculated trise value.
 *
 * @Note 		-
 */
uint8_t Get_TriseValue(I2C_Handle_t *pI2CHandle){

	uint8_t trise_val;
	uint32_t pclk1;

	uint32_t SCLSpeed = pI2CHandle->I2C_Config.I2C_SCLSpeed;

	pclk1 = RCC_GetPCLK1Value();


	if(SCLSpeed == I2C_SCL_SPEED_STANDARD){
		//if SCLSpeed is standard mode use 1000ns as numerator as per i2c spec
		trise_val = ( 1000e-9 * pclk1 ) + 1;
	}else{
		//If SCLSpeed is fast mode use 300ns as numerator as per i2c spec
		trise_val = ( 300e-9 * pclk1 ) + 1;
	}

	return trise_val;
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
		flag_status = (pI2Cx->SR2 >> FlagName) & 0x0001;
	}

	return flag_status;
}


/*********************************************************************************************
 * @fn			- i2c_start_address_phase
 *
 * #brief		- helper function to generate START condition and send slave address.
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, pointer variable, input the address of the structure variable
 * @param[in]	- uint8_t SlaveAddr, input the target slave address
 * @param[in]	- uint8_t rw_bit, read/write bit, Read = 1 & write = 0.
 *
 * @return		- uint8_t, returns the flag status
 *
 * @Note 		-
 */
static void i2c_startphase_addressphase(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t rw_bit)
{

	//generate START condition
	pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_START;

	//wait for SB flag to be set which means start bit executed successfully at the same tme read SR1
	while( I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB) == 0 );

	//sets the r/nw bit to write mode
	SlaveAddr = SlaveAddr << 1;

	//master is in write mode
	if(rw_bit == I2C_MASTER_WRITE)
		SlaveAddr &= ~(1 << 0);

	//master is in read mode
	else
		SlaveAddr |= 1 << 0;

	//write the slave address to be sent thru which also clears the SB flag.
	pI2CHandle->pI2Cx->DR = SlaveAddr;

	//waits for ADDR flag to be set which means address bit sent successfully at the same time reads SR1
	while( I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR) == 0 );

}


/*********************************************************************************************
 * @fn			- I2C_IRQInterruptConfig
 *
 * #brief		- Enables or disables the interrupt for the given IRQ number.
 *
 * @param[in]	- IRQNumber of the SPI peripheral. Has already defined macros refer to @IRQ_NUMBERS
 * @param[in]	- IRQpriority, values 0 - 255 the lower the value the higher the priority of the interrupt
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	/*
	 * iNVIC selects which NVIC_ISER register to program by dividing the IRQNumber by 32 which
	 * returns the NVIC_ISER index.
	 * EX. IRQNumber = 76 (which belongs to NVIC_ISER2 register), iNVIC = 76 / 32 = 2
	 *     Integer part only gets stored since variable is uint8_t.
	 * Lshift_val calculates the amount of left shift needed to program the specific bitfield in the NVIC
	 * ISER register.
	 * EX. IRQNumber = 76, Lshift_val = 76 % 32 = 12, so the needed left shift to program the bitfield
	 * for IRQNumber 76 is 12.
	 */
	uint8_t iNVIC = IRQNumber / 32;
	uint8_t Lshift_val = IRQNumber % 32;

	if(EnorDi == ENABLE){
		*(NVIC_ISER_BASE_ADDR + iNVIC) |= (1 << Lshift_val);
	}
	else{
		*(NVIC_ICER_BASE_ADDR + iNVIC) |= (1 << Lshift_val);
	}
}


/*********************************************************************************************
 * @fn			- I2C_IRQPriorityConfig
 *
 * #brief		- Sets the priority number of a specific IRQ number. Kinda like priority number queuing for interrupts.
 *
 * @param[in]	- IRQNumber of the SPI peripheral. Has already defined macros refer to @IRQ_NUMBERS
 * @param[in]	- IRQpriority, values 0 - 255 the lower the value the higher the priority of the interrupt
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4; //gets the IPR register index for setting
	uint8_t iprx_section = IRQNumber % 4; //determines which section of the IPR register to set.
	uint8_t Lshift_val = (8 - NUM_PRIO_BITS_IMPLEMENTED) + (8 * iprx_section);

	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << Lshift_val);
}


/*********************************************************************************************
 * @fn			- I2C_Event_IRQHandling
 *
 * #brief		- Handles interrupts triggered by events during I2C transmission and reception
 *
 * @param[in]	- I2C_Handle_t *pI2CHandle, address of I2C Handle
 *
 * @return		- none
 *
 * @Note 		-
 */
void I2C_Event_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//interrupt handling for both master and slave mode of device
	uint8_t temp1, temp2, temp3;
	uint16_t dummy_read __unused;

	//check if ITEVTEN is set.
	temp1 = (pI2CHandle->pI2Cx->CR2 >> I2C_CR2_ITEVTEN) & 0x0001;

	//check if ITBUFEN is set. (For Tx and Rx buffer interrupts)
	temp2 = (pI2CHandle->pI2Cx->CR2 >> I2C_CR2_ITBUFEN) & 0x0001;


/*********** Handle interrupt generated by SB event. Note: SB flag is only applicable in Master mode *****************/
	//check SB flag if it is set
	temp3 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB);
	//check if both interrupt flag are set for SB to trigger
	if(temp1 && temp3)
	{
		//SB flag is set, start with address phase

		//left shift device address value by one to make space for R/nW bit
		pI2CHandle->DeviceAddr <<= 1;

		//SB flag was triggered by master write mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			pI2CHandle->DeviceAddr &= ~(1 << 0);

		//SB flag was triggered by master read
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			pI2CHandle->DeviceAddr |= 1 << 0;

		//write the slave address to data register to be transmitted also clears the SB flag
		pI2CHandle->pI2Cx->DR = pI2CHandle->DeviceAddr;
	}


/************************ Handle interrupt generated by ADDR event ****************************************/
	//Note: When master mode: Address is sent ; When slave mode: address is matched with own address
	temp3 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR);
	if(temp1 && temp3)
	{	//ADDR flag is set

		//if i2c is busy in Rx & only send 1 byte of data & in master mode. Most likely requesting for data length from slave at the moment execute this
		//note that i did not read from SR1 AND SR2  to clear the ADDR flag because we've already read from SR1 flag from initializing temp3 above
		//we've already read from SR2 in the if statement below, wouldn't that clear the ADDR flag before disabling the ack right? IDK really well just see. haha
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX && pI2CHandle->RxSize == 1 && I2C_GetSR2FlagStatus(pI2CHandle->pI2Cx, I2C_SR2_MSL) == 1)
		{
			//disable acking
			pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

			//read into SR1 and SR2 to clear ADDR flag hypothetically but i think we dont need this part here na see previous comment
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;

		}else{

			//clear ADDR flag only if using as TX mode (master mode) or using as Rx mode with len > 1 (master mode)
			//or using the device as slave mode
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
		}
	}


/***************** Handle interrupt generated by BTF (byte transfer finished) event ************************/
	temp3 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		/*
		 * BTF flag is set plus if busy in TX (transmitting data) & TXE = 1 & TxLen = 0,
		 * which all means that data transmission is finished and close the data transmission
		 */
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX && I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE) == 1 && pI2CHandle->TxLen == 0)
		{
			if(pI2CHandle->Sr == I2C_REPEATED_START_DI)
				//generate STOP condition
				pI2CHandle->pI2Cx->CR1 |= I2C_CR1_STOP;

			//Reset all member elements of the I2C handle structure
			I2C_CloseSendData(pI2CHandle);


			//notify the application that transmission is complete
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
		}
		// note: if TxRxState == BUSY IN RX, do nothing, we don't terminate the reception in this code block
	}

	//Handle interrupt generated by STOPF event (Slave mode only)
	temp3 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOPF flag is set

		//followed by write into CR1 to clear the STOP Flag (dummy write)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify the application that STOP is detected by slave
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);

	}


/******************** Handle interrupt generated by TxE event ******************************************/
	temp3 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		//TXE flag is set
		//check if device is busy in transmission & TxLen > 0 (has data left to send) & device is in master mode.
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX && pI2CHandle->TxLen > 0 && I2C_GetSR2FlagStatus(pI2CHandle->pI2Cx, I2C_SR2_MSL) == 1)
		{
			//load the data into Data Register
			pI2CHandle->pI2Cx->DR = *(pI2CHandle->pRxBuffer);

			//decrement Txlen
			(pI2CHandle->TxLen)--;

			//increment TxBuffer address
			(pI2CHandle->pTxBuffer)++;
		}
	}


/*********************** Handle interrupt generated by RxNE event ******************************/
	temp3 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		//RXNE flag is set
		//if if it RX mode and in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX && I2C_GetSR2FlagStatus(pI2CHandle->pI2Cx, I2C_SR2_MSL) == 1)
		{

			I2C_MasterRxNEInterrupt_Handler(pI2CHandle);
		}
	}


}


/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             - Handles the interrupt generated by error events during i2c data transmission/reception
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle, base address of i2c handle
 *
 * @return            -
 *
 * @Note              - its mostly just clearing the error bit field.

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2 >> I2C_CR2_ITERREN) & 0x0001;


/***********************Check for Bus error************************************/
	temp1 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ARLO);
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}



//helper function because code is too long, for readability
static void I2C_MasterRxNEInterrupt_Handler(I2C_Handle_t *pI2CHandle){

	//if only 1 byte is to be received. Most likely retrieving data length from slave
	if(pI2CHandle->RxSize == 1){

		//read data register and write into RxBuffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

		//decrement Rxlen since 1 byte has been received
		(pI2CHandle->RxLen)--;
<<<<<<< HEAD
=======

>>>>>>> i2c_driver
	}

	//if multiple bytes need to be received execute this
	if(pI2CHandle->RxSize > 1){
		//if only 2 bytes left to receive prepare the master to NACK the last byte
		if(pI2CHandle->RxLen == 2)
			//disable acking
			pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);


		//read data register and write into RxBuffer (receive the data)
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

		//decrement Rxlen since 1 byte has been received
		(pI2CHandle->RxLen)--;

		//incement rxBuffer address to store the next byte into
		(pI2CHandle->pRxBuffer)++;
	}

	//if all bytes are received, close i2c reception and notify application
	if(pI2CHandle->RxLen == 0){

		//generate stop condition
		if(pI2CHandle->Sr == I2C_REPEATED_START_DI)
			pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_STOP;

		//close the I2c Rx
		I2C_CloseReceiveData(pI2CHandle); // to be implemented


		//Notify Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT); //to be implemented
	}


}


static void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){

	//disable I2C_CR2_ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//disable I2C_CR2_ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		//re-enable ACKing
		pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_ACK;
	}
}


static void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){

	//disable I2C_CR2_ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//disable I2C_CR2_ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;


}



__weak void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t event)
{

}


































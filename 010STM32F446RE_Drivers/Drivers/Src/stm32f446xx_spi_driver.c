/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 23, 2025
 *      Author: eldon
 */

#include <stdint.h>
#include "stm32f446xx_spi_driver.h"

/*
 * Set as static functions since function will only be accessed in this driver file.
 * User will not have access to these functions.
 */
static void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_InterruptHandle(SPI_Handle_t *pSPIHandle);

/*********************************************************************************************
 * @fn			- SPI_PeripheralClkCtrl
 *
 * #brief		- enables or disables the peripheral clock for the given SPI port
 *
 * @param[in]	- SPI_RegDef_t *pSPIx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t EnorDi, input peripheral clock state here either "ENABLE" or "DISABLE"
 *
 * @return		- none
 *
 * @Note 		-
 */
void SPI_PeripheralClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	uint8_t temp1 = SPI_BASEADDR_TO_CODE(pSPIx);

	if(EnorDi == ENABLE){
		/*
		 * I've used switch case statements for better readability compared to previous
		 * version of else if statements.
		 */
		switch(temp1){
		case 0:
			SPI1_PCLK_EN();
			break;
		case 1:
			SPI2_PCLK_EN();
			break;
		case 2:
			SPI3_PCLK_EN();
			break;
		case 3:
			SPI4_PCLK_EN();
			break;
		default:
			break;
		}

	}
	else{
		switch(temp1){
		case 0:
			SPI1_PCLK_DI();
			break;
		case 1:
			SPI2_PCLK_DI();
			break;
		case 2:
			SPI3_PCLK_DI();
			break;
		case 3:
			SPI4_PCLK_DI();
			break;
		default:
			break;
		}

	}
}



/*********************************************************************************************
 * @fn			- SPI_Init
 *
 * #brief		- Initializes the SPI peripheral according to the user settings
 *
 * @param[in]	- SPI_Handle_t *pSPIHandle, pointer variable, input the address of the structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//tempreg acts a placeholer for the CR1 register.
	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;


	//2. configure bus config
	switch(pSPIHandle->SPIConfig.SPI_BusConfig)
	{
	case 1:		//SPI_BUSCONF_FULLDUPLEX
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		break;
	case 2:		//SPI_BUSCONF_HALFDUPLEX
		//bidi mode should be set
		tempreg |= 1 << SPI_CR1_BIDIMODE;
		break;
	case 3:		//SPI_BUSCONF_SIMPLEX_RXONLY
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

		//rxonly bit must be set
		tempreg |= 1 << SPI_CR1_RXONLY;
		break;
	}

	//3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BAUDRATE;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. Configure the Software Slave Management (SSM)
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	//writes the value of tempreg to the control register 1 to initialize the SPI peripheral
	pSPIHandle->pSPIx->CR1 = tempreg;
}


/*********************************************************************************************
 * @fn			- SPI_DeInit
 *
 * #brief		- Resets the SPI peripheral settings and reverts it back to reset values.
 *
 * @param[in]	- SPI_Handle_t *pSPIHandle, pointer variable, input the address of the structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	uint8_t temp1 = SPI_BASEADDR_TO_CODE(pSPIx);

	switch(temp1){
	case 0:
		SPI1_REG_RESET();
		break;
	case 1:
		SPI2_REG_RESET();
		break;
	case 2:
		SPI3_REG_RESET();
		break;
	case 3:
		SPI4_REG_RESET();
		break;
	default:
		break;
	}
}


/*********************************************************************************************
 * @fn			- SPI_Periperal_EnDi
 *
 * #brief		- Enables or disables the SPI peripheral
 *
 * @param[in]	- SPI_RegDef_t *pSPIx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t EnorDi, input peripheral clock state here either "ENABLE" or "DISABLE"
 *
 * @return		- none
 *
 * @Note 		-
 */
void SPI_PeriperalCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*********************************************************************************************
 * @fn			- SPI_SSI_EnDi
 *
 * #brief		- Enables or disables the SSI bit in the SPI Control register 1
 *
 * @param[in]	- SPI_RegDef_t *pSPIx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t EnorDi, input peripheral clock state here either "ENABLE" or "DISABLE"
 *
 * @return		- none
 *
 * @Note 		-
 */
void SPI_SSI_EnDi(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*********************************************************************************************
 * @fn			- SPI_SendData
 *
 * #brief		- Sends data using the SPI peripheral
 *
 * @param[in]	- SPI_RegDef_t *pSPIx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t *pTxBuffer, pointer variable, input the address of the data buffer to be sent
 * @param[in]	- uint32_t Len, input the length of the data to be sent in bytes.
 *
 * @return		- none
 *
 * @Note 		- This is a blocking call, the function will not return until all the data is sent. Inefficient.
 *                there is a better way to do this using interrupts or DMA
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//extracts data from TXE and check if its equal to 0 (TXBuffer is not empty).
		//breaks from the loop if TXE = 1, which means TXBuffer is empty and ready to write data in.
		while( SPI_GetFlagStatus(pSPIx, SPI_SR_TXE) == FLAG_RESET );


		//checks whether DFF is set to 8bit = 0 or 16bit = 1 data frame.
		uint8_t temp1 = ((pSPIx->CR1 >> SPI_CR1_DFF) & 0x0001);

		switch(temp1)
		{
		case 0: //8Bit DFF
			//Write the data into the SPI Data Register. (To send)
			pSPIx->DR = *pTxBuffer;

			//Decrements data length by 1 since 1 byte of data had been sent.
			Len--;

			//Increments pointer address to next byte to be sent in the next loop
			pTxBuffer++;
			break;

		case 1: //16Bit DFF
			//Typecaseted to uint16_t since sending 2 bytes data. Write the data into the SPI Data Register. (To send)
			pSPIx->DR = *((uint16_t*)pTxBuffer);

			//Decrements data length by 2 since 2 bytes of data had been sent.
			Len -= 2;

			//Increments pointer address to next 2 bytes to be sent in the next loop
			(uint16_t*)pTxBuffer++;
			break;
		default:
			break;
		}

	}
}



/*********************************************************************************************
 * @fn			- SPI_SendDataIT
 *
 * #brief		- Sends data using the SPI peripheral. Using interrupts.
 *
 * @param[in]	- SPI_RegDef_t *pSPIx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t *pRxBuffer, pointer variable, input the address of the data buffer to be received
 * @param[in]	- uint32_t Len, input the length of the data to be received in bytes.
 *
 * @return		- none
 *
 * @Note 		- This is not a blocking call, the function will return immediately after enabling the interrupt. More efficient
 * 				- This gets executed once only during data transmission, the SPI IRQ handling does the rest of the data transmission
 * 				- this set-upd the address of the TX buffer, length of data in bytes, and unmask the TSEIE so interrupt will push through.
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	//Check if SPI is busy in transmission.
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		//1. Store the Tx buffer address and length information in the handle structure
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission. So no other code can take over the SPI peripheral until the transmission is done.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE bit to enable interrupt generation when TXE flag is set in the SR (TXBuffer empty and ready to write data)
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled in the ISR code.


	}

	return state;

}


/*********************************************************************************************
 * @fn			- SPI_ReceiveData
 *
 * #brief		- Receives data from a master or slave using the SPI peripheral
 *
 * @param[in]	- SPI_RegDef_t *pSPIx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t *pRxBuffer, pointer variable, input the address of the data buffer to be received
 * @param[in]	- uint32_t Len, input the length of the data to be received in bytes.
 *
 * @return		- none
 *
 * @Note 		- This is a blocking call, the function will not return until all the data is received. Inefficient.
 *                there is a better way to do this using interrupts or DMA
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//extracts data from RXNE and check if its equal to 1 (RXBuffer is not empty).
		//breaks from the loop if RXNE = 0, which means RXBuffer is empty and ready to write data in.
		while( ((pSPIx->SR >> SPI_SR_RXNE) & 0x0001 ) == FLAG_SET );


		//checks whether DFF is set to 8bit = 0 or 16bit = 1 data frame.
		uint8_t temp1 = ((pSPIx->CR1 >> SPI_CR1_DFF) & 0x0001);

		switch(temp1)
		{
		case 0: //8Bit DFF
			//Write the data into the RXBuffer. (To receive)
			*pRxBuffer = (uint8_t) pSPIx->DR;

			//Decrements data length by 1 since 1 byte of data had been received.
			Len--;

			//Increments pointer address to next byte to be received in the next loop
			pRxBuffer++;
			break;

		case 1: //16Bit DFF
			//Typecaseted to uint16_t since receving 2 bytes data. Write the data into the RXBuffer. (To receive)
			*((uint16_t*)pRxBuffer) = (uint16_t) pSPIx->DR;

			//Decrements data length by 2 since 2 bytes of data had been received.
			Len -= 2;

			//Increments pointer address to next 2 bytes to be sent in the next loop
			(uint16_t*)pRxBuffer ++;
			break;
		default:
			break;
		}

	}
}


/*********************************************************************************************
 * @fn			- SPI_ReceiveDataIT
 *
 * #brief		- Receives data from a master or slave using the SPI peripheral. Using interrupts.
 *
 * @param[in]	- SPI_RegDef_t *pSPIx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t *pRxBuffer, pointer variable, input the address of the data buffer to be received
 * @param[in]	- uint32_t Len, input the length of the data to be received in bytes.
 *
 * @return		- none
 *
 * @Note 		- This is not a blocking call, the function will return immediately after enabling the interrupt. More efficient
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Check if SPI is busy in reception.
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {
		//1. Store the Rx buffer address and length information in the handle structure
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in reception. So no other code can take over the SPI peripheral until the reception is done.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE bit to enable interrupt generation when RXNE flag is set in the SR (RXBuffer not empty)
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		return pSPIHandle->TxState;

		//4. Data reception will be handled in the ISR code.
	}

	return state;
}


/*********************************************************************************************
 * @fn			- SPI_GetFlagStatus
 *
 * #brief		- Extracts the status of a flag from the status register
 *
 * @param[in]	- SPI_RegDef_t *pSPIx, pointer variable, input the SPI port base address here
 * @param[in]	- uint8_t FlagName, input the flag name to be checked refer to @SPI_SR_BIT_FIELDS
 *
 * @return		- returns the flag status.
 *
 * @Note 		-
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName) {

	//gets the status of the flag from the status register
	uint8_t flag_status = (pSPIx->SR >> FlagName) & 0x0001;

	//return the flag status
	return flag_status;

}


/*********************************************************************************************
 * @fn			- SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	/*
	 * Previously used switch statements in choosing the ISER & ICER index, switched to pointer arithmetic
	 * algorithm for lesser lines of code and better readability.
	 */

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
 * @fn			- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4; //gets the IPR register index for setting
	uint8_t iprx_section = IRQNumber % 4; //determines which section of the IPR register to set.
	uint8_t Lshift_val = (8 - NUM_PRIO_BITS_IMPLEMENTED) + (8 * iprx_section);

	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << Lshift_val);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//1. Check for TXE flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		//Handle TXE
		SPI_TXE_InterruptHandle(pSPIHandle);
	}

	//2. Check for RXNE flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2) {
		//Handle RXNE
		SPI_RXNE_InterruptHandle(pSPIHandle);
	}

	//3. Check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2) {
		//Handle OVR
		SPI_OVR_InterruptHandle(pSPIHandle);
	}


}


/*********************************************************************************************
 * @fn			- SPI_TXE_InterruptHandle
 *
 * #brief		- Handles the TXE interrupt for the SPI peripheral.
 *
 * @param[in]	- SPI_Handle_t *pSPIHandle, pointer variable, input the address of the structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
static void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	//checks whether DFF is 8bits = 0 or 16bits = 1
	uint8_t DFF = pSPIHandle->SPIConfig.SPI_DFF;

	//If there is still data to be sent in the Tx buffer.
	if(pSPIHandle->TxLen > 0)
	{
		switch(DFF)
		{
		case 0: //8bit DFF
			//Write the data into the SPI Data Register. (To send)
			pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);

			//Decrements data length by 1 since 1 byte of data had been sent.
			pSPIHandle->TxLen--;

			//Increments pointer address to next byte to be sent in the next interrupt
			pSPIHandle->pTxBuffer++;
			break;

		case 1: //16Bit DFF
			//Typecasted to uint16_t since sending 2 bytes data. Write the data into the SPI Data Register. (To send)
			pSPIHandle->pSPIx->DR = *((uint16_t*)(pSPIHandle->pTxBuffer));

			//Decrements data length by 2 since 2 bytes of data had been sent.
			pSPIHandle->TxLen -= 2;

			//Increments pointer address to next 2 bytes to be sent in the next interrupt
			(uint16_t*)pSPIHandle->pTxBuffer++;
			break;

		default:
			break;
		}
	}
	else //if TXlen = 0, which means all data has been sent
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


/*********************************************************************************************
 * @fn			- SPI_RXNE_InterruptHandle
 *
 * #brief		- Handles the RX interrupt for the SPI peripheral.
 *
 * @param[in]	- SPI_Handle_t *pSPIHandle, pointer variable, input the address of the structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
static void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	//checks whether DFF is 8bits = 0 or 16bits = 1
	uint8_t DFF = pSPIHandle->SPIConfig.SPI_DFF;

	//If there is still data to be received from the Rx buffer.
	if(pSPIHandle->RxLen > 0)
	{
		switch(DFF)
		{
		case 0: //8bit DFF
			//Read the data inside the SPI Data Register and store it in the RXBuffer. (To receive)
			*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;

			//Decrements data length by 1 since 1 byte of data had been received.
			pSPIHandle->RxLen--;

			//Increments pointer address to next byte to be received
			pSPIHandle->pRxBuffer++;
			break;

		case 1: //16Bit DFF
			//Read the data inside the SPI Data Register and store it in the RXBuffer. (To receive)
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;

			//Decrements data length by 2 since 2 byte of data had been received.
			pSPIHandle->RxLen -= 2;

			//Increments pointer address to next 2 bytes to be received
			(uint16_t*)pSPIHandle->pRxBuffer++;
			break;

		default:
			break;
		}
	}
	else //if RXlen = 0, which means all data has been received
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}

}


/*********************************************************************************************
 * @fn			- SPI_OVR_InterruptHandle
 *
 * #brief		- Handles the OVR interrupt for the SPI peripheral.
 *
 * @param[in]	- SPI_Handle_t *pSPIHandle, pointer variable, input the address of the structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
static void SPI_OVR_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	//Clears the OVR flag by reading the DR register followed by reading the SR register.
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	(void)temp; // Suppress unused variable warning

	// Notify application about the error
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

//Manual clearing of OVR flag
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp;
}



void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//Disable the TXEIE interrupt so it wont keep trigering since all the data has been sent
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	//Reset the SPI TX state to ready since transmission is done
	pSPIHandle->TxState = SPI_READY;

	//Reset the Tx buffer address and length information in the handle structure
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
}




void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//Disable the RXNEIE interrupt so it wont keep trigerring since all the data has been received
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	//Reset the SPI RX state to ready since transmission is done
	pSPIHandle->RxState = SPI_READY;

	//Reset the Tx buffer address and length information in the handle structure
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;

}

/*
 * This is a weak implementation, application may override this function for a custom
 * Application event Callback
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{

}




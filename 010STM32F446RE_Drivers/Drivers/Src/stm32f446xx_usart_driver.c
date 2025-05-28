/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: May 19, 2025
 *      Author: eldon
 */

#include<stdint.h>
#include "stm32f446xx_usart_driver.h"

/*********************************************************************************************
 * @fn			- USART_PeripheralClkCtrl
 *
 * #brief		- enables or disables the peripheral clock for the given USART/UART port
 *
 * @param[in]	- USART_RegDef_t *pUSARTx pointer variable, input the USARTT/UART port base address here
 * @param[in]	- uint8_t EnorDi, input peripheral clock state here either "ENABLE" or "DISABLE"
 *
 * @return		- none
 *
 * @Note 		-
 */
void USART_PeripheralClkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	uint8_t temp1 = USART_BASEADDR_TO_CODE(pUSARTx);

	if(EnorDi == ENABLE){
		switch(temp1){
		case 0:
			USART1_PCLK_EN();
			break;
		case 1:
			USART2_PCLK_EN();
			break;
		case 2:
			USART3_PCLK_EN();
			break;
		case 3:
			UART4_PCLK_EN();
			break;
		case 4:
			UART5_PCLK_EN();
			break;
		case 5:
			USART6_PCLK_EN();
			break;
		default:
			break;
		}
	}
	else{
		switch(temp1){
		case 0:
			USART1_PCLK_EN();
			break;
		case 1:
			USART2_PCLK_DI();
			break;
		case 2:
			USART3_PCLK_DI();
			break;
		case 3:
			UART4_PCLK_DI();
			break;
		case 4:
			UART5_PCLK_DI();
			break;
		case 5:
			USART6_PCLK_DI();
			break;
		default:
			break;
		}
	}
}


/*********************************************************************************************
 * @fn			- USART_DeInit
 *
 * #brief		- Resets the USART peripheral settings and reverts it back to reset values.
 *
 * @param[in]	- USART_RegDef_t *pUSARTx, pointer variable, USART peripheral base address
 *
 * @return		- none
 *
 * @Note 		-
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	uint8_t temp1 = USART_BASEADDR_TO_CODE(pUSARTx);

	switch (temp1) {
	case 0:
		USART1_REG_RESET();
		break;
	case 1:
		USART2_REG_RESET();
		break;
	case 2:
		USART3_REG_RESET();
		break;
	case 3:
		UART4_REG_RESET();
		break;
	case 4:
		UART5_REG_RESET();
		break;
	case 5:
		USART6_REG_RESET();
		break;
	default:
		break;
	}
}



/*********************************************************************************************
 * @fn			- USART_Init
 *
 * #brief		- Initializes the USART/UART peripheral according to the user settings
 *
 * @param[in]	- USART_Handle_t *pUSARTHandle, pointer variable, input the address of the handle structure variable
 *
 * @return		- none
 *
 * @Note 		-
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg=0;

	/******************************** Configuration of CR1******************************************/

	//enable the Clock for given USART peripheral
	USART_PeripheralClkCtrl(pUSARTHandle->pUSARTx, ENABLE);

	//enable Tx and Rx USART/UART engines according to user configuration
	switch(pUSARTHandle->USART_Config.USART_Mode)
	{
	case USART_MODE_ONLY_RX:
		tempreg |= 1 << USART_CR1_RE; //enable receiver
		break;
	case USART_MODE_ONLY_TX:
		tempreg |= 1 << USART_CR1_TE; //enable transmitter
		break;
	case USART_MODE_TXRX:
		tempreg |= ( (1 << USART_CR1_TE) | (1 << USART_CR1_RE) ); //enable both transmitter and receiver
		break;
	default:
		break;
	}

	//configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	//configure the parity control
	switch(pUSARTHandle->USART_Config.USART_ParityControl)
	{
	case USART_PARITY_EN_EVEN:
		tempreg |= 1 << USART_CR1_PCE; //enable parity control at the same time will enable even parity by default
		break;
	case USART_PARITY_EN_ODD:
		tempreg |= ( (1 << USART_CR1_PCE) | (1 << USART_CR1_PS) ); //enable parity control & configure to odd parity
		//tempreg |= 1 << USART_CR1_PS; //enable odd parity
		break;
	default:
		break;
	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_StopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	switch(pUSARTHandle->USART_Config.USART_HWFlowControl)
	{
	case USART_HW_FLOW_CTRL_CTS:
		tempreg |= 1 << USART_CR3_CTSE; //enable CTS flow control
		break;
	case USART_HW_FLOW_CTRL_RTS:
		tempreg |= 1 << USART_CR3_RTSE; //enable RTS flow control
		break;
	case USART_HW_FLOW_CTRL_CTS_RTS:
		tempreg |= ( (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE) ); //enable CTS flow control & RTS flow control
		//tempreg |= 1 << USART_CR3_RTSE; //enable RTS flow control
		break;
	default:
		break;
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}


/*********************************************************************************************
 * @fn			- USART_SendData
 *
 * #brief		- Sends data through the USART/UART peripheral
 *
 * @param[in]	- USART_Handle_t *pUSARTHandle, pointer variable, input the address of the handle structure variable
 * @param[in]	- uint8_t *pTxBuffer, pointer variable, input the address of the data to be sent
 * @param[in]	- uint32_t Len, input the length of the data to be sent
 *
 * @return		- none
 *
 * @Note 		-
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	for(uint32_t i = 0 ; i < Len; i++)
	{
		//wait until TXE flag is set in the SR
		while( USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_TXE) == 0 );

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

				//increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//increment the buffer address
			pTxBuffer++;
		}

	}

	//wait till TC flag is set in the SR
	while( USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_TC) == 0 );

}



/*********************************************************************************************
 * @fn			- USART_SendDataIT
 *
 * #brief		- Sends data through the USART/UART peripheral using interrupts
 *
 * @param[in]	- USART_Handle_t *pUSARTHandle, pointer variable, input the address of the handle structure variable
 * @param[in]	- uint8_t *pTxBuffer, pointer variable, input the address of the data to be sent
 * @param[in]	- uint32_t Len, input the length of the data to be sent
 *
 * @return		- none
 *
 * @Note 		-
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE); //enable TXE interrupt

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE); //enable TC interrupt

	}

	return txstate;
}



/*********************************************************************************************
 * @fn			- USART_ReceiveData
 *
 * #brief		- Receives data through the USART/UART peripheral
 *
 * @param[in]	- USART_Handle_t *pUSARTHandle, pointer variable, input the address of the handle structure variable
 * @param[in]	- uint8_t *pRxBuffer, pointer variable, input the address of the data to be received
 * @param[in]	- uint32_t Len, input the length of the data to be received
 *
 * @return		- none
 *
 * @Note 		-
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{

	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while( USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_RXNE) == 0);

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//receiveing 9 bits data frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//receiving 8 bits data frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = pUSARTHandle->pUSARTx->DR;;
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}


/*********************************************************************************************
 * @fn			- USART_ReceiveDataIT
 *
 * #brief		- Receives data through the USART/UART peripheral using interrupts
 *
 * @param[in]	- USART_Handle_t *pUSARTHandle, pointer variable, input the address of the handle structure variable
 * @param[in]	- uint8_t *pRxBuffer, pointer variable, input the address of the data to be received
 * @param[in]	- uint32_t Len, input the length of the data to be received
 *
 * @return		- none
 *
 * @Note 		-
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE); //enable RXNE interrupt

	}

	return rxstate;

}


/*********************************************************************************************
 * @fn			- USART_IRQInterruptConfig
 *
 * #brief		- Enables or disables the interrupt for the given IRQ number.
 *
 * @param[in]	- IRQNumber of the USART/UART peripheral. Has already defined macros refer to @IRQ_NUMBERS
 * @param[in]	- IRQpriority, values 0 - 255 the lower the value the higher the priority of the interrupt
 *
 * @return		- none
 *
 * @Note 		-
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	uint8_t iNVIC = IRQNumber / 32; //Gets the NVIC_ISER register index to program
	uint8_t Lshift_val = IRQNumber % 32; //CALCULATES THE LEFT SHIFT VALUE FOR THE BITFIELD

	if(EnorDi == ENABLE){
		//ENABLE THE IRQ NUMBER
		*(NVIC_ISER_BASE_ADDR + iNVIC) |= (1 << Lshift_val);
	}
	else{
		//DISABLE THE IRQ NUMBER
		*(NVIC_ICER_BASE_ADDR + iNVIC) |= (1 << Lshift_val);
	}
}



/*********************************************************************************************
 * @fn			- USART_IRQPriorityConfig
 *
 * #brief		- Sets the priority number of a specific IRQ number. Kinda like priority number queuing for interrupts.
 *
 * @param[in]	- IRQNumber of the USART/UART peripheral. Has already defined macros refer to @IRQ_NUMBERS
 * @param[in]	- IRQpriority, values 0 - 255 the lower the value the higher the priority of the interrupt
 *
 * @return		- none
 *
 * @Note 		-
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4; //gets the IPR register index for setting
	uint8_t iprx_section = IRQNumber % 4; //determines which section of the IPR register to set.
	uint8_t Lshift_val = (8 - NUM_PRIO_BITS_IMPLEMENTED) + (8 * iprx_section);


	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << Lshift_val);
}




/*********************************************************************************************
 * @fn			- USART_GetFlagStatus
 *
 * #brief		- fetches the current flag status of the SR register
 *
 * @param[in]	- USART_RegDef_t *pUSARTx, USART/UART peripheral base address
 * @param[in]	- uint8_t FlagName, input the flag name to be checked
 *
 * @return		- uint8_t, returns the flag status
 *
 * @Note 		-
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{

	//gets the flag status from SR
	uint8_t flag_status = (pUSARTx->SR >> FlagName) & 0x0001;

	//return the flag status
	return flag_status;
}


/*********************************************************************************************
 * @fn			- USART_ClearFlag
 *
 * #brief		- clears the given flag in the SR register
 *
 * @param[in]	- USART_RegDef_t *pUSARTx, USART/UART peripheral base address
 * @param[in]	- uint8_t FlagName, input the flag name to be checked
 *
 * @return		- uint8_t, returns the flag status
 *
 * @Note 		-
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{
	//clears the flag by writing 1 to it
	pUSARTx->SR &= ~(1 << FlagName);
}


/*********************************************************************************************
 * @fn			- USART_PeripheralControl
 *
 * #brief		- Enables or disables the USART/UART peripheral
 *
 * @param[in]	- USART_RegDef_t *pUSARTx, USART/UART peripheral base address
 * @param[in]	- uint8_t EnorDi, input peripheral clock state here either "ENABLE" or "DISABLE"
 *
 * @return		- none
 *
 * @Note 		-
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE); //Enable the peripheral
	} else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE); //Disable the peripheral
	}
}


/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Sets the baudrate of the USART/USART peripheral
 *
 * @param[in]         -	USART_RegDef_t *pUSARTx, USART peripheral base address
 * @param[in]         - uint32_t BuadRate, set baudrate by user.
 *
 * @return            -
 *
 * @Note              -
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t PCLKx;
	uint32_t usartdiv;

	//Mantissa and Fraction variables
	uint32_t M_part,F_part;
	uint32_t tempreg=0;

	//fetch the APB clock value
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		//the rest of USART peripheral are in APB1 bus
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check if OVER8 = 1
	if( pUSARTx->CR1 & (1 << USART_CR1_OVER8) )
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ( (25 * PCLKx) / (2 *BaudRate) );
	}else
	{
		//over sampling by 16
		usartdiv = ( (25 * PCLKx) / (4 * BaudRate) );
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	//check if OVER8 = 1
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		/*
		 * the "+50" is for the roundup factor
		 */
		F_part = ( ( (F_part * 8)+ 50 ) / 100 ) & ((uint8_t)0x07 ); //mask only the first 3 bits
		//as indicated in reference manual to keep 4th bit clear

	}else
	{
		//over sampling by 16
		F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}





/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             - This is the interrupt handler for the USART/UART peripheral
 *
 * @param[in]         - USART_Handle_t *pUSARTHandle, pointer variable, input the address of the handle structure variable
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;

	/*************************Check for TC flag ********************************************/

	//check the state of TC bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TC);

	//check the state of TCEIE bit
	temp2 = (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_TCIE) & 0x0001;

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(pUSARTHandle->TxLen == 0 )
			{
				//clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);

				//Reset application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset length to zero
				pUSARTHandle->TxLen = 0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*************************Check for TXE flag ********************************************/

	//check the state of TXE bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TXE);

	//check the state of TXEIE bit in CR1
	temp2 = (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_TXEIE) & 0x0001;


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX && pUSARTHandle->TxLen > 0 )
		{
			//Keep sending data until Txlen reaches to zero

			//Check USART_WordLength 9BIT or 8BIT data frame
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				uint16_t *pdata;

				//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
				pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

				//check for USART_ParityControl
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used in this transfer , so, 9bits of user data will be sent
					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//Implement the code to increment pTxBuffer twice
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->pTxBuffer++;

					//decrement the length
					pUSARTHandle->TxLen--;
				}
				else
				{
					//Parity bit is used in this transfer . so , 8bits of user data will be sent
					pUSARTHandle->pUSARTx->DR = (*(pUSARTHandle->pTxBuffer)  & (uint8_t)0xFF);

					//The 9th bit will be replaced by parity bit by the hardware
					pUSARTHandle->pTxBuffer++;

					//decrement the length
					pUSARTHandle->TxLen--;
				}
			}
			else
			{
				//This is 8bit data transfer
				pUSARTHandle->pUSARTx->DR = (*(pUSARTHandle->pTxBuffer)  & (uint8_t)0xFF);

				//Implement the code to increment the buffer address
				pUSARTHandle->pTxBuffer++;

				//Implement the code to decrement the length
				pUSARTHandle->TxLen--;
			}

			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE );
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/

	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_RXNE);

	temp2 = (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_RXNEIE) & 0x0001;


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne

		//check if the RxBusyState is USART_BUSY_IN_RX and RxLen > 0
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX && pUSARTHandle->RxLen > 0)
		{
			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				//receive 9bit data in a frame

				//Now, check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used. so, all 9bits will be of user data

					//read only first 9 bits so mask the DR with 0x01FF
					*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

					//Now increment the pRxBuffer two times
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen -= 2;
				}
				else
				{
					//Parity is used. so, 8bits will be of user data and 1 bit is parity
					*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					//Now increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen --;
				}
			}
			else
			{
				//receive 8bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used 8bits will be of user data

					//read 8 bits from DR
					*(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				}

				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					*(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

				}

				//increment the pRxBuffer
				pUSARTHandle->pRxBuffer++;

				//decrement the length
				pUSARTHandle->RxLen --;
			}

			if(pUSARTHandle->RxLen == 0)
			{
				//disable the rxneie
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


	/*************************Check for CTS flag ********************************************/
	//Note : CTS feature is not applicable for UART4 and UART5

	//check the status of CTS bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_CTS);

	//check the state of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 && temp3)
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

	/*************************Check for IDLE detection flag ********************************************/

	//check the status of IDLE flag bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_IDLE);

	//check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);


	if(temp1 && temp2)
	{
		uint32_t dummyread __unusedatr;

		dummyread = pUSARTHandle->pUSARTx->SR; //Read SR to clear IDLE flag
		dummyread = pUSARTHandle->pUSARTx->DR; //Read DR to clear IDLE flag

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

	/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_ORE);
	}



	/*************************Check for Error Flag ********************************************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We dont discuss multibuffer communication in this course. please refer to the RM
	//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}

}





__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{

}











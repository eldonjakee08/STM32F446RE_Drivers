/*
 * stm32f446xx_gpio.c
 *
 *  Created on: Apr 14, 2025
 *      Author: eldon
 */


#include "stm32f446xx_gpio_driver.h"

/*********************************************************************************************
 * @fn			- GPIO_PeriphClkCtrl
 *
 * #brief		- This function enables or disable the peripheral clock for the given GPIO port
 *
 * @param[in]	- GPIO_RegDef_t *pGPIOx, pointer variable, input the GPIO port base address here
 * @param[in]	- uint8_t EnorDi, input peripheral clock state here either "ENABLE" or "DISABLE"
 * 				  macros as defined in stm32f446xx.h file
 *
 * @return		- none
 *
 * @Note 		-
 */
void GPIO_PeripheralClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	/*
	 * translates the GPIO port base addresses into primitive integer data for
	 * ease of use in switch case statement or other applications that require
	 * a more primitive type of data.
	 * note: GPIO port base addresses which are pointers are not primitive data types
	 */
	uint8_t temp1 = GPIO_BASEADDR_TO_CODE(pGPIOx);

	if(EnorDi == ENABLE){
		/*
		 * I've used switch case statements for better readability compared to previous
		 * version of else if statements.
		 */
		switch(temp1){
		case 0:
			GPIOA_PCLK_EN();
			break;
		case 1:
			GPIOB_PCLK_EN();
			break;
		case 2:
			GPIOC_PCLK_EN();
			break;
		case 3:
			GPIOD_PCLK_EN();
			break;
		case 4:
			GPIOE_PCLK_EN();
			break;
		case 5:
			GPIOF_PCLK_EN();
			break;
		case 6:
			GPIOG_PCLK_EN();
			break;
		case 7:
			GPIOH_PCLK_EN();
			break;
		default:
			break;
		}

	}
	else{
		switch(temp1){
		case 0:
			GPIOA_PCLK_DI();
			break;
		case 1:
			GPIOB_PCLK_DI();
			break;
		case 2:
			GPIOC_PCLK_DI();
			break;
		case 3:
			GPIOD_PCLK_DI();
			break;
		case 4:
			GPIOE_PCLK_DI();
			break;
		case 5:
			GPIOF_PCLK_DI();
			break;
		case 6:
			GPIOG_PCLK_DI();
			break;
		case 7:
			GPIOH_PCLK_DI();
			break;
		default:
			break;
		}

	}

}


/*********************************************************************************************
 * @fn			- GPIO_Init
 *
 * #brief		- Initializes the GPIO of the intended settings by the user.
 *
 * @param[in]	- GPIO_Handle_t *pGPIOHandle, address of the structure variable GPIO_Handle_t.
 *
 * @return		- none
 *
 * @Note 		-
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_PeripheralClkCtrl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp=0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG ){

		//1. Configure mode of GPIO pin
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) ); //clears the bit fields first before setting
		pGPIOHandle->pGPIOx->MODER |= temp; //sets the bit fields in the register
		temp = 0;
	}
	else{
		//interrupt mode GPIO initialization
		//1. pin must be in input configuration
		//2. configure the edge trigger (RT,FT,RFT)
		//3. ENABLE INTERRUPT DELIVERY FROM PERIPHERAL TO THE PROCEssor (on peripheral side)

		//clears bits at the same time setting to input mode since input mode is "00".
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

		switch(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode){
		case GPIO_MODE_IT_FT:
			//1. configure FTSR (Falling edge trigger interrupt)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit can you want it solely as falling edge trigger
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			break;
		case GPIO_MODE_IT_RT:
			//1. configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit can you want it solely as rising edge trigger
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			break;
		case GPIO_MODE_IT_RFT:
			//1. configure both FTSR & RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			break;
		default:
			break;
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		/*
		 * same logic with selecting which alternate function mode peripheral to program.
		 * This part is where it chooses the specific EXTI line to program.
		 * temp1 chooses the specific EXTICR peripheral register (out of 4) to program
		 * With the given pin number, we then calculate how much left shift the bits need to program the specific
		 * EXTI which the GPIO number is associated with and stored into Lshift_val. EXTI0 = PIN 0's ----> EXTI15 = PIN 15's.
		 */
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t Lshift_val = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		/*translates the given GPIO port address into a specific GPIO port code for programming
		 * the SYSCFG EXTICR peripheral register.*/
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] &= ~(15 << (4*Lshift_val) ); //clears peripheral register bitfield
		SYSCFG->EXTICR[temp1] |= ( portcode << (4*Lshift_val) ); //clears peripheral register bitfield


		/*
		 * /3. enable the EXTI interrupt delivery using IMR.
		 *     Basically unmasks the interrupt that will run through the specific EXTI line
		 */
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2. Configure pup pud settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) ); //clears the bit fields first before setting
	pGPIOHandle->pGPIOx->PUPDR |= temp; //sets the bit fields
	temp = 0;

	//3. Configure output type & output speed

	//Only executes if user wants to set GPIO as output mode because no pushpull or open drain config for GPIO input mode
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); //clears the bit fields
	pGPIOHandle->pGPIOx->OTYPER |= temp; //sets the bit fields
	temp = 0;

	//sets GPIO output speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) ); //clears the bit fields
	pGPIOHandle->pGPIOx->OSPEEDER |= temp; //sets the bit fields
	temp = 0;


	//4. Configure alternate function setting (if GPIO is set at alternate function mode)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		/*
		 * The resulting quotient selects whether AFRL = 0 or AFRH = 1 to be used as index in the AFR array
		 * Logic behind: any number that is less than 8 divided by 8 will result in a quotient of 0 since only the integer part is stored
		 * in the variable hence it will  use the 0 index which the AFRL address is at.
		 * If the number (pin number) is greater than 8, the resulting quotient is 1 hence will use the index 1 which the AFRH address is at
		 */
		uint8_t temp1 = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 8);

		/*
		 * the resulting modulo determines the left shift needed for setting the peripheral register
		 */
		uint8_t Lshift_val = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8);

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAFMode << (4 * Lshift_val) );
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(15 << (4 * Lshift_val) ); //clears the bit fields in the register before setting
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;  //Sets the bit fields in the register
		temp = 0;
	}

}



/*********************************************************************************************
 * @fn			- GPIO_Init2
 *
 * #brief		- Initializes the GPIO of the intended settings by the user. Instead of using a structure to define the GPIO configuration
 * 	              I've modified it to input the GPIO configuration directly into the function input parameters it's less hassle for me. Trade-off is
 * 	              the function has 7 input parameters which could be long.
 *
 * @param[in]	- GPIO_RegDef_t *pGPIOPort_BaseAddr, base address of the GPIO port.
 * @param[in]	- uint8_t PinNumber, pin number of GPIO pin 0 to 15. Possible inputs @GPIO_PIN_NUMBERS at GPIO driver header file.
 * @param[in]	- uint8_t PinMode, desired pin mode for GPIO pin. Possible inputs @GPIO_PIN_MODES at GPIO driver header file.
 * @param[in]	- uint8_t PUPD_ResMd, enable GPIO pull-up or pull-down resistors. Possible inputs @GPIO_PUPD at GPIO driver header file.
 *
 ************************************************ OUTPUT MODE SETTING ONLY *******************************************************************
 * @param[in]	- uint8_t OP_PinSpeed, desired pin speed (slew rate) of output pin. Possible inputs @GPIO_OUTPUT_SPEED at GPIO driver header file.
 * @param[in]	- uint8_t OPType, selects output type between push-pull or open-drain. Possible inputs @GPIO_OUTPUT_TYPES at GPIO driver header file.

 ************************************************ ALTERNATE FUNCTION MODE SETTING ONLY***********************************************************
 * @param[in]	- uint8_t AFMode_Sel, Selects the alternate function of the GPIO pin. Possible inputs @GPIO_ALTERNATE_FUNCTION_MODES at GPIo driver header file.
 *
 *
 * @return		- none
 *
 * @Note 		-
 */
void GPIO_Init2(GPIO_RegDef_t *pGPIOPort_BaseAddr, uint8_t PinNumber, uint8_t PinMode,  uint8_t PUPD_ResMd, uint8_t OP_PinSpeed, uint8_t OPType, uint8_t AFMode_Sel)
{

	if(PinMode <=GPIO_MODE_ANALOG ){

		//1. Configure mode of GPIO pin
		pGPIOPort_BaseAddr->MODER &= ~(3 << (2 * PinNumber) ); //clears the bit fields first before setting
		pGPIOPort_BaseAddr->MODER |= (PinMode << (2 * PinNumber) ); //sets the bit fields in the register

	}
	else{
		//interrupt mode GPIO initialization
		//1. pin must be in input configuration
		//2. configure the edge trigger (RT,FT,RFT)
		//3. ENABLE INTERRUPT DELIVERY FROM PERIPHERAL TO THE PROCEssor (on peripheral side)

		//clears bits at the same time setting to input mode since input mode is "00".
		pGPIOPort_BaseAddr->MODER &= ~(3 << (2 * PinNumber) );

		switch(PinMode){
		case GPIO_MODE_IT_FT:
			//1. configure FTSR (Falling edge trigger interrupt)
			EXTI->FTSR |= (1 << PinNumber);
			//clear the corresponding RTSR bit can you want it solely as falling edge trigger
			EXTI->RTSR &= ~(1 << PinNumber);
			break;
		case GPIO_MODE_IT_RT:
			//1. configure RTSR
			EXTI->RTSR |= (1 << PinNumber);
			//clear the corresponding FTSR bit can you want it solely as rising edge trigger
			EXTI->FTSR &= ~(1 << PinNumber);
			break;
		case GPIO_MODE_IT_RFT:
			//1. configure both FTSR & RTSR
			EXTI->FTSR |= (1 << PinNumber);
			EXTI->RTSR |= (1 << PinNumber);
			break;
		default:
			break;
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		/*
		 * same logic with selecting which alternate function mode peripheral to program.
		 * This part is where it chooses the specific EXTI line to program.
		 * temp1 chooses the specific EXTICR peripheral register (out of 4) to program
		 * With the given pin number, we then calculate how much left shift the bits need to program the specific
		 * EXTI which the GPIO number is associated with and stored into Lshift_val. EXTI0 = PIN 0's ----> EXTI15 = PIN 15's.
		 */
		uint8_t temp1 = PinNumber / 4;
		uint8_t Lshift_val = PinNumber % 4;

		/*translates the given GPIO port address into a specific GPIO port code for programming
		 * the SYSCFG EXTICR peripheral register.*/
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOPort_BaseAddr);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] &= ~(15 << (4*Lshift_val) ); //clears peripheral register bitfield
		SYSCFG->EXTICR[temp1] |= ( portcode << (4*Lshift_val) ); //clears peripheral register bitfield


		/*
		 * /3. enable the EXTI interrupt delivery using IMR.
		 *     Basically unmasks the interrupt that will run through the specific EXTI line
		 */
		EXTI->IMR |= (1 << PinNumber);


	}

	//2. Configure pup pud settings
	pGPIOPort_BaseAddr->PUPDR &= ~(3 << (2 * PinNumber) ); //clears the bit fields first before setting
	pGPIOPort_BaseAddr->PUPDR |= (PUPD_ResMd << (2 * PinNumber) ); //sets the bit fields

	//3. Configure output type & output speed(if set only at output mode)
	if(PinMode == GPIO_MODE_OUTPUT){

		//Only executes if user wants to set GPIO as output mode because no pushpull or open drain config for GPIO input mode
		pGPIOPort_BaseAddr->OTYPER &= ~(1 << PinNumber ); //clears the bit fields
		pGPIOPort_BaseAddr->OTYPER |= (OPType << PinNumber); //sets the bit fields

		//sets GPIO output speed
		pGPIOPort_BaseAddr->OSPEEDER &= ~(3 << (2 * PinNumber) ); //clears the bit fields
		pGPIOPort_BaseAddr->OSPEEDER |= (OP_PinSpeed << (2 * PinNumber) ); //sets the bit fields

	}

	//4. Configure alternate function setting (if GPIO is set at alternate function mode)
	if(PinMode == GPIO_MODE_ALTFN){
		/*
		 * The resulting quotient selects whether AFRL = 0 or AFRH = 1 to be used as index in the AFR array
		 * Logic behind: any number that is less than 8 divided by 8 will result in a quotient of 0 since only the integer part is stored
		 * in the variable hence it will  use the 0 index which the AFRL address is at.
		 * If the number (pin number) is greater than 8, the resulting quotient is 1 hence will use the index 1 which the AFRH address is at
		 */
		uint8_t temp1 = (PinNumber / 8);

		/*
		 * the resulting modulo determines the left shift needed for setting the peripheral register
		 */
		uint8_t Lshift_val = (PinNumber % 8);
		pGPIOPort_BaseAddr->AFR[temp1] &= ~(15 << (4 * Lshift_val) ); //clears the bit fields in the register before setting
		pGPIOPort_BaseAddr->AFR[temp1] |= (AFMode_Sel << (4 * Lshift_val) );  //Sets the bit fields in the register

	}

}


/*********************************************************************************************
 * @fn			- GPIO_DeInit
 *
 * #brief		- Resets the GPIO port settings and reverts it back to reset values.
 *
 * @param[in]	- GPIO_RegDef_t *pGPIOx, GPIO base address.
 *
 * @return		- none
 *
 * @Note 		-
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	uint8_t temp1 = GPIO_BASEADDR_TO_CODE(pGPIOx);

	switch(temp1){
	case 0:
		GPIOA_REG_RESET();
		break;
	case 1:
		GPIOB_REG_RESET();
		break;
	case 2:
		GPIOC_REG_RESET();
		break;
	case 3:
		GPIOD_REG_RESET();
		break;
	case 4:
		GPIOE_REG_RESET();
		break;
	case 5:
		GPIOF_REG_RESET();
		break;
	case 6:
		GPIOG_REG_RESET();
		break;
	case 7:
		GPIOH_REG_RESET();
		break;
	default:
		break;
	}
}


/*********************************************************************************************
 * @fn			- GPIO_ReadInputPin
 *
 * #brief		- Reads the GPIO input of a specific GPIO pin from the GPIO input data register
 *
 * @param[in]	- GPIO_RegDef_t *pGPIOx, GPIO port base address.
 * @param[in]	- uint8_t PinNumber, GPIO Pin Numbers 0 to 15.
 *
 * @return		- uint8_t
 *
 * @Note 		-
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	/*
	 * Logic behind: extracts the value from the Input Data register
	 * then right shift the data according the the pin number putting the
	 * target data to the 0th bit (LSB) it is then put through a bitwise AND operator
	 * with 0th bit left at 1 to only extract the 0th bit value and clears the remaining 31 bits.
	 */
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*********************************************************************************************
 * @fn			- GPIO_ReadInputPort
 *
 * #brief		- Reads the entire GPIO input port
 *
 * @param[in]	- GPIO_RegDef_t *pGPIOx, GPIO base address.
 *
 * @return		- uint16_t
 *
 * @Note 		-
 */
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}


/*********************************************************************************************
 * @fn			- GPIO_WriteOutputPin
 *
 * #brief		- Writes an output value to a specific GPIO output pin.
 *
 * @param[in]	- GPIO_RegDef_t *pGPIOx, GPIO port base address.
 * @param[in]	- uint8_t PinNumber, Pin numbers 0 to 15
 * @param[in]	- uint8_t Value, value you want to write to output pin, just HIGH = 1, & LOW =0;
 *
 * @return		- none
 *
 * @Note 		-
 */
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == HIGH){

		//write 1 to desired output pin
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		//clearing the desired output pin
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*********************************************************************************************
 * @fn			- GPIO_WriteOutputPort
 *
 * #brief		- Writes an output value to the whole GPIO port.
 *
 * @param[in]	- GPIO_RegDef_t *pGPIOx, GPIO base address.
 * @param[in]	- uint8_t Value, value you want to write to output pin, just HIGH = 1, & LOW =0;
 *
 * @return		- none
 *
 * @Note 		-
 */
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*********************************************************************************************
 * @fn			- GPIO_WriteOutputPin
 *
 * #brief		- Toggles the state of a specific output pin
 *
 * @param[in]	- GPIO_RegDef_t *pGPIOx, GPIO port base address.
 * @param[in]	- uint8_t PinNumber, Pin numbers 0 to 15.
 *
 * @return		- none
 *
 * @Note 		-
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*********************************************************************************************
 * @fn			- GPIO_IRQInterruptConfig
 *
 * #brief		- Enables or disables IRQNumber from being processed by the CPU.
 *
 * @param[in]	- IRQNumber, IRG number of the corresponding EXTI line.
 * @param[in]	- ENABLE = 1 OR DISABLE = 0
 *
 * @return		- none
 *
 * @Note 		-
 */
//4. identify the IRQ number on which the processor accepts the interrupt from that pin
//5. Configure the IRQ priority for the identified IRQ number (Processor side)
//6. enable interrupt reception on that IRQ number (processor side)
//7. implement the IRQ handler
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
	 *
	 *
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
 * @fn			- GPIO_IRQPriorityConfig
 *
 * #brief		- Sets the priority number of a specific IRQ number. Kinda like priority number queuing for interrupts.
 *
 * @param[in]	- IRQNumber, IRQ number of the EXTI line which the interrupt will come from.
 * @param[in]	- IRQpriority, values 0 - 255 the lower the value the higher the priority of the interrupt
 *
 * @return		- none
 *
 * @Note 		-
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4; //gets the IPR register index for setting
	uint8_t iprx_section = IRQNumber % 4; //determines which section of the IPR register to set.
	uint8_t Lshift_val = (8 - NUM_PRIO_BITS_IMPLEMENTED) + (8 * iprx_section);

	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << Lshift_val);


}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clears the EXTI PR register with the corresponding pin number
	if(EXTI->PR & (1<<PinNumber)){
		//clear
		EXTI->PR |= (1 << PinNumber);
	}

}

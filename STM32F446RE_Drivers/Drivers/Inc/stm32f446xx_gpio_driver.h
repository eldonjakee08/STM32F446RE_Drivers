/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Apr 14, 2025
 *      Author: eldon
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"
#include <stdint.h>

/*
 * This is a handle structure for a GPIO pin
 */

typedef struct
{

	uint8_t GPIO_PinNumber;		/*REFER TO @GPIO_PIN_NUMBERS FOR POSSIBLE INPUTS*/
	uint8_t GPIO_PinMode;		/*REFER TO @GPIO_PIN_MODES FOR POSSIBLE INPUTS*/
	uint8_t GPIO_PinSpeed;		/*FOR OUTPUT MODE ONLY!!! refer to @GPIO_OUTPUT_SPEED for possible inputs */
	uint8_t GPIO_PinPuPdCtrl;	/*REFER TO @GPIO_PUPD FOR POSSIBLE INPUTS*/
	uint8_t GPIO_PinOType;		/*FOR OUTPUT MODE ONLY!!! refer to @GPIO_OUTPUT_TYPES for possible inputs*/
	uint8_t GPIO_PinAFMode;		/*FOR ALTERNATE FUNCTION MODE ONLY!!! refer to @GPIO_ALTERNATE_FUNCTION_MODES for possible inputs*/

}GPIO_PinConfig_t;

typedef struct
{

	GPIO_RegDef_t 		*pGPIOx;			//This hold the base address of the GPIO port of which the pin belongs
	GPIO_PinConfig_t 	GPIO_PinConfig; 		//This hold the GPIO pin configuration settings
}GPIO_Handle_t;

/*
 * @GPIO_ALTERNATE_FUNCTION_MODES
 *GPIO pin possible alternate function modes
 */
#define GPIO_AF0			0
#define GPIO_AF1			1
#define GPIO_AF2			2
#define GPIO_AF3			3
#define GPIO_AF4			4
#define GPIO_AF5			5
#define GPIO_AF6			6
#define GPIO_AF7			7
#define GPIO_AF8			8
#define GPIO_AF9			9
#define GPIO_AF10			10
#define GPIO_AF11			11
#define GPIO_AF12			12
#define GPIO_AF13			13
#define GPIO_AF14			14
#define GPIO_AF15			15


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers macros
 */
#define GPIO_PIN0			0
#define GPIO_PIN1			1
#define GPIO_PIN2			2
#define GPIO_PIN3			3
#define GPIO_PIN4			4
#define GPIO_PIN5			5
#define GPIO_PIN6			6
#define GPIO_PIN7			7
#define GPIO_PIN8			8
#define GPIO_PIN9			9
#define GPIO_PIN10			10
#define GPIO_PIN11			11
#define GPIO_PIN12			12
#define GPIO_PIN13			13
#define GPIO_PIN14			14
#define GPIO_PIN15			15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes macros
 */
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4 //GPIO INTERRUPT MODE FALLING EDGE TRIGGER
#define GPIO_MODE_IT_RT			5 //GPIO INTERRUPT MODE RISING EDGE TRIGGER
#define GPIO_MODE_IT_RFT		6 //GPIO INTERRUPT MODE RISING AND FALLING EDGE TRIGGER


/*
 * @GPIO_OUTPUT_TYPES
 * GPIO possible output types
 */
#define GPIO_OPTYPE_PUSHPULL 			0 //GPIO OUTPUT TYPE PUSH-PULL MODE
#define GPIO_OPTYPE_OPENDRAIN			1 //GPIO OUTPUT TYPE OPEN DRAIN


/*
 * @GPIO_OUTPUT_SPEED
 * GPIO possible output speeds
 */
#define GPIO_LOW_SPD			0 //GPIO LOW SPEED
#define GPIO_MED_SPD			1 //GPIO MEDIUM SPEED
#define GPIO_FAST_SPD			2 //GPIO FAST SPEED
#define GPIO_HIGH_SPD			3 //GPIO HIGH SPEED (FASTEST)


/*
 * @GPIO_PUPD
 * GPIO internal pull-ip & pull-down configuration macros
 */
#define GPIO_NO_PULLUP_PULLDOWN		0 //NO INTERNAL PULLUP OR PULLDOWN RESISTOR ENABLED
#define GPIO_PULLUP_EN				1 //PULLUP RESISTOR ENABLED
#define GPIO_PULLDOWN_EN			2 //PULLDOWN RESISTOR ENABLED


/***********************************************************************************************************************
 * 									APIs supported by this driver
 * 					for more information about the APIs check the function definitions
 ***********************************************************************************************************************/

/*
 * Peripheral Clock set-up
 */
void GPIO_PeripheralClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO Initialization & De-initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Init2(GPIO_RegDef_t *pGPIOPort_BaseAddr, uint8_t PinNumber, uint8_t PinMode, uint8_t PUPD_ResMd, uint8_t OP_PinSpeed, uint8_t OPType, uint8_t AFMode_Sel);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO read and write functionality
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Interrupt functionality
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */

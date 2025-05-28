/*
 * tm32f446xx_rcc_driver.h
 *
 *  Created on: May 24, 2025
 *      Author: eldon
 */

#include <stm32f446xx.h>

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

/*
 * RCC fuction prototypes
 */
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */

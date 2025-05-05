/*
 * stm32f446xx.h
 *
 *  Created on: Apr 12, 2025
 *      Author: eldon
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>


#define __vo 		volatile
#define __weak 		__attribute__((weak))

/***************************************START: Processor Specific Details ****************************/
/*
 * ARM Cortex M4 processor NVIC ISERx register addresses
 * Interrupt Set-enable Registers
 * The NVIC_ISER0-NVIC_ISER7 registers enable interrupts, and show which interrupts are
 * enabled.
 */
#define NVIC_ISER0		((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10CU)
#define NVIC_ISER4		((__vo uint32_t*)0xE000E110U)
#define NVIC_ISER5		((__vo uint32_t*)0xE000E114U)
#define NVIC_ISER6		((__vo uint32_t*)0xE000E118U)
#define NVIC_ISER7		((__vo uint32_t*)0xE000E11CU)
#define NVIC_ISER_BASE_ADDR		NVIC_ISER0


/*
 * ARM Cortex M4 processor NVIC ICERx register addresses
 * Interrupt Clear-enable Registers
 * The NVIC_ICER0-NVIC_ICER7 registers disable interrupts, and show which interrupts are
 * enabled.
 */
#define NVIC_ICER0		((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1		((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2		((__vo uint32_t*)0XE000E188U)
#define NVIC_ICER3		((__vo uint32_t*)0XE000E18CU)
#define NVIC_ICER4		((__vo uint32_t*)0XE000E190U)
#define NVIC_ICER5		((__vo uint32_t*)0XE000E194U)
#define NVIC_ICER6		((__vo uint32_t*)0XE000E198U)
#define NVIC_ICER7		((__vo uint32_t*)0XE000E19CU)
#define NVIC_ICER_BASE_ADDR		NVIC_ICER0

/*
 * NVIC_IPR base address
 */
#define NVIC_IPR_BASE_ADDR 		((__vo uint32_t*)0xE000E400U)

/*
 * Number of priority bits implemented in Interrupt Priority Register.
 */
#define NUM_PRIO_BITS_IMPLEMENTED		4


/*
 * Generic macro definitions
 */
#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define HIGH				1
#define LOW 				0
#define NONE				16
#define FLAG_RESET			0
#define FLAG_SET			1

/*MCU memory base addresses*/
#define FLASH_BASEADDR 		0x08000000U 	/*Flash base address*/
#define SRAM1_BASEADDR 		0x20000000U 	/*SRAM1 base address mostly used*/
#define SRAM2_BASEADDR		0x2001C000U 	/*SRAM2 base address rarely used*/
#define ROM_BASEADDR		0x1FFF0000U 	/*System memory base address also called ROM*/
#define SRAM 				SRAM1_BASEADDR 	/*Just for shortening the SRAM1 to SRAM easier to call out*/

/*AHBx & APBx Peripheral busses base addresses*/
#define APB1PERIPH_BASEADDR 	0x40000000U 	/*APB1 peripheral base address*/
#define APB2PERIPH_BASEADDR		0x40010000U		/*APB2 peripheral base address*/
#define AHB1PERIPH_BASEADDR		0x40020000U		/*AHB1 peripheral base address*/
#define AHB2PERIPH_BASEADDR		0x50000000U		/*AHB2 peripheral base address*/
#define AHB3PERIPH_BASEADDR		0x60000000U		/*AHB3 peripheral base address I think this one is used rarely*/

/*Base addresses of peripherals hanging on AHB1 bus
 * Note: can also be defined using AHB1PERIPH_BASEADDR + Offset value
 * Note: EX. GPIOB_BASEADDR = AHB1PERIPH_BASEADDR + 0x0400 = 0x40020400
 * Note: but its easier to just copy directly from reference manual.*/
#define GPIOA_BASEADDR		0x40020000U 	/*GPIOA BASE ADDRESS*/
#define GPIOB_BASEADDR		0x40020400U 	/*GPIOB BASE ADDRESS*/
#define GPIOC_BASEADDR		0x40020800U 	/*GPIOB BASE ADDRESS*/
#define GPIOD_BASEADDR		0x40020C00U  	/*GPIOD BASE ADDRESS*/
#define GPIOE_BASEADDR		0x40021000U  	/*GPIOE BASE ADDRESS*/
#define GPIOF_BASEADDR		0x40021400U 	/*GPIOF BASE ADDRESS*/
#define GPIOG_BASEADDR		0x40021800U  	/*GPIOG BASE ADDRESS*/
#define GPIOH_BASEADDR		0x40021C00U  	/*GPIOH BASE ADDRESS*/

#define CRC_BASEADDR		0x40023000U		/*cyclic redundancy check for error checking*/
#define	RCC_BASEADDR		0x40023800U		/*RCC peripheral base address, this controls MCU clocks*/
#define FLASHINT_BASEADDR	0x40023C00U		/*Flash interface register base address*/
#define BKPSRAM_BASEADDR	0x40024000U		/*BKPSRAM base address IDK what this does for now*/
#define DMA1_BASEADDR		0x40026000U 	/*DMA1 base address might be for the DMA controller*/
#define DMA2_BASEADDR		0x40026400U		/*DMA1 base address might be for the DMA controller*/
#define USBOTGHS_BASEADDR	0x40040000U		/*USB OTG HS base address*/

/*Base addresses of peripherals hanging on APB2 bus
 * Note: just like AHB1 bus, can also be defined using APB2PERIPH_BASEADDR + Offset value
 * Note: but its easier to just copy directly from reference manual.*/
#define TIM1_BASEADDR 		0x40010000U
#define TIM8_BASEADDR		0x40010400U
#define USART1_BASEADDR		0x40011000U
#define USART6_BASEADDR		0x40011400U
#define ADC1TO3_BASEADDR	0x40012000U 	/*ADC1-ADC2-ADC3 is whats written in reference manual*/
#define SDMMC_BASEADDR		0x40012C00U 	/*IDK what SDMMC means for now*/
#define SPI1_BASEADDR	 	0x40013000U
#define SPI4_BASEADDR		0x40013400U
#define SYSCFG_BASEADDR		0x40013800U
#define EXTI_BASEADDR		0x40013C00U
#define	TIM9_BASEADDR		0x40014000U
#define TIM10_ABASEADDR		0x40014400U
#define TIM11_BASEADDR		0x40014800U
#define SAI1_BASEADDR		0x40015800U		/*this seems to be a new peripheral for me IDK what this does yet*/
#define SAI2_BASEADDR 		0x40015C00U


/*Base addresses of peripherals hanging on APB1 bus
 * Note: just like AHB1 bus, can also be defined using APB1PERIPH_BASEADDR + Offset value
 * Note: but its easier to just copy directly from reference manual.*/
#define TIM2_BASEADDR 		0X40000000U
#define TIM3_BASEADDR		0X40000400U
#define TIM4_BASEADDR		0x40000800U
#define TIM5_BASEADDR		0x40000C00U
#define TIM6_BASEADDR		0x40001000U
#define TIM7_BASEADDR		0x40001400U
#define TIM12_BASEADDR		0x40001800U
#define TIM13_BASEADDR		0x40001C00U
#define TIM14_BASEADDR		0x40002000U
#define RTCBKP_BASEADDR		0x40002800U		/*RTC & BKP Registers base address IDK what this does yet*/
#define WWATCHDOG_BASEADDR	0x40002C00U		/*Window Watchdog base address curious about how watchdog works*/
#define IWATCHDOG_BASEADDR	0x40003000U		/*Independent watchdog peripheral base address, curious as well on what this does*/
#define SPI2_I2S2_BASEADDR	0x40003800U		/*SPI2 BASE ADDR*/
#define SPI3_I2S3_BASEADDR 	0x40003C00U
#define SPDIF_RX_BASEADDR	0x40004000U
#define USART2_BASEADDR		0x40004400U
#define USART3_BASEADDR		0x40004800U
#define UART4_BASEADDR		0x40004C00U
#define UART5_BASEADDR		0x40005000U
#define I2C1_BASEADDR		0x40005400U
#define I2C2_BASEADDR		0x40005800U
#define I2C3_BASEADDR		0x40005C00U
#define CAN1_BASEADDR		0x40006400U
#define CAN2_BASEADDR		0x40006800U
#define HDMI_CEC_BASEADDR	0x40006C00U
#define PWR_BASEADDR		0x40007000U		/*Power controller peripheral registers base address, curious on what this guy does*/
#define DAC_BASEADDR		0x40007400U

/*Base addresses of peripherals hanging on AHB2 bus
 * Note: just like AHB1 bus, can also be defined using AHB2PERIPH_BASEADDR + Offset value
 * Note: but its easier to just copy directly from reference manual.*/
#define USBOTGFS_BASEADDR	0x50000000U
#define DCMI_BASEADDR		0x50050000U			/*DCMI base address, this is the digital camera interface cool!*/

/*Base addresses of peripherals hanging on AHB3 bus
 * Note: just like AHB1 bus, can also be defined using AHB3PERIPH_BASEADDR + Offset value
 * Note: but its easier to just copy directly from reference manual.*/
#define QUADSPI_BASEADDR	0xA0001000U			/*QUAD SPI BASE ADDRESS, curious on what this does*/
#define FMCCTRL_BASEADDR	0xA0000000U			/*FMC Register base addr, curious as well on what this does*/

/*********************************peripheral registers definition structures*********************///

/*GPIO Peripheral registers definitions*/
typedef struct
{
	/* Physical register of the MCU
	 * Note: notice how we declared each member of the structure element to volatile because, values of these peripheral registers are always changing,
	 * to prevent the compiler from ignoring these variable when using higher optimization levels like -O3.
	 */

	__vo uint32_t MODER;		/* Offset: 0x00, GPIO port mode register, Sets the GPIO pin as either input,output,Alternate function & Analog mode*/
	__vo uint32_t OTYPER;		/* Offset: 0x04, GPIO port output type register, This sets the GPIO (if in output mode) to either in push-pull state or in open drain state*/
	__vo uint32_t OSPEEDER; 	/* Offset: 0x08, GPIO port output speed register, Sets the GPIO Slew rate to control its speed of operation*/
	__vo uint32_t PUPDR;		/* Offset: 0x0C, GPIO port pull-up/pull-down register, Enables the GPIO internal pull-up or pull-down resistors*/
	__vo uint32_t IDR;			/* Offset: 0x10, GPIO port input data register, Read only register, allows you to read the current value of an input GPIO*/
	__vo uint32_t ODR;			/* Offset: 0x14, GPIO port output data register, Read and write register, if GPIO is in output mode this register allows you to change the GPIO state and also read its current state*/
	__vo uint32_t BSRR;			/* Offset: 0x18, GPIO port bit set/reset register, IDK pa what this does actually, I'd have to learn it yet*/
	__vo uint32_t LCKR; 		/* Offset: 0x1C, GPIO port configuration lock register, IDK much about this register, but it says in the manual that it locks the GPIO configuration*/
	__vo uint32_t AFR[2];		/* Offset: 0x20-0x24, GPIO alternate function  register, AF[0] = alternate function low  AF0 to AF7. AF[1] = alternate function high AF8 to AF15*/

}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t CR; 			/*RCC clock control register*/
	__vo uint32_t PLL_CFGR;		/*RCC PLL configuration register*/
	__vo uint32_t CFGR;			/*RCC clock configuration register*/
	__vo uint32_t CIR;			//RCC clock interrupt register
	__vo uint32_t AHB1_RSTR;	//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2_RSTR;	//RCC AHB2 peripheral reset register
	__vo uint32_t AHB3_RSTR;	//RCC AHB3 peripheral reset register
	__vo uint32_t RESERVED1;	//reserved register
	__vo uint32_t APB1_RSTR;	//RCC APB1 peripheral reset register
	__vo uint32_t APB2_RSTR;	//RCC APB2 peripheral reset register
	uint32_t RESERVED2;		//reserved register
	uint32_t RESERVED3;		//reserved register
	__vo uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;		//RCC AHB3 peripheral clock enable register
	uint32_t RESERVED4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED5;
	uint32_t RESERVED6;
	__vo uint32_t AHB1_LPENR;
	__vo uint32_t AHB2_LPENR;
	__vo uint32_t AHB3_LPENR;
	uint32_t RESERVED7;
	__vo uint32_t APB1_LPENR;
	__vo uint32_t APB2_LPENR ;
	uint32_t RESERVED8;
	uint32_t RESERVED9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED10;
	uint32_t RESERVED11;
	__vo uint32_t SS_CGR;
	__vo uint32_t PLLI2_SCFGR;
	__vo uint32_t PLL_SAI_CFGR;
	__vo uint32_t DCK_CFGR;
	__vo uint32_t CK_GATEENR;
	__vo uint32_t DCK_CFGR2;

}RCC_RegDef_t;

/*
 * peripheral register definition for EXTI (External Interrupts)
 */
typedef struct
{
	__vo uint32_t IMR;		//Interrupt mask register (EXTI_IMR)
	__vo uint32_t EMR;		//Event mask register (EXTI_EMR)
	__vo uint32_t RTSR;		//Rising trigger selection register (EXTI_RTSR)
	__vo uint32_t FTSR;		//Falling trigger selection register (EXTI_FTSR)
	__vo uint32_t SWIER;	//Software interrupt event register (EXTI_SWIER)
	__vo uint32_t PR;		//Pending register (EXTI_PR)

}EXTI_RegDef_t;


/*
 * Peripheral register definition for SYCFG (System Configuration Controller)
 */
typedef struct
{
	__vo uint32_t MEMRMP;		//SYSCFG memory remap register (SYSCFG_MEMRMP)
	__vo uint32_t PMC;			//SYSCFG peripheral mode configuration register (SYSCFG_PMC)
	__vo uint32_t EXTICR[4];	//SYSCFG external interrupt configuration register 1 to 4, chooses which GPIO port the EXTI line runs through
	__vo uint32_t CMPCR;		//Compensation cell control register (SYSCFG_CMPCR)
	__vo uint32_t CFGR;			//SYSCFG configuration register (SYSCFG_CFGR)

}SYSCFG_RegDef_t;


/*
 * peripheral register definition of SPI.
 */
typedef struct
{
	__vo uint32_t CR1;			//SPI control register 1 (SPI_CR1) (not used in I2S mode)
	__vo uint32_t CR2;			//SPI control register 2 (SPI_CR2)
	__vo uint32_t SR;			//SPI status register (SPI_SR)
	__vo uint32_t DR;			//SPI data register (SPI_DR)
	__vo uint32_t CRCPR;		//SPI CRC polynomial register (SPI_CRCPR) (not used in I2S mode)
	__vo uint32_t RXCRCR;		//SPI RX CRC register (SPI_RXCRCR) (not used in I2S mode)
	__vo uint32_t TXCRCR;		//SPI TX CRC register (SPI_TXCRCR) (not used in I2S mode)
	__vo uint32_t I2SCFGR;		//SPI_I2S configuration register (SPI_I2SCFGR)
	__vo uint32_t I2SPR;		//SPI_I2S prescaler register (SPI_I2SPR)

}SPI_RegDef_t;


/*
 * peripheral register definition of I2C
 */
typedef struct
{
	__vo uint32_t CR1;		//I2C control register 1 (I2C_CR1)
	__vo uint32_t CR2;		//I2C control register 2 (I2C_CR2)
	__vo uint32_t OAR1;		//I2C own address register 1 (I2C_OAR1)
	__vo uint32_t OAR2;		//I2C own address register 2 (I2C_OAR2)
	__vo uint32_t DR;		//I2C data register (I2C_DR)
	__vo uint32_t SR1;		//I2C status register 1 (I2C_SR1)
	__vo uint32_t SR2;		//I2C status register 2 (I2C_SR2)
	__vo uint32_t CCR;		//I2C clock control register (I2C_CCR)
	__vo uint32_t TRISE;	//I2C TRISE register (I2C_TRISE)
	__vo uint32_t FLTR;		//I2C FLTR register (I2C_FLTR)

}I2C_RegDef_t;



/*stm32f446rexx pointer macros for ease of typing when initializing the pointer values*/
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR) /*GPIOA_BASEADDR typecasted as a pointer of type GPIO_RegDef_t */
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR) /*GPIOB_BASEADDR typecasted as a pointer of type GPIO_RegDef_t */
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR) /*GPIOC_BASEADDR typecasted as a pointer of type GPIO_RegDef_t */
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR) /*GPIOD_BASEADDR typecasted as a pointer of type GPIO_RegDef_t */
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR) /*GPIOE_BASEADDR typecasted as a pointer of type GPIO_RegDef_t */
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR) /*GPIOF_BASEADDR typecasted as a pointer of type GPIO_RegDef_t */
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR) /*GPIOG_BASEADDR typecasted as a pointer of type GPIO_RegDef_t */
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR) /*GPIOH_BASEADDR typecasted as a pointer of type GPIO_RegDef_t */

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)	 	/*RCC_BASEADDR typecasted as a pointer of type RCC_RegDef_t */
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)	 	/*EXTI base address typecasted as a pointer of type EXTI_RegDef_t */
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)	/*SYSCFG base address typecasted as a pointer of type SYSCFG_RegDef_t */

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)		//SPI1 base address typecasted as a pointer of type SPI_RegDef_t
#define SPI2		((SPI_RegDef_t*)SPI2_I2S2_BASEADDR)		//SPI2 base address typecasted as a pointer of type SPI_RegDef_t
#define SPI3		((SPI_RegDef_t*)SPI3_I2S3_BASEADDR)		//SPI3 base address typecasted as a pointer of type SPI_RegDef_t
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)		//SPI4 base address typecasted as a pointer of type SPI_RegDef_t

#define I2C1		((I2C_RegDef_t)*I2C1_BASEADDR)		//I2C1 base address typecasted as a pointer of type I2C_RegDef_t (represent the physical registers of I2C peripheral)
#define I2C2		((I2C_RegDef_t)*I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t)*I2C3_BASEADDR)



/*
 * Clock Enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))


/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))


/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1<<13))


/*
 * Clock enable macros for USARTx & UARTx peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1<<19)) 	//not capable of synchronous comms
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1<<20))	//not capable of synchronous comms
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1<<5))


/*
 * Clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))


/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))


/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))


/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<13))


/*
 * Clock disable macros for USARTx & UARTx peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1<<19)) 	//not capable of synchronous comms
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1<<20))	//not capable of synchronous comms
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1<<5))


/*
 * Clock disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14))

/*
 * Macros to reset GPIOx peripherals
 * notice the do/while loop, its pretty clever for using it to execute multiple lines of
 * code with only one macro definition.
 * Logic behind: do while loop executes the code block first before evaluating if the condition is true.
 * so in this case it executes the code clock first which are the setting and clearing of the peripheral register
 * then it reads the "while(0)" which is false so it breaks off from the loop. NICEEE!!
 */

/*
 * sets the RCC_AHB1RSTR to 1 then clears it to zero (reset value)
 * as instructed by reference manual. To prevent it from resetting
 * the GPIO Port  repeatedly.
 *
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB1_RSTR |= (1 << 0) ); (RCC->AHB1_RSTR &= ~(1 << 0) );} while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1_RSTR |= (1 << 1) ); (RCC->AHB1_RSTR &= ~(1 << 1) );} while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1_RSTR |= (1 << 2) ); (RCC->AHB1_RSTR &= ~(1 << 2) );} while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1_RSTR |= (1 << 3) ); (RCC->AHB1_RSTR &= ~(1 << 3) );} while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1_RSTR |= (1 << 4) ); (RCC->AHB1_RSTR &= ~(1 << 4) );} while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1_RSTR |= (1 << 5) ); (RCC->AHB1_RSTR &= ~(1 << 5) );} while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1_RSTR |= (1 << 6) ); (RCC->AHB1_RSTR &= ~(1 << 6) );} while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1_RSTR |= (1 << 7) ); (RCC->AHB1_RSTR &= ~(1 << 7) );} while(0)


/*
 * Resets the SPI peripherals with the use of RCC APB2 peripheral reset register
 */
#define SPI1_REG_RESET()		do{ (RCC->APB2_RSTR |= (1 << 12) ); (RCC->APB2_RSTR &= ~(1 << 12) );} while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1_RSTR |= (1 << 14) ); (RCC->APB1_RSTR &= ~(1 << 14) );} while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1_RSTR |= (1 << 15) ); (RCC->APB1_RSTR &= ~(1 << 15) );} while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2_RSTR |= (1 << 13) ); (RCC->APB2_RSTR &= ~(1 << 13) );} while(0)


/*
 * This macro translates the GPIO port base address to SYSCFG EXTI code for programming the SYSCFG_EXTICR peripheral register
 * This macro definition accepts an input parameter, which is the 1st time I've encountered this.
 * This uses ternary operator, if x == GPIOA is true then it returns the "0" value, if statement is false it moves on
 * to next line to evaluate if condition is true till it reaches the end. Notice the "\" at the end of each condition
 * it probably says "proceed to next line" for the succeeding conditions.
 *
 * This can come in handy as well in translating the base addresses of the GPIO ports, which are pointers,
 * into a primitive data type for ease of coding especially when using in switch statements.
 */
#define GPIO_BASEADDR_TO_CODE(x)  (	(x == GPIOA)?0:\
		(x == GPIOB)?1:\
				(x == GPIOC)?2:\
						(x == GPIOD)?3:\
								(x == GPIOE)?4:\
										(x == GPIOF)?5:\
												(x == GPIOG)?6:\
														(x == GPIOH)?7:0)

#define SPI_BASEADDR_TO_CODE(x)	  (	(x == SPI1)?0:\
		(x == SPI2)?1:\
				(x == SPI3)?2:\
						(x == SPI4)?3:0)


/*
 * @IRQ_NUMBERS
 * macro definitions of IRQ numbers
 */
#define IRQ_NUMBER_EXTI0		6
#define IRQ_NUMBER_EXTI1		7
#define IRQ_NUMBER_EXTI2		8
#define IRQ_NUMBER_EXTI3		9
#define IRQ_NUMBER_EXTI4		10
#define IRQ_NUMBER_EXTI5_9		23
#define IRQ_NUMBER_EXTI10_15	40

#define IRQ_NUMBER_SPI1			35
#define IRQ_NUMBER_SPI2			36
#define IRQ_NUMBER_SPI3			51
#define IRQ_NUMBER_SPI4			84

/*
 * @BIT_FIELDS_SPI_PERIPH
 * Bit fields position of SPI Peripherals
 */
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BAUDRATE	3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * @SPI_SR_BIT_FIELDS
 * STATUS REGISTER FLAG NAMES AND BIT FIELDS
 */
#define SPI_SR_RXNE 		0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define	SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define	SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3


/*
 * @I2C_PERIPH_BIT_POSITION
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

#define I2C_CR2_FREQ5_0		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD7_1		1
#define I2C_OAR1_ADD9_8		8
#define I2C_OAR1_ADDMODE	15

#define I2C_OAR2_ENDUAL		0
#define I2C_OAR2_ADD2		1

#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMB_DEFAULT		5
#define I2C_SR2_SMB_HOST		6
#define I2C_SR2_DUALF			7
#define I2C_ST2_PEC15_8			8

#include 	"stm32f446xx_gpio_driver.h"
#include 	"stm32f446xx_spi_driver.h"
#include 	"stm32f446xx_i2c_driver.h"
#endif /* INC_STM32F446XX_H_ */

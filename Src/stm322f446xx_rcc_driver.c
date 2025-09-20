



#include <stdint.h>
#include <stm32f446xx_rcc_driver.h>

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
 * @fn			- RCC_GetPCLK2Value
 *
 * #brief		- fetches the current APB2 bus clock value
 *
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note 		-
 */
uint32_t RCC_GetPCLK2Value(void) {
	uint32_t pclk2, system_clk;
	uint8_t clk_src, ahb_prescaler, apb2_prescaler;

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

	// Get APB2 prescaler
	apb2_prescaler = (RCC->CFGR >> 10) & 0x7;
	if (apb2_prescaler < 4) {
		apb2_prescaler = 1; // No division
	} else {
		apb2_prescaler = 1 << (apb2_prescaler - 3); // 2, 4, 8, 16
	}

	// Calculate PCLK1
	pclk2 = (system_clk / ahb_prescaler) / apb2_prescaler;

	return pclk2;
}

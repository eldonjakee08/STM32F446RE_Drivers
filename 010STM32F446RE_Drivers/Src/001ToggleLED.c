/*
 * 001ToggleLED.c
 *
 *  Created on: Apr 15, 2025
 *      Author: eldon
 */

#include "stm32f446xx.h"


void delay1(){
	for(uint32_t i=0;i<=1000;i++){
		//delay lang ni
	}
}

void delay2(){
	for(uint32_t i=0;i<=300000;i++){
		//delay lang ni
	}
}



int main(void)
{

	GPIO_PeripheralClkCtrl(GPIOA, ENABLE);
	GPIO_PeripheralClkCtrl(GPIOC, ENABLE);

	//SET-UP & initialization of PA9 PIN as output pin
	GPIO_Init2(GPIOA, GPIO_PIN9, GPIO_MODE_OUTPUT,  GPIO_NO_PULLUP_PULLDOWN, GPIO_MED_SPD, GPIO_OPTYPE_PUSHPULL, NONE);

	//SET-UP & initialization of PC13 as input pin
	GPIO_Init2(GPIOC, GPIO_PIN13, GPIO_MODE_INPUT, GPIO_NO_PULLUP_PULLDOWN, NONE, NONE, NONE);


	for(;;){
		//loop forever
		if(GPIO_ReadInputPin(GPIOC,GPIO_PIN13) == LOW){
			delay2();

			while(GPIO_ReadInputPin(GPIOC,GPIO_PIN13) == HIGH){
			delay1();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN9);

			if(GPIO_ReadInputPin(GPIOC,GPIO_PIN13) == LOW){
				break;
			}

			}
		}


	}
	return 0;

}

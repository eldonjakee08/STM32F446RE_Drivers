


#include "stm32f446xx.h"
#include <string.h>

/*

pB6 - usart1_tx
pB7 - usart_rx
*/

USART_Handle_t USARTHandle;

void usart_gpioinit(){
	GPIO_Handle_t GPIOHandle;


	GPIOHandle.pGPIOx = GPIOB;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOType = GPIO_OPTYPE_PUSHPULL;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPD;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAFMode = GPIO_AF7;


	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6; //tx
	GPIO_Init(&GPIOHandle);

	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN7; //rx
	GPIO_Init(&GPIOHandle);
}

void usart_init(){

	USARTHandle.pUSARTx = USART1;
	USARTHandle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USARTHandle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USARTHandle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USARTHandle.USART_Config.USART_StopBits = USART_STOPBITS_1;
	USARTHandle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USARTHandle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USARTHandle);
}

void button_init(){
	GPIO_Handle_t GPIOButtonHandle;

	GPIOButtonHandle.pGPIOx = GPIOC;
	GPIOButtonHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOButtonHandle.GPIO_PinConfig.GPIO_PinOType = GPIO_OPTYPE_PUSHPULL;
	GPIOButtonHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PULLUP_EN;
	GPIOButtonHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPD;

	GPIOButtonHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	GPIO_Init(&GPIOButtonHandle);
}


int main(void)
{
	char *msg[3] = {"Hello from STM32\n", "YOW yow YOW!\n", "I'm BURNING\n"};

	button_init();

	usart_gpioinit();

	usart_init();

	USART_PeripheralControl(USART1, ENABLE);

	while(1){
		//wait until button is pressed
		while ( GPIO_ReadInputPin(GPIOC, GPIO_PIN13) == HIGH );

		//delay
		for (uint32_t i = 0; i < 500000; i++);

		USART_SendData(&USARTHandle, msg, strlen((char*)msg));


	}

	return 0;
}



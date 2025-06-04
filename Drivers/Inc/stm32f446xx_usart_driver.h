/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: May 19, 2025
 *      Author: eldon
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"


/*
 * Configuration structure for USARTx peripheral
 */
typedef struct{
	uint8_t 	USART_Mode;				//possible values: USART_MODE_TX, USART_MODE_RX, USART_MODE_TXRX
	uint32_t	USART_Baud;			//possible values: 9600, 115200, etc.
	uint8_t 	USART_WordLength;		//possible values: USART_WORDLEN_8BITS, USART_WORDLEN_9BITS
	uint8_t 	USART_StopBits;			//possible values: USART_STOPBITS_1, USART_STOPBITS_0_5, USART_STOPBITS_2, USART_STOPBITS_1_5
	uint8_t 	USART_ParityControl;	//possible values: USART_PARITY_DISABLE, USART_PARITY_ENABLE_EVEN, USART_PARITY_ENABLE_ODD
	uint8_t 	USART_HWFlowControl;	//possible values: USART_HW_FLOW_CTRL_NONE, USART_HW_FLOW_CTRL_CTS, USART_HW_FLOW_CTRL_RTS, USART_HW_FLOW_CTRL_CTS_RTS
}USART_Config_t;


/*
 * Handle structure for USART peripheral
 */
typedef struct{
	USART_RegDef_t 	*pUSARTx;			//pointer to USARTx peripheral
	USART_Config_t 	USART_Config;		//USART configuration settings
	uint8_t 		*pTxBuffer;			//To store the application Tx Buffer address
	uint8_t 		*pRxBuffer;			//To store the application Rx Buffer address
	uint32_t 		TxLen;				//to store the Tx Len
	uint32_t 		RxLen;				//to store the Rx Len
	uint8_t 		TxBusyState;			//to store the communication state
	uint8_t 		RxBusyState;			//to store the communication state

}USART_Handle_t;


/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


/*
 * USART/UART application states
 */
#define USART_READY 				0
#define USART_BUSY_IN_RX			1
#define USART_BUSY_IN_TX			2


/*
 * USART/UART application events
 */
#define USART_EVENT_TX_CMPLT 		1
#define USART_EVENT_RX_CMPLT 		2
#define USART_EVENT_IDLE 			3
#define USART_EVENT_CTS 			4
#define USART_EVENT_PE 				5
#define USART_ERREVENT_FE 			6
#define USART_EVENT_ORE				7
#define USART_ERREVENT_NE 			8
#define USART_ERREVENT_ORE 			9

/***********************************************************************************************************************
 * 									APIs supported by this driver
 * 					for more information about the APIs check the function definitions
 **********************************************************************************************************************/

/*
 * USART/UART Peripheral Clock set-up
 */
void USART_PeripheralClkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi); //done

/*
 * USART/UART peripheral control
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi); //done


/*
 * USART/UART Initialization & De-initialization
 */
void USART_Init(USART_Handle_t *pUSARTHandle); //done Baud rate nalay kuwang
void USART_DeInit(USART_RegDef_t *pUSARTx); //done


/*
 * USART/UART data send and receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len); //done
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);



/*
 * Other APIs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName); //done
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t FlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent);

/*
 * IRQ config and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); 	//done
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority); //done
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


#endif /* INC_STM32F446XX_USART_DRIVER_H_ */



























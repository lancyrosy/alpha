// ---------------------------------------------------------------------------------
// libUart.c
//
// Created on: 07-Jan-2010
// Author: nbk
// Desc  :
// UARTx_init() to initialise UARTx
// RXCharISR() & TxCharISR() to get receive/transmit char from/to UART
// GetChar() & PutChar() to get or put char on stdio.
// ---------------------------------------------------------------------------------

#include "project.h"

#define BUFSIZE		128
static unsigned char rxBuffer[5][BUFSIZE];
static unsigned char txBuffer[5][BUFSIZE];
volatile static int16_t rxReadIndex[5], rxWriteIndex[5];
volatile static int16_t txReadIndex[5], txWriteIndex[5];
bool txEnableFlag[5];
USART_TypeDef* usartLut[5] = {USART1, USART2, USART3, UART4, UART5};
int16_t usartIRQnLut[5] = {USART1_IRQn, USART2_IRQn,USART3_IRQn,UART4_IRQn,UART5_IRQn };

// @brief : To get a character from UART through interrupt
// @param : uartNum is the USART number minus 1, so USART1 -> 0
// @retval: character received
int RxCharISR(int16_t uartNum) {
	unsigned char ch;

	while(rxWriteIndex[uartNum]==rxReadIndex[uartNum]);

	ch = rxBuffer[uartNum][rxReadIndex[uartNum]];
DI;
	rxReadIndex[uartNum]++;
	if (rxReadIndex[uartNum]>=BUFSIZE)
		rxReadIndex[uartNum]=0;
EI;
	return ch;
}

// @brief : To check if a character is available for reading
// @param : none
// @retval: true if character is available, else false.
int CheckRxBuf(int16_t uartNum) {
	return (rxWriteIndex[uartNum]!=rxReadIndex[uartNum]);
}
// ---------------------------------------------------------------------------------
// @brief : To put a character into usart through interrupt
// @param : c-character to send, usartNum-usart#
// @retval: none
int TxCharISR(int16_t uartNum, char c) {

	// If tx buffer is full, wait
	while((txWriteIndex[uartNum]==(txReadIndex[uartNum]-1)) ||
		(txWriteIndex[uartNum]==(txReadIndex[uartNum]+BUFSIZE-1)) );

	txBuffer[uartNum][txWriteIndex[uartNum]] = c;
DI;
	txWriteIndex[uartNum]++;
	if (txWriteIndex[uartNum]>=BUFSIZE)
		txWriteIndex[uartNum]=0;
EI;
	if (!txEnableFlag[uartNum]) {
		// turn ON tx interrupt to begin the transfer
		txEnableFlag[uartNum] = TRUE;
		USART_ITConfig(usartLut[uartNum], USART_IT_TXE, ENABLE);
	}
	return c;
}

// ---------------------------------------------------------------------------------
// @brief : To put a string into usart through interrupt
// @param : s-string to send, usartNum-usart#
// @retval: none
void PutStrISR(int16_t uartNum, char *s) {
	while(*s)
		TxCharISR(uartNum, *s++);
}

// @brief : To check if a character is available for reading
// @param : none
// @retval: true if character is available, else false.
bool kbhit(){
	return (rxWriteIndex[0]!=rxReadIndex[0]);
//	return (USART_GetFlagStatus(USART1, USART_IT_RXNE) != RESET);
}



// @brief : To get a character from stdio, which is USART1
// @param : none
// @retval: character received
int GetChar() {
	return RxCharISR(0);
//	return RxChar(USART1);
}

// ---------------------------------------------------------------------------------
// @brief : To put a character into stdio, which is USART1
// @param c: character to send
// @retval: none
int PutChar(char c) {
	return TxCharISR(0, c);
}
// ---------------------------------------------------------------------------------
// @brief : To send a string to stdio, which is UART1
// @param s : string to send
// @retval: none
int puts(char *s) {
	while(*s)
		PutChar(*s++);
	PutChar('\n');
	return 0;
}

// ---------------------------------------------------------------------------------
// @brief : Common routine for initialising UART
// @param : uartNum is usart#-1. so USART1 -> 0
//		  : baudrate
// @retval: none
void UART_init(int16_t uartNum, int baudrate){
	switch(uartNum){
	case 0:UART1_init(baudrate); break;
	//case 1:UART2_init(baudrate); break;
	case 2:UART3_init(baudrate); break;
	case 3:UART4_init(baudrate); break;
	case 4:UART5_init(baudrate); break;
	}
}

// ---------------------------------------------------------------------------------
// @brief : Common routine for initialising UART
// @param : baudrate , uartNum
// @retval: none
void __UART_init(int16_t uartNum, int baudrate){
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Configure the USARTx */
    USART_Init(usartLut[uartNum], &USART_InitStructure);

    /* Configure and enable USART interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = usartIRQnLut[uartNum];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART_ISR_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(usartLut[uartNum], USART_IT_RXNE, ENABLE);	// Enable Rx interrupt

    /* Enable the USARTx */
    USART_Cmd(usartLut[uartNum], ENABLE);
}

#define USART1_RX		GPIOA, GPIO_Pin_10
#define USART1_TX		GPIOA, GPIO_Pin_9
// ---------------------------------------------------------------------------------
// @brief : To init UART1 for downloading and debugging
// @param : baudrate
// @retval: none
void UART1_init(int baudrate){


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_Init_Mode(USART1_TX, GPIO_Mode_AF_PP);
	GPIO_Init_Mode(USART1_RX, GPIO_Mode_IN_FLOATING);

	__UART_init(0, baudrate);

}

#define USART3_RX		GPIOB, GPIO_Pin_11
#define USART3_TX		GPIOB, GPIO_Pin_10
void UART3_init(int baudrate){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_Init_Mode(USART3_TX, GPIO_Mode_AF_PP);
	GPIO_Init_Mode(USART3_RX, GPIO_Mode_IN_FLOATING);

	__UART_init(2, baudrate);

}

#define USART4_RX		GPIOC, GPIO_Pin_11
#define USART4_TX		GPIOC, GPIO_Pin_10
void UART4_init(int baudrate){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	GPIO_Init_Mode(USART4_TX, GPIO_Mode_AF_PP);
	GPIO_Init_Mode(USART4_RX, GPIO_Mode_IN_FLOATING);

	__UART_init(3, baudrate);

}

#define USART5_RX		GPIOD, GPIO_Pin_2
#define USART5_TX		GPIOC, GPIO_Pin_12
void UART5_init(int baudrate){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	GPIO_Init_Mode(USART5_TX, GPIO_Mode_AF_PP);
	GPIO_Init_Mode(USART5_RX, GPIO_Mode_IN_FLOATING);

	__UART_init(4, baudrate);

}

// @brief : General usart ISR
// @param : uartNum is the USART number minus 1, so USART1 -> 0
// @retval: none
void USART_ISR(int16_t uartNum) {
	char ch;

	if (USART_GetITStatus(usartLut[uartNum], USART_IT_RXNE)==SET)
	{
		rxBuffer[uartNum][rxWriteIndex[uartNum]++] = USART_ReceiveData(usartLut[uartNum]);

		if (rxWriteIndex[uartNum]>=BUFSIZE)
			rxWriteIndex[uartNum]=0;
	}

	if (USART_GetITStatus(usartLut[uartNum], USART_IT_TXE)==SET) {
		if (txWriteIndex[uartNum]!=txReadIndex[uartNum]) {

			ch = txBuffer[uartNum][txReadIndex[uartNum]];
			USART_SendData(usartLut[uartNum], ch);

			txReadIndex[uartNum]++;
			if (txReadIndex[uartNum]>=BUFSIZE)
				txReadIndex[uartNum]=0;

		}
		else {
		    USART_ITConfig(usartLut[uartNum], USART_IT_TXE, DISABLE);
		    txEnableFlag[uartNum] = FALSE;
		}
	}
}

/////////////////////////////////////////////////////////////////////
// USARTs ISRs. For convenience, they all call USART_ISR(n);
/////////////////////////////////////////////////////////////////////
// @brief : usart1 ISR handler. Uses USART_ISR(n) for sevice.
void USART1_IRQHandler(void){
	USART_ISR(0);
}

void USART2_IRQHandler(void){
	USART_ISR(1);
}

void USART3_IRQHandler(void){
	USART_ISR(2);
}

void UART4_IRQHandler(void){
	USART_ISR(3);
}

void UART5_IRQHandler(void){
	USART_ISR(4);
}

/////////////////////////////////////////////////////////////////////
// Polling based functions. Used for initial testing of UARTs
/////////////////////////////////////////////////////////////////////

// @brief : To get a character from UART through polling
// @param : none
// @retval: character received
int RxChar(USART_TypeDef* usart) {
	while(USART_GetFlagStatus(usart, USART_IT_RXNE) == RESET);
	return USART_ReceiveData(usart);
}

// @brief : To put a character into UART through polling
// @param c: character to send
// @retval: none
int TxChar(USART_TypeDef* usart, char c) {
	while(USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET);
   	USART_SendData(usart,c);
   	return 0;
}

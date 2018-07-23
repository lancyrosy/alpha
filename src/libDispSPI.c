// ---------------------------------------------------------------------------------
// libDispSPI.c
// Created on: 07-Jan-2010
// Author: nbk
// Desc  :
// DispInit() to initialise the SPI before using the display routines
// DispDotMatrix() to display a string on the dot matrix
// DispDotMatrixWait() to display a string on the dot matrix and wait for user to press
//     user switch
// ---------------------------------------------------------------------------------
#include "project.h"

const char lut5x7[] = { 0x00, 0x00, 0x00, 0x00, 0x00, // sp
		0x00, 0x00, 0x2f, 0x00, 0x00, // !
		0x00, 0x07, 0x00, 0x07, 0x00, // "
		0x14, 0x7f, 0x14, 0x7f, 0x14, // #
		0x24, 0x2a, 0x7f, 0x2a, 0x12, // $
		0xc4, 0xc8, 0x10, 0x26, 0x46, // %
		0x36, 0x49, 0x55, 0x22, 0x50, // &
		0x00, 0x05, 0x03, 0x00, 0x00, // '
		0x00, 0x1c, 0x22, 0x41, 0x00, // (
		0x00, 0x41, 0x22, 0x1c, 0x00, // )
		0x14, 0x08, 0x3E, 0x08, 0x14, // *
		0x08, 0x08, 0x3E, 0x08, 0x08, // +
		0x00, 0x00, 0x50, 0x30, 0x00, // ,
		0x10, 0x10, 0x10, 0x10, 0x10, // -
		0x00, 0x60, 0x60, 0x00, 0x00, // .
		0x20, 0x10, 0x08, 0x04, 0x02, // /
		0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
		0x00, 0x42, 0x7F, 0x40, 0x00, // 1
		0x42, 0x61, 0x51, 0x49, 0x46, // 2
		0x21, 0x41, 0x45, 0x4B, 0x31, // 3
		0x18, 0x14, 0x12, 0x7F, 0x10, // 4
		0x27, 0x45, 0x45, 0x45, 0x39, // 5
		0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
		0x01, 0x71, 0x09, 0x05, 0x03, // 7
		0x36, 0x49, 0x49, 0x49, 0x36, // 8
		0x06, 0x49, 0x49, 0x29, 0x1E, // 9
		0x00, 0x36, 0x36, 0x00, 0x00, // :
		0x00, 0x56, 0x36, 0x00, 0x00, // ;
		0x08, 0x14, 0x22, 0x41, 0x00, // <
		0x14, 0x14, 0x14, 0x14, 0x14, // =
		0x00, 0x41, 0x22, 0x14, 0x08, // >
		0x02, 0x01, 0x51, 0x09, 0x06, // ?
		0x32, 0x49, 0x59, 0x51, 0x3E, // @
		0x7E, 0x11, 0x11, 0x11, 0x7E, // A
		0x7F, 0x49, 0x49, 0x49, 0x36, // B
		0x3E, 0x41, 0x41, 0x41, 0x22, // C
		0x7F, 0x41, 0x41, 0x22, 0x1C, // D
		0x7F, 0x49, 0x49, 0x49, 0x41, // E
		0x7F, 0x09, 0x09, 0x09, 0x01, // F
		0x3E, 0x41, 0x49, 0x49, 0x7A, // G
		0x7F, 0x08, 0x08, 0x08, 0x7F, // H
		0x00, 0x41, 0x7F, 0x41, 0x00, // I
		0x20, 0x40, 0x41, 0x3F, 0x01, // J
		0x7F, 0x08, 0x14, 0x22, 0x41, // K
		0x7F, 0x40, 0x40, 0x40, 0x40, // L
		0x7F, 0x02, 0x0C, 0x02, 0x7F, // M
		0x7F, 0x04, 0x08, 0x10, 0x7F, // N
		0x3E, 0x41, 0x41, 0x41, 0x3E, // O
		0x7F, 0x09, 0x09, 0x09, 0x06, // P
		0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
		0x7F, 0x09, 0x19, 0x29, 0x46, // R
		0x46, 0x49, 0x49, 0x49, 0x31, // S
		0x01, 0x01, 0x7F, 0x01, 0x01, // T
		0x3F, 0x40, 0x40, 0x40, 0x3F, // U
		0x1F, 0x20, 0x40, 0x20, 0x1F, // V
		0x3F, 0x40, 0x38, 0x40, 0x3F, // W
		0x63, 0x14, 0x08, 0x14, 0x63, // X
		0x07, 0x08, 0x70, 0x08, 0x07, // Y
		0x61, 0x51, 0x49, 0x45, 0x43, // Z
		0x00, 0x7F, 0x41, 0x41, 0x00, // [
		0x55, 0x2A, 0x55, 0x2A, 0x55, // 55
		0x00, 0x41, 0x41, 0x7F, 0x00, // ]
		0x04, 0x02, 0x01, 0x02, 0x04, // ^
		0x40, 0x40, 0x40, 0x40, 0x40, // _
		0x00, 0x01, 0x02, 0x04, 0x00, // '
		0x20, 0x54, 0x54, 0x54, 0x78, // a
		0x7F, 0x48, 0x44, 0x44, 0x38, // b
		0x38, 0x44, 0x44, 0x44, 0x20, // c
		0x38, 0x44, 0x44, 0x48, 0x7F, // d
		0x38, 0x54, 0x54, 0x54, 0x18, // e
		0x08, 0x7E, 0x09, 0x01, 0x02, // f
		0x0C, 0x52, 0x52, 0x52, 0x3E, // g
		0x7F, 0x08, 0x04, 0x04, 0x78, // h
		0x00, 0x44, 0x7D, 0x40, 0x00, // i
		0x20, 0x40, 0x44, 0x3D, 0x00, // j
		0x7F, 0x10, 0x28, 0x44, 0x00, // k
		0x00, 0x41, 0x7F, 0x40, 0x00, // l
		0x7C, 0x04, 0x18, 0x04, 0x78, // m
		0x7C, 0x08, 0x04, 0x04, 0x78, // n
		0x38, 0x44, 0x44, 0x44, 0x38, // o
		0x7C, 0x14, 0x14, 0x14, 0x08, // p
		0x08, 0x14, 0x14, 0x18, 0x7C, // q
		0x7C, 0x08, 0x04, 0x04, 0x08, // r
		0x48, 0x54, 0x54, 0x54, 0x20, // s
		0x04, 0x3F, 0x44, 0x40, 0x20, // t
		0x3C, 0x40, 0x40, 0x20, 0x7C, // u
		0x1C, 0x20, 0x40, 0x20, 0x1C, // v
		0x3C, 0x40, 0x30, 0x40, 0x3C, // w
		0x44, 0x28, 0x10, 0x28, 0x44, // x
		0x0C, 0x50, 0x50, 0x50, 0x3C, // y
		0x44, 0x64, 0x54, 0x4C, 0x44 // z
		};
const char waitMsg[] = {
		0x00, 0x00, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x04, 0x00, 0x00,
		0x00, 0x00, 0x08, 0x00, 0x00,
		0x00, 0x00, 0x10, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x40, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x10, 0x00, 0x00,
		0x00, 0x00, 0x08, 0x00, 0x00,
		0x00, 0x00, 0x04, 0x00, 0x00,
		0x00, 0x00, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x04, 0x00, 0x00,
};
const char waitMsg2[] = {
		0x01, 0x01, 0x01, 0x02, 0x02,
		0x04, 0x04, 0x04, 0x08, 0x08,
		0x10, 0x10, 0x10, 0x20, 0x20,
		0x40, 0x40, 0x40, 0x40, 0x40,
		0x20, 0x20, 0x20, 0x10, 0x10,
		0x08, 0x08, 0x08, 0x04, 0x04,
		0x02, 0x02, 0x02, 0x01, 0x01,
};
const int8_t waitMsgSize=sizeof(waitMsg);
char dispBuffer[256];
volatile char *dispBufPtr, *dispBufEndPtr;
u16 dispCnt, charDelay;
bool bBlinkFlag, bScrollFlag, bRawFlag;
int16_t blinkDelay;

// @brief : Write a control character to the dot matrix
// @param c : control register value
// @retval
void DispWriteCtrlReg(char c) {
	DISP_RS_1;
	DISP_CE_0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPI2, c);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
		;
	DISP_CE_1;
	DISP_RS_0;
}

// @brief : To write a column of data into the dot magtrix using polling method
// @param c: column data
// @retval
void DispWriteCol(char c) {

	DISP_RS_0;
	DISP_CE_0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPI2, c);

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
		;
	DISP_CE_1;

}

void DispWriteChar(char c) {
	int j, index;
	char const *ptr;
	DISP_RS_0;
	DISP_CE_0;

	if (c >= ' ' && c <= 'z') {
		index = c - ' ';
		ptr = &lut5x7[index * 5];
		for (j = 0; j < 5; j++) {
			while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
				;
			SPI_I2S_SendData(SPI2, *ptr++);
		}
	}

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
		;
	DISP_CE_1;

}

// @brief : To display the ascii string onto the dotmatrix using polling method
// @param s: the character string
// @retval
void DispDotMatrixPoll(char *s) {
	int i, j, index;
	char const *ptr;
	DISP_RS_0;
	DISP_CE_0;

	for (i = 0; i < 4; i++) {
		if (s[i] >= ' ' && s[i] <= 'z') {
			index = s[i] - ' ';
			ptr = &lut5x7[index * 5];
			for (j = 0; j < 5; j++) {
				while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
					;
				SPI_I2S_SendData(SPI2, *ptr++);
			}
		}
	}

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
		;
	DISP_CE_1;

}

// @brief : To display the string onto the dotmatrix & wait for switch press
// @param s: the character string
// @retval
int DispDotMatrixWait(const char *s) {
	DispDotMatrix(s);
	return ReadSWSpecial();
}

// @brief : To display the ascii string onto the dotmatrix using dma
// @param s: the character string
// @retval
void DispDotMatrix(const char *s) {
	int i, j, index;
	char const *ptr;
	char *dest;
	dest = dispBuffer;


	for (i = 0; *s; s++, i++) {
		if (*s >= ' ' && *s <= 'z') {
			index = *s - ' '; // index to 5x7 segment lut
			ptr = &lut5x7[index * 5];
			for (j = 0; j < 5; j++) {
				*dest++ = *ptr++;
			}
		}
	}
	for (; i < 4; i++) {
		for (j = 0; j < 5; j++) {
			*dest++ = 0;
		}

	}
	dispBufEndPtr = dest;
	dispBufPtr = dispBuffer;

	bRawFlag = FALSE;
	dispCnt = 0;


#ifdef POLLING
	while(!DMA_GetFlagStatus(DMA1_FLAG_TC5));

	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
	//DMA_ClearFlag(DMA1_FLAG_TC5);

	DISP_CE_1;
#endif
}
// @brief : To display the raw set of column data on to dot matrix
// @param s: pointer to raw data
// @param len : length of raw data
// @retval
void DispDotMatrixRaw(const char *s, uint16_t len) {
	int16_t i ;
	char *dest;
	dest = dispBuffer;


	for (i = 0; i<len; i++) {
		*dest++ = *s++;
	}

	dispBufEndPtr = dest;
	dispBufPtr = dispBuffer;

	bRawFlag = TRUE;
	dispCnt = 0;

}

// @brief : To initialise SPI2 for dot matrix display interface
// @param : none
// @retval: none
void DispInit() {
	SPI_InitTypeDef SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Enable DMA1 clocks */
	// Use DMA for display x'fer
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	// SPI2 TX is tied to DMA1 channel5
	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) &(SPI2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) dispBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 20;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	// Enable SPI2 dma tx. It is linked o DMA1 channel5
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

	// Enable dma interrupt on TC5 (transfer complete)
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DISPLAY_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

	/* Configure SPI_MOSI as alternate function push-pull */
	GPIO_Init_Mode(GPIOB, GPIO_Pin_13 | GPIO_Pin_15, GPIO_Mode_AF_PP);

	// Configure PB14 as DISP RS pin
	GPIO_Init_Mode(GPIOB, GPIO_Pin_14, GPIO_Mode_Out_PP);

	//	 Configure PB12 as DISP CE pin. NSS don't work
	GPIO_Init_Mode(GPIOB, GPIO_Pin_12, GPIO_Mode_Out_PP);

	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;

	SPI_Init(SPI2, &SPI_InitStruct);
	SPI_SSOutputCmd(SPI2, ENABLE);

	SPI_Cmd(SPI2, ENABLE);
	SPI_NSSInternalSoftwareConfig(SPI2, ENABLE);

	DISP_CE_1;
	bScrollFlag=TRUE;
	DispWriteCtrlReg(0x77);
}

void DMA1_Channel5_IRQHandler(void) {
	DMA_ClearITPendingBit(DMA1_IT_TC5);

	// This is necessary to make sure that the last byte is transferred before
	// setting DISP_CE to 1.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
		;

	DISP_CE_1;
}

// @brief : To blink the display. For menu select indication
// @param
// @retval
void DispBlink(void){
	bBlinkFlag=TRUE;
	blinkDelay=80;
	dispCnt=0;
}

// @brief : To enable or diable display character scrolling
// @param state : true to enable, false to disable
// @retval
void DispScroll(bool state) {
	bScrollFlag = state;
}

uint8_t dispBufBlank[20];
// @brief : To setup dma for dot matrix display
// @param
// @retval
// Disp_dma_service() is called from 1ms ISR.
// It handles the scrolling of the display. Setup the dma to service the SPI transfer.
void Disp_dma_service(void) {
	if (blinkDelay) {
		if (bBlinkFlag){
			DISP_CE_0;
			DMA_Cmd(DMA1_Channel5, DISABLE); // Must disable in order to load CNDTR
			DMA1_Channel5->CMAR = (u32) (dispBufBlank);
			DMA1_Channel5->CNDTR = 20;
			DMA_Cmd(DMA1_Channel5, ENABLE);
			bBlinkFlag = FALSE;
		}
		blinkDelay--;
		return;
	}
	if (!dispCnt) {
		// delay longer for first and last 4 characters
		if ((dispBufEndPtr - dispBufPtr) < 20) {
			dispBufPtr = dispBuffer;
		}
		if ((dispBufPtr==dispBuffer) || (dispBufEndPtr - dispBufPtr) == 20) {
			charDelay = 800;
		} else
			charDelay = 250;
		if (bRawFlag==TRUE) charDelay=100;
		//DISP_RS_0;
		DISP_CE_0;
		DMA_Cmd(DMA1_Channel5, DISABLE); // Must disable in order to load CNDTR
		DMA1_Channel5->CMAR = (u32) (dispBufPtr);
		DMA1_Channel5->CNDTR = 20;
		DMA_Cmd(DMA1_Channel5, ENABLE);
	}

	dispCnt++;
	if (dispCnt >= charDelay) {
		dispCnt = 0;
		if (bScrollFlag)
			dispBufPtr += 5;
	}
}

void DispDotMatrixInt16(int16_t *num) {
	char s[10];

	while(1) {
		sprintf(s,"%4d", *num);
		DispDotMatrix(s);
		DelaymSec(50);

		if (bSWFlag) {
			bSWFlag=FALSE;
			break;
		}
	}
}

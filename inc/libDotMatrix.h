// ---------------------------------------------------------------------------------
// libDotMatrix.h
// Created on: 12-Feb-2010
// Author: nbk
// ---------------------------------------------------------------------------------
#ifndef LIBDOTMATRIX_H_
#define LIBDOTMATRIX_H_

// ---------------------------------------------------------------------------------
//  Macros
// ---------------------------------------------------------------------------------
#define DISP_RS_0	GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define DISP_RS_1	GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define DISP_CE_0	GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define DISP_CE_1	GPIO_SetBits(GPIOB, GPIO_Pin_12)

// ---------------------------------------------------------------------------------
//  Function prototypes
// ---------------------------------------------------------------------------------
void DispInit(void);
void DispWriteCol(char c);
void DispWriteChar(char c);
void DispDotMatrixPoll(char *s);
void DispDotMatrix(const char *s);
int DispDotMatrixWait(const char *s);
void DispDotMatrixRaw(const char *s, uint16_t len);
void DispWriteCtrlReg(char c);
void DispConfig(char c);
void Disp_dma_service(void);
void DispBlink(void);
void DispScroll(bool state);

#endif /* LIBDOTMATRIX_H_ */

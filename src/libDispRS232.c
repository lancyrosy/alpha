// ---------------------------------------------------------------------------------
// * libDispRS232.c
// *
// *  Created on: 07-Jan-2010
// *  Author: nbk
// *  Desc  : the rountines here are for simulating the SPI-driven dot matrix display on the
// 			: rs232 port.
// ---------------------------------------------------------------------------------
#include "string.h"
#include "project.h"
#define DEBUG
#include "libdebug.h"

u16 dispCnt, charDelay;
bool bBlinkFlag, bScrollFlag, bRawFlag;
int16_t blinkDelay;
bool bDisplayDelay;
const char waitMsg[] = {
0
};
const int8_t waitMsgSize=sizeof(waitMsg);

#define DISP_RS232  	// turn this on to use winH8 to simulate dotmatrix
static char dotMatrixSimData[7]="[    ]";
// @brief : To display the string onto the dotmatrix & wait for switch press
// @param s: the character string
// @retval
int DispDotMatrixWait(const char *s) {

	register unsigned i;
	const char *dispStrg;
	DispDotMatrix(s);

#ifdef DISP_RS232  	// turn this on to use winH8 to simulate dotmatrix
	bSWFlag = 0;

	if (bDisplayDelay) {
		/* To ensure that some important message will be shown at least for 0. sec */
		DelaymSec(500);
		bDisplayDelay = 0;
	}


	while(1) {

		dispStrg = s;

		for (i=0; i<(strlen(dispStrg)-3); i++) {

			DispDotMatrix(dispStrg+i);


			if (i==0) {
				DelaymSec(800);
			}
			else {
				DelaymSec(200);
			}

			if (bSWFlag==1) {

				return ReadSWSpecial();

			}
		}

		DelaymSec(500);

	}
#endif
}

// @brief : To display the ascii string onto the dotmatrix using dma
// @param s: the character string
// @retval

void DispDotMatrix(const char *s) {

	char *dest;
	dest = dispBuffer;
#ifdef DISP_RS232  	// turn this on to use winH8 to simulate dotmatrix
	dotMatrixSimData[1] = s[0];
	dotMatrixSimData[2] = s[1];
	dotMatrixSimData[3] = s[2];
	dotMatrixSimData[4] = s[3];
	PRINTDOTMATRIX(dotMatrixSimData);
#endif

}

// @brief : To initialise SPI2 for dot matrix display interface
// @param : none
// @retval: none
void DispInit() {
}


// @brief : To blink the display. For menu select indication
// @param
// @retval
void DispBlink(void){
	ledValue = 0x00;
	OffDotMatrix();
	DelaymSec(200);
	OnDotMatrix();
	ledValue = 0x0f;
}

void DispScroll(bool state) {
}

void DispDotMatrixRaw(const char *s, uint16_t len) {
	DispDotMatrix("....");
}
void Disp_dma_service(void) {
}

void OnDotMatrix( ){
#ifdef DISP_RS232  	// turn this on to use winH8 to simulate dotmatrix
	PRINTDOTMATRIX(dotMatrixSimData);
#endif

}

void OffDotMatrix( ){
#ifdef DISP_RS232  	// turn this on to use winH8 to simulate dotmatrix
	PRINTDOTMATRIX("[    ]");
#endif

}

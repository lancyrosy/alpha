// ---------------------------------------------------------------------------------
//   File        :main.c
//   Version     :v1.0.1
//   Date        :06Feb2011
//   Author      :NgBK
//   Description :main()
// ---------------------------------------------------------------------------------

//......................................................................................
// To do list (reminder on what to do in this project)
// test input port pins
// buzzer
// flash data
//......................................................................................

#include "project.h"
#include <string.h>

static int16_t CAM_UartNum;

#define CAMCheckRx()	CheckRxBuf(CAM_UartNum)
#define CAMSendChar(c)	TxCharISR(CAM_UartNum, c)
#define CAMGetChar()	RxCharISR(CAM_UartNum)
#define CAMPutStr(s)	PutStrISR(CAM_UartNum, s)

volatile char camDataChar[CAM3_NUMOFCOLOR][50];
volatile int camData[CAM3_NUMOFCOLOR][CAM3_NUMOFPARA];
volatile bool camDataFlag[CAM3_NUMOFCOLOR];
int alignColorNum=2, alignXref=44;		// yellow line is color 2
										// default reference is 44, centre of image
int alignColorPara=0;					// A few parameters can be used for alignment
bool bColorTrackFlag=FALSE;

// Refer to libCmuCam.h
// camdata[0][CAM3_NUMOFPARA] - color 0 parameters
// camdata[1][CAM3_NUMOFPARA] - color 1 parameters
// camdata[2][CAM3_NUMOFPARA] - color 2 parameters
void PrintCamValues(){	// To print CMUCAM3 values for debugging purposes
	int num;

	clrscr();
	while(1) {
		for (num=0; num<3; num++) {
			gotoxy(12,10+num);
			printf("%d %3d %3d %3d %3d %3d %3d %3d %3d", num, camData[num][0], camData[num][1],
			camData[num][2], camData[num][3],camData[num][4], camData[num][5],
			camData[num][6], camData[num][7]);
		}
		if (bSWFlag) {
			bSWFlag = FALSE;
			//Beep(SOUND_ACK);
			DelaymSec(200);
			break;
		}
	}
}

// Use the output of CMUCAM3 to align robot
// alignColorNum must be initialised to 0, 1 or 2.
// alignColorNum must also be initialised to the parameters for alignment
int AlignCAM3CG() {
	int xsize, alignOffset;

	xsize = camData[alignColorNum][CAM3_X1] - camData[alignColorNum][CAM3_X0];
	if ((camData[alignColorNum][CAM3_AREA]>20)) {
		alignOffset = camData[alignColorNum][alignColorPara]-alignXref;
	}
	else {
		alignOffset = 0;
	}
	return alignOffset*SPEED_deg_oc(-10)/4;
}

// @brief : To initialise USART for CMUCAM and start CMUCAM tracking
// @param : uartNum is usart#-1. so USART1 -> 0, baudrate
// @retval: None
void StartCMUCAM(int16_t uartNum, int baudrate) {
	char s[8];
	bool bCamFlag;

	UART_init(uartNum, baudrate);

	CAM_UartNum = uartNum;
	// Test CMUCAM3
	// Send '\r' and wait for "ACK\r"
	CAMSendChar('\r');
	CAMSendChar('\r');
	StartElapsedTimer();
	bCamFlag = FALSE;

	while(1) {
		// wait for reply with timeout in 500msec
		if (GetElapsedTime()>500) break;
		if (CAMCheckRx()) {
			s[0] = s[1];
			s[1] = s[2];
			s[2] = s[3];
			s[3] = CAMGetChar();
//			PutChar(s[3]);
			if (!strncmp(s,"ACK\r",4)) {
				bCamFlag = TRUE;
				break;
			}
		}

	}

	if (bCamFlag) {
		DispDotMatrixWait("CAM3-OK");
		bColorTrackFlag = TRUE;
		CAMPutStr("RM 1\r");		// raw mode
		CAMPutStr("T3\r");			// Track 3 colors
	}
	else
		DispDotMatrixWait("CAM3-BAD");

}

uint8_t CAMRawData[CAM3_NUMOFPARA+2];
int16_t CAMRawDataIndex;
// @brief : To process CMUCAM3 data. Must be called regularly, say, every msec
// @param :
// @retval: None
void CAMRxISR() {

	if (bColorTrackFlag) {
		while(CAMCheckRx()) {
			CAMRawData[CAMRawDataIndex++] = CAMGetChar();
			if (CAMRawData[0] != 255) { 	// First byte is 255
				CAMRawDataIndex = 0;
			}

			// In CMUCAM3 text output mode
			// Format is "Tn cx cy x0 y0 x1 y1 a q xL xR"
			// n - color number from 0 to 2
			// cx, cy - centroid xy coordinate
			// x0, y0, x1, y1 - bounding box
			// a - area in pixel ?
			// q - quality, (pixel area/bounding box area)
			// xL, xR - left and right edge centroid
			//
			// In CMUCAM3 raw output mode,
			// Format is "Hc0123456789A"
			else if (CAMRawDataIndex==(CAM3_NUMOFPARA+2)) {
				int i;
				CAMRawDataIndex = 0;
				if (CAMRawData[1]<=2) {
					// rxBuffer[1] is the color number. Value from 0-2
					camDataFlag[CAMRawData[1]] = TRUE;
					for (i=0; i<CAM3_NUMOFPARA; i++) {
						camData[CAMRawData[1]][i]=CAMRawData[i+2];
					}
				}
			}
		}
	}
}

// @brief : To connect CMUCAM to WinM3
// @param : None
// @retval: None
void CAM_to_WinM3(){

    bColorTrackFlag = FALSE;
    TxCharISR(CAM_UartNum, '\n');

    while(1){
    	if (CheckRxBuf(CAM_UartNum)) {
    		TxCharISR(0, RxCharISR(CAM_UartNum));
    	}
    	if (CheckRxBuf(0)) {
    		TxCharISR(CAM_UartNum, RxCharISR(0));
    	}
    }

}

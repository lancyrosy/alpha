// ---------------------------------------------------------------------------------
// * libLED.c
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

char ledTmp;
char ledValue;
int ledCnt;
char ledValue;
char ledBlinkPat2[50]={1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1};
char ledBlinkPat1[50]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//char ledBlinkPat1[50]={0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,};
char ledBlinkPat3[50]={0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1};
char *ledBlinkPat=ledBlinkPat1;
char cnt20ms;
int16_t ledBlinkPatIndex;
bool bBlinkFlag = FALSE;
bool bPulseFlag = FALSE;
unsigned pulseDuration[4];
void BlinkLEDOnce() {
	ledValue = 0x0;
	DelaymSec(100);
	ledValue = 0x07;
}

void PulseLED(int16_t ledNum, unsigned duration) {
	pulseDuration[ledNum] = duration;
	bPulseFlag = TRUE;
	bBlinkFlag = FALSE;
}

int16_t ledRotatePat;
bool dirFlag;
void RotateLED() {
	DelaymSec(60);
	ledRotatePat>>=1;
	if(ledRotatePat==0) ledRotatePat = 0x800;
	ledValue = ledRotatePat;
}

void SwingLED() {
	DelaymSec(100);
	if(ledRotatePat==0) {
		ledRotatePat = 0x1;
		dirFlag = 0;
	}
	else if(ledRotatePat==0x8){
		ledRotatePat = 0x4;
		dirFlag = 1;
	}

	if (dirFlag) {
		ledRotatePat>>=1;
	}
	else {
		ledRotatePat<<=1;
	}
	ledValue = ledRotatePat;

}
void DispLEDError() {
	bBlinkFlag = TRUE;
	ledValue = 0x0f;
	ledBlinkPat = ledBlinkPat3;
}

void LEDISR() {
	if (bBlinkFlag) {
		cnt20ms++;
		if (cnt20ms>=10) {
			cnt20ms = 0;
			if (ledValue==0 ) {
				// Flash quickly to indicate zero. Else there's no indication.
				ledTmp = (ledBlinkPatIndex==0)?0x0f:0;
			}
			else {
				ledTmp = (ledBlinkPat[ledBlinkPatIndex])?ledValue:0;
			}
			ledBlinkPatIndex++;
			if (ledBlinkPatIndex>=50) ledBlinkPatIndex=0;
		}
	}
	else {
		if (bPulseFlag==TRUE) {
			if (pulseDuration[0]) {
				pulseDuration[0]--;
				ledValue |= 0x01;
			} else ledValue &= 0xfe;
			if (pulseDuration[1]) {
				pulseDuration[1]--;
				ledValue |= 0x02;
			} else ledValue &= 0xfd;
			if (pulseDuration[2]) {
				pulseDuration[2]--;
				ledValue |= 0x04;
			} else ledValue &= 0xfb;
			if (pulseDuration[3]) {
				pulseDuration[3]--;
				ledValue |= 0x08;
			} else ledValue &= 0xf7;

		}
		ledTmp = ledValue;
	}
	if (ledTmp&0x01)LED0_ON; else LED0_OFF;
	if (ledTmp&0x02)LED1_ON; else LED1_OFF;
	if (ledTmp&0x04)LED2_ON; else LED2_OFF;
	if (ledTmp&0x08)LED3_ON; else LED3_OFF;

}
void SetLED( char value) {
	if (value>=0 && value <=15) {
		ledValue = value;
		//ledBlinkPat = ledBlinkPat1;
	}
}


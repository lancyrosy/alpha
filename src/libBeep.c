/*****************************************************************************
  File : libbeep.c
  Date : 28 Jan 99
  Name : NBK
  Desc : Beep the buzzer
  Modi :
*****************************************************************************/

#include "Project.h"


void InitBeeper() {
}


void Bird();
void Bird2();

#define TUNEBEEP_NO
#ifdef TUNEBEEP
/* For tuning beep sound */
void TuneBeep() {
	register int i, n;
	int sound=0;

	i=0;
	clrscr();
	while(1) {
		gotoxy(2,2);
		printf("base=%d divisor=%d ", base, divisor);

		switch(getch()){
		case 'b' : scanf("%d", &base);
			 	break;
		case 'd' : scanf("%d", &divisor);
			 	break;
		}
		BeepOK();
//		Bird();
	}
}
#endif
/* intCnt is required as there may be a interrupt during Bird() execution */
/* This variable is used to decide whether to disable TIM3 */
/* Without this protection, if TIM is disabled, the interrupted function */
/* may wait forever in the while(!IF0H.5) loop */
void Bird() {
	DelaymSec(50);
}

void Bird2() {
	DelaymSec(50);
}

void Beep(int sound) {
	if (sound == SOUND_ACK) {
		Bird2();
	}
	else {
		Bird();
	}
	//DelaymSec(50);
}

void BeepError() {
	int i;

	for (i=0; i<2; i++) {
		BeepOK();
		if (bSWFlag)
			break;
	}
}

void BeepOK() {
	int i;
	for (i=0; i<4; i++) {
		Bird();
		DelaymSec(20);
	}
}


void BeepConstant(int freq) {
}

void BeepOnce() {
}

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
#include "stdlib.h"
#include "MyFunction.h"

unsigned pulseDuration[2];
unsigned aveSensorBlack[15];
int pulseBuzzerDuration = 0;
int sensoroffsetsqr = 0;
int xSpeed = 0;
volatile int LSumMarker,RSumMarker,sumJunction,disL,disR;
int LState,RState,JLState,JRState;
int i=0;

#define L_MARKER_SEN sensorBlack[13]
#define R_MARKER_SEN sensorBlack[14]

bool bPulseFlag = FALSE;
bool bJunFlag = FALSE;

void collectBlackValue();
void pulseLED(int num, int duration);
void pulseBuzzer( int per, int duration);
void LMarketDetect();
void RMarketDetect();
void JMarkerDetect();
void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc);



void pulseLED(int num, int duration){
	bPulseFlag = TRUE;
	pulseDuration[num] = duration;
}

void pulseBuzzer( int period, int duration){
	TIM2->ARR = period;
    TIM2->CCR2 = TIM2->ARR/2;
	pulseBuzzerDuration = duration;
}

void TestRun(){

	DelaymSec(1000);
	ClearMarkerFlag();
	char s[8];
	while(!RSumMarker==2) {
		if(sensoroffset<-150) sensoroffset = -250;
		if(sensoroffset>150)  sensoroffset = 250;

		sensoroffsetsqr = sensoroffset*sensoroffset;

		xSpeed = -(0.1)*sensoroffsetsqr+1000;
		SetRobotSpeedX(xSpeed);
		// Do other stuff here!!!
		//printf("\ncurPos0=%-5d s=%5d", (int16_t)(curPos[0]/DIST_mm_oc(1)), curSpeed[0]);
		// like checking for sensors to detect object etc
		sprintf(s,"4%d", xSpeed);
		DispDotMatrix(s);
		if (bSWFlag ) {	// user switch break!
			break;
		}
	}
	StopRobot();
	WaitSW();

}


void LMarketDetect(){
	if (sensorCal[13] >= 600) {
		LState = 1;
		JLState = 0;
	}
	if (sensorCal[13] <= 400 && LState == 1){
		LState = 0;
		JLState = 1;
		disL = curPos[0]/DIST_mm_oc(1);
		JMarkerDetect();
	}
	if (JLState==1) {
		uint16_t tDist = curPos[0]/DIST_mm_oc(1);
		if ((tDist-disL)>25) {
			JLState = 0;
			LSumMarker ++;
			pulseLED(0,100);
			//pulseBuzzer(1000, 50);
		}
	}
}

void RMarketDetect(){
	if (sensorCal[14] >= 600){
		RState = 1;
		JRState = 0;
	}
	if (sensorCal[14] <= 400 && RState == 1){
		RState = 0;
		JRState = 1;
		disR = curPos[0]/DIST_mm_oc(1);
		JMarkerDetect();
	}
	if (JRState==1) {
		uint16_t tDist = curPos[0]/DIST_mm_oc(1);
		if ((tDist-disR)>25) {
			JRState = 0;
			RSumMarker ++;
			pulseLED(1,100);
			pulseBuzzer(4000, 50);
		}
	}
}
void JMarkerDetect(){
	if ((JLState == 1) && (JRState == 1)) {
		if (abs(disL - disR) < 20) {
			sumJunction++;
			//LSumMarker--;
			//RSumMarker--;
			pulseBuzzer(2500, 50);
		}
		JLState=JRState=0;
	}
}
void ClearMarkerFlag(){
	JRState=JLState=0;
	LState=RState=0;
	RSumMarker=LSumMarker=sumJunction=0;
}

void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc) {

	SetMoveCommand(speedType, dist, brakeDist,  topSpeed, endSpeed, acc);
	bAlignFlag = FALSE;

	while(!EndOfMove(speedType)) {

		while(!bMotorISRFlag);
		bMotorISRFlag=FALSE;
		DispDotMatrix("Black");

		for (i=0; i<15; i++) {
			if(aveSensorBlack[i] == 0) aveSensorBlack[i] = sensor[i];
			else aveSensorBlack[i] = (aveSensorBlack[i] + sensor[i])/2;
			sensorBlack[i] = aveSensorBlack[i];
		}

		// Do other stuff here!!!
		//printf("\ncurPos0=%-5d s=%5d", (int16_t)(curPos[0]/DIST_mm_oc(1)), curSpeed[0]);
		// like checking for sensors to detect object etc
		if (bSWFlag) {	// user switch break!
			bSWFlag=FALSE;
			break;
		}
	}

	bAlignFlag = TRUE;
}
void MoveRobotExplore(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc) {
	char s[8];


	while(!RSumMarker==2) {
		if(sensoroffset<-150) sensoroffset = -250;
		if(sensoroffset>150)  sensoroffset = 250;

		sensoroffsetsqr = sensoroffset*sensoroffset;

		xSpeed = -(0.1)*sensoroffsetsqr+1000;
		SetRobotSpeedX(xSpeed);
		// Do other stuff here!!!
		//printf("\ncurPos0=%-5d s=%5d", (int16_t)(curPos[0]/DIST_mm_oc(1)), curSpeed[0]);
		// like checking for sensors to detect object etc
		sprintf(s,"4%d", xSpeed);
		DispDotMatrix(s);
		if (bSWFlag ) {	// user switch break!
			break;
		}
	}
}

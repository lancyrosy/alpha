#include "project.h"
#include "stdlib.h"
#include "MyFunction.h"
#include "Math.h"

//#define Fixed Speed Run
#define LOGSIZE	12000
int logData[LOGSIZE];
int logExplore[1000];
int logIndex;
bool logFlag = FALSE;

unsigned pulseDuration[2];
unsigned aveSensorBlack[15];
int pulseBuzzerDuration = 0;

int sensoroffsetsqr = 0;
int tsensoroffset = 0;
int xSpeed = 0;
int sl,f;

volatile int LSumMarker,RSumMarker,sumJunction,disL,disR;
int LState,RState,JLState,JRState;
int i=0;

#define L_MARKER_SEN sensorBlack[13]
#define R_MARKER_SEN sensorBlack[14]

bool bPulseFlag = FALSE;
bool bJunFlag = FALSE;

void StartLog(){
	logIndex = 0;
	logFlag = TRUE;
}
void LogData(int data) {
	if (logFlag==TRUE && logIndex<LOGSIZE) {
		logData[logIndex] = data;
		logIndex++;
	}
}
void PrintLog() {
	int i;
	logFlag=FALSE;
	logIndex=0;
	for (i=0; i<LOGSIZE; ) {
		printf("\n%5d", logData[i++]);
		printf(" %5d", logData[i++]);
		printf(" %5d", logData[i++]);
	}
	/*for (i=0;i <1000;i++){
		printf("\n%5d",logExplore[i]);
	}*/
}

void LogExplore (){
	//int i,count,Rcount,Lcount;

	for (i=2;i<1000;i=i+3){
		if(logData[i] <= 80 && logData[i] >= -80){
			//count++;
			logExplore[i] = 0; //If sensoroffset(abs) is less than 80 for 500ms, Straight line
			//if(logData[i] >80 || logData[i] <-80) count = 0; //Clear count to 0
		}
		else if(logData[i] > 80){
			//Rcount++;
			logExplore[i] = -1;//If sensoroffset is higher than 800 for 500ms, Right curve
			//if(logData[i] <= 80) Rcount = 0;
		}
		else if(logData[i] < -80){
			//Lcount++;
			logExplore[i] = 1;//If sensoroffset is lower than -800 for 500ms, Left curve
			//if(logData[i] >= -80) Lcount = 0;
		}
	}
}


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

#define y0 1000
#define a 800.0f
#define b 500.0f
void TestRun(){
	DelaymSec(1000);
	EnWheelMotor();
	SetRobotAccX(10000);

	ClearMarkerFlag();
	int ty0;

	char s[8];
	StartLog();
	while(RSumMarker!=2) {
		if(slowFlag == TRUE){
			ty0=y0-300;
		}
		else{
			ty0=y0+300;
		}

		/*if(abs(sensoroffset) > abs(sensoroffset2)){
			tsensoroffset = sensoroffset;
		}
		else
			tsensoroffset = sensoroffset2;
		*/
		if(tsensoroffset<-a) tsensoroffset = -a;
		if(tsensoroffset>a)  tsensoroffset = a;

		sensoroffsetsqr = (long)tsensoroffset*tsensoroffset;

		//xSpeed = -0.000781f*sensoroffsetsqr+1000;

		xSpeed=(b/a)*sqrt(a*a-sensoroffsetsqr)+ty0;
		SetRobotSpeedX(xSpeed);
		// Do other stuff here!!!
		//printf("\ncurPos0=%-5d s=%5d", (int16_t)(curPos[0]/DIST_mm_oc(1)), curSpeed[0]);
		// like checking for sensors to detect object etc
		sprintf(s,"%d", timeCount/1000);
		//gotoxy(5,5);
		//printf(" 2nd row %d   1st row  %d  tsen   %d  xspeed  %d   ",sensoroffset,sensoroffset2,tsensoroffset,xSpeed);
		//gotoxy(5,10);
		//printf(" slowFlag:%d  fastFlag:%d  s1:%d  s2:%d  s3:%d   ",sl,f,sensorCal[0],sensorCal[1],sensorCal[2]);
		DispDotMatrix(s);
		if (bSWFlag ) {	// user switch break!
			break;
		}
	}

	StopRobot();
	WaitSW();
}

//Marker detect
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


//Collect black value
void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc) {
	DelaymSec(1000);
	bAlignFlag = FALSE;
	SetMoveCommand(speedType, dist, brakeDist,  topSpeed, endSpeed, acc);

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
	//change target speed then call SetRobotSpeedX() which inside libProfile
	while(!RSumMarker==2) {
		// Do other stuff here!!!
		//printf("\ncurPos0=%-5d s=%5d", (int16_t)(curPos[0]/DIST_mm_oc(1)), curSpeed[0]);
		// like checking for sensors to detect object etc
		if (sensoroffset < -150)
			sensoroffset = -250;
		if (sensoroffset > 150)
			sensoroffset = 250;

		sensoroffsetsqr = sensoroffset * sensoroffset;
		xSpeed = -(0.1) * sensoroffsetsqr + 1000;
		SetRobotSpeedX(xSpeed);

		sprintf(s,"%4d", xSpeed);
		DispDotMatrix(s);
		if (bSWFlag) {	// user switch break!
			break;
		}
	}
}


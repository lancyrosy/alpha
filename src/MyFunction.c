#include "project.h"
#include "stdlib.h"
#include "MyFunction.h"
#include "Math.h"

//#define Fixed Speed Run
void FindSegments(void);
void FilterSegments(void);
#define LOGSIZE	9000
#define SEGSIZE 200
int logData[LOGSIZE];
int segment[SEGSIZE], segType[SEGSIZE], segNum;
int segmentF[SEGSIZE], segTypeF[SEGSIZE], segNumF;
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


void LogData(int data) {
	if (logFlag==TRUE && logIndex<LOGSIZE) {
		logData[logIndex] = data;
		logIndex++;
	}
}
void PrintLog() {
	int i;
	logFlag=FALSE;

	for (i=0; i<logIndex; ) {
		printf("\n%5d", logData[i++]);
		//printf(" %5d", logData[i++]);
		//printf(" %5d", logData[i++]);
	}
}

void PrintSegment() {
	int i;
	for (i=0; i<segNum; i++ ) {
		printf("%5d  %2d\n", segment[i], segType[i]);
	}
	printf("\n\n\n");
	for (i=0; i<segNumF; i++ ) {
		printf("%5d  %2d\n", segmentF[i], segTypeF[i]);
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

void TestRun(){

	timeCount = 0;

	DelaymSec(1000);
	EnWheelMotor();
	ClearMarkerFlag();
	char s[8];

	logIndex = 0;
	logFlag = FALSE;

	while(RSumMarker!=2){

		SetRobotSpeedX(1000);

		// Do other stuff here!!!
		//printf("\ncurPos0=%-5d s=%5d", (int16_t)(curPos[0]/DIST_mm_oc(1)), curSpeed[0]);
		// like checking for sensors to detect object etc
		sprintf(s,"%4d", (int)(timeCount/100));
		//gotoxy(5,5);
		//printf(" 2nd row %d   1st row  %d  tsen   %d  xspeed  %d mark %d  ",sensoroffset,sensoroffset2,tsensoroffset,xSpeed,RSumMarker);
		//gotoxy(5,10);
		//printf(" slowFlag:%d  fastFlag:%d  s1:%d  s2:%d  s3:%d   ",sl,f,sensorCal[0],sensorCal[1],sensorCal[2]);
		DispDotMatrix(s);
		if (bSWFlag ) {	// user switch break!
			break;
		}
	}
	logFlag = FALSE;
	StopRobot();
	WaitSW();
	FindSegments();
}

#define thres 70
void FindSegments(void) {
	int i, offset;
	int cFlag;		//0:str, 1:+, -1:-ve
	cFlag = 0;
	segNum = 0;

	for (i=0; i<logIndex; i++) {
		offset = logData[i];

		if(offset >= thres) {
			if(cFlag != 1 ){
				segment[segNum] = i;
				segType[segNum++] = cFlag;
				cFlag = 1;
			}
		}
		else if(offset <= -thres) {
			if(cFlag != -1 ){
				segment[segNum] = i;
				segType[segNum++] = cFlag;
				cFlag = -1;
			}
		}
		else  {
			if((cFlag != 0)){
				segment[segNum] = i;
				segType[segNum++] = cFlag;
				cFlag = 0;
			}
		}
	}
	segment[segNum] = i;
	segType[segNum++] = cFlag;
	FilterSegments();
}

void FilterSegments(void) {
	int dif, i;
	segNumF = 0;
	segmentF[0] = segment[0];
	segTypeF[0] = segType[0];
	for (i = 1; i < segNum; i++) {
		dif = segment[i] - segment[i-1];
		printf("d=%4d s=%d\n",dif, segNumF);
		if (dif > 20) {
			segNumF++;
			segmentF[segNumF] = segment[i];
			segTypeF[segNumF] = segType[i];
		}
		else {
			segmentF[segNumF] = segment[i];
		}
	}
	segNumF++;
}

#define y0 1200
#define a 250.0f
#define b 600.0f

void DumbRun(void){
	logIndex = 0;
	timeCount = 0;
	DelaymSec(1000);

	EnWheelMotor();

	SetRobotAccX(3000,8000);

	ClearMarkerFlag();
	int ty0=y0;
	//int tmpOffset, tmpSpeed;
	//tsensoroffset = sensoroffset;

	char s[8];
	while(RSumMarker!=2) {
		//change the value of y
//		if(slowFlag == TRUE){
//			ty0=y0-300;
//		}
//		else{
//			ty0=y0+300;
//		}
		//change tsensoroffset
//		tmpOffset = sensoroffset2;
//		tmpSpeed = curSpeed[0]/SPEED_mm_oc(1);
//		if(tmpSpeed<y0) tmpSpeed = y0;
//		tmpOffset = tmpOffset*(long)(tmpSpeed-600)/400;


		if(abs(sensoroffset) > abs(sensoroffset2)){
			tsensoroffset = sensoroffset;
		}
		else
			tsensoroffset = sensoroffset2;

		if(tsensoroffset<-a) tsensoroffset = -a;
		if(tsensoroffset>a)  tsensoroffset = a;


		sensoroffsetsqr = (long)tsensoroffset*tsensoroffset;
		xSpeed=(b/a)*sqrt(a*a-sensoroffsetsqr)+ty0;
		SetRobotSpeedX(xSpeed);

		// Do other stuff here!!!
		//printf("\ncurPos0=%-5d s=%5d", (int16_t)(curPos[0]/DIST_mm_oc(1)), curSpeed[0]);
		// like checking for sensors to detect object etc
		sprintf(s,"%4d", (int)(timeCount/100));
		//gotoxy(5,5);
		//printf(" 2nd row %d   1st row  %d  tsen   %d  xspeed  %d mark %d  ",sensoroffset,sensoroffset2,tsensoroffset,xSpeed,RSumMarker);
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

void FastRun(){

}
//Marker detection
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
	while(RSumMarker!=2) {
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


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
int segmentL[SEGSIZE], segTypeL[SEGSIZE], segNumL;
int segmentF1[SEGSIZE], segTypeF1[SEGSIZE], segNumF1;
int segmentF2[SEGSIZE], segTypeF2[SEGSIZE], segNumF2;
int segmentF3[SEGSIZE], segTypeF3[SEGSIZE], segNumF3;
int logIndex;
int left[100];
bool logFlag = FALSE;

unsigned pulseDuration[2];
unsigned aveSensorBlack[15];
int pulseBuzzerDuration = 0;

int leftNum = 0;
int sensoroffsetsqr = 0;
int tsensoroffset = 0;
int xSpeed = 0;

volatile int LSumMarker,RSumMarker,sumJunction,disL,disR;
int LState,RState,JLState,JRState;

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
	//analyseSegment();
	int i;
	for (i=0; i<segNum; i++ ) {
		printf("%5d  %2d\n", segment[i], segType[i]);
	}
	printf("\n\n\n");
	for (i=0; i<segNumF1; i++ ) {
		printf("%5d  %2d\n", segmentF1[i], segTypeF1[i]);
	}
	printf("\n\n\n");
	for (i=0; i<segNumF2; i++ ) {
			printf("%5d  %2d\n", segmentF2[i], segTypeF2[i]);
	}

	printf("\n\n\n");
	for (i=0; i<segNumF3; i++ ) {
			printf("%5d  %2d\n", segmentF3[i], segTypeF3[i]);
	}

	printf("\n\n\n");
	for (i=0; i<segNumL; i++ ) {
		printf("%5d  %2d\n", segmentL[i], segTypeL[i]);
	}

	printf("\n\n\n");
	for (i=0; i<leftNum; i++ ) {
		printf("%5d\n", left[i]);
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
	int difNum, difType,i;


	//Filter out spike. Make it straight
	for (i = 1; i <segNum; i++) {
		difNum = segment[i] - segment[i-1];
		if (difNum < 10) {
			segType[i] = 0;
		}
	}

	//Join all adjacent same segment types
	segmentF1[0] = segment[0];
	segTypeF1[0] = segType[0];
	for (i = 1; i < segNum; i++) {
		difType = segType[i] - segType[i - 1];
		if (difType != 0) {
			segNumF1++;
			segmentF1[segNumF1] = segment[i];
			segTypeF1[segNumF1] = segType[i];
		}
		else {
			segmentF1[segNumF1] = segment[i];
		}
	}
	segNumF1++;

	// Join short segment to previous segment
	segNumF2=0;
	segmentF2[0] = segmentF1[0];
	segTypeF2[0] = segTypeF1[0];
	for (i = 1; i <segNumF1; i++) {
		difNum = segmentF1[i] - segmentF1[i-1];
		if (difNum > 20) {
			segNumF2++;
			segmentF2[segNumF2] = segmentF1[i];
			segTypeF2[segNumF2] = segTypeF1[i];
		}
		else {
			segmentF2[segNumF2] = segmentF1[i];
		}
	}
	segNumF2++;

	//Join all adjacent same segment types
	segmentF3[0] = segmentF2[0];
	segTypeF3[0] = segTypeF2[0];
	for (i = 1; i < segNumF2; i++) {
		difType = segTypeF2[i] - segTypeF2[i - 1];
		if (difType != 0) {
			segNumF3++;
			segmentF3[segNumF3] = segmentF2[i];
			segTypeF3[segNumF3] = segTypeF2[i];
		}
		else {
			segmentF3[segNumF3] = segmentF2[i];
		}
	}
	segNumF3++;
}


//Filter based on left segment, update segment position
//void FilterLeft(){
//	int i,a;
//	if(segNumF3>segNumL)
//		a = segNumF3;
//	else
//		a = segNumL;
//	for (i = 0; i < a; i++) {
//		if(segTypeF3[i] != segTypeL[i]){
//
//		}
//		else{
//			segmentF4[segNumF4] = segmentF3[i];
//			segTypeF4[segNumF4] = segTypeF3[i];
//		}
//	}
//}

//Find segments based on left markers
void SegmentLeft(){
	int i, offset;
	int lNum=0;
	int num = 0;
	int sumoffset = 0;
	int aveoffset = 0;
	int cFlag;		//0:str, 1:+, -1:-ve
	cFlag = 0;
	segNumL = 0;

	for (i=0; i<logIndex; i++) {
		offset = logData[i];
		sumoffset +=offset;
		num++;
		if(left[lNum]==(i*5)){
			aveoffset = sumoffset/num;
			if(aveoffset>50){
				segmentL[segNumL] = i;
				segTypeL[segNumL] = 1;
				sumoffset = 0;
				num =0;
			}
			else if(aveoffset<-50){
				segmentL[segNumL] = i;
				segTypeL[segNumL] = -1;
				sumoffset = 0;
				num =0;
			}
			else{
				segmentL[segNumL] = i;
				segTypeL[segNumL] = 0;
				sumoffset = 0;
				num =0;
			}
		}
	}
	segmentL[segNumL] = i;
	segTypeL[segNumL] = 0;
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

void ExploreRun(){

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
			left[leftNum]=timeCount;
			leftNum++;
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
	int i;
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


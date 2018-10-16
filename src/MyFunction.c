#include "project.h"
#include "stdlib.h"
#include "MyFunction.h"
#include "Math.h"

//#define Fixed Speed Run
void FindSegments(void);
void FilterSegments(void);
void AnalyseCurve(void);


#define LOGSIZE	10000

#define SEGSIZE 300
int logData[LOGSIZE];
int t = 0;
int segment[SEGSIZE], segType[SEGSIZE], segNum;
int segmentF1[SEGSIZE], segTypeF1[SEGSIZE], segNumF1;
int segmentF2[SEGSIZE], segTypeF2[SEGSIZE], segNumF2;
int segmentF3[SEGSIZE], segTypeF3[SEGSIZE], segNumF3;
int segmentFL[SEGSIZE], segTypeFL[SEGSIZE], segNumFL;
int dis[SEGSIZE];
int rad[SEGSIZE];
int arcAngle[SEGSIZE];
int curveSpeed[SEGSIZE];
int logIndex;
bool logFlag = FALSE;
bool exploreFlag=FALSE;

unsigned pulseDuration[2];
unsigned aveSensorBlack[15];
int pulseBuzzerDuration = 0;

//for left marker
int left[200];
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


void pulseLED(int num, int duration);
void pulseBuzzer( int per, int duration);
void LMarkerDetect(void);
void RMarkerDetect(void);
void JMarkerDetect(void);
void ClearMarkerFlag(void);
void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc,int16_t dcc);

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
		printf(" %5d", logData[i++]);
		printf(" %5d", logData[i++]);
		printf(" %5d", logData[i++]);
	}
}


void PrintSegment() {
	int i;
	for (i=0; i< segNum; i++ ) {
		printf("%5d  %2d\n", segment[i], segType[i]);
	}
	printf("\n\n\n");
	for (i=0; i< segNumF1; i++ ) {
		printf("%5d  %2d\n", segmentF1[i], segTypeF1[i]);
	}
	printf("\n\n\n");
	for (i=0; i< segNumF2; i++ ) {
		printf("%5d  %2d\n", segmentF2[i], segTypeF2[i]);
	}
	printf("\n\n\n");
	for (i=0; i< segNumF3; i++ ) {
		printf("%5d  %2d\n ", segmentF3[i], segTypeF3[i]);
	}
	printf("\n\n\n");
	for (i=0; i< segNumFL; i++ ) {
		printf("%5d  %2d  %5d  %5d  %5d  %5d\n ", segmentFL[i], segTypeFL[i], arcAngle[i], rad[i], dis[i], curveSpeed[i]);
	}
//	printf("\n\n\n");
//	for (i=0; i<segNumFL; i++ ) {
//		printf("%5d  %2d\n ", segmentFL[i], segTypeFL[i]);
//	}
//	printf("\n\n\n");
//	for (i=0; i<leftNum; i++ ) {
//		printf("%5d\n ", left[i]);
//	}
}
void pulseLED(int num, int duration){
	bPulseFlag = TRUE;
	pulseDuration[num] = duration;
}
void pulseBuzzer( int period, int duration){
	TIM2->ARR = period;
    TIM2->CCR2 = TIM2->ARR/2;
	pulseBuzzerDuration = duration;
}

void ExploreRun(){
	timeCount = 0;
	DelaymSec(1000);
	EnWheelMotor();
	ClearMarkerFlag();
	logIndex = 0;
	logFlag = FALSE;
	char s[8];

	while(RSumMarker!=2)
	{
		if (RSumMarker==1)
			logFlag = TRUE;
		SetRobotSpeedX(1000);
		sprintf(s, "%4d", (int) (timeCount / 100));
		DispDotMatrix(s);
		if (bSWFlag ) {	// user switch break!
			break;
		}
	}
	logFlag = FALSE;
	MoveRobot(XSPEED, 400, 0, 1500, 0, 4000, 4000);
	StopRobot();
	FindSegments();
	exploreFlag=TRUE;
	//DelaymSec(1000);
}

#define thres 70
void FindSegments(void) {
	int i, offset;
	int cFlag;		//0:str, 1:+, -1:-ve
	cFlag = 0;
	segNum = 0;

	for (i=0; i<logIndex; i++) {
		offset = logData[i];

		if(offset > thres) {
			if(cFlag != 1 ){
				segment[segNum] = i;
				segType[segNum++] = cFlag;
				cFlag = 1;
			}
		}
		else if(offset < -thres) {
			if(cFlag != -1 ){
				segment[segNum] = i;
				segType[segNum++] = cFlag;
				cFlag = -1;
			}
		}
		else  {
			if((cFlag != 0)){
				if ( abs(offset<thres/2)) {
					segment[segNum] = i;
					segType[segNum++] = cFlag;
					cFlag = 0;
				}
			}
		}
	}
	segment[segNum] = i-1;
	segType[segNum++] = cFlag;
	FilterSegments();
}
void FilterSegments(void) {
	int difNum, difType, i;

	//Filter out spike. Make it straight

	for (i = 1; i < segNum; i++) {
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
		} else {
			segmentF1[segNumF1] = segment[i];
		}
	}
	segNumF1++;

	// Join short segment to previous segment
	segNumF2 = 0;
	segmentF2[0] = segmentF1[0];
	segTypeF2[0] = segTypeF1[0];
	for (i = 1; i < segNumF1; i++) {
		difNum = segmentF1[i] - segmentF1[i-1];
		if (difNum > 20) {
			segNumF2++;
			segmentF2[segNumF2] = segmentF1[i];
			segTypeF2[segNumF2] = segTypeF1[i];
		} else {
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


	//Filter Based on left marker
	segmentFL[0] = segmentF3[0];
	segTypeFL[0] = segTypeF3[0];
	for (i = 1; i < segNumF3; i++){
		segNumFL++;
		if(segmentF3[i] - segmentF3[i-1] > 4000){	//4000 to exclude this code
//			if(segmentF3[i] - segmentF3[i-1] > 400){
			int index,index2,sum,ave,num;
			bool leftFlag = FALSE;
			sum = ave = num =0;
			for(index = 0; index < leftNum; index++){
				if((left[index] > segmentF3[i-1])&&(left[index] < segmentF3[i])){
					//Assume only one left marker in one long seg
					//if more than one leftmarker in a long segment, may cause problem
					leftFlag = TRUE;
					segmentFL[segNumFL] = left[index];
					for(index2 = segmentF3[i-1]; index2 < segmentF3[i]; index2++){
						sum += logData[index2];
//						num++;
//						ave = sum/num;
					}
				}
			}
			if(leftFlag){
				if(sum >= 0){
					segTypeFL[segNumFL] = 1;
				}
				else{
					segTypeFL[segNumFL] = -1;
				}
				segNumFL++;
			}
		}
		segmentFL[segNumFL] = segmentF3[i];
		segTypeFL[segNumFL] = segTypeF3[i];
	}
	segNumFL++;

//FL
//	int index,sum,num,ave,a;
//	bool aFlag,bFlag;
//	segmentFL[0] = segmentF3[0];
//	segTypeFL[0] = segTypeF3[0];
//	for (i = 1; i < segNumF3; i++) {
//		segNumFL++;
//		if((segmentF3[i]-segmentF3[i-1])>400){
//			sum = 0;
//			num = 0;
//			ave = 0;
//			a = 0;
//			aFlag = FALSE;
//			bFlag = FALSE;
//			for(index=segmentF3[i-1]; index<(segmentF3[i]);index++){
//				sum += logData[index];
//				num++;
//				ave = sum/num;
//				if(ave > 50){
//					if(logData[index] < 20){
//						if(aFlag == FALSE){
//							a = index;
//							ave = sum = num =0;
//							aFlag = TRUE;
//							segNumFL++;
//						}
//						if(aFlag == TRUE && bFlag == TRUE){
//							a = index;
//							ave = sum = num =0;
//							bFlag = FALSE;
//							segNumFL++;
//						}
//
//					}
//					if(logData[index] > 70){
//						bFlag = TRUE;
//					}
//				}
//				else if(ave < -50){
//					if(logData[index] > -20){
//						if(aFlag == FALSE){
//							a = index;
//							ave = sum = num =0;
//							aFlag = TRUE;
//							segNumFL++;
//						}
//						if(aFlag == TRUE && bFlag == TRUE){
//							a = index;
//							ave = sum = num =0;
//							bFlag = FALSE;
//							segNumFL++;
//						}
//
//					}
//					if(logData[index] < -70){
//						bFlag = TRUE;
//					}
//				}
//
//			}
//			segmentFL[segNum] = a;
//			segTypeFL[segNum] = 1;
//
//		}
//		segmentFL[segNumFL]=segmentF3[i];
//		segTypeFL[segNumFL]=segTypeF3[i];
//	}
//	segNumFL++;


	AnalyseCurve();

}
void AnalyseCurve(void) {
	int i,arcLength,radian,angle;
	int w=1;
	int offset;
	long sum=0;
	//radian=arcength/angle to get arc--- adjust speed
	for (i = 0; i < segmentFL[0]; i++) {
			w=0;
			offset=logData[i];
			sum=sum+offset;
			if(i==segmentFL[0]){
				arcLength=segmentFL[0]*5;
				dis[w]=arcLength;
				angle=sum/13;
				arcAngle[w]=angle;
				radian=(int)(arcLength/(angle*3.142f/1800));
				rad[w]=radian;
//				angle = radian = 0; //if straight, clear angle and radius value
				w++;
				sum=0;
			}
	}

	for (i = segmentFL[0]; i < logIndex; i++) {

		offset=logData[i];
		if(i < segmentFL[w]){ //  <=
			sum=sum+offset;
		}
		else {
			sum=sum+offset;
			arcLength=(segmentFL[w]-segmentFL[w-1])*5;
			dis[w]=arcLength;
			angle=sum/14;
			arcAngle[w]=angle;
			radian=(int)arcLength/(angle*3.142f/1800);
			rad[w]=radian;
//			if(segTypeFL[w]==0){   //if straight, clear angle and radius value
//				angle = radian =0;
//			}
			w++;
			sum=0;
		}
	}
	for (i = 0; i < segNumFL; i++) {
		//curveSpeed[i]= (int) ((0.7f) * sqrt((abs(rad[i]) + 712) * acc));
		curveSpeed[i]= (int)(sqrt(fabs(rad[i])*10600.0f));
	}
	//finish analysing
//	pulseBuzzer(5000,50);
//	DispDotMatrix("OK");
}

void FastRun(void) {
	int i = 0;
	int endSpeed=2000;
	int acc, dec;
	char s[8];
	int constSpeed;
	DelaymSec(1000);
	EnWheelMotor();
	ClearMarkerFlag();

	MoveRobotCheck(XSPEED, 1000, 50, 1500, 1200, 2000, 2000, 1); //before first marker
	timeCount = 0;
	for (i = 0; i < segNumFL; i++) {
		if (segTypeFL[i] == 0) {
			if (i != (segNumFL-1) ) {
				constSpeed = curveSpeed[i+1];
			}
			else {
				constSpeed = endSpeed;
			}

			MoveRobotStraight(XSPEED, dis[i], 50+dis[i]/20, 3000, constSpeed, 5000, 8000);
		}
		else {
			int curveEndSpeed;
			acc=2000;
			dec=3000;
			//constSpeed=(int)((0.7f)*sqrt((abs(rad[i])+712)*acc));
			curveEndSpeed = curveSpeed[i];
			if (segTypeFL[i+1] != 0 ) {
				if (curveEndSpeed > curveSpeed[i+1])
					curveEndSpeed = curveSpeed[i+1];
			}
			MoveRobotCurve(XSPEED, dis[i], 50, curveSpeed[i], curveEndSpeed, acc, dec);
		}
	}
	sprintf(s, "%4d", (int) (timeCount / 100));
	DispDotMatrix(s);
	pulseBuzzer(5000,100);

	MoveRobotCheck(XSPEED, 1000, 0, endSpeed, endSpeed, 3000, 8000, 2);
	MoveRobot(XSPEED, 400, 0, endSpeed, 40, 3000, 8000);
	StopRobot();
	pulseBuzzer(2000,100);
	WaitSW();
}


#define y0 1450

#define a 250.0f
#define b 600.0f
void DumbRun(void){
	logIndex = 0;
	timeCount = 0;
	DelaymSec(1000);
	EnWheelMotor();
	SetRobotAccX(3000,9000);
	logIndex = 0;
	logFlag = FALSE;
	ClearMarkerFlag();
	int ty0=y0;
	//int tmpOffset, tmpSpeed;
	//tsensoroffset = sensoroffset;

	char s[8];
	while(RSumMarker!=2) {

//		tmpOffset = sensoroffset2;
//		tmpSpeed = curSpeed[0]/SPEED_mm_oc(1);
//		if(tmpSpeed<y0) tmpSpeed = y0;
//		tmpOffset = tmpOffset*(long)(tmpSpeed-600)/400;

		if(abs(sensoroffset) > abs(sensoroffset2))
			tsensoroffset = sensoroffset;
		else
			tsensoroffset = sensoroffset2;

		if(tsensoroffset<-a) tsensoroffset = -a;
		if(tsensoroffset>a)  tsensoroffset = a;

		sensoroffsetsqr = (long)tsensoroffset*tsensoroffset;
		xSpeed=(b/a)*sqrt(a*a-sensoroffsetsqr)+ty0;
		SetRobotSpeedX(xSpeed);

		if (bSWFlag ) {
			break;
		}
	}
	sprintf(s,"%4d", (int)(timeCount/100));
	DispDotMatrix(s);
	StopRobot();
	WaitSW();
}

void TestRun(void){
	timeCount = 0;
	DelaymSec(1000);
	EnWheelMotor();
	ClearMarkerFlag();
	char s[8];
	logIndex = 0;
	logFlag = FALSE;

	while(RSumMarker!=2)
	{
//		int i;
//		int n=7;
//		int disTest[]={330,400,175,130,480,515,360};
//		int endSpeed[]={1200,1200,1200,1200,1200,1200,50};
//
//		for(i=0;i<n;i++){
//			MoveRobot(XSPEED, disTest[i], 0, 1200, endSpeed[i], 3000,3000);
//			pulseBuzzer(2050,50);
//		}
//		break;

		SetRobotSpeedX(1000);

		// Do other stuff here!!!
		//printf("\ncurPos0=%-5d s=%5d", (int16_t)(curPos[0]/DIST_mm_oc(1)), curSpeed[0]);
		// like checking for sensors to detect object etc
		//gotoxy(5,5);
		//printf(" 2nd row %d   1st row  %d  tsen   %d  xspeed  %d mark %d  ",sensoroffset,sensoroffset2,tsensoroffset,xSpeed,RSumMarker);
		//gotoxy(5,10);
		//printf(" slowFlag:%d  fastFlag:%d  s1:%d  s2:%d  s3:%d   ",sl,f,sensorCal[0],sensorCal[1],sensorCal[2]);
		sprintf(s, "%4d", (int) (timeCount / 100));
		DispDotMatrix(s);
		if (bSWFlag ) {
			break;
		}
	}
	logFlag = FALSE;
	StopRobot();
	FindSegments();
	WaitSW();
}


//Marker detection
void LMarkerDetect(){
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
			left[leftNum] = timeCount/5;
			leftNum++;
			pulseLED(0,100);
			//pulseBuzzer(1000, 50);
		}
	}
}
void RMarkerDetect(){
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
void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc, int16_t dcc) {
	DelaymSec(1000);
	int i;
	bAlignFlag = FALSE;
	SetMoveCommand(speedType, dist, brakeDist,  topSpeed, endSpeed, acc,dcc);

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

void MoveRobotExplore(int16_t speedType, int16_t dist, int16_t brakeDist,int16_t topSpeed, int16_t endSpeed, int16_t acc) {
		char s[8];
		//change target speed then call SetRobotSpeedX() which inside libProfile
		while (RSumMarker != 2) {
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

			sprintf(s, "%4d", xSpeed);
			DispDotMatrix(s);
			if (bSWFlag) {	// user switch break!
				break;
			}
		}
}


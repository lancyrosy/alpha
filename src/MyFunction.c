#include "project.h"
#include "stdlib.h"
#include "MyFunction.h"
#include "Math.h"

//#define Fixed Speed Run
void FindSegments(void);
void FilterSegments(void);
void AnalyseCurve(void);
void AnalyseJunction(void);
#define LOGSIZE	12200
#define SEGSIZE 300
int logData[LOGSIZE];
int t = 0;
volatile uint16_t segment[SEGSIZE*2], segNum;
volatile uint16_t segmentF1[SEGSIZE*2], segNumF1;
volatile uint16_t segmentF2[SEGSIZE], segNumF2;
volatile uint16_t segmentF3[SEGSIZE], segNumF3;
volatile uint16_t segmentF4[SEGSIZE], segNumF4;
volatile uint16_t segmentFL[SEGSIZE], segNumFL;
volatile char segType[SEGSIZE*2];
volatile char segTypeF1[SEGSIZE*2];
volatile char segTypeF2[SEGSIZE];
volatile char segTypeF3[SEGSIZE];
volatile char segTypeF4[SEGSIZE];
volatile char segTypeFL[SEGSIZE];
uint16_t dis[SEGSIZE];
int16_t rad[SEGSIZE];
int16_t arcAngle[SEGSIZE];
uint16_t curveSpeed[SEGSIZE];
uint16_t junctionPos[SEGSIZE];
uint16_t logIndex;
int junction[100];
int JunctionTotal;
int Index=0;
int JIndex=0;
volatile bool fastFlag=FALSE;
volatile bool exploreFlag=FALSE;
volatile bool logFlag = FALSE;
volatile bool logFastFlag = FALSE;
volatile bool LMarkerFlag=FALSE;
volatile bool JMarkerFlag=FALSE;
volatile int LMarkerFlagPos;
volatile int JMarkerFlagPos;
volatile int fastModeX=1;
unsigned pulseDuration[2];
int pulseBuzzerDuration = 0;

unsigned int strEndSpeed;
unsigned int strTopSpeed;
//for left marker
volatile uint16_t  LeftMarker[300];
volatile int LeftNum = 0;
int leftm[50];
//for Junction marker
volatile uint16_t  JMarker[100];
volatile int JMarkerNum = 0;

int sensoroffsetsqr = 0;
int tsensoroffset = 0;
int xSpeed = 0;

volatile int16_t maxSpeed;
volatile int16_t minSpeed;
volatile int16_t maxRad;
volatile int16_t minRad;



volatile int RSumMarker,sumJunction;
volatile long disL,disR;
int LState,RState,JLState,JRState;


bool bPulseFlag = FALSE;
bool bJunFlag = FALSE;

void LMarkerDetect();
void RMarkerDetect();
void JMarkerDetect();
void CurveSpeed();


void LogData(int data) {
	if (logFlag==TRUE && logIndex<LOGSIZE) {
		logData[logIndex] = data;
		logIndex++;
		if (logIndex==LOGSIZE)
			logIndex=0;
	}
}
void PrintLog() {
	int i=0;
	logFlag=FALSE;
	if(logFastFlag == TRUE){
		printf("\n   \n");
		for (i=0; i<LOGSIZE; ) {
			printf("\n%5d   %5d   %5d   %5d   %5d",logData[i++],logData[i++],logData[i++],logData[i++],logData[i++]);
		}
		printf("\n %5d",logIndex);
	}
	else{
		printf("int logData[LOGSIZE]={");
		for (i=0; i<logIndex; ) {
			printf("\n%5d,", logData[i++]);
		}
		printf("};");
	}
}


void PrintSegment() {
	int i;
	for (i=0; i<= segNum; i++ ) {
		printf("%5d  %2d\n", segment[i], segType[i]);
	}
	printf("\n\n\n");
	for (i=0; i<= segNumF1; i++ ) {
		printf("%5d  %2d\n", segmentF1[i], segTypeF1[i]);
	}
	printf("\n\n\n");
	for (i=0; i<= segNumF2; i++ ) {
		printf("%5d  %2d\n", segmentF2[i], segTypeF2[i]);
	}
	printf("\n\n\n");
	for (i=0; i<= segNumF3; i++ ) {
		printf("%5d  %2d\n ", segmentF3[i], segTypeF3[i]);
	}
	printf("\n\n\n");
	for (i=0; i<= segNumF4; i++ ) {
		printf("%5d  %2d\n ", segmentF4[i], segTypeF4[i]);
	}
	printf("\n\n\n");
	for (i=0; i<LeftNum; i++ ) {
		printf("%5d\n ", LeftMarker[i]);
	}
	printf("\n\n\n");
	printf("          arcAngle  rad  dis  curveSpeed\n");
	for (i=0; i<= segNumFL; i++ ) {
		printf("%5d  %2d  %5d  %5d  %5d  %5d\n ", segmentFL[i], segTypeFL[i], arcAngle[i], rad[i], dis[i], curveSpeed[i]);
	}
	printf("\n\n\n");
	for (i=0; i<JunctionTotal; i++ ) {
		printf("%5d %5d\n ", JMarker[i],junction[i]);
	}

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
	fastFlag=FALSE;
	char s[8];

	while(RSumMarker!=2)
	{
		if (RSumMarker==1)
			logFlag = TRUE;
		SetRobotSpeedX(1000);
		if (bSWFlag ) {
			break;
		}
	}
	sprintf(s, "%4d", (int) (timeCount / 100));
	DispDotMatrix(s);
	logFlag = FALSE;
	MoveRobot(XSPEED, 400, 0, 1500, 0, 4000, 4000);
	StopRobot();
	JunctionTotal=sumJunction;				//store the sum of junction
	FindSegments();
	exploreFlag=TRUE;
	DelaymSec(1000);
	fastFlag=TRUE;
}

#define thres 60
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
				if(abs(offset)<thres/2){
				segment[segNum] = i;
				segType[segNum++] = cFlag;
				cFlag = 0;
				}
			}
		}
	}
	segment[segNum] = i-1;
	segType[segNum] = cFlag;
	FilterSegments();
}

void FilterSegments(void) {
	int difNum, difType, i, leftmNum;;

	//Filter out spike. Make it straight
	for (i = 1; i <= segNum; i++) {
		difNum = segment[i] - segment[i-1];
		if (difNum < 10) {
			segType[i] = 0;
		}
	}

	//make segments with junction straight
	segNumF1 = 0;
	segmentF1[0] = segment[0];
	segTypeF1[0] = segType[0];
	int index,index2,index3,sum,num,ave;
	index = index2 = index3 = 0;
	sum = num = ave = 0;
	for (i=1; i <= segNum; i++){
		if((JMarker[index]>=segment[i-1])&&(JMarker[index]<segment[i])){
			leftmNum = 0;
			for(index2 = 0; index2 < LeftNum; index2++){ //check left marker inside this segment
				if((LeftMarker[index2] > (segment[i-1]+20))&&(LeftMarker[index2] < JMarker[index])){
					leftm[leftmNum] = LeftMarker[index2];
					leftmNum++;
				}
			}
			if(leftmNum!=0){
				segmentF1[segNumF1] = leftm[leftmNum-1];
				for(index3=segment[i-1];index3<leftm[leftmNum-1];index3++){
					sum += logData[index3];
					num++;
					ave = sum/num;
				}
				if(ave>35){
					segmentF1[segNumF1] = leftm[leftmNum-1];
					segTypeF1[segNumF1] = 1;
				}
				else if(ave<-35){
					segmentF1[segNumF1] = leftm[leftmNum-1];
					segTypeF1[segNumF1] = -1;
				}
				else{
					segmentF1[segNumF1] = leftm[leftmNum-1];
					segTypeF1[segNumF1] = 0;
				}
			}
			segNumF1++;
			segmentF1[segNumF1] = segment[i];
			segTypeF1[segNumF1] = 0;
			index++;
			while((JMarker[index]>segment[i-1])&&(JMarker[index]<segment[i])){
				index++;
			}
		}
		else{
			segNumF1++;
			segmentF1[segNumF1] = segment[i];
			segTypeF1[segNumF1] = segType[i];
		}
	}

	//Join all adjacent same segment types (for straight only)
	segNumF2=0;
	segmentF2[0] = segmentF1[0];
	segTypeF2[0] = segTypeF1[0];
	for (i = 1; i <= segNumF1; i++) {
		if ((segTypeF1[i]==0)&&(segTypeF1[i-1]==0)) {
			segmentF2[segNumF2] = segmentF1[i];
		}
		else {
			segNumF2++;
			segmentF2[segNumF2] = segmentF1[i];
			segTypeF2[segNumF2] = segTypeF1[i];
		}
	}

	// Join short segments to previous segments (different segType)
	segNumF3 = 0;
	segmentF3[0] = segmentF2[0];
	segTypeF3[0] = segTypeF2[0];
	bool JFlag = FALSE;
	for (i = 1; i <= segNumF2; i++) {
		difNum = segmentF2[i] - segmentF2[i-1];
		difType = segTypeF2[i] - segTypeF2[i-1];
		if (difNum < 20){
			if(abs(difType) == 1){
				for(index=0; index<JMarkerNum;index++){
					if((JMarker[index]>=segmentF2[i-1])&&(JMarker[index]<segmentF2[i])){
						JFlag = TRUE;
					}
				}
				if(JFlag==TRUE){
					segNumF3++;
					segmentF3[segNumF3] = segmentF2[i];
					segTypeF3[segNumF3] = segTypeF2[i];
					JFlag = FALSE;
				}
				else{ // JFlag = FALSE
					segmentF3[segNumF3] = segmentF2[i];
				}
			}
			else{
				segNumF3++;
				segmentF3[segNumF3] = segmentF2[i];
				segTypeF3[segNumF3] = segTypeF2[i];
			}
		}
		else {
			segNumF3++;
			segmentF3[segNumF3] = segmentF2[i];
			segTypeF3[segNumF3] = segTypeF2[i];
		}
	}



	//Join all adjacent same segment types (based on left marker)
	segNumF4=0;
	segmentF4[0] = segmentF3[0];
	segTypeF4[0] = segTypeF3[0];
	for (i = 1; i <= segNumF3; i++) {
		difType = segTypeF3[i] - segTypeF3[i-1];
		if (difType == 0) {
			leftmNum = 0;
			for(index = 0; index < LeftNum; index++){ //check left marker inside this segment
				if((LeftMarker[index] > (segmentF3[i-1]-20))&&(LeftMarker[index] < (segmentF3[i-1]+20))){
					leftmNum++;
				}
			}
			if(leftmNum!=0){// left marker within segments
				segNumF4++;
				segmentF4[segNumF4] = segmentF3[i];
				segTypeF4[segNumF4] = segTypeF3[i];
			}
			else{
				segmentF4[segNumF4] = segmentF3[i];
			}
		}
		else {
			segNumF4++;
			segmentF4[segNumF4] = segmentF3[i];
			segTypeF4[segNumF4] = segTypeF3[i];
		}
	}


	//Filter Based on left marker, check long segment
	segNumFL=0;
	segmentFL[0] = segmentF4[0];
	segTypeFL[0] = segTypeF4[0];
	for (i = 1; i <= segNumF4; i++){
		segNumFL++;

		if(segmentF4[i] - segmentF4[i-1] > 250){ //if one segment more than 300(1500mm)
			int index,index2,index3,sum,num,ave;
			sum = ave = num = 0;
			leftmNum = 0;
			for(index = 0; index < LeftNum; index++){ //check left marker inside this segment
				if((LeftMarker[index] > (segmentF4[i-1]+20))&&(LeftMarker[index] < (segmentF4[i]-20))){
					leftm[leftmNum] = LeftMarker[index];
					leftmNum++;
				}
			}
			if(leftmNum != 0){
				for(index2=0;index2<leftmNum;index2++){
					if(index2==0){
						for(index3=segmentF4[i-1];index3<leftm[index2];index3++){
							sum += logData[index3];
							num++;
							ave = sum/num;
						}
					}
					else{
						for(index3=leftm[index2-1];index3<leftm[index2];index3++){
							sum += logData[index3];
							num++;
							ave = sum/num;
						}
					}
					if(ave>35){
						segmentFL[segNumFL] = leftm[index2];
						segTypeFL[segNumFL] = 1;
					}
					else if(ave<-35){
						segmentFL[segNumFL] = leftm[index2];
						segTypeFL[segNumFL] = -1;
					}
					else{
						segmentFL[segNumFL] = leftm[index2];
						segTypeFL[segNumFL] = 0;
					}
					sum = ave = num = 0;
					if(index2==(leftmNum-1)){
						for(index3=leftm[index2];index3<segmentF4[i];index3++){
							sum += logData[index3];
							num++;
							ave = sum/num;
						}
						if(ave>35){
							segTypeF4[i] = 1;
						}
						else if(ave<-35){
							segTypeF4[i] = -1;
						}
						else{
							segTypeF4[i] = 0;
						}
					}
					segNumFL++;
				}
			}
		}
		segmentFL[segNumFL] = segmentF4[i];
		segTypeFL[segNumFL] = segTypeF4[i];
	}


	AnalyseCurve();
	AnalyseJunction();
	CurveSpeed();
}

void AnalyseCurve(void) {
	int i,arcLength,radian,angle;
	int w=1;
	int offset;
	long sum=0;
	long calSum=0;
	//radian=arcength/angle to get arc--- adjust speed
	for (i = 0; i <=segmentFL[0]; i++) {
			w=0;
			offset=logData[i];
			sum=sum+offset;
			if(i==segmentFL[0]){
				arcLength=segmentFL[0]*5;
				dis[w]=arcLength;
				angle=sum/14;
				arcAngle[w]=angle;
				radian=(int)(arcLength/(angle*3.142f/1800));
				rad[w]=radian;

//				angle = radian = 0; //if straight, clear angle and radius value
				w++;
				sum=0;
			}
	}

	for (i = segmentFL[0]; i <logIndex; i++) {

		offset=logData[i];
		if(i<segmentFL[w]){
			sum=sum+offset;
			//calSum=calSum+offset;
		}
		else {
			sum=sum+offset;
			//calSum=calSum+offset;
			arcLength=(segmentFL[w]-segmentFL[w-1])*5;
			dis[w]=arcLength;
			angle=sum/14;
			//angle=calSum/14;
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
}

int accStr=9000, decStr=10000;
int accCur=2000, decCur=3000;
int maxCurSpeed=3000, minCurSpeed=1300;

void CurveSpeed(void){
	int i;
	double x,m;
	for (i = 0; i <= segNumFL; i++) {
	switch (fastModeX) {
		case 1:
			x=9000;
			accStr=8000;
			decStr=9000;
		    accCur=2000;
		    decCur=3000;
		    maxCurSpeed=2500;
		    minCurSpeed=1100;
		    strTopSpeed=3500;
			break;
		case 2:
			x=10500;
			accStr=8500;
			decStr=9500;
			accCur=2250;
			decCur=3250;
			maxCurSpeed=2750;
			minCurSpeed=1200;
			strTopSpeed=3750;
			break;
		case 3:
			x=12000;
			accStr=9000;
			decStr=10000;
			accCur=2500;
			decCur=3500;
			maxCurSpeed=2750;
			minCurSpeed=1300;
			strTopSpeed=4000;
			break;
		}

		m = x;

		//small curve
		if (dis[i] <= 200) {
			if (rad[i] <= 200) {
				m = x * (0.90f);
			}
			else {
				m = x * (0.95f);
			}
		}
		//medium curve
		else if ((dis[i] > 200) && (dis[i] <= 400)) {
			if (rad[i] <= 200) {
				m = x * (1.1f);
				maxCurSpeed=1650;
			}
			else if ((rad[i] > 200) && (rad[i] <= 300)) {
				m = x * (1.15f);
				maxCurSpeed=1800;
			}
			else {
				m = x * (1.2f);
			}
		}
		//big curve (dis>400)
		else if ((dis[i] > 400) && (dis[i] <= 600)) {
			if (rad[i] <= 200) {
				m = x * dis[i] * (0.0022f);
				maxCurSpeed=1650;
			} else if ((rad[i] > 200) && (rad[i] <= 300)) {
				m = x * dis[i] * (0.0025f);
				maxCurSpeed=1800;
			} else {
				m = x * dis[i] * (0.0030f);
			}
		} else if ((dis[i] > 600) && (dis[i] <= 800)) {
			if (rad[i] <= 200) {
				m = x * dis[i] * (0.0022f);
				maxCurSpeed=1650;
			} else if ((rad[i] > 200) && (rad[i] <= 300)) {
				m = x * dis[i] * (0.0026f);
				maxCurSpeed=1800;
			} else {
				m = x * dis[i] * (0.0031f);
			}
		}
		//big curve (dis>800)
		else {
			if (rad[i] <= 200) {
				m = x * dis[i] * (0.0022f);
				maxCurSpeed=1650;
			} else if ((rad[i] > 200) && (rad[i] <= 300)) {
				m = x * dis[i] * (0.0027f);
				maxCurSpeed=1800;
			} else {
				m = x * dis[i] * (0.003f);
			}
		}
		//m=dis[i]*(0.002f)*x;
		curveSpeed[i] = (int) (sqrt(fabs(rad[i]) * m));

		// Limit max/min speed
		if (curveSpeed[i] > maxCurSpeed)
			curveSpeed[i] = maxCurSpeed;
		if (curveSpeed[i] < minCurSpeed)
			curveSpeed[i] = minCurSpeed;

		//		int tRad = fabs(rad[i]);
		//		if (tRad<minRad) tRad = minRad;
		//		curveSpeed[i]= minSpeed + (int)((long)(sqrt(fabs(rad[i])-minRad))*(long)(maxSpeed-minSpeed)/sqrt(maxRad-minRad));

	}
	int tCurveSpeed = 0;
	for (i = segNumFL; i > 0; i--) {
		if ((segTypeFL[i] != 0) && (segTypeFL[i - 1] != 0)) {
			tCurveSpeed = (int) (sqrt(
					2.0f * dis[i - 1] * decCur
							+ (long) curveSpeed[i] * (long) curveSpeed[i]));
			if (tCurveSpeed < curveSpeed[i - 1])
				curveSpeed[i - 1] = tCurveSpeed;
		}
	}


}
void AnalyseJunction(void){
	int index,i;
	int numJunction=0;
	unsigned int startSeg = 0;
	for (index = 0; index <= segNumFL; index++) {
		for (i = numJunction;i < JunctionTotal;i++) { // Check junctions
			if ((JMarker[i] >= startSeg) && (JMarker[i] < segmentFL[index])) {
				junction[i] = index;	 //store junction segment index
			}
			else
				break;
		}
		startSeg = segmentFL[index];
		numJunction=i;
	}
	junction[numJunction] = -1;
}

int SegmentNum=0;
int endSpeed = 2500;
void FastRun(void) {
	int i = 0;
	char s[8];
	timeCount = 0;
	JIndex = 0;
	logIndex = 0;
	DelaymSec(1000);
	EnWheelMotor();
	ClearMarkerFlag();
	fastFlag=TRUE;
	CurveSpeed();
	MoveRobotCheck(XSPEED, 1000, 50, 1500, 1200, 2000, 2000, 1); //before first marker
	if(RSumMarker == 1){
		logFlag = TRUE;
	}
	for (i = 0; i <= segNumFL; i++) {
		SegmentNum=i;
		if (segTypeFL[i] == 0) {		//Straight
			if (i != segNumFL){					//Not last segment
				strEndSpeed = curveSpeed[i+1];
			}
			else {								//Last segment
				strEndSpeed = endSpeed;
			}
			MoveRobotStraight(XSPEED, dis[i], 20+dis[i]/25, strTopSpeed, strEndSpeed, accStr, decStr, 2, SegmentNum);
			//pulseBuzzer(250,50);
		}
		else {							//Curve
			int curveEndSpeed;
			curveEndSpeed = curveSpeed[i];
			if (segTypeFL[i+1] != 0 ) {			//Next segment is curve(Curve-Curve)
				if (curveEndSpeed > curveSpeed[i+1])
					curveEndSpeed = curveSpeed[i+1];
			}
			else{								//Next segment is straight (Curve-Straight)
				accCur=4000;
			}
			MoveRobotCurve(XSPEED, dis[i], 50, curveSpeed[i], curveEndSpeed, accCur, decCur, SegmentNum);
			//pulseBuzzer(250,50);
		}
	}
	logFlag = FALSE;
	sprintf(s, "%4d", (int) (timeCount / 100));
	DispDotMatrix(s);
	pulseBuzzer(5000,100);

	MoveRobot(XSPEED, 400, 0, endSpeed, 40, 3000, 8000);  // enter start-finish area
	StopRobot();
	pulseBuzzer(2000,100);

	WaitSW();
}


#define y0 1450
#define a 250.0f
#define b 600.0f
void DumbRun(void){
	timeCount = 0;
	DelaymSec(1000);
	EnWheelMotor();
	SetRobotAccX(4000,10000);
	ClearMarkerFlag();
	int ty0=y0;

	char s[8];
	while(RSumMarker!=2) {

		if(abs(sensoroffset) > abs(sensoroffset2))  //entering the curve
			tsensoroffset = abs(sensoroffset);
		else										// straight
			tsensoroffset = abs(sensoroffset2);
		tsensoroffset -= 100;
		if (tsensoroffset<0) tsensoroffset = 0;
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
	//Run as constant speed
	timeCount = 0;
	DelaymSec(1000);
	EnWheelMotor();
	ClearMarkerFlag();
	char s[8];

	while(RSumMarker!=2)
	{
		MoveRobot(XSPEED, 3500, 20+3500/25, 3000, 0, 11000, 11000);
		if (bSWFlag ) {
			break;
		}
	}
	sprintf(s, "%4d", (int) (timeCount / 100));
	DispDotMatrix(s);
	StopRobot();
	WaitSW();
}


#define LEFT_SEN	13
#define RIGHT_SEN	14
//Marker detection
void LMarkerDetect(){
	if (sensorCal[LEFT_SEN] >= 400) {
		LState = 1;
		JLState = 0;
	}
	if (sensorCal[LEFT_SEN] <= 300 && LState == 1){
		LState = 0;
		JLState = 1;
		disL = curPosTotal[0]/DIST_mm_oc(1);
		JMarkerDetect();
	}
	if (JLState==1) {
		uint16_t tDist = curPosTotal[0]/DIST_mm_oc(1);
		if ((tDist-disL)>30) {
			JLState = 0;
			LMarkerFlagPos=curPos[0]/DIST_mm_oc(1);
			LMarkerFlag=TRUE;
			JMarkerFlag = FALSE;
			if(fastFlag==FALSE){
				LeftMarker[LeftNum] = (timeCount-30-60)/5;
				LeftNum++;
				//pulseBuzzer(1000, 50);
			}
			pulseLED(0,100);

		}
	}
}
void RMarkerDetect(){
	if (sensorCal[RIGHT_SEN] >= 400){
		RState = 1;
		JRState = 0;
	}
	if (sensorCal[RIGHT_SEN] <= 300 && RState == 1){
		RState = 0;
		JRState = 1;
		disR = curPosTotal[0]/DIST_mm_oc(1);
		JMarkerDetect();
	}
	if (JRState==1) {
		uint16_t tDist = curPosTotal[0]/DIST_mm_oc(1);
		if ((tDist-disR)>30) {
			JRState = 0;
			JMarkerFlag = FALSE;
			RSumMarker ++;
			pulseLED(1,100);
			pulseBuzzer(4000, 50);
		}
	}
}
void JMarkerDetect(){
	if ((JLState == 1) && (JRState == 1)) {
		if (abs(disL - disR) < 40) {
			sumJunction++;
			JMarkerFlag=TRUE;
			if(fastFlag==FALSE){
				//JMarkerFlagPos=curPos[0]/DIST_mm_oc(1);
				JMarker[JMarkerNum] = (timeCount-60)/5;
				JMarkerNum++;
				pulseBuzzer(2500, 50);
			}
		}
		JLState=JRState=0;
	}
}
void ClearMarkerFlag(){
	JRState=JLState=0;
	JMarkerFlag = 0;
	LState=RState=0;
	timeCount = 0;
	RSumMarker=sumJunction=JMarkerNum=0;
}

//Collect black value
void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc, int16_t dcc) {

	bAlignFlag = FALSE;
	DelaymSec(1000);
	int i;
	for(i=0; i<15; i++){
		sensorCalMax[i] = 1000;
		sensorBlack[i] = 1000;
	}

	SetMoveCommand(speedType, dist, brakeDist,  topSpeed, endSpeed, acc,dcc);

	while(!EndOfMove(speedType)) {

		while(!bMotorISRFlag);
		bMotorISRFlag=FALSE;
		DispDotMatrix("Black");

		for (i=0; i<15; i++) {
			if(sensor[i] > sensorCalMax[i])
				sensorCalMax[i] = sensor[i];//-(sensor[i]-sensorCalMax[i])/10;
			if(sensor[i] < sensorBlack[i])
				sensorBlack[i] = sensor[i];//+(sensorBlack[i]-sensor[i])/10;;
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

void PrintBlackValue(){
	int i;
	clrscr();
	printf("\n\nSensorCalMax value:\n");
	for(i=0; i<15; i++){
		printf("S %2u   %4u\n",i+1,sensorCalMax[i]);
	}
	printf("\n\nSensorBlack value:\n");
	for(i=0; i<15; i++){
		printf("S %2u   %4u\n",i+1,sensorBlack[i]);
	}
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


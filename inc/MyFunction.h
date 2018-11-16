extern unsigned pulseDuration[2];
extern int pulseBuzzerDuration;
extern int xSpeed;
extern int tsensoroffset;
unsigned int constSpeed;
extern volatile int RSumMarker;
extern volatile bool exploreFlag;
extern volatile bool fastFlag;
extern volatile bool logFlag;
extern volatile bool logFastFlag;
extern volatile bool LMarkerFlag;
extern volatile bool JMarkerFlag;
extern volatile int LMarkerFlagPos;
extern volatile int JMarkerFlagPos;
extern volatile uint16_t  LeftMarker[300];
extern volatile uint16_t JMarker[100];
extern int MarkerNum;
extern int JunctionTotal;
extern int JIndex;
extern int Index;
extern int junction[100];
extern int SegmentNum;

#define SEGSIZE 300
extern volatile uint16_t segment[SEGSIZE], segNum;
extern volatile uint16_t segmentF1[SEGSIZE], segNumF1;
extern volatile uint16_t segmentF2[SEGSIZE], segNumF2;
extern volatile uint16_t segmentF3[SEGSIZE], segNumF3;
extern volatile uint16_t segmentFL[SEGSIZE], segNumFL;
extern volatile int16_t segType[SEGSIZE];
extern volatile int16_t segTypeF1[SEGSIZE];
extern volatile int16_t segTypeF2[SEGSIZE];
extern volatile int16_t segTypeF3[SEGSIZE];
extern volatile int16_t segTypeFL[SEGSIZE];



void pulseBuzzer( int per, int duration);
void pulseLED(int num, int duration);
void LMarkerDetect();
void RMarkerDetect();
void ClearMarkerFlag();
int16_t Cen1();
void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc,int16_t dcc);
void MoveRobotExplore(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc);
void LogData(int data);
void PrintLog();
void PrintSegment();
void TestRun();
void DumbRun();
void ExploreRun();
void FastRun();
void PrintBlackValue();

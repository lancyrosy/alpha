extern unsigned pulseDuration[2];
extern int pulseBuzzerDuration;
extern int xSpeed;
extern int tsensoroffset;
unsigned int constSpeed;
extern volatile int RSumMarker;
extern bool logFlag;
extern bool exploreFlag;
extern volatile bool LMarkerFlag;
extern volatile bool JMarkerFlag;
extern volatile int LMarkerFlagPos;
extern volatile int JMarkerFlagPos;
extern volatile int LeftMarker[300];
extern volatile int JMarker[100];
extern volatile int MarkerNum;
extern volatile int JunctionTotal;
extern volatile int JIndex;
extern volatile int Index;
extern volatile int junction[100];

#define SEGSIZE 300
extern volatile int segment[SEGSIZE], segType[SEGSIZE], segNum;
extern volatile int segmentF1[SEGSIZE], segTypeF1[SEGSIZE], segNumF1;
extern volatile int segmentF2[SEGSIZE], segTypeF2[SEGSIZE], segNumF2;
extern volatile int segmentF3[SEGSIZE], segTypeF3[SEGSIZE], segNumF3;
extern volatile int segmentFL[SEGSIZE], segTypeFL[SEGSIZE], segNumFL;

#define L_MARKER_SEN sensorBlack[13]
#define R_MARKER_SEN sensorBlack[14]

void pulseBuzzer( int per, int duration);
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

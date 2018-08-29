extern unsigned pulseDuration[2];
extern int pulseBuzzerDuration;
extern int xSpeed;
extern int sl,f;
extern int tsensoroffset;
extern volatile int RSumMarker;
extern bool logFlag;


#define L_MARKER_SEN sensorBlack[13]
#define R_MARKER_SEN sensorBlack[14]

void LMarketDetect();
void RMarketDetect();
void ClearMarkerFlag();
int16_t Cen1();
void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc);
void MoveRobotExplore(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc);
void LogData(int data);
void StartLog();
void PrintLog();
void PrintSegment();
void LogOffset(int data);
void TestRun();
void DumbRun();
void ExploreRun();
void FastRun();

extern unsigned pulseDuration[2];
extern int pulseBuzzerDuration;


#define L_MARKER_SEN sensorBlack[13]
#define R_MARKER_SEN sensorBlack[14]

void LMarketDetect();
void RMarketDetect();
void ClearMarkerFlag();
void MoveRobotCalibrate(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc);
int16_t Cen1();
void MoveRobotExplore(int16_t speedType, int16_t dist, int16_t brakeDist, int16_t topSpeed, int16_t endSpeed, int16_t acc);

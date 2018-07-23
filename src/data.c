////////////////////////////////////////////////////////////////////////////////////
// file : libData.c
// name : nbk
// desc : Global variable files
//
// The global variables here are required in many library routines. For convenience sake, they
// are grouped here so that it will minimise the number of files you need to include.
////////////////////////////////////////////////////////////////////////////////////
#include "project.h"


int debugData0, debugData1;
int debugData2, debugData3;
volatile bool bSWFlag;
int beepCnt;


volatile bool bEncoderClickFlag;
int16_t encoderClickType;

int specialVariable1=2, specialVariable2=2;
void InitDataCold()    /* Cold start global data initialisation     */
{


}
void InitDataWarm()    /* Warm start (after crash) global data init */
{

}


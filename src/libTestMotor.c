// ---------------------------------------------------------------------------------
// libTestMotor.c
// Date : 3/04/2007
// Name : nbk
// Desc : PrintMotorMenu() is for testing the various motors and speed profile functions.
// ---------------------------------------------------------------------------------
#include "project.h"
#include <ctype.h>


void PrintMotorMenu();

void PrintMotorMenu() {
	clrscr();
    printf("\n ? - Print this menu");
    printf("\n 1 - Drive motor %s",(bWheelMotorEnable)?"enabled":"Disable");
    printf("\n 2 - fwd Straighht run");
    printf("\n 3 - RWD Straight  run");
    printf("\n 4 - Turn on spot");
    printf("\n 5 - Move forward 20cm");
    printf("\n 6 - Move backward 20cm");
    printf("\n 7 - Move 90 deg CCW");
    printf("\n 8 - Move 90 deg CW");
    printf("\n 9 - Move forward, curve turn and forward");
}

#define START_ROW		17
void PrintMotorStatus() {
	    gotoxy(12,START_ROW);
		printf("encoder0=%-6d  1=%-6d",
		oldEncoderData[0], oldEncoderData[1]);
		
	    gotoxy(12,START_ROW+2);
		printf("posErr0=%-6d 1=%-6d ", posErr[0], posErr[1]);
		
	    gotoxy(12,START_ROW+4);
		printf("encDiff0=%-6d  1=%-6d ", encoderSpeed[0], encoderSpeed[1]);

	    gotoxy(12,START_ROW+6);
		printf("WheelPWM0=%-6d  1=%-6d ", wheelPWM[0], wheelPWM[1]);
		
	    gotoxy(12,START_ROW+8);
		printf("curPos0=%-6d  1=%-6d ", curPos[0]/DIST_mm_oc(1), curPos[1]);

	    gotoxy(12,START_ROW+10);
		printf("targetPos0=%-6d  1=%-6d ", finalPos[0], finalPos[1]);

		gotoxy(12,START_ROW+12);
		printf("sensoroffset=%4d",sensoroffset);
}

// Menu for testing various speed profile functions
void TestMotorMenu() {           
	char c;        
	
	EnWheelMotor();
	DelaymSec(5);
	SetRobotSpeedX(0);
	SetRobotSpeedW(0);
	
	PrintMotorMenu();
	while(1)  {                                                    

		if (bSWFlag) {
			bSWFlag = FALSE;
			return;
		}
		if (kbhit()) {

			c = toupper(GetChar());

			switch(c) {
			case  '1':
					if (bWheelMotorEnable)
						DisWheelMotor();
					else
						EnWheelMotor();
					PrintMotorMenu();
					break;                            
			case '2':               
					// No auto speed profile
					SetRobotAccX(500);
					SetRobotSpeedX(500);
					DelaymSec(500);                                                
					SetRobotSpeedX(0);
					DelaymSec(500);                                                
					break;
			case '3':               
					// No auto speed profile
					SetRobotSpeedX(-500);
					DelaymSec(500);                                                
					SetRobotSpeedX(0);
					DelaymSec(500);                                                
					break;
			case '4':               
					// No auto speed profile
					SetRobotAccW(1000);
					SetRobotSpeedW(360);
					DelaymSec(500);                                                
					SetRobotSpeedW(0);
					DelaymSec(500);                                                
					break;
			case '5':     
					// Auto speed profile
					// Robot will travel a designated distance using the
					// speed and acceleration parameters given.
					MoveRobot(XSPEED, 200, 0, 400, 0, 1000);
					StopRobot();       
					break;
			case '6':     
					MoveRobot(XSPEED, -100, 0, 500, 0, 1000);
					StopRobot();       
					break;
			case '7':     
					MoveRobot(WSPEED, 90, 0, 360, 0, 1800);
					StopRobot();       
					break;
			case '8':     
					MoveRobot(WSPEED, -90, 0, 360, 0, 1800);
					StopRobot();       
					break;
			case '9':                                                    
					// Move 2cm
					MoveRobot(XSPEED, 30, 0, 300, 300, 500);
					// Curve turn 90 degrees
					MoveRobot(WSPEED, -45, 0, 1800, 10, 1800);
					// Move 2cm
					MoveRobot(XSPEED, 30, 0, 300, 0, 500);
					StopRobot();       
					break;
			}
		}
	PrintMotorStatus();

	}
}

sPos positions[4] = {
	{500,500,0},	// direction is not important
	{1000,0,0},
	{0,0,0}
};

void TestGotoPos() {
	int i;
	char s[20];
		
	// Initialise robot position.
	robotPos.x=robotPos.y=robotPos.dir=0;
	EnWheelMotor();
	DelaymSec(200);
	
	for (i=0; i<3; i++) {
//		GotoPos(&positions[i]);
		sprintf(s,"x%d,y%d,w%d", robotPos.x, robotPos.y,robotPos.dir/10);
		DispDotMatrixWait(s);
	}
						
	DisWheelMotor();
}

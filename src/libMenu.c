// ---------------------------------------------------------------------------------
// libMenu.c
// Created on: 30-Jan-2010
// Author: nbk
// Desc:
// MainMenu() is the menu entry function. It is use to call the other submenus
// ---------------------------------------------------------------------------------
#include "project.h"
#define DEBUGno
#include "libdebug.h"

/* macros for mouse menu */
#define MENU_SIZE				(int)(sizeof(menuStrg)/sizeof(*menuStrg))

#define MENU_SPECIAL			127
#define MENU_SPECIAL2			126
#define MENU_EXIT               0
#define MENU_MAIN			  	0

void MenuTest();
void MenuRun();
void MenuMotor();
void MenuSensor();
void MyTest();


int16_t specialVariable1;

char s[80];
uint8_t menuNum;
// @brief : main menu function of program
// @param : none
// @retval: none
void MainMenu(){
	static const char * const menuStrg[] = {
			"RUN",
			"TEST",

	};
	int8_t itemNum=0, selectedItem;

	DispDotMatrixWait("AlphaCentre RobotKit-"__DATE__);
	DelaymSec(200);
	while(1){
		char a[10];
		sprintf(a,"%4u",ReadBatteryVolt() );
		DispDotMatrix(a);
		if(bSWFlag)
			break;
	}

	while(1) {
		DisableSensor();
		menuNum = MENU_MAIN;
	    selectedItem=SelectMenuItem(&itemNum, MENU_SIZE, menuStrg);

			switch(selectedItem) {
	     	case 0:
				MenuRun();
				break;
			case 1:
				MenuTest();
				break;
	     	}
	}
}

void MenuRun(){
	int8_t itemNum=1, selectedItem;
	static const char *const menuStrg[] = {
				"----",
				"expR",
				"fR1",
				"fR2",
				"fR3",
				"dumR",
				"tesR",
				"pLog",
				"pSeg"
	};
	while(1) {
		menuNum = 1;
		selectedItem = SelectMenuItem(&itemNum, MENU_SIZE, menuStrg);

		switch (selectedItem) {
		case 1:
			EnableSensor();
			EnWheelMotor();
			ExploreRun();
			DisableSensor();
			DisWheelMotor();
			break;
		case 2:
			EnableSensor();
			EnWheelMotor();
			if (exploreFlag == TRUE) {
				fastModeX = 1;
				FastRun();
			}
			DisableSensor();
			DisWheelMotor();
			break;
		case 3:
			EnableSensor();
			EnWheelMotor();
			if (exploreFlag == TRUE) {
				fastModeX = 2;
				FastRun();
			}
			DisableSensor();
			DisWheelMotor();
			break;
		case 4:
			EnableSensor();
			EnWheelMotor();
			if (exploreFlag == TRUE) {
				fastModeX = 3;
				FastRun();
			}
			DisableSensor();
			DisWheelMotor();
			break;
		case 5:
			EnableSensor();
			EnWheelMotor();
			DumbRun();
			DisableSensor();
			DisWheelMotor();
			break;
		case 6:
			EnableSensor();
			EnWheelMotor();
			TestRun();
			DisableSensor();
			DisWheelMotor();
			break;
		case 7:
			PrintLog();
			break;
		case 8:
			PrintSegment();
			break;
		case MENU_EXIT:
			return;
		}
	}
}
void MenuTest(){
	int8_t itemNum=1, selectedItem;

	const const static char *const menuStrg[] = {
				"----",
				"Sensor",
				"Motor",
				"MyTest"
	};

	while(1) {
		DisableSensor();
		menuNum = MENU_MAIN;
	    selectedItem=SelectMenuItem(&itemNum, MENU_SIZE, menuStrg);

		switch(selectedItem) {
		case 1:
			EnableSensor();
			MenuSensor(); //sensor
			DisableSensor();
			break;
		case 2:
			EnWheelMotor();
			MenuMotor();  //motor
			DisWheelMotor();
			break;
		case 3:
			EnableSensor();
			MyTest();    //MyTest
			DisableSensor();
			break;
	    case MENU_EXIT:
			return;
     	}
	}
}

void MenuMotor(){
	int8_t itemNum=1, selectedItem;
	const const static char *const menuStrg[] = {
				"----",
				"DriveMotor",
				"ServoMotor",
				"Move 2.0m",
				"T360",
				"Variable"

	};


	while(1) {

	     selectedItem=SelectMenuItem(&itemNum, MENU_SIZE, menuStrg);

		switch(selectedItem) {
        case MENU_EXIT:
			return;
		case 1:
			TestMotorMenu();
			break;
     	case 2:
			 //MenuServoMotor();
	          break;

		case 3:
			fastFlag = logFlag = 1;
			MoveRobot(XSPEED, 500, 0, 1300, 1300, 2000,3000);
			MoveRobot(XSPEED, 3600, 0, 3300, 0, 9000,9000);
			StopRobot();
			fastFlag = logFlag = 0;

			break;
     	case 4:
			MoveRobot(WSPEED, 360, 0, 360, 0, 1000,1000);
			//StopRobot();
			DelaymSec(200);
	          break;
		case 5:
			MenuChangeVariable(&specialVariable1);
			break;
     	}
	}
}
void MenuSensor(){
	int8_t sensorNum=1;
	static const char *const menuStrg[] = {
				"----",
				"All ",
				"sOff",
				"sen1",
				"sen2",
				"sen3",
				"sen4",
				"sen5",

	};
	while(1) {
	    switch(SelectMenuItem(&sensorNum, MENU_SIZE, menuStrg)) {
		case 1:
			DispAllSensorValues();
			break;
		case 2:
			while(!bSWFlag) {
				sprintf(s,"%4d", sensoroffsetX2);
				DispDotMatrix(s);
			}
			break;

		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
			DisplaySensorOnDotMatrix(sensorNum-1);
			break;


        case MENU_EXIT:
        case MENU_SPECIAL:
			return;
		}
     }
}
void MyTest(){
	int8_t sensorNum=1;
		static const char *const menuStrg[]={
				"----",
				"TestR",
				"PLog",
				"pSeg",
				"ColBlack",
				"pBlack"
		};
		while(1) {
		    switch(SelectMenuItem(&sensorNum, MENU_SIZE, menuStrg)) {
			case 1:
				EnableSensor();
				EnWheelMotor();
				TestRun();
				DisableSensor();
				DisWheelMotor();
				break;
			case 2:
				PrintLog();
				break;
			case 3:
				PrintSegment();
				break;
			case 4:
				EnableSensor();
				EnWheelMotor();
				MoveRobotCalibrate(XSPEED, 300, 0, 200, 0, 1000,1000);
				DisableSensor();
				DisWheelMotor();
				break;
			case 5:
				PrintBlackValue();
				break;
			case MENU_EXIT:
				return;
		    }
		}
}

// ---------------------------------------------------------------------------------
// Function for display menu number and reading the number selected.
// Display is on 7 seg. Reading is from interrupt s/w. A short key press (<0.5sec)
// denotes scrolling of menu items. A long key press (>1 sec) select
// that item. For consistency, '0' should be the exit-menu number
// ---------------------------------------------------------------------------------
char SelectMenuItem(int8_t *startNum, int8_t numOfItem, const char *const *menuStrg) {
	int8_t itemNum = *startNum;

	numOfItem--;
	if (itemNum>numOfItem)
		itemNum = 0;

	bBlinkFlag = TRUE;

	while(1) {

		switch(DispDotMatrixWait(menuStrg[itemNum]) ) {
		case 1:
			DispDotMatrix(menuStrg[itemNum]);
			*startNum = itemNum;
			bBlinkFlag = FALSE;
			return itemNum;
		case 0:
			itemNum++;
			if (itemNum>numOfItem)
				itemNum = 0;
			break;
		case 2:
			itemNum--;
			if (itemNum<0 )
				itemNum = numOfItem;
			break;
		case 3:
			*startNum = itemNum;
			bBlinkFlag = FALSE;
			return MENU_SPECIAL;
		case 4:
			*startNum = itemNum;
			bBlinkFlag = FALSE;
			return MENU_SPECIAL2;
		}
	}
}

void MenuChangeVariable(int16_t *variable) {
	char flag;
	flag = TRUE;
	bUseEncoderClickFlag=TRUE;
	while(flag) {
		char s[8];
		sprintf(s, "%4d", *variable);
		switch (DispDotMatrixWait(s)){
		case 0:
			*variable+=1;
			break;
		case 1:
			*variable-=1;
			break;
		case 2:
			flag = FALSE;
			break;
		}
	}
	bUseEncoderClickFlag=FALSE;
}

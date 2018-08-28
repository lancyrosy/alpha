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
			"RUN ",
			"Test Menu",

	};
	int8_t itemNum=0, selectedItem;


	DispDotMatrixWait("AlphaCentre RobotKit-"__DATE__);


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
			MenuSensor();
			DisableSensor();
			break;
		case 2:
			EnWheelMotor();
			MenuMotor();
			DisWheelMotor();
			break;
		case 3:

			MyTest();

			break;
	    case MENU_EXIT:
			return;
     	}
	}
}


void MenuRun(){
	int8_t itemNum=1, selectedItem;
	static const char *const menuStrg[] = {
				"----",
				"explore",
				"fastR"
	};


	while(1) {
		menuNum = 1;
	     selectedItem=SelectMenuItem(&itemNum, MENU_SIZE, menuStrg);

		switch(selectedItem) {
		case 1:
			ExploreRun();
			break;
		case 2:
			FastRun();
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
			MoveRobot(XSPEED, 2000, 0, 100, 0, 2000);

			//StopRobot();
			DelaymSec(200);

			break;
     	case 4:
			MoveRobot(WSPEED, 360, 0, 360, 0, 720 );
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
				"sen1",
				"sen2",
				"sen3",
				"sen4",
				"sen5",
				"L1  ",
				"L2  ",
				"ADC8",
				"ADC9",
				"AlignValue",
				"Compass",
				"Sharp",		// sharp sensor
				"CAM3",
				"0color",
				"1color",
				"2color",
	};
	while(1) {
	    switch(SelectMenuItem(&sensorNum, MENU_SIZE, menuStrg)) {
		case 1:
			DispAllSensorValues();
			break;

		case 2:
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
		case 11 :
			bAlignLineFlag = TRUE;
			DispDotMatrixInt16((int16_t*)&alignSpeed);
			bAlignLineFlag = FALSE;
			break;

		case 12:
			  //PrintCompass();
 	          break;
		case 13:
			while(1) {
//				unsigned num= GetSharpGP2D12Dist(senADC9);
//				sprintf(s,"%4d", num);
				DispDotMatrix(s);
				DelaymSec(50);

				if (bSWFlag) {
					bSWFlag=FALSE;
					break;
				}
			}
			 break;
		case 14:
//			PrintCamValues();
			break;
		case 15:
//			MenuDispCAM3Value(0);
			break;
		case 16:
//			MenuDispCAM3Value(1);
			break;
		case 17:
//			MenuDispCAM3Value(2);
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
				"TestRun",
				"PrintLog",
				"CollectBlack",
				"DisplaySensor",
				"Battery"
		};
		while(1) {
		    switch(SelectMenuItem(&sensorNum, MENU_SIZE, menuStrg)) {
			case 1:
				EnableSensor();
				EnWheelMotor();
				TestRun();
				//MoveRobotExplore( 0,  5000,  200,  1000,  50,  5000);
				DisableSensor();
				DisWheelMotor();
				break;
			case 2:
				PrintLog();
				break;
			case 3:
				EnableSensor();
				EnWheelMotor();
				MoveRobotCalibrate(XSPEED, 200, 0, 200, 0, 1000);
				DisableSensor();
				DisWheelMotor();
				break;
			case 4:
				DispAllSensorValues();
				break;
			case 5:
				while(1){
					char a[10];
					sprintf(a,"%4u",ReadBatteryVolt() );
					DispDotMatrix(a);
					if(bSWFlag)
						break;
				}
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

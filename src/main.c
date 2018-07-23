// ---------------------------------------------------------------------------------
//   File        :main.c
//   Version     :v1.0.1
//   Date        :06Feb2011
//   Author      :NgBK
//   Description :main()
// ---------------------------------------------------------------------------------

//......................................................................................
// To do list (reminder on what to do in this project)
// test input port pins
// buzzer
// flash data
//......................................................................................

#include "project.h"

#define mydebugNo
#ifdef mydebug
	DispDotMatrix("debug");
#endif


int main(void){

	InitIO();	// Must be initialised before any printf()

	clrscr();


   	puts("I'm alive! abcde");

   	InitDataCold();

	InitDataWarm();


	EnWheelMotor();
	//DispAllSensorValues();
	//ReadDataFromFlash();
	//StartCMUCAM(4, 57600);
	//PrintCamValues();
	//CAM_to_WinM3();
	//TestMotorMenu();


	MainMenu();

	return 0;
}

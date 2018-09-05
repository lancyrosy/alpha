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
	int pwm=0;

	InitIO();	// Must be initialised before any printf()

	clrscr();

   	puts("I'm alive!12345999");


   	InitDataCold();

	InitDataWarm();


	//EnWheelMotor();
	//DispAllSensorValues();
	//ReadDataFromFlash();
	//StartCMUCAM(4, 57600);
	//PrintCamValues();
	//CAM_to_WinM3();
	//TestMotorMenu();
#ifdef xxx
	while(1){
		char c;
		gotoxy(5,5);
		printf("%5d", pwm);
		c = GetChar();
		if (c=='u')
			pwm+=20;
		if (c=='d')
			pwm-=20;
		SetPWM0(pwm);
		SetPWM1(pwm);
	}
#endif


	MainMenu();

	return 0;
}

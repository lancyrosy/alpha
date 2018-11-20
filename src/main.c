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

   	printf("I'm alive!123 %u", DIST_mm_oc(3655/20 + 3655/10 + 150));


   	InitDataCold();

	InitDataWarm();


	//EnWheelMotor();
	//DispAllSensorValues();
	//ReadDataFromFlash();
	//StartCMUCAM(4, 57600);
	//PrintCamValues();
	//CAM_to_WinM3();
	//TestMotorMenu();
#define xxxx
#ifdef xxx
	int pwm=0;
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
	//IO_Write(LED1_PIN,1);
	//pulseBuzzer(1000,1);

	return 0;
}

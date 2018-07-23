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

int main(void){
	InitIO();	// Must be initialised before any printf()

	clrscr();

   	puts("Ning666 is alive!");

   	InitDataCold();

	InitDataWarm();

   	ReadDataFromFlash();

	MainMenu();

	return 0;
}

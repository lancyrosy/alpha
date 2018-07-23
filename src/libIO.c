// ---------------------------------------------------------------------------------
// File        :libIO.c
// Date        :06Feb2011
// Author      :NgBK
// Description :All other IO initialisation functions are called from InitIO();
// InitIO() should be the first function to be called from main()
// ---------------------------------------------------------------------------------
#include "project.h"

// @brief : Initialise all IO periperhals and port pins
// @param : none
// @retval: none
void InitIO() {
	DI;
	stm32_init();

	// All general input and output port pins
	GPIO_Init_Mode(LED0_PIN, GPIO_Mode_Out_PP);
	GPIO_Init_Mode(LED1_PIN, GPIO_Mode_Out_PP);



	InitMotor();

	SensorInit();

	// serial port for downloading program and debugging
	UART_init(0, 57600);

	// dotmatrix display or serial display on winM3
   	DispInit();

   	// User switch input
	InitSwitch();

	InitTimer2Buzzer();


	EI;




}

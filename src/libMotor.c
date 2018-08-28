// ---------------------------------------------------------------------------------
// libMotor.c
// Created on: 12-Feb-2010
// Author: nbk
// Desc:
// InitMotor() is used to initalise the IOs required for motor PID.
// EnWheelMotor() is used to enable motor PID on wheel motor
// DisWheelMotor() is used to disable motor PID on wheel motor
// ---------------------------------------------------------------------------------

#include "Project.h"
#include "libDebug.h"

/* Data for motor functions */
// PID gain constant
int16_t kp[NUM_OF_SPEED] = {4,		// kp[0] is for X-component
							1};		// kp[1] is for W-component
int16_t kd[NUM_OF_SPEED] = {8,		// kd[0] is for X-component
							0};		// kd[1] is for W-component

bool bWheelMotorEnable;
volatile bool bMotorISRFlag;
// These variables relate to individual wheel control
volatile int16_t wheelPWM[NUM_OF_WHEEL];		// wheelPWM[0] is for left wheel
												// wheelPWM[1] is for right wheel

volatile int16_t newEncoderData[NUM_OF_WHEEL];
volatile int16_t encoderSpeed[NUM_OF_WHEEL];
volatile int16_t oldEncoderData[NUM_OF_WHEEL];

// PID control variables
volatile int16_t posPWM[NUM_OF_SPEED];
volatile int16_t posErr[NUM_OF_SPEED];
volatile int16_t posErrOld[NUM_OF_SPEED];
volatile int16_t PIDFeedback[NUM_OF_SPEED];
volatile int16_t PIDInput[NUM_OF_SPEED];

int16_t encoderChangeCnt;
bool bUseEncoderClickFlag;

#define LEFTMOTORPWM		TIM1->CCR1
#define RIGHTMOTORPWM		TIM1->CCR4

#define MAXPWMVALUE			1000


int16_t tmplog1, tmplog2, tmplog3, tmplog4;


void InitMotor(void)
{
   	// Phase counting input for motor encoders
	InitTimer3_PhaseCountingRemap();
	InitTimer4_PhaseCounting();

	// PWM for DC motor and servo motor
	InitTimer1_PWM();

	// Motor direction pins
	GPIO_Init_Mode( LMOTORDIR_PIN, GPIO_Mode_Out_PP);
	GPIO_Init_Mode( RMOTORDIR_PIN, GPIO_Mode_Out_PP);

    IO_Write(LMOTORDIR_PIN, 0);
    IO_Write(RMOTORDIR_PIN, 0);

	LEFTMOTORPWM = 0;
	RIGHTMOTORPWM = 0;

	DisWheelMotor();
}

void EnWheelMotor() {
	bWheelMotorEnable = 1;
}

void DisWheelMotor() {
	bWheelMotorEnable = 0;
}

// ---------------------------------------------------------------------------------
// Motor PID for a wheel chair configuration robot
// Robot movement comprised 2 components, translational (forward & backward)
// and rotational movements.
// The PID is implemented according to the 2 movement components. This is because
// the Kp & Kd parameters might be different for both. Hence the performance for
// such a system should be better.
// ---------------------------------------------------------------------------------
void MotorPID(void)
{
	int i;

	// Read in encoder values.
	// The encoder values, encoderSpeed[] should be positive when robot is moving forward
	newEncoderData[0] = -TIM_GetCounter(TIM3);		// left wheel motor encoder
	newEncoderData[1] = -TIM_GetCounter(TIM4);		// right wheel motor

	// wheel0 - left wheel
	// wheel1 - right wheel
	// speed0 - x speed
	// speed1 - w speed

	// Calculate encoder difference and accumulate it into the wheel error
	// Calculate encoder difference and accumulate it into the wheel error
	for (i=0; i<NUM_OF_WHEEL; i++) {
		encoderSpeed[i] = (newEncoderData[i] - oldEncoderData[i]);
		oldEncoderData[i] = newEncoderData[i];
	}


	// XSPEED is the sum of left and right wheels speed
	PIDFeedback[XSPEED] = encoderSpeed[0] + encoderSpeed[1];

	// WSPEED is the difference between left and right wheel
	// SInce antiClockwise is positive, we take right-left wheel speed
	PIDFeedback[WSPEED] = encoderSpeed[1] - encoderSpeed[0];


	// ---------------------------------------------------------------------------------
	// PID position control
	// ---------------------------------------------------------------------------------
	i = 0;
	{
		// Accumulate the speed error to get position error
		posErr[i] += PIDInput[i]-PIDFeedback[i];

		// Simple PD control
		posPWM[i] = (kp[i]*posErr[i] + kd[i]*(posErr[i]-posErrOld[i]));
		posErrOld[i] = posErr[i];

	}

	posPWM[1] = (0-sensoroffset)/2 + ((sensoroffsetold-sensoroffset)*12);
	sensoroffsetold = sensoroffset;
	/////////////////////////////////////////////////////////
	// Calculate individual wheels PWM from X & W components
	/////////////////////////////////////////////////////////
	wheelPWM[0] = posPWM[0] - posPWM[1];
	wheelPWM[1] = posPWM[0] + posPWM[1];

	// Limit maximum PWM value
	for (i=0; i<NUM_OF_WHEEL; i++) {
		CLIP(wheelPWM[i], -(MAXPWMVALUE-1), (MAXPWMVALUE-1) );
	}


	DI;		// Disable interrupt so that motor pwm and dir pin are set without interruption


	// In the following, the dir and pwm sign depends on the motor connection
	// To check SetPWM0() & SetPWM1() functions, set the values below.
	//wheelPWM[0] = wheelPWM[1] = 0;

	if (bWheelMotorEnable){
		SetPWM0(wheelPWM[0]);
		SetPWM1(wheelPWM[1]);

	}
	else {
		if (!bUseEncoderClickFlag) {
			SetPWM0(0);
			SetPWM1(0);
		}
	}

	EI;


	bMotorISRFlag = TRUE;

}

// ---------------------------------------------------------------------------------
// Functions to set the motor PWM values
// pwm from -254 to +254
// For positive values, the robot should be moving forward
// Depending on motor mounting, motor driver connection etc, these functions need
// to be customised for different robot design.
// Left motor
// ---------------------------------------------------------------------------------
void SetPWM0(int16_t pwm){

	if (pwm>=0)
	{
		IO_Write(LMOTORDIR_PIN, 0);
		LEFTMOTORPWM = pwm;
	}
	else
	{
		IO_Write(LMOTORDIR_PIN, 1);
		LEFTMOTORPWM = MAXPWMVALUE+pwm;

	}
}

//right
void SetPWM1(int16_t pwm){

	if (pwm>0)
	{
		IO_Write(RMOTORDIR_PIN, 1);
		RIGHTMOTORPWM = MAXPWMVALUE-pwm;
	}
	else
	{
		IO_Write(RMOTORDIR_PIN, 0);
		RIGHTMOTORPWM = -pwm;

	}
}



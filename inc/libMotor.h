// ---------------------------------------------------------------------------------
// file : libmotor.h
// date : 05/7/2007
// name : nbk
// desc : header file for libmotor.c
// This is to serve as an example on how motor control is done.
// This is meant for 2 wheel robot.
// A simple speed profile controller and motor PID is implmented.
// Students must modify the various macros and c- source code to suit their own robot.
// ---------------------------------------------------------------------------------

#ifndef _LIBMOTOR_H_
#define _LIBMOTOR_H_
//#include <stdio.h>
//#include <stdbool.h>

typedef struct {
	int x;		// unit in mm
	int y;
	int dir;	// unit in 0.1 degree
} sPos;

typedef union {
	int fp;
	struct {
		int integer:10;
		int fraction:6;
	} part;
} fixedPt;

// ---------------------------------------------------------------------------------
//	Robot data (AlphaCentre Training robot)
//  Robot speed and distance control are done in real unit (mm and mm/sec)
//	The following macros allo us to easily convert real unit in mm & mm/sec into encoder counts
//  Change these constants to match that of your robot
#define ENCODER		(256*4)		//encoder resolution per revolution
#define W_DIAMETER	26			// Wheel diameter
#define W_DISTANCE	80			// Wheel to wheel distance
#define GEAR_RATIO	(45/12.0)	// Wheel to motor gear ratio
#define _PI_		3.142
#define CNT_PER_CM	(int)((ENCODER*10)*(GEAR_RATIO)/(W_DIAMETER*_PI_))

// Speed and acceleration variables used fixed point representation with lower n bits
// for decimal point. This is necessary as slower finer speed/acceleration is required
// Macro convert speed to virtual encoder count per PID_period
// PID loop interval is 1 msec
// Conversion factor (cm/sec to encoder cnt/msec) = 450*FIXED_PT_SCALING/1000
// (*FIXED_PT_SCALING) for fixed pt representation scaling
// (/1000) as the time unit is 1msec
#define CNT_PER_CM_PER_MS	(int)((CNT_PER_CM*FIXED_PT_SCALING)/1000)

// This assumes a wheel chair configuration for the robot
// For the robot to turn 360 degrees, the wheel has to travel PI*W_DISTANCE
#define CNT_PER_360DEG			(int)((ENCODER*W_DISTANCE*_PI_)*(GEAR_RATIO)/(W_DIAMETER*_PI_))
#define CNT_PER_360DEG_PER_MS	(int)((CNT_PER_360DEG*FIXED_PT_SCALING)/1000)

// ---------------------------------------------------------------------------------
// These macros are used to convert real unit to optical count
// This is necessary as ultimately, the motor feedback is in term of optical counts
// Speed and acceleration are in fix-point representation
// ---------------------------------------------------------------------------------
#define FIXED_PT_SCALING	128

#define SPEED_mm_oc(n)	((CNT_PER_CM_PER_MS*(n))/10)
											// Convert speed (mm/s) to (oc/ms)
											// oc refers to optical count
											// Note max speed is about 4m/s, before overflow
#define ACC_mm_oc(n)	((CNT_PER_CM_PER_MS*(n))/10000)
											// Convert acceleration (mm/s/s) to
											// (oc/ms/ms)
#define SPEED_deg_oc(n)	((CNT_PER_360DEG_PER_MS*(n))/360)

#define ACC_deg_oc(n)	((CNT_PER_360DEG_PER_MS*(n))/360000)


#define DIST_mm_oc(dist)	(((CNT_PER_CM)*(dist))/10)	// Convert distance to optical count
#define ANGLE_ADJUST		0							// adjust this value to make 360 accurate if necessary
#define DIST_deg_oc(dist)	(((CNT_PER_360DEG+ANGLE_ADJUST)*(dist))/360)


////////////////////////////////////////////////////////////////////////////////////
// There are 2 wheels. It is convenient to store wheels' related data into array
////////////////////////////////////////////////////////////////////////////////////
#define NUM_OF_WHEEL	2		// 0 - left
								// 1 - right
#define LEFT_WHEEL		0
#define RIGHT_WHEEL		1

// There are 2 kind of speeds and distance, linear x-speed and rotational w-speed.
// It is also convenient to theis 2 kinds of related data into array
#define NUM_OF_SPEED	2		// 0 - xSpeed
								// 1 - wSpeed (angular)
#define 	XSPEED		0
#define 	WSPEED		1

////////////////////////////////////////////////////////////////////////////////////
// Global data
////////////////////////////////////////////////////////////////////////////////////
extern bool bUseEncoderClickFlag;
extern volatile bool bEncoderClickFlag;
extern int16_t encoderClickType;

extern bool bAlignCamFlag;
extern bool bAlignLineFlag;
extern volatile bool bMotorISRFlag;
extern bool bWheelMotorEnable;
volatile extern sPos robotPos;
volatile extern int uncertainty;

extern volatile int16_t posErr[NUM_OF_SPEED];
extern volatile int16_t posErrOld[NUM_OF_SPEED];
extern volatile int16_t PIDFeedback[NUM_OF_SPEED];
extern volatile int16_t PIDInput[NUM_OF_SPEED];
extern volatile int16_t oldEncoderData[NUM_OF_WHEEL];
extern volatile int16_t encoderSpeed[NUM_OF_WHEEL];
extern volatile int16_t wheelPWM[NUM_OF_WHEEL];
extern volatile int32_t curPos[NUM_OF_SPEED];
extern volatile int32_t finalPos[NUM_OF_SPEED];
extern volatile int16_t curSpeed[NUM_OF_SPEED];
extern volatile int16_t targetSpeed[NUM_OF_SPEED];
extern volatile int16_t alignSpeed;
extern int16_t curAcc[NUM_OF_SPEED];
extern int16_t curDec[NUM_OF_SPEED];
extern int16_t afterBrakeDist[NUM_OF_SPEED];	// Speed profile use
											// after brake(deceleration) distance
											// In case there is not enough distance to brake
extern bool bDistDirFlag[NUM_OF_SPEED];		// To indicate direction of move in speed profile

// ---------------------------------------------------------------------------------
// Function prototypes
// Some functions are used internally by the libraries.
// You should never have to call them directly.
// For your convenience, the functions you are likely to use are placed in front.
// The internal functions are place behind.
// ---------------------------------------------------------------------------------

void InitMotor();		// To initialise the 2 drive motor hardware.
						// You need to invoke this function once in main()
						// before enabling the interrupt

void DisWheelMotor();	// Disable the 2 drive wheels
void EnWheelMotor() ;	// Enable the 2 drive wheels

void StopRobot(void);	// Stop robot movement immediately
						// May result in robot skidding if current speed is high.

// ---------------------------------------------------------------------------------
// The following functions are used to set the current target speed of the robot.
// Not to be used with the MoveRobot() or SetMoveCommand()
// ---------------------------------------------------------------------------------
void SetRobotSpeedX( int16_t xSpeed);
void SetRobotSpeedW( int16_t wSpeed);
void SetRobotAccX( int16_t acc, int16_t dec);
void SetRobotAccW( int16_t acc);

// ---------------------------------------------------------------------------------
// Set the speed profile parameters. It makes sure the robot travel the dist according
// to the speed and acceleration parameters.
// ---------------------------------------------------------------------------------
void MoveRobot(int16_t speedType, int16_t dist, int16_t brakeDistAllowance, int16_t topSpeed, int16_t endSpeed, int16_t acc,int16_t dcc);
void MoveRobotStraight(int16_t speedType, int16_t dist, int16_t brakeDistAllowance, int16_t topSpeed, int16_t endSpeed, int16_t acc,int16_t dcc,int16_t marker);
void MoveRobotCurve(int16_t speedType, int16_t dist, int16_t brakeDistAllowance, int16_t topSpeed, int16_t endSpeed, int16_t acc,int16_t dcc);
void MoveRobotCheck(int16_t speedType, int16_t dist, int16_t brakeDistAllowance, int16_t topSpeed, int16_t endSpeed, int16_t acc,int16_t dcc, int16_t marker);
// ---------------------------------------------------------------------------------
// Wait for the robot to travel certain distance. Must make sure the distance is less than
// final target distance
// ---------------------------------------------------------------------------------
void WaitDist(int16_t speedType, int16_t dist);

// ---------------------------------------------------------------------------------
// SetMoveCommand() is the same as MoveRobot() except that it doesn't wait for the
// robot to travel the distance required.
// ---------------------------------------------------------------------------------
void SetMoveCommand(int16_t speedType, int16_t dist, int16_t brakeDistAllowance, int16_t topSpeed, int16_t endSpeed, int16_t acc,int16_t dcc);

// ---------------------------------------------------------------------------------
// Wait for end of speed profile. Basically, the functions just check if the distance to
// travel is reached.
// ---------------------------------------------------------------------------------
int EndOfMove(int moveCmd);

// ---------------------------------------------------------------------------------
// Move robot to position x,y
// Based on cartesian x,y coordinates
// ---------------------------------------------------------------------------------
void GotoPos(sPos *pos);
void TestGotoPos();			// function to test GotoPos()
void PrintMotorStatus();	// Print motor internal status. For debugging
							// You can use this to observe the motor encoder values


// ---------------------------------------------------------------------------------
// For casual users of this library, you should never have to call the functions below.
// ---------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------
// Control the speed profile of robot based on distance left to travel, current speed,
// deceleration and end speed. It has 2 different speed control, the translational and
// rotational speed control
// User should never have to call this function directly
// ---------------------------------------------------------------------------------
void DoMoveCommand();


void MotorPID(void);		// Does the motor PID and robot move speed profile
							// Called from a regular interrupt routine

// ---------------------------------------------------------------------------------
// This function will keep adjusting the curSpeed of the robot until it reaches the
// targetSpeed. The main function would have to set the targetSpeed, and the curAcc[] in
// order to control the speed profile.
// Called from MotorPID().
// ---------------------------------------------------------------------------------
void UpdateCurSpeed();
void UpdateWheelPos();
void UpdateRobotPos();


// Calculate the deceleration required based on input parameters.
unsigned GetDecRequired(int32_t dist, int16_t brakeDist, int16_t curSpeed, int16_t endSpeed) __attribute__ ((noinline));

// ---------------------------------------------------------------------------------
// Set the PWM output for the 2 drive motors.
// Functions to set the motor PWM values
// pwm from -999 to +999
// For positive values, the robot should be moving forward
//
// Depending on motor mounting, motor driver connection etc, these functions need
// to be customised for different robot design.
//
// This function is used by the motor PID controller
// Do not call these functions unless you disable the motor PID
// ---------------------------------------------------------------------------------
void SetPWM0(int16_t pwm);	// left motor
void SetPWM1(int16_t pwm);	// right motor

// ---------------------------------------------------------------------------------
// This function implements the speed profile generator.
// Should be called only from the motor ISR
// ---------------------------------------------------------------------------------
void DoSpeedProfile();

// ---------------------------------------------------------------------------------
// Reset the drive motor speed profile internal data
// This should be called once only after the motor drive wheels are just enabled
// User should never have to call this function directly.
// ---------------------------------------------------------------------------------
void ResetSpeedProfileData();

#endif

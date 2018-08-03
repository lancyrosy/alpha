// ---------------------------------------------------------------------------------
//   File        :project.h
//   Version     :v1.0.1
//   Date        :2011/02/17
//   Author      :nbk
//   Description :
// ---------------------------------------------------------------------------------
#ifndef __PROJECT_H___
#define __PROJECT_H___

#ifndef __cplusplus
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif

#include "libstm32.h"
#include "stm32f10x.h"
#include "libSysTick.h"
#include "libSensor.h"
#include "libDotMatrix.H"
#include "libSwitch.h"
#include "libMotor.h"
#include "libCMUCAM.h"
#include "Myfunction.h"
// ---------------------------------------------------------------------------------
// Data type definition
// ---------------------------------------------------------------------------------
typedef void (*type_fp) (void);

// -------------------------------------------------------------------------------
//  Macros
// ---------------------------------------------------------------------------------

#define DI			__disable_irq()
#define EI			__enable_irq()
#define ABS(n)	((n)>=0?(n):(-n))		// Get absolute value
#define CLIP(n, min, max)	((n)=(n)>(max)?(max):(n)<(min)?(min):(n))

#define NULL			0
#define fputc(c, f)		PutChar(c)
#define fputs(s, f) 	PutStr(s)

#define SENSOR_ISR_PRIORITY			0		// lower number -> higher priority
#define USART_ISR_PRIORITY			1		// lower number -> higher priority
#define MOTOR_ISR_PRIORITY			2
#define SYSTICK_PRIORITY			3
#define DISPLAY_PRIORITY			4

#define HIGH					1
#define LOW						0
#define IO_Write(pin, value)	GPIO_WriteBit(pin, value)
#define IO_Read(pin)			GPIO_ReadInputDataBit(pin)

// IO pins definition. These macros make it easier to write & modify your program
#define LED0_PIN	GPIOA,GPIO_Pin_12
#define LED1_PIN	GPIOB,GPIO_Pin_9


#define LMOTORDIR_PIN	GPIOA, GPIO_Pin_13
#define RMOTORDIR_PIN	GPIOB, GPIO_Pin_8

#define TIM_SensorISR	TIM5

#define SOUND_TWEET				0
#define SOUND_ACK				1

#define MHZ						1000000
// ---------------------------------------------------------------------------------
//  Global variables
// ---------------------------------------------------------------------------------
extern int32_t	SYSCLK, APB1CLK, APB1TIMERCLK, APB2CLK;
extern const char waitMsg[];
extern const int8_t waitMsgSize;

extern volatile bool bSWFlag;

extern bool bAlignEnableFlag;
extern int16_t specialTestVariable;
extern int16_t specialTestVariable2;

extern bool bSWBeep;
extern uint8_t menuNum;		/* Define the current active menu */
extern uint8_t beeperOnOff;
volatile extern int16_t debugCode, debugCode1, debugCode2;
extern uint16_t elapsedTime;
extern bool	bDispSensorValue;


extern char debugChar;


extern bool bSensorEnableFlag;



extern volatile uint16_t sensorSwitchTime;
extern volatile uint16_t sensorSwitchOffTime;


extern bool bDisplayDelay;
extern bool bBlinkFlag, bScrollFlag, bRawFlag, bAlignFlag;
extern unsigned pulseDuration[2];
extern int pulseBuzzerDuration;
extern volatile int LSumMarker,RSumMarker,sumJunction;

// ---------------------------------------------------------------------------------
//  Function prototypes
// ---------------------------------------------------------------------------------
void InitIO();

/* Initialisation of global data (data.c) */
void InitDataCold();    /* Cold start global data initialisation     */
void InitDataWarm();    /* Warm start (after crash) global data init */

                                        /* 7 segment display. */
/* Menu related routines (menu.c) */
void MainMenu();                        /* main or run menu */
char SelectMenuItem(int8_t *startNum, int8_t numOfItem, const char *const *strg);
                                        /* Function for display menu number */
                                        /* and reading the number selected */
                                        /* Display is on 7 seg */
                                        /* Reading is from s/w */
                                        /* A short key press (<0.5sec) denotes scrolling*/
                                        /* of menu item. A long key press (>1 sec) select */
                                        /* that item. For consistency, '0' */
                                        /* should be the exit-menu number */
void MenuChangeVariable(int16_t *variable);

/* Switch, 7seg and beeper routines ( switch.c, s7seg.c, beep.c) */
#define BEEP_LEFT()			BeepConstant(2500)
#define BEEP_RIGHT()		BeepConstant(3000)
#define BEEP_FRONT()		BeepConstant(4000)
#define BEEP_DIAG()			BeepConstant(2500)
#define BEEP_STOP()			BeepConstant(5000)

void InitBeeper();
void Beep(int sound);                   /* Beep certain sound */
void BeepPWM();					/* Used PWM to beep (sensor.c) */
void BeepOK();
void BeepError();
void BeepConstant(int freq);
void BeepOnce();

void InitDotMatrix();
void OffDotMatrix();
void OnDotMatrix();
void BlinkDotMatrix();
void DispDotMatrix(const char *strg);
int  DispDotMatrixWait( const char *strg );
void DispDotMatrixInt16(int16_t *num);



/* Side sensors routines (sensor.c) */
void InitSensor();
void ReadSensorsAndUpdate();
void OffSensorTx();
void StartSensorISR();
void EnableSensor();
void DisableSensor();
void DisplaySensorOnDotMatrix(int sensorNum);
void PrintSensorValues();
void PrintSideSensorCaliValue();
void PrintFrontSensorCaliValue();
void CollectSensorStatistic();

/* */


/* */

/* Motor related routines (motor.c) */
void InitMotor(void);		/* motor initialisation */
void MotorISR(void);
void SetMotorCtrl( int mode );
void PauseMotor();
void UpdateWheelPos();
void InitMotorData();
void PrintMotorValues(void);
void PrintLog();
void ClearLog();
void TestMotorMenu();


// Serial port fns
void UART1_init(int baudrate);
void UART3_init(int baudrate);
void UART4_init(int baudrate);
void UART5_init(int baudrate);
int RxCharISR(int16_t uartNum);
int CheckRxBuf(int16_t uartNum);
int TxCharISR(int16_t uartNum, char c);
void PutStrISR(int16_t uartNum, char *s);

// Functions to tx and rx characters from UART1
// Same as calling TxCharISR() & RxCharISR()
int PutChar(char c);
int GetChar(void);
bool kbhit();
int PutStr(char *s);
void gotoxy(int x, int y);
void clrscr(void);

// Functions to tx and rx characters from UARTs by polling.
// Not compatible with above functions. Used for testing...
int RxChar(USART_TypeDef* usart);
int TxChar(USART_TypeDef* usart, char c);

// timer stuff
void InitTimer2_PhaseCountingRemap(void);
void InitTimer4_PhaseCounting(void);
void InitTimer1_PWM(void);
void InitTimer3_servoPWM(bool bCH1, bool bCH2, bool bCH3, bool bCH4);
void InitTimer8_servoPWM(bool bCH1, bool bCH2, bool bCH3, bool bCH4);
void SetTimer3PWM(int16_t ch, int16_t pwm);
void SetTimer8PWM(int16_t ch, int16_t pwm);
void InitTimer6_1msISR(void);
void InitTimer5_SensorISR(void);

uint16_t GetElapsedTime();
void StartElapsedTimer();
extern void Delayms(u32 time_ms);
extern void Handler_SysTick(void);

// GPIO stuff
void GPIO_Init_Mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode);

bool ReadDataFromFlash();
void WriteDataToFlash();

// stdio
int sprintf(char *out, const char *format, ...) __attribute__((noinline));
int printf(const char *format, ...) __attribute__((noinline));
int puts( char *str );
#endif

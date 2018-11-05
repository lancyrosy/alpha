// Author: nbk
// Desc  :
// Handler_SysTick() is a 1ms regular ISR. It is used for motor PID, speed profile etc.
// ---------------------------------------------------------------------------------

#include "project.h"

u8 	wait_timer_flag=0;
volatile u32 waitDelay;
uint16_t elapsedTime;
int encoderChangeCnt;
int count=0;
int sumoffset = 0;
int fOffset;
int sumoffset2 = 0;
int sumtOffset=0;
float timeCount=0;

bool bAlignFlag = TRUE;

void LED_ISR(void){
//	if(JMarkerFlag==TRUE){
//		IO_Write(LED1_PIN,1);
//	}
//	else{
//		IO_Write(LED1_PIN,0);
//	}

	if(pulseDuration[0]){
		pulseDuration[0]--;
		IO_Write(LED0_PIN,1);
	}
	else IO_Write(LED0_PIN,0);

	if(pulseDuration[1]){
		pulseDuration[1]--;
		IO_Write(LED1_PIN,1);
	}
	else IO_Write(LED1_PIN,0);

	if(pulseBuzzerDuration) pulseBuzzerDuration--;
	else     TIM2->CCR2 = 0;
}

// SysTick timer is setup in stm32_NVIC_Configuration()
// It is a regular 1ms interrupt

void Handler_SysTick(void){
	waitDelay--;
	elapsedTime++;

	// Start IR sensor interrupt
	//if (bSensorEnableFlag)
	if(RSumMarker==1){
		timeCount++;
	}
	count++;
	fOffset = (sensoroffset*2+fOffset*8)/10; //filtering
	sumoffset += sensoroffset;
	sumoffset2 += sensoroffset2;
	sumtOffset += tsensoroffset;
	if(count==5){
//		LogData(targetSpeed[0]/SPEED_mm_oc(1));
//		LogData(curSpeed[0] / SPEED_mm_oc(1));
//		LogData(sumoffset / 5);
		LogData(fOffset);
//		LogData(sumoffset2 / 5);
//		LogData(sumtOffset);
		count = 0;
		sumoffset = 0;
		sumoffset2 = 0;
		sumtOffset = 0;
	}
	StartSensorISR();

	DoSpeedProfile();

	if (bAlignFlag)
		Cen1();
	else sensoroffset = 0;
	MotorPID();

	LMarkerDetect();
	RMarkerDetect();

	// alphanumeric display service routine
	Disp_dma_service();

	// User switch service routine
	User_sw_service();

	LED_ISR();

	if (bUseEncoderClickFlag) {
		// Check encoder count for user interface
		// Only the right wheel is used
		// This allows the user to change a variable conveniently.
		// Very useful for testing.
		encoderChangeCnt+=encoderSpeed[1];

		if (encoderChangeCnt>20) {
			bEncoderClickFlag = TRUE;
			encoderClickType=0;
			encoderChangeCnt=0;
		}
		if (encoderChangeCnt<-20) {
			bEncoderClickFlag = TRUE;
			encoderClickType=1;
			encoderChangeCnt=0;
		}
		// Provide tactile feedback to user
		// Change the gain according to your preference
		SetPWM1(-encoderChangeCnt/2);
	}

}

#define SysTick_Counter_Disable        ((u32)0xFFFFFFFE)
#define SysTick_Counter_Enable         ((u32)0x00000001)

//SysTick
#define CTRL_TICKINT_Set      ((u32)0x00000002)
#define CTRL_TICKINT_Reset    ((u32)0xFFFFFFFD)

// @brief  delay function
// @param  delay in msec
// @retval None
void DelaymSec(uint32_t time_ms){
	wait_timer_flag=1;
	waitDelay = time_ms;
	while(waitDelay>0)
		if (bSWFlag==1 ) {
			break;
		}
}

void StartElapsedTimer() {
	elapsedTime = 0;
}


uint16_t GetElapsedTime() {
	return elapsedTime;
}

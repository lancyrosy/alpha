// ---------------------------------------------------------------------------------
// libAdc.c
// Created on: 15-Jan-2010
// Author: nbk
// Description :
// SensorInit() : To initialise for sensor use
// StartSensorISR() : To trigger sensor ISR routine. This should be called regularly every msec.
// DispAllSensorValues() : Display all sensors value for debugging
// DisableSensor() & EnableSensor() - Enable or disable sensor ISR through flag
// DisplaySensorOnDotMatrix() - To display a sensor value on dot matrix display
// ReadBatteryVolt() - Read and convert battery adc input
// DMA2_Channel4_5_IRQHandler() - 	Sensor ISR, which is actually a DMA interrupt. Need to modify
// 									according to your robot needs.
// ---------------------------------------------------------------------------------

#include "project.h"

#define RobotNumber 2

#if RobotNumber == 1
volatile int16_t sensorCalMax[NUM_SENSOR]={2500,2500,2500,2450,1800,2000,2100,2050,2000,1950,1900,1850,1900,1950,1750};
volatile int16_t sensorBlack[NUM_SENSOR]={140,120,125,115,125,120,130,125,130,120,120,120,115,120,110};
#elif RobotNumber == 2
volatile int16_t sensorCalMax[NUM_SENSOR]={1500,1600,1650,1600,1650,1700,1750,1700,1650,1650,1500,1450,1400,1750,1750};
volatile int16_t sensorBlack[NUM_SENSOR]={75,85,95,90,95,95,105,100,95,95,95,90,75,100,100};
#endif

volatile int16_t  sensorCal[NUM_SENSOR];
volatile int16_t sensor[NUM_SENSOR];
volatile int16_t sensorOld[NUM_SENSOR];
volatile int16_t sensorMin[NUM_SENSOR];
volatile uint16_t sensorMax[NUM_SENSOR];
volatile uint16_t adc1_dma_buf[16];
volatile uint16_t adc2_dma_buf[16];
volatile uint16_t adc3_dma_buf[16];

volatile int senfla;
volatile int sensoroffset, sensoroffsetX2, sensoroffset2, sensoroffsetold, cenval;
volatile int state, substate;
volatile int adcCnt;

bool bDispSensorValue;
bool bEndSensorISRFlag;


bool bSensorEnableFlag;
void CalibrateADC(ADC_TypeDef* ADCx);

// ---------------------------------------------------------------------------------
// @brief : To init ADC and port pins for analog sensors
// @param : none
// @retval: none
// ---------------------------------------------------------------------------------
void SensorInit(void) {

	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//ADC i/p port pin initialisation
	//Change this according to your design
	//MODIFY HERE


	GPIO_Init_Mode(GPIOA, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_AIN);
	GPIO_Init_Mode(GPIOB, GPIO_Pin_0|GPIO_Pin_1, GPIO_Mode_AIN);
	GPIO_Init_Mode(GPIOC, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5, GPIO_Mode_AIN);



	//------------------------------------------------------------------
	// ADC1 is tied to DMA1 channel1
	//------------------------------------------------------------------
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);   /* Enable ADC1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (ADC1->DR);	//JDR1
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adc1_dma_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = NUM_ADC1_INPUT;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Convert a few channels
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 	//ADC_ExternalTrigInjecConv_None
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = NUM_ADC1_INPUT;
	ADC_Init(ADC1, &ADC_InitStructure);

	// Add all the non-pulsed adc inputs here
	//MODIFY HERE
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_1, 1, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_2, 2, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_3, 3, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_4, 4, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_5, 5, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_6, 6, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_7, 7, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_8, 8, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_9, 9, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_10, 10, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_11, 11, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_12, 12, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_13, 13, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_14, 14, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_15, 15, ADC_SampleTime_28Cycles5);
			ADC_RegularChannelConfig(ADC1, ADC_CH_SEN_BATTERY, 16, ADC_SampleTime_28Cycles5);


	// Enable dma interrupt on TC5 (transfer complete)
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SENSOR_ISR_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

		ADC_Cmd(ADC1, ENABLE);
		CalibrateADC(ADC1);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);		// ADC actively converting all the time
												// result in adc1_dma_buf


}

// ---------------------------------------------------------------------------------
// @brief : To start sensor isr. ADC3 is triggered by TIM_SensorISR
//		  : ADC1 is software started
// @param : none
// @retval: none
// ---------------------------------------------------------------------------------
void StartSensorISR() {
	state = 0;
	substate = 0;
	bEndSensorISRFlag = FALSE;

	//TIM_Cmd(TIM_SensorISR, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//Start ADC1 conversion
}

// StartSensorISR() starts the sensor read-in process.
// TIM_SensorISR(TIM5) is setup to trigger ADC3 conversion.
// ADC3 is setup to convert a few channels at a time. scan mode ON.
// ADC3 is served by dma2 channel-5
// Upon converting all adc channels, the dma will trigger this interrupt.

void DMA1_Channel1_IRQHandler(void) {
	DMA_ClearITPendingBit(DMA1_IT_TC1);
		int a=0;


	for(a=0;a<16;a++){
    	 sensor[a] = (adc1_dma_buf[a]+sensor[a])/2;
     }



			// store dc values of IR sensors


}


// ---------------------------------------------------------------------------------
// @brief : To calibrate ADC during initialisation
// @param ADCx: which adc
// @retval: none
// ---------------------------------------------------------------------------------
void CalibrateADC(ADC_TypeDef* ADCx) {

	/* Enable ADCx reset calibaration register */
	ADC_ResetCalibration(ADCx);

	/* Check the end of ADCx reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADCx))
		;

	/* Start ADCx calibaration */
	ADC_StartCalibration(ADCx);

	/* Check the end of ADCx calibration */
	while (ADC_GetCalibrationStatus(ADCx))
		;
}

// @brief : To collect sensors' minimum and maximum values
// @param : none
// @retval: none
void CollectSensorStatistic() {
	uint16_t i;

	for (i = 0; i < NUM_SENSOR; i++) {
		if (sensor[i] > sensorMax[i])
			sensorMax[i] = sensor[i];
		if (sensor[i] < sensorMin[i])
			sensorMin[i] = sensor[i];
	}

}

// ---------------------------------------------------------------------------------
// @brief : To display all sensor values on UART
// @param : none
// @retval: none
// ---------------------------------------------------------------------------------
#define ROW1	4
#define COL1	8
void DispAllSensorValues() {
	int i;

	bDispSensorValue = TRUE;
	clrscr();
	DispDotMatrixRaw(waitMsg, waitMsgSize);
	DelaymSec(500);

	//	SetMotorCtrl(ENABLE_MOTOR);
	//	SetMotorCtrl(FORWARDDIR);
	//	targetXSpeed = 1000;

	//	SetNextCommand( CMD_STRAIGHT_RUN, 0);
	//	curTargetPosX = 0;
	//	finalTargetPosX = CD(1800);


	for (i = 0; i < NUM_SENSOR; i++) {
		sensorMax[i] = 0;
		sensorMin[i] = 4096;
	}



	while (1) {
			char a[10];
	        sprintf(a,"%4u",ReadBatteryVolt() );
			gotoxy(COL1, ROW1+2);
			printf(" S1   S2   S3   S4");		// dc value
			gotoxy(COL1, ROW1+4);
			printf("%4u %4u %4u %4u", sensorCal[0], sensorCal[1], sensorCal[2], sensorCal[3]);
	        gotoxy(COL1, ROW1+6);
	        printf(" S5   S6   S7   S8   S9   S10   S11   S12   S13");
	        gotoxy(COL1, ROW1+8);
			printf("%4u %4u %4u %4u %4u %4u %4u %4u %4u",sensorCal[4], sensorCal[5], sensorCal[6], sensorCal[7],
					sensorCal[8], sensorCal[9], sensorCal[10], sensorCal[11], sensorCal[12]);
	        gotoxy(COL1, ROW1+10);
			printf(" S14  S15");
			gotoxy(COL1, ROW1+12);
			printf("%4u %4u",sensorCal[13], sensorCal[14]);
	        gotoxy(COL1, ROW1+14);
			printf("Battery voltage: %4uV ", ReadBatteryVolt());
		   	gotoxy(COL1, ROW1+16);
		   	printf("Sensor Offset: second row: %5d,  first row: %5d", sensoroffset, sensoroffset2);
		 	gotoxy(COL1, ROW1+18);
		 	printf("Black sensor value:");
		   	gotoxy(COL1, ROW1+20);
		   	printf(" S1   S2   S3   S4");		// dc value
		   	gotoxy(COL1, ROW1+22);
		   	printf("%4u %4u %4u %4u", sensorBlack[0], sensorBlack[1], sensorBlack[2], sensorBlack[3]);
		   	gotoxy(COL1, ROW1+24);
		   	printf(" S5   S6   S7   S8   S9   S10   S11   S12   S13");
		   	gotoxy(COL1, ROW1+26);
		   	printf("%4u %4u %4u %4u %4u %4u %4u %4u %4u",sensorBlack[4], sensorBlack[5], sensorBlack[6], sensorBlack[7],
		   			sensorBlack[8], sensorBlack[9], sensorBlack[10], sensorBlack[11], sensorBlack[12]);
		   	gotoxy(COL1, ROW1+28);
		   	printf(" S14  S15");
		   	gotoxy(COL1, ROW1+30);
		   	printf("%4u %4u",sensorBlack[13], sensorBlack[14]);

		   	DispDotMatrix(a);
			if (bSWFlag) {
				bSWFlag = FALSE;
				DelaymSec(200);
				break;
			}
		}
	//	SetMotorCtrl(DISABLE_MOTOR);
	bDispSensorValue = FALSE;
}

// @brief : To disable IR sensor isr()
void DisableSensor() {
	bSensorEnableFlag = 0;
}

// @brief : To enable IR sensor isr()
void EnableSensor() {
	bSensorEnableFlag = 1;
}

void DisplaySensorOnDotMatrix(int sensorNum) {

	uint16_t value;
	uint16_t *ptr = NULL;

	switch (sensorNum) {
	case 1:
		ptr = (uint16_t *) &sensor[0];
		break;
	case 2:
		ptr = (uint16_t *) &sensor[1];
		break;
	case 3:
		ptr = (uint16_t *) &sensor[2];
		break;
	case 4:
		ptr = (uint16_t *) &sensor[3];
		break;
	case 5:
		ptr = (uint16_t *) &sensor[4];
		break;
	case 6:
		ptr = (uint16_t *) &sensor[5];
		break;
	case 7:
		ptr = (uint16_t *) &sensor[6];
		break;
	case 8:
		ptr = (uint16_t *) &sensor[7];
		break;


	}

	while (1) {
		char s[10];
		value = *ptr;

		sprintf(s, "%04d", value);
		DispDotMatrix(s);
		DelaymSec(50);

		if (bSWFlag) {
			//Beep(SOUND_ACK);
			bSWFlag = 0;
			break;
		}
	}
}

void DisplayBatteryVoltOnDotMatrix() {
	while (1) {
		char s[10];

		sprintf(s, "%04d", ReadBatteryVolt());
		DispDotMatrix(s);
		DelaymSec(100);

		if (bSWFlag) {
			bSWFlag = 0;
			break;
		}
	}
}

// Convert battery adc value into 0.01 volts. So 8.4 volts is returned as 840.
uint16_t ReadBatteryVolt() {
	// R series = 4k7 & 20k
	// ratio = 0.190
	// adc is 4096 for 3.3V
	// 8.4V -> adc = 8.4V*0.190/3.3V*4096 = 1980
	// The 2nd parameter needs to be calibrated
	return (senBattery * 840) / 1980;
}
volatile bool senLowFlag = FALSE;
int16_t Cen1(){
	int i=0;
	int sumHigh=0;
	int sumLowFst=0;
	int sumLowScd =0;
	int tmpOffset;
	for (i=0; i<15; i++) {
		sensorCal[i] = (sensor[i]-0)*2000l/sensorCalMax[i]; //sensorBlack[i]
		if (sensorCal[i] <= 0) sensorCal[i] = 0;

	}
	for (i=0; i<4; i++){
		if (sensorCal[i] < 150) sumLowFst++;
	}
	for (i=4; i<13; i++) {
		if (sensorCal[i] > 1000) sumHigh++;
		if (sensorCal[i] < 150) sumLowScd++;
	}

	tmpOffset = (sensorCal[4]*(-1600l)+sensorCal[5]*(-1200l)+sensorCal[6]*(-800l)+sensorCal[7]*(-400l)+sensorCal[9]*(400l)
						+sensorCal[10]*(800l)+sensorCal[11]*(1200l)+sensorCal[12]*(1600l))*2
					/(sensorCal[4]+sensorCal[5]+sensorCal[6]+sensorCal[7]+sensorCal[9]+sensorCal[10]
						+sensorCal[11]+sensorCal[12]);

	sensoroffset2 = (sensorCal[0]*(-600l)+sensorCal[1]*(-200l)+sensorCal[2]*(200l)+sensorCal[3]*(600l))
					/(sensorCal[0]+sensorCal[1]+sensorCal[2]+sensorCal[3]);

	if (sumHigh >= 7) {
		sensoroffset = sensoroffsetX2 = -1;// meet junction
	}
	if (sumLowFst == 4) sensoroffset2 = 1;
	if (sumLowScd == 9) {
		sensoroffset = 1;
	}
	else {
		sensoroffsetX2 += (tmpOffset-sensoroffsetX2);
		sensoroffset = sensoroffsetX2/2;
	}
	if (sumLowFst == 4 && sumLowScd == 9)
		senLowFlag = TRUE;
	else
		senLowFlag = FALSE;

    return sensoroffset;


}



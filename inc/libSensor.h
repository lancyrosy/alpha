// ---------------------------------------------------------------------------------
// libSensor.h
// Created on: 22-Feb-2011
// Author: nbk
// ---------------------------------------------------------------------------------

#ifndef LIBSENSOR_H_
#define LIBSENSOR_H_

// ---------------------------------------------------------------------------------
//  Macros
// ---------------------------------------------------------------------------------
// These sensors need to be actively pulsed. They are converted thru ADC1
#define ADC_CH_SEN_1		ADC_Channel_9
#define ADC_CH_SEN_2		ADC_Channel_8
#define ADC_CH_SEN_3		ADC_Channel_15
#define ADC_CH_SEN_4		ADC_Channel_14
#define ADC_CH_SEN_5		ADC_Channel_7
#define ADC_CH_SEN_6		ADC_Channel_6
#define ADC_CH_SEN_7		ADC_Channel_5
#define ADC_CH_SEN_8		ADC_Channel_3
#define ADC_CH_SEN_9		ADC_Channel_2
#define ADC_CH_SEN_10		ADC_Channel_1
#define ADC_CH_SEN_11		ADC_Channel_0
#define ADC_CH_SEN_12		ADC_Channel_13
#define ADC_CH_SEN_13	    ADC_Channel_12
#define ADC_CH_SEN_14	    ADC_Channel_10
#define ADC_CH_SEN_15	    ADC_Channel_11

// These sensors provide analog output directly. They are converted thru ADC1
#define ADC_CH_SEN_BATTERY	ADC_Channel_4
#define NUM_ADC1_INPUT		16

#define NUM_SENSOR		(NUM_ADC1_INPUT)

// IO pins used for IR sensor transmit
#define SEN_RX_15		GPIOC,GPIO_Pin_1
#define SEN_RX_14		GPIOC,GPIO_Pin_0
#define SEN_RX_13		GPIOC,GPIO_Pin_2
#define SEN_RX_12		GPIOC,GPIO_Pin_3
#define SEN_RX_11		GPIOA,GPIO_Pin_0
#define SEN_RX_10		GPIOA,GPIO_Pin_1
#define SEN_RX_9		GPIOA,GPIO_Pin_2
#define SEN_RX_8		GPIOA,GPIO_Pin_3
#define SEN_RX_7		GPIOA,GPIO_Pin_5
#define SEN_RX_6		GPIOA,GPIO_Pin_6
#define SEN_RX_5		GPIOA,GPIO_Pin_7
#define SEN_RX_4		GPIOC,GPIO_Pin_4
#define SEN_RX_3		GPIOC,GPIO_Pin_5
#define SEN_RX_2		GPIOB,GPIO_Pin_0
#define SEN_RX_1		GPIOB,GPIO_Pin_1
#define SEN_BATTERY		GPIOA,GPIO_Pin_4

// Macros to give meaningful names to the sensors
#define sen1		sensor[0]
#define sen2		sensor[1]
#define sen3		sensor[2]
#define sen4		sensor[3]
#define sen5		sensor[4]
#define sen6		sensor[5]
#define sen7        sensor[6]
#define sen8        sensor[7]
#define sen9        sensor[8]
#define sen10        sensor[9]
#define sen11       sensor[10]
#define sen12       sensor[11]
#define sen13       sensor[12]
#define sen14		sensor[13]
#define sen15		sensor[14]
#define senBattery  sensor[15]


// ---------------------------------------------------------------------------------
//  Global variables
// ---------------------------------------------------------------------------------
extern volatile uint16_t adc1_dma_buf[16];	// For adc->dma buffer



extern volatile uint16_t sensor[NUM_SENSOR];		// actual sensor values
extern volatile uint16_t sensorOld[NUM_SENSOR];
extern volatile int sensoroffset, sensoroffset2, sensoroffsetold, senfla, cenval;
extern volatile int16_t  sensorCal[NUM_SENSOR];
extern volatile int16_t  sensorBlack[NUM_SENSOR];

extern char dispBuffer[256];
extern bool	bDispSensorValue;
extern bool bSensorEnableFlag;
extern bool bFlashDetectFlag;
extern bool bEndSensorISRFlag;
extern bool slowFlag;
extern bool fastFlag;

// ---------------------------------------------------------------------------------
//  Function prototypes
// ---------------------------------------------------------------------------------
void SensorInit(void);
void StartSensorISR(void);
void DispAllSensorValues(void);
void CollectSensorStatistic();
void EnableSensor();
void DisableSensor();
void DisplaySensorOnDotMatrix(int sensorNum);
void DisplayBatteryVoltOnDotMatrix();

uint16_t ReadBatteryVolt();

#endif /* LIBSENSOR_H_ */

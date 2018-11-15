// ---------------------------------------------------------------------------------
// libTimer.c
// Created on: 11-Jan-2010
// Author: nbk
// Desc:
// InitTimer2_PhaseCountingRemap() - Init timer2 for quadrature encoder counting
// InitTimer4_PhaseCounting() - Init timer4 for quadrature encoder counting
// InitTimer1_PWM() - Init timer1 to generate 2 PWM signals for DC motors
// InitTimer3_servoPWM() - Init timer2 to generate up to 4 servo motor signals
// InitTimer8_servoPWM() - Init timer8 to generate up to 4 servo motor signals
// SetTimer3PWM() & SetTimer8PWM() - Set the PWM pulse width
// ---------------------------------------------------------------------------------
#include "project.h"

// @brief : To init TIM2 for quadrature encoder input
//		  : Timer2 partial remap to  PB3 & PA15
// @param : none
// @retval: none
void InitTimer2_PhaseCountingRemap(void){

	// ---------------------------------------------------------------------------------
	// Timer2 partial remap to  PB3 & PA15
	// ---------------------------------------------------------------------------------
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

	GPIO_Init_Mode(GPIOB, GPIO_Pin_3, GPIO_Mode_IN_FLOATING);
	GPIO_Init_Mode(GPIOA, GPIO_Pin_15, GPIO_Mode_IN_FLOATING);

	/* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
    TIM_SetAutoreload(TIM2,0xffff);

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
}
void InitTimer3_PhaseCountingRemap(void){

	// ---------------------------------------------------------------------------------
	// Timer2 partial remap to  PB3 & PA15
	// ---------------------------------------------------------------------------------
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

	GPIO_Init_Mode(GPIOB, GPIO_Pin_4, GPIO_Mode_IN_FLOATING);
	GPIO_Init_Mode(GPIOB, GPIO_Pin_5, GPIO_Mode_IN_FLOATING);

	/* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
    TIM_SetAutoreload(TIM3,0xffff);

    /* TIM2 enable counter */
    TIM_Cmd(TIM3, ENABLE);
}

// @brief : To init TIM4 for quadrature encoder input, Use PB6&PB7
// @param : none
// @retval: none
void InitTimer4_PhaseCounting(void){

	// Init PB6 & PB7 as input pins
	GPIO_Init_Mode(GPIOB, GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_IN_FLOATING);

	/* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
    TIM_SetAutoreload(TIM4,0xffff);

    /* TIM2 enable counter */
    TIM_Cmd(TIM4, ENABLE);
}

// @brief : To init TIM1 for DC motor pwm, PA8 & PA11
// @param : none
// @retval: none
void InitTimer1_PWM(){
#define TIM_PWM		TIM1
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;



	//Timer1_CH1(PA8) Timer1_CH4(PA11)
	GPIO_Init_Mode(GPIOA, GPIO_Pin_8|GPIO_Pin_11, GPIO_Mode_AF_PP);

	/* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

     /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = (4-1);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM_PWM, &TIM_TimeBaseStructure);

    /* Output Compare Active Mode configuration: Channel1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    /* Output Compare Timing Mode configuration: Channel1 */
    TIM_OC1Init(TIM_PWM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM_PWM, TIM_OCPreload_Enable);

     /* Output Compare Active Mode configuration: Channel4 */
    TIM_OC4Init(TIM_PWM, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM_PWM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM_PWM, ENABLE);


    // output enable for TIM1 only
    TIM_CtrlPWMOutputs(TIM_PWM, ENABLE);
    TIM_Cmd(TIM_PWM, ENABLE);

 }

// @brief : To init TIM3 for servo motor pwm, PA6, PA7, PB0, PB1
// @param : none
// @retval: none
void InitTimer3_servoPWM(bool bCH1, bool bCH2, bool bCH3, bool bCH4){
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

     /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 20000;
    TIM_TimeBaseStructure.TIM_Prescaler = (APB2CLK/MHZ-1);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* Output Compare Active Mode configuration: Channel1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//Timer3_CH1(PA6) Timer3_CH2(PA7) Timer3_CH3(PB0) Timer3_CH4(PB1)
    if (bCH1) {
    	GPIO_Init_Mode(GPIOA, GPIO_Pin_6, GPIO_Mode_AF_PP);
    	/* Output Compare Timing Mode configuration: Channel1 */
    	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }

    if (bCH2) {
    	GPIO_Init_Mode(GPIOA, GPIO_Pin_7, GPIO_Mode_AF_PP);
       	/* Output Compare Timing Mode configuration: Channel1 */
    	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }

    if (bCH3) {
    	GPIO_Init_Mode(GPIOB, GPIO_Pin_0, GPIO_Mode_AF_PP);
       	/* Output Compare Timing Mode configuration: Channel1 */
       	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
       	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }

    if (bCH4) {
    	GPIO_Init_Mode(GPIOB, GPIO_Pin_1, GPIO_Mode_AF_PP);
       	/* Output Compare Active Mode configuration: Channel4 */
       	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
       	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // output enable for TIM3 only
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

 }

void InitTimer2Buzzer(void){
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

	/* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

     /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 1000;
    TIM_TimeBaseStructure.TIM_Prescaler = (APB2CLK/MHZ-1);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Output Compare Active Mode configuration: Channel1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//Timer3_CH1(PA6) Timer3_CH2(PA7) Timer3_CH3(PB0) Timer3_CH4(PB1)

    GPIO_Init_Mode(GPIOB, GPIO_Pin_3, GPIO_Mode_AF_PP);
    /* Output Compare Timing Mode configuration: Channel1 */
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);



    TIM_ARRPreloadConfig(TIM2, ENABLE);

    // output enable for TIM3 only
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    TIM2->CCR2 = 0;

 }

// @brief : To init TIM8 for servo motor pwm, PC6, PC7, PC8, PC9
// @param : none
// @retval: none
void InitTimer8_servoPWM(bool bCH1, bool bCH2, bool bCH3, bool bCH4){
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* TIM8 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

     /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 20000;
    TIM_TimeBaseStructure.TIM_Prescaler = (APB2CLK/MHZ-1);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    /* Output Compare Active Mode configuration: Channel1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//Timer8_CH1(PC6) Timer8_CH2(PC7) Timer8_CH3(PC8) Timer8_CH4(PC9)
    if (bCH1) {
    	GPIO_Init_Mode(GPIOC, GPIO_Pin_6, GPIO_Mode_AF_PP);
    	/* Output Compare Timing Mode configuration: Channel1 */
    	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    }

    if (bCH2) {
    	GPIO_Init_Mode(GPIOC, GPIO_Pin_7, GPIO_Mode_AF_PP);
       	/* Output Compare Timing Mode configuration: Channel1 */
    	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    }

    if (bCH3) {
    	GPIO_Init_Mode(GPIOC, GPIO_Pin_8, GPIO_Mode_AF_PP);
       	/* Output Compare Timing Mode configuration: Channel1 */
       	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
       	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    }

    if (bCH4) {
    	GPIO_Init_Mode(GPIOC, GPIO_Pin_9, GPIO_Mode_AF_PP);
       	/* Output Compare Active Mode configuration: Channel4 */
       	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
       	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM8, ENABLE);

    // output enable for TIM3 only
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);

 }

void SetTimer3PWM(int16_t ch, int16_t pwm) {
	switch(ch) {
	case 1 : TIM3->CCR1 = pwm; break;
	case 2 : TIM3->CCR2 = pwm; break;
	case 3 : TIM3->CCR3 = pwm; break;
	case 4 : TIM3->CCR4 = pwm; break;
	}
}

void SetTimer8PWM(int16_t ch, int16_t pwm) {
	switch(ch) {
	case 1 : TIM8->CCR1 = pwm; break;
	case 2 : TIM8->CCR2 = pwm; break;
	case 3 : TIM8->CCR3 = pwm; break;
	case 4 : TIM8->CCR4 = pwm; break;
	}
}

#define TIM_1msISR	TIM6
// @brief : To init TIM6 for regular 1ms isr
// @param : none
// @retval: none
void InitTimer6_1msISR() {
    NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	// TIM6 clock is APB1CLK, maximum
	// Total division is APB1CLK/1000 to give 1msec
	TIM_PrescalerConfig(TIM_1msISR, 0, TIM_PSCReloadMode_Immediate);
	TIM_SetAutoreload(TIM_1msISR, APB1TIMERCLK/1000-1);		// count from 0. So subtract 1

    /* Configure and enable TIM interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MOTOR_ISR_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM_1msISR, TIM_IT_Update, ENABLE);
	TIM_GenerateEvent(TIM_1msISR, TIM_EventSource_Update);
    TIM_Cmd(TIM_1msISR, ENABLE);


}

volatile int timer6Cnt;
// @brief : TIM6 1ms isr. For PID, display etc....
// @param : none
// @retval: none
void TIM6_IRQHandler(void) {

	TIM_ClearFlag(TIM_1msISR, TIM_FLAG_Update);	// Clear interrupt flag
	if (timer6Cnt&0x01)
		IO_Write(LED0_PIN, HIGH);
	else
		IO_Write(LED0_PIN, LOW);
	timer6Cnt++;


}

// @brief : To init TIM5 for IR sensor . It is used as a trigger for ADC conversion
//			Does not generate it's own interrupt
// @param : none
// @retval: none
#define SENSOR_PULSE_WIDTH	80			//usec
void InitTimer5_SensorISR() {
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	// TIM5 clock is APB1CLK, maximum
	// Total division is APB1CLK/MHZ
	TIM_PrescalerConfig(TIM_SensorISR, (APB1CLK/MHZ)-1, TIM_PSCReloadMode_Immediate);
	TIM_SetAutoreload(TIM_SensorISR, SENSOR_PULSE_WIDTH-1);		// count from 0. So subtract 1

    /* Configure and enable TIM interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);		// to enable interrupt, uncomment

    TIM_OCStructInit( &TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OC3Init(TIM_SensorISR,&TIM_OCInitStruct);
    TIM_SetCompare3(TIM_SensorISR, SENSOR_PULSE_WIDTH-1);
    TIM_CCxCmd(TIM_SensorISR,TIM_Channel_3,TIM_CCx_Enable);


}

volatile int timer5Cnt;
void TIM5_IRQHandler(void) {

	//TIM_ClearFlag(TIM_SensorISR, TIM_FLAG_CC3);

	timer5Cnt++;

}


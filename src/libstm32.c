// ---------------------------------------------------------------------------------
// File        :libstm32
// Version     :
// Date        :
// Author      :nbk
// Description :
// stm32_init() initalises the stm32 clock subsystem & enable the GPIO clock
// GPIO_Init_Out_PP() initialises port pin as a normal pushpull output
// GPIO_Init_Out_OD() initialises port pin as a normal open drain output
// GPIO_Init_In_Floating() initialises port pin as a digital input pin
// GPIO_Init_AF_PP() initialises port pin as alternate function (e.g. timer) pushpull output
// GPIO_Init_AF_PP() initialises port pin as alternate function (e.g. timer) opendrain output
// GPIO_Init_In_Adc() initialises port pin as adc input
// ---------------------------------------------------------------------------------
#include "project.h"

int32_t	SYSCLK, APB1CLK, APB1TIMERCLK, APB2CLK;

void stm32_TIM_Configuration(void);
void stm32_GPIO_Configuration(void);
void stm32_NVIC_Configuration(void);
void stm32_RCC_Configuration(bool bHSEFlag);

// @brief : stm32 clock subsystem initialisation
//		  : GPIO peripherals clock enabled
// @param : none
// @retval: none
void stm32_init() {
	//
	stm32_RCC_Configuration(FALSE); //true(HSE), false(HSI/no external resonator)
	//
	stm32_NVIC_Configuration();

}

// @brief : set SYSCLK to 64MHz.
//		  : APB1 set to 32MHz & APB2 set to 64MHz
// @param : bHSEFlag - true(external clk), false(internal clk)
// 		  : External resonator must be 8MHz, else adjust program accordingly
// @retval: none
void stm32_RCC_Configuration(bool bHSEFlag) {
	ErrorStatus HSEStartUpStatus;
	// SYSCLK, HCLK, PCLK2 and PCLK1 configuration
	// RCC system reset(for debug purpose)
	RCC_DeInit();

	// Disable LSE so that PC13, PC14 & PC15 can be used.
	RCC_LSEConfig(RCC_LSE_OFF);

	if (bHSEFlag == TRUE) {
		// Enable HSE, external oscillator
		RCC_HSEConfig(RCC_HSE_ON);

		// Wait till HSE is ready
		HSEStartUpStatus = RCC_WaitForHSEStartUp();

		if (HSEStartUpStatus == SUCCESS) {
			// Enable Prefetch Buffer
			FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
			// Flash 2 wait state
			FLASH_SetLatency(FLASH_Latency_2);
			// HCLK = SYSCLK; AHB=SYSCLK
			RCC_HCLKConfig(RCC_SYSCLK_Div1);
			// PCLK2 = HCLK; APB2=SYSCLK
			RCC_PCLK2Config(RCC_HCLK_Div1);
			// PCLK1 = HCLK/2; APB1 = SYSCLK/2
			RCC_PCLK1Config(RCC_HCLK_Div2);
			// ADCCLK = PCLK2/6
			RCC_ADCCLKConfig(RCC_PCLK2_Div6);
			/* PLLCLK = 8MHz * 8 = 64 MHz */
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_8);
			// Enable PLL
			RCC_PLLCmd(ENABLE);
			// Wait till PLL is ready
			while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
				;
			// Select PLL as system clock source
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
			// Wait till PLL is used as system clock source
			while (RCC_GetSYSCLKSource() != 0x08)
				;
		}
	} else {
		// Enable HSI (High Speed Internal) clock, which is 8MHz
		// Enable Prefetch Buffer
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		// Flash 2 wait state
		FLASH_SetLatency(FLASH_Latency_2);
		// HCLK = SYSCLK; AHB=SYSCLK
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		// PCLK2 = HCLK; APB2=SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);
		// PCLK1 = HCLK/2; APB1 = SYSCLK/2
		RCC_PCLK1Config(RCC_HCLK_Div2);
		// ADCCLK = PCLK2/6
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);

		// PLL provides frequency multiplier of (HSI/2)
		// So (HSI/2)*16 = 64MHz
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
		// Enable PLL
		RCC_PLLCmd(ENABLE);
		// Wait till PLL is ready
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
			;
		// Select PLL as system clock source
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		// Wait till PLL is used as system clock source
		while (RCC_GetSYSCLKSource() != 0x08)
			;
	}
	SYSCLK = APB2CLK = 64000000;
	APB1CLK = 32000000;
	APB1TIMERCLK = APB1CLK*2;		// APB1 timer clock x2 if APB1 prescaler != 1
									// Refer to datasheet clock tree block diagram


	/* AFIO GPIOA,GPIOB,GPIOC clock enable */
	// AFIO enable is necessary for remap
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA
			| RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
			| RCC_APB2Periph_GPIOD, ENABLE);

	//nbk
	// Not using JTAG or SW debug. Disable them so that pins can be remapped.
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	// OSC not used. Remapped PD0 & PD1 for GPIO
	GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
}


// @brief : Init interrupt vector table
//		  : Init system tick for 1 msec interrupt
// @param : none
// @retval: none
void stm32_NVIC_Configuration(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
	// Set the Vector Table base location at 0x20000000
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  // VECT_TAB_FLASH
	// Set the Vector Table base location at 0x08003000 for DFU
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Setup SysTick Timer for 1 msec interrupts */
	if (SysTick_Config(SystemCoreClock / 1000)) /* SystemFrequency is
	 defined in “system_stm32f10x.h” and equal to HCLK frequency */
	{
		/* Capture error */
		//    while (1);
	}
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SYSTICK_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// @brief : To init GPIO pin mode
// @param : GPIOx - Port#, GPIO_Pin - Pin#,
// 			mode-GPIO mode
// 			GPIO_Mode_AIN 			;Analog in
// 			GPIO_Mode_IN_FLOATING 	;input floating
// 			GPIO_Mode_IPD 			;input pulled down
// 			GPIO_Mode_IPU 			;input pulled up
// 			GPIO_Mode_Out_OD		;output open drain
// 			GPIO_Mode_Out_PP		;output push-pull
// 			GPIO_Mode_AF_OD			;alternate function open drain
// 			GPIO_Mode_AF_PP			;alternate function push pull
// @retval: none
void GPIO_Init_Mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode) {
	 GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

#include "Init.h"
extern uint16_t ADC_DMA_Values[2];

/*========================================================
 * RCC_init
 *========================================================
 * Function configures System Clock Source to be an output of PLL.
 * As an input PLL takes HSI signal (16MHz) and converts it into 80MHz signal at the output.
 *
 * Also, function configures clock source for all the necessary peripherals like
 * timers, GPIOs, ADC and USART.
 *
 * @param  None
 * @retval None
 * */
void RCC_init(void){
	RCC -> CFGR |= 0b00 << 21;		// MCO_1 is a PLL output
	RCC -> CFGR |= 0b000 << 24;		// MCO_1 prescaler = 1
	RCC -> CFGR |= 0b00 << 30;		// MCO_2 is a PLL output
	RCC -> CFGR |= 0b000 << 27;		// MCO_2 prescaler = 2

	RCC -> CFGR &= ~RCC_CFGR_SW;	// HSI as System Clock Source
	RCC -> CR &= ~RCC_CR_PLLON;		// PLL OFF
	while(((RCC->CR) & (RCC_CR_PLLRDY)) > 0);	// Wait until PLLRDY=0 (PLL OFF)
	RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; 		// PLLSRC=0. HSI=16MHz is a PLL's input

	/* PLL CONFIGURATION in order to get 80MHz a the output
	* PLL_OUTPUT = HSI * (PLLN/PLLM) * (1/PLLP)
	* 192 < PLLN < 432   =>   PLLN = 300
	* 2  < PLLM < 63     =>   PLLM = 30
	*      				   PLLP = 2
	* PLL_OUTPUT = 16MHz * (300/30) * 1/2 = 80MHz
	*/
	RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLN;  		// Clear PLLN bits
	RCC -> PLLCFGR |= (300 << 6);		  		// PLLN = 300
	RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLM;  		// Clear PLLM bits
	RCC -> PLLCFGR |= 30;  				  		// PLLM = 30
	RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLP;  		// Clear PLLP bits. PLLP = 2 (PLLP bits = 0b00)
	RCC -> CR  |= RCC_CR_PLLON;           		// PLLON = 1
	while (((RCC -> CR) & (1<<25)) == 0 );		// Wait until PLLRDY bit is set (PLL locked)
	RCC -> CFGR &= ~RCC_CFGR_SW;   		  		// Select PLL as System Clock Source (SW=0b10)
	RCC -> CFGR |= RCC_CFGR_SW_1;  		  		// Set bit SW_1
	while(((RCC -> CFGR) & 0b1100) != 0b1000 );	// Wait until System Clock Status is set to PLL (SWS=0b10)
	RCC -> CFGR &= ~(1 << 7);			  		// AHB prescaler = 1
	RCC -> CFGR &= ~(1 << 15);			  		// APB2 Prescaler=1 (AHB not divided)


	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN
					  | RCC_APB1ENR_TIM5EN;		// Enable clock for TIM2, TIM3, TIM4 and TIM5
	RCC -> AHB1ENR |= 0b11111111 | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN; 	// Enable clock for GPIOA/B/C/D/E/H
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN; 	// Enable clock for SYSCFG
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN
					  | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN;	// Enable clock for ADC1, TIM1, TIM9 and TIM10
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_PWREN; 	// Enable clock for USART2 and PWR domain

	PWR -> CR |= PWR_CR_DBP;				// Enable access to BKP domain

	RCC -> BDCR |= RCC_BDCR_RTCSEL_0;		// LSE clock for RTC
	RTC -> ISR &= ~RTC_ISR_TAMP1F;
	RCC -> BDCR |= RCC_BDCR_RTCEN;			// RTC enabled
	RCC -> BDCR |= RCC_BDCR_LSEON;			// Enable LSE

}


/*========================================================
 * GPIO_init
 *========================================================
 *Function configures all the necessary input and output pins of the MCU.
 *
 * @param  None
 * @retval None
 */
void GPIO_init(void){
//**********************OUTPUT STAGE************************
	// TIM4 Channel 1 output configuration
	GPIOB -> MODER |= 0b10 << 12;		// GPIOB_Pin_6 used as alternative function
	GPIOB -> OSPEEDR |= 0b11 << 12;		// GPIOB_Pin_6 HIGH SPEED output
	GPIOB -> OTYPER |= 0b0 << 6;		// GPIOB_Pin_6 is Push-Pull output
	GPIOB -> AFR[0]  |= 0b0010 << 24;	// GPIOB_Pin_6 alternative function TIM4

	// TIM4 Channel 2 output configuration
	GPIOB -> MODER |= (0b10 << 14);		// GPIOB_Pin_7 used as alternative function
	GPIOB -> OSPEEDR |= (0b11 << 14);	// GPIOB_Pin_7 HIGH SPEED output
	GPIOB -> OTYPER |= (0b0 << 7);		// GPIOB_Pin_7 is Push-Pull output
	GPIOB -> AFR[0]  |= (0b0010 << 28);	// GPIOB_Pin_7 alternative function TIM4

	// TIM4 Channel 3 output configuration
	GPIOB -> MODER |= (0b10 << 16);		// GPIOB_Pin_8 used as alternative function
	GPIOB -> OSPEEDR |= (0b11 << 16);	// GPIOB_Pin_8 HIGH SPEED output
	GPIOB -> OTYPER |= (0b0 << 8);		// GPIOB_Pin_8 is Push-Pull output
	GPIOB -> AFR[1]  |= (0b0010 << 0);	// GPIOB_Pin_8 alternative function TIM4

	GPIOB -> MODER |= 0b10 << 18;		// GPIOB_Pin_6 used as alternative function
	GPIOB -> OSPEEDR |= 0b11 << 18;		// GPIOB_Pin_6 HIGH SPEED output
	GPIOB -> OTYPER |= 0b0 << 9;		// GPIOB_Pin_6 is Push-Pull output
	GPIOB -> PUPDR |= 0b00 << 18;
	GPIOB -> AFR[1]  |= 0b0010 << 4;	// GPIOB_Pin_6 alternative function TIM4


//*********** NEW LOWSIDES **********************
	// Outputs used while switching lowside MOSFETS with TIM3 channels
	// W LOWSIDE TIM3 CC1
	// UNUSED
	GPIOC -> MODER |= 0b10 << 12;		// GPIOC_Pin_6 used as alternative function
	GPIOC -> OSPEEDR |= 0b11 << 12;		// GPIOC_Pin_6 HIGH SPEED output
	GPIOC -> OTYPER |= 0b0 << 6;		// GPIOC_Pin_6 is Push-Pull output
	GPIOC -> PUPDR |= 0b00 << 12;
	GPIOC -> AFR[0]  |= 0b0010 << 24;	// GPIOC_Pin_6 alternative function TIM3

	// V LOWSIDE TIM3 CC2
	GPIOC -> MODER |= 0b10 << 14;		// GPIOC_Pin_7 used as alternative function
	GPIOC -> OSPEEDR |= 0b11 << 14;		// GPIOC_Pin_7 HIGH SPEED output
	GPIOC -> OTYPER |= 0b0 << 7;		// GPIOC_Pin_7 is Push-Pull output
	GPIOC -> AFR[0]  |= 0b0010 << 28;	// GPIOC_Pin_7 alternative function TIM1

	// U LOWSIDE TIM3 CC3
	GPIOC -> MODER |= 0b10 << 16;		// GPIOC_Pin_8 used as alternative function
	GPIOC -> OSPEEDR |= 0b11 << 16;		// GPIOC_Pin_8 HIGH SPEED output
	GPIOC -> OTYPER |= 0b0 << 8;		// GPIOC_Pin_8 is Push-Pull output
	GPIOC -> AFR[1]  |= 0b0010 << 0;	// GPIOC_Pin_8 alternative function TIM1

//**********************************
	// Outputs used to switch lowside MOSFETS without any timer
	// W LOWSIDE INPUT
	GPIOB -> MODER |= (0b01 << 0);		// GPIOB_Pin_0 is an output
	GPIOB -> OTYPER |= 0 << 0;			// GPIOB_Pin_0 is Push-Pull output
	GPIOB -> OSPEEDR |= (0b11 << 0);	// GPIOB_Pin_0 is High-Speed output

	// V LOWSIDE INPUT
	GPIOB -> MODER |= (0b01 << 2);		// GPIOB_Pin_1 is an output
	GPIOB -> OTYPER |= 0 << 1;			// GPIOB_Pin_1 is Push-Pull output
	GPIOB -> OSPEEDR |= (0b11 << 2);	// GPIOB_Pin_1 is High-Speed output

	// U LOWSIDE INPUT
	GPIOB -> MODER |= (0b01 << 4);		// GPIOB_Pin_2 is an output
	GPIOB -> OTYPER |= 0 << 2;			// GPIOB_Pin_2 is Push-Pull output
	GPIOB -> OSPEEDR |= (0b11 << 4);	// GPIOB_Pin_2 is High-Speed output

////**********************NUCLEO 64************************
	// MCO2
	GPIOC -> MODER |= (0b10 << 18);		// GPIOA_Pin_5 is an output
	GPIOC -> OTYPER |= 0 << 9;			// GPIOA_Pin_5 is Push-Pull output
	GPIOC -> OSPEEDR |= 0b11 << 18;
	GPIOC -> AFR[1]  |= 0b0000 << 4;	// GPIOC_Pin_8 alternative function TIM1

	// MCO1
	GPIOA -> MODER |= (0b10 << 16);		// GPIOA_Pin_5 is an output
	GPIOA -> OTYPER |= 0 << 9;			// GPIOA_Pin_5 is Push-Pull output
	GPIOA -> OSPEEDR |= 0b11 << 16;
	GPIOA -> AFR[1]  |= 0b0000 << 0;	// GPIOC_Pin_8 alternative function TIM1

	// User diode on Nucleo64 board
	GPIOA -> MODER |= (0b01 << 10);		// GPIOA_Pin_5 is an output
	GPIOA -> OTYPER |= 0 << 5;			// GPIOA_Pin_5 is Push-Pull output
	GPIOA -> OSPEEDR |= (0b11 << 10);	// GPIOA_Pin_5 is High-Speed output

	// User button on Nucleo64 board
	GPIOC -> MODER |= (0b00 << 26);		// GPIOC_Pin_13 is an input
	GPIOC -> OSPEEDR |= (0b11 << 12);	// GPIOC_Pin_13 is High-Speed input

//**********************INPUTS FOR THE COMPARATOR************************
	// Input for EXTI interrupt from W phase
	GPIOB -> MODER |= (0b00 << 4);		// GPIOB_Pin_2 is an input
	GPIOB -> OSPEEDR |= (0b11 << 4);	// GPIOB_Pin_2 is High-Speed input

	// Input for EXTI interrupt from V phase
	GPIOB -> MODER |= (0b00 << 6);		// GPIOB_Pin_3 is an input
	GPIOB -> OSPEEDR |= (0b11 << 6);	// GPIOB_Pin_3 is High-Speed input

	// Input for EXTI interrupt from U phase
	GPIOD -> MODER |= (0b00 << 4);		// GPIOD_Pin_2 is an input
	GPIOD -> OSPEEDR |= (0b11 << 4);	// GPIOD_Pin_2 is High-Speed input

//**********************ADC INPUTS************************
	// ANALOG INPUTS CONFIGURATION
	// Connected to the op-amps outputs
	// ADC1_IN10
	GPIOC -> MODER |= 0b11 << 0;		// GPIOC_Pin_0 is in analog mode

	// ADC1_IN11
	GPIOC -> MODER |= 0b11 << 2;		// GPIOC_Pin_1 is in analog mode

	// ADC1_IN12
	GPIOC -> MODER |= 0b11 << 4;		// GPIOC_Pin_2 is in analog mode


//**********************USART PINS************************
	// TX pin GPIOA_Pin_2
	GPIOA -> MODER |= 0b10 << 4; 		// GPIOA_Pin_2 is an alternative function pin
	GPIOA -> OTYPER |= 0 << 2;			// GPIOA_Pin_2 is push-pull pin
	GPIOA -> AFR[0] |= 0b0111 << 8;		// GPIOA_Pin_2 alternative function is USART 2

	// RX pin GPIOA_Pin_3
	GPIOA -> MODER |= 0b10 << 6; 		// GPIOA_Pin_3 is an alternative function pin
	GPIOA -> PUPDR |= 0b00 << 6;		// GPIOA_Pin_3 is a floating pin
	GPIOA -> AFR[0] |= 0b0111 << 12;	// GPIOA_Pin_3 alternative function is USART 2

//**********************OTHER************************
	// Test output pins configuration
	GPIOC -> MODER |= (0b01 << 20);
	GPIOC -> OTYPER |= 0 << 10;
	GPIOC -> OSPEEDR |= (0b11 << 20);
	GPIOC -> ODR |= 0b0 << 10;

	GPIOC -> MODER |= (0b01 << 22);
	GPIOC -> OTYPER |= 0 << 11;
	GPIOC -> OSPEEDR |= (0b11 << 22);
	GPIOC -> ODR |= 0b0 << 11;

	GPIOC -> MODER |= (0b01 << 24);
	GPIOC -> OTYPER |= 0 << 12;
	GPIOC -> OSPEEDR |= (0b11 << 24);
	GPIOC -> ODR |= 0b0 << 12;
}

/*========================================================
 * TIM_init
 *========================================================
 * Function configures timers and their channels which are used to generate
 * PWM signal using Channels 1/2/3 of TIM4. PWM frequency is set to 10kHz. TIM4 CC4 is
 * used as a trigger for ADC conversion. TIM5 is used to produce an interrupt for the PID
 * regulator. Also TIM2 *is configured in order to create delay(int) function which
 * doesn't use any interrupts.
 *
 * @param  None
 * @retval None
 */
void TIM_init(void){
	// TIM1 configuration
	// TIM1 is clocked with 2*APB2CLK = 80MHz
	// TIM1_EN signal is used to trigger the TIM4 and TIM3 at the same time
	TIM1 -> CR1 &= ~TIM_CR1_CKD;	// Clock division = 1
	TIM1 -> CR1 &= ~TIM_CR1_ARPE;	// TIM4_ARR is not buffered
	TIM1 -> CR1 &= ~TIM_CR1_CMS;    // Edge-aligned mode
	TIM1 -> CR1 &= ~TIM_CR1_DIR;	// Counter used as up-counter
	TIM1 -> CR1 &= ~TIM_CR1_OPM;    // Counter is not stopped at update event
	TIM1 -> CR1 |= TIM_CR1_URS;		// Only counter overflow generates and update interrupt
	TIM1 -> CR1 &= ~TIM_CR1_UDIS;	// Update event enabled

	TIM1 -> CR2 |= 0b001 << 4;		// CNT_EN is connected do TRGO (Trigger signal for other timers)

	TIM1 -> EGR |= TIM_EGR_UG;		// Registers update and re-initialization of the counter ENABLED

	TIM1 -> PSC = 80 - 1;			// Prescaler = 79 (1Mhz at its output)
	TIM1 -> ARR = 65500 - 1;		// Auto-reload register = 49 999;
	TIM1 -> CNT = 0;				// Counter = 0


//=========================================================================
	// TIM4 configuration which is used to generate a PWM signal for the highside switches.
	// Timer is clocked with 40MHz signal.
	// TIM4 channels are the outputs for the HIGHSIDES.
	// TIM4 generates 10kHz PWM signal.
	TIM4 -> CR1 &= ~TIM_CR1_CKD;	// Clock division = 1
	TIM4 -> CR1 &= ~TIM_CR1_ARPE;	// TIM4_ARR is buffered
	TIM4 -> CR1 &= ~TIM_CR1_CMS;    // Edge-aligned mode
	TIM4 -> CR1 &= ~TIM_CR1_DIR;	// Counter used as up-counter
	TIM4 -> CR1 &= ~TIM_CR1_OPM;    // Counter is not stopped at update event
	TIM4 -> CR1 |= TIM_CR1_URS;		// Only counter overflow generates and update interrupt
	TIM4 -> CR1 &= ~TIM_CR1_UDIS;	// Update event enabled

	TIM4 -> EGR |= TIM_EGR_UG;		// Registers update and re-initialization of the counter ENABLED
	TIM4 -> EGR |= TIM_EGR_CC1G;	// Channel 1 event generation ENABLED (setting CC1F flag ENABLED)
	TIM4 -> EGR |= TIM_EGR_CC2G;	// Channel 2 event generation ENABLED (setting CC2F flag ENABLED)
	TIM4 -> EGR |= TIM_EGR_CC3G;	// Channel 3 event generation ENABLED (setting CC3F flag ENABLED)
	TIM4 -> EGR |= TIM_EGR_CC4G;	// Channel 4 event generation ENABLED (setting CC4F flag ENABLED)

	TIM4 -> PSC = 20 - 1;			// Prescaler = 19 (2Mhz at its output)
	TIM4 -> ARR = 200 - 1;			// Auto-reload register = 199;
	TIM4 -> CNT = 0;				// Counter = 0

	// TIM4 Channel 1 configuration
	TIM4 -> CCMR1 &= ~TIM_CCMR1_CC1S;// CC1 channel configured as output
	TIM4 -> CCMR1 &= ~TIM_CCMR1_OC1M;// Resets OC1M bits (output compare mode configuration)
	TIM4 -> CCMR1 |= (0b110 << 4);   // PWM Mode. CC1 active as long as CNT<CCR1
	TIM4 -> CCMR1 |= TIM_CCMR1_OC1PE;// CCR1 can be written at any time. Value taken in account instantly
	TIM4 -> CCR1 = 25;				 // Value to be compared with the counter

	// TIM4 Channel 2 configuration
	TIM4 -> CCMR1 &= ~TIM_CCMR1_CC2S;// CC2 channel configured as output
	TIM4 -> CCMR1 &= ~TIM_CCMR1_OC2M;// Resets OC2M bits (output compare mode configuration)
	TIM4 -> CCMR1 |= (0b110 << 12);  // PWM Mode. CC2 active as long as CNT<CCR2
	TIM4 -> CCMR1 |= TIM_CCMR1_OC2PE;// CCR2 can be written at any time. Value taken in account instantly
	TIM4 -> CCR2 = 50;				 // Value to be compared with the counter

	// TIM4 Channel 3 configuration
	TIM4 -> CCMR2 &= ~TIM_CCMR2_CC3S;// CC3 channel configured as output
	TIM4 -> CCMR2 &= ~TIM_CCMR2_OC3M;// Resets OC3M bits (output compare mode configuration)
	TIM4 -> CCMR2 |= (0b110 << 4);   // PWM Mode. CC3 active as long as CNT<CCR3
	TIM4 -> CCMR2 |= TIM_CCMR2_OC3PE;// CCR3 can be written at any time. Value taken in account instantly
	TIM4 -> CCR3 = 50;				 // Value to be compared with the counter

	// TIM4 Channel 4 configuration
	TIM4 -> CCMR2 &= ~TIM_CCMR2_CC4S;// CC4 channel configured as output
	TIM4 -> CCMR2 &= ~TIM_CCMR2_OC4M;// Resets OC4M bits (output compare mode configuration)
	TIM4 -> CCMR2 |= (0b110 << 12);  // PWM Mode. CC4 active as long as CNT<CCR4
	TIM4 -> CCMR2 |= TIM_CCMR2_OC4PE;// CCR4 can be written at any time. Value taken in account instantly
	TIM4 -> CCR4 = 100;				 // Value to be compared with the counter

	TIM4 -> CCER &= ~TIM_CCER_CC1P;	 // Channel 1 active high polarity
	TIM4 -> CCER &= ~TIM_CCER_CC2P;	 // Channel 2 active high polarity
	TIM4 -> CCER &= ~TIM_CCER_CC3P;	 // Channel 3 active high polarity
	TIM4 -> CCER &= ~TIM_CCER_CC4P;	 // Channel 4 active high polarity

	TIM4 -> CCR1 = 0;				 // TIM4 CC1 Duty_Cycle=0%
	TIM4 -> CCR2 = 0;				 // TIM4 CC2 Duty_Cycle=0%
	TIM4 -> CCR3 = 0;				 // TIM4 CC3 Duty_Cycle=0%

	TIM4 -> SMCR |= 0b110 << 0;		 // Trigger slave mode enabled
	TIM4 -> SMCR |= 0b00 << 4;		 // ITR0 (CNT_EN from TIM1) is the trigger for the timer


//=========================================================================
	// TIM3 configuration
	// Timer is triggered with 40MHz signal
	// TIM3 CC1 is used as a trigger for A/D conversion for ADC1
	// TIM3 generates 100kHz PWM signal
	TIM3 -> CR1 &= ~TIM_CR1_CKD;	// Clock division = 1
	TIM3 -> CR1 &= ~TIM_CR1_ARPE;	// TIM3_ARR is buffered
	TIM3 -> CR1 &= ~TIM_CR1_CMS;    // Edge-aligned mode
	TIM3 -> CR1 &= ~TIM_CR1_DIR;	// Counter used as up-counter
	TIM3 -> CR1 &= ~TIM_CR1_OPM;    // Counter is not stopped at update event
	TIM3 -> CR1 |= TIM_CR1_URS;		// Only counter overflow generates and update interrupt
	TIM3 -> CR1 &= ~TIM_CR1_UDIS;	// Update event enabled

	TIM3 -> EGR |= TIM_EGR_UG;		// Registers update and re-initialisation of the counter ENABLED
	TIM3 -> EGR |= TIM_EGR_CC1G;	// Channel 1 event generation ENABLED (setting CC1F flag ENABLED)
	TIM3 -> EGR |= TIM_EGR_CC2G;	// Channel 2 event generation ENABLED (setting CC2F flag ENABLED)
	TIM3 -> EGR |= TIM_EGR_CC3G;	// Channel 3 event generation ENABLED (setting CC3F flag ENABLED)
	TIM3 -> EGR |= TIM_EGR_CC4G;	// Channel 4 event generation ENABLED (setting CC4F flag ENABLED)

	TIM3 -> PSC = 5 - 1;			// Prescaler = 4 (10Mhz at its output)
	TIM3 -> ARR = 80 - 1;			// Auto-reload register = 80;
	TIM3 -> CNT = 0;				// Counter = 0

	// TIM3 Channel 1 configuration
	TIM3 -> CCMR1 &= ~TIM_CCMR1_CC1S;// CC1 channel configured as output
	TIM3 -> CCMR1 &= ~TIM_CCMR1_OC1M;// Resets OC1M bits (output compare mode configuration)
	TIM3 -> CCMR1 |= (0b110 << 4);   // PWM Mode. CC1 active as long as CNT<CCR1
	TIM3 -> CCMR1 |= TIM_CCMR1_OC1PE;// CCR1 can be written at any time. Value taken in account instantly
	TIM3 -> CCR1 = 25;				 // Value to be compared with the counter

	// TIM3 Channel 2 configuration
	TIM3 -> CCMR1 &= ~TIM_CCMR1_CC2S;// CC2 channel configured as output
	TIM3 -> CCMR1 &= ~TIM_CCMR1_OC2M;// Resets OC2M bits (output compare mode configuration)
	TIM3 -> CCMR1 |= (0b110 << 12);  // PWM Mode. CC2 active as long as CNT<CCR2
	TIM3 -> CCMR1 |= TIM_CCMR1_OC2PE;// CCR2 can be written at any time. Value taken in account instantly
	TIM3 -> CCR2 = 50;				 // Value to be compared with the counter

	// TIM3 Channel 3 configuration
	TIM3 -> CCMR2 &= ~TIM_CCMR2_CC3S;// CC3 channel configured as output
	TIM3 -> CCMR2 &= ~TIM_CCMR2_OC3M;// Resets OC3M bits (output compare mode configuration)
	TIM3 -> CCMR2 |= (0b110 << 4);   // PWM Mode. CC3 active as long as CNT<CCR3
	TIM3 -> CCMR2 |= TIM_CCMR2_OC3PE;// CCR3 can be written at any time. Value taken in account instantly
	TIM3 -> CCR3 = 50;				 // Value to be compared with the counter

	// TIM3 Channel 4 configuration
	TIM3 -> CCMR2 &= ~TIM_CCMR2_CC4S;// CC4 channel configured as output
	TIM3 -> CCMR2 &= ~TIM_CCMR2_OC4M;// Resets OC4M bits (output compare mode configuration)
	TIM3 -> CCMR2 |= (0b110 << 12);  // PWM Mode. CC4 active as long as CNT<CCR4
	TIM3 -> CCMR2 |= TIM_CCMR2_OC4PE;// CCR4 can be written at any time. Value taken in account instantly
	TIM3 -> CCR4 = 50;				 // Value to be compared with the counter

	TIM3 -> CCER &= ~TIM_CCER_CC1P;	 // Channel 1 active high polarity
	TIM3 -> CCER &= ~TIM_CCER_CC2P;	 // Channel 2 active high polarity
	TIM3 -> CCER &= ~TIM_CCER_CC3P;	 // Channel 3 active high polarity
	TIM3 -> CCER &= ~TIM_CCER_CC4P;	 // Channel 4 active high polarity

	TIM3 -> CCR1 = TIM3->ARR/2;		// TIM3 CC1 Duty_Cycle=50%
	TIM3 -> CCR2 = 100;				// TIM3 CC2 Duty_Cycle=100%
	TIM3 -> CCR3 = 100;				// TIM3 CC3 Duty_Cycle=100%

	TIM3 -> SMCR |= 0b110 << 0;		// Trigger slave mode enabled
	TIM3 -> SMCR |= 0b00 << 4;		// ITR0 (CNT_EN from TIM1) is the trigger for the timer

//=========================================================================
	// TIM2 configuration
	// Counter overloads after 10 seconds
	// Timer is triggered with 40MHz signal
	// Counter is used by the delay_ms(int) function.
	TIM2 -> CR1 &= ~TIM_CR1_CKD;	// Clock division = 1
	TIM2 -> CR1 &= ~TIM_CR1_ARPE;	// TIM2_ARR is buffered
	TIM2 -> CR1 &= ~TIM_CR1_CMS;    // Edge-aligned mode
	TIM2 -> CR1 &= ~TIM_CR1_DIR;	// Counter used as up-counter
	TIM2 -> CR1 &= ~TIM_CR1_OPM;    // Counter is not stopped at update event
	TIM2 -> CR1 &= ~TIM_CR1_UDIS;	// Update enabled

	TIM2 -> DIER |= TIM_DIER_UIE;
	TIM2 -> EGR |= TIM_EGR_UG;

	TIM2 -> PSC = 40000 - 1;		// Prescaler = 39 999 (1kHz at its output)
	TIM2 -> ARR = 10000 - 1;		// Auto-reload register = 9 999;
	TIM2 -> CNT = 0;				// Counter = 0

	TIM2 -> CR1 |= TIM_CR1_CEN;		 // Enable counter

//=========================================================================
	// TIM5 configuration
	// Timer is triggered with 40MHz signal
	// Timer used to synchronize the PI controller's work.
	// Interrupt generation frequency = 2kHz
	TIM5 -> CR1 &= ~TIM_CR1_CKD;	// Clock division = 1
	TIM5 -> CR1 &= ~TIM_CR1_ARPE;	// TIM5_ARR is not buffered
	TIM5 -> CR1 &= ~TIM_CR1_CMS;    // Edge-aligned mode
	TIM5 -> CR1 &= ~TIM_CR1_DIR;	// Counter used as upcounter
	TIM5 -> CR1 &= ~TIM_CR1_OPM;    // Counter is not stopped at update event
	TIM5 -> CR1 |= TIM_CR1_URS;		// Only counter overflow generates and update interrupt
	TIM5 -> DIER |= TIM_DIER_UIE;	// Update interrupt enabled

	TIM5 -> PSC = 20 - 1;			// Prescaler = 19 (2MHz at its output)
	int PI_freq = Bw;
	if (PI_freq == 1000){
		TIM5 -> ARR = 2000 - 1;			// Auto-reload register = 999;
	}else if(PI_freq == 2000){
		TIM5 -> ARR = 1000 - 1;
	}else TIM5 -> ARR = 1000 - 1;
	TIM5 -> CNT = 0;				// Counter = 0

	TIM5 -> CR1 |= TIM_CR1_CEN;		 // Enable counter

//=========================================================================
	// TIM9 configuration
	// Timer is triggered with 80MHz signal
	TIM9 -> CR1 &= ~TIM_CR1_CKD;	// Clock division = 1
	TIM9 -> CR1 &= ~TIM_CR1_ARPE;	// TIM9_ARR is not buffered
	TIM9 -> CR1 |= TIM_CR1_URS;		// Only counter overflow generates and update interrupt
	TIM9 -> DIER |= TIM_DIER_UIE;	// Update interrupt enabled

	TIM9 -> EGR |= TIM_EGR_UG;

	TIM9 -> PSC = 80 - 1;			// Prescaler = 79 (1MHz at its output)
	TIM9 -> ARR = 60000 - 1;		// Auto-reload register = 59 999;
	TIM9 -> CNT = 0;				// Counter = 0

	TIM9 -> CR1 |= TIM_CR1_CEN;		 // Enable counter
//=========================================================================
	// TIM10 configuration
	// Timer is triggered with 80MHz signal
	// TIM10 is used to calculate the time between two consecutive
	// rotor's zero crossing
	TIM10 -> CR1 &= ~TIM_CR1_CKD;		// Clock division = 1
	TIM10 -> CR1 &= ~TIM_CR1_ARPE;		// TIM10_ARR is buffered
	TIM10 -> CR1 |= TIM_CR1_URS;		// Only counter overflow generates and update interrupt
	TIM10 -> DIER |= TIM_DIER_UIE;		// Update interrupt enabled

	TIM10 -> DIER |= TIM_DIER_CC1IE;

	TIM10 -> EGR |= TIM_EGR_UG;

	TIM10 -> CCMR1 |= 0b110 << 4;		// CC1 PWM mode

	TIM10 -> CCMR1 &= ~TIM_CCMR1_CC1S;

	TIM10 -> CCR1 = 50;


	TIM10 -> PSC = 80 - 1;				// Prescaler = 79 (1MHz at its output)
	TIM10 -> ARR = 500 - 1;				// Auto-reload register = 499;
	TIM10 -> CNT = 0;					// Counter = 0

	TIM10 -> CR1 |= TIM_CR1_CEN;		// Enable counter
}

/*========================================================
 * NVIC_init
 *========================================================
 *  Interrupts configuration.
 *
 *  @param  None
 *  @retval None
 */
void NVIC_init(void){
	// Interrupt configuration from user button on Nucleo64 board (GPIOC_Pin_13)
	SYSCFG -> EXTICR[3] |= (0b0010 << 4);	// GPIOC_Pin_13 is the source input for EXTI external interrupt
	EXTI -> RTSR &= ~EXTI_RTSR_TR13;		// Rising edge trigger disabled
	EXTI -> FTSR |= EXTI_FTSR_TR13;			// Falling edge trigger enabled
	EXTI -> IMR |= EXTI_IMR_MR13;			// Interrupt request from line 13 not masked
	NVIC_SetPriority(EXTI15_10_IRQn, 3);	// EXTI13 interrupt priority=1
	NVIC_EnableIRQ(EXTI15_10_IRQn);			// Enable EXTI interrupts for lines 15:10

	// Interrupt configuration from W phase (GPIOH_Pin_0)
	SYSCFG -> EXTICR[1] |= (0b0001 << 0);	// GPIOH_Pin_0 is the source input for EXTI internal interrupt
	EXTI -> RTSR |= EXTI_RTSR_TR4;			// Rising edge trigger enabled
	EXTI -> FTSR |= EXTI_FTSR_TR4;		 	// Falling edge trigger enabled
	EXTI -> IMR |= EXTI_IMR_MR4;		 	// Interrupt request from line 0 not masked
	NVIC_SetPriority(EXTI4_IRQn, 0);

	// Interrupt configuration from V phase (GPIOH_Pin_1)
	SYSCFG -> EXTICR[0] |= (0b0001 << 12);	// GPIOH_Pin_1 is the source input for EXTI internal interrupt
	EXTI -> RTSR |= EXTI_RTSR_TR3;		 	// Rising edge trigger enabled
	EXTI -> FTSR |= EXTI_FTSR_TR3;		 	// Falling edge trigger enabled
	EXTI -> IMR |= EXTI_IMR_MR3;		 	// Interrupt request from line 1 not masked
	NVIC_SetPriority(EXTI3_IRQn, 0);	 	// EXTI1 interrupt maximum priority

	// Interrupt configuration from U phase (GPIOD_Pin_2)
	SYSCFG -> EXTICR[0] |= (0b0011 << 8);	// GPIOD_Pin_2 is the source input for EXTI internal interrupt
	EXTI -> RTSR |= EXTI_RTSR_TR2;		 	// Rising edge trigger enabled
	EXTI -> FTSR |= EXTI_FTSR_TR2;		 	// Falling edge trigger enabled
	EXTI -> IMR |= EXTI_IMR_MR2;		 	// Interrupt request from line 2 not masked
	NVIC_SetPriority(EXTI2_IRQn, 0);	 	// EXTI2 interrupt maximum priority

	// Interrupt configuration for the measurement start
	SYSCFG -> EXTICR[1] |= (0b0010 << 4);	// GPIOC_Pin_5 is the source input for EXTI internal interrupt
	EXTI -> RTSR |= EXTI_RTSR_TR5;		 	// Rising edge trigger enabled
	EXTI -> IMR |= EXTI_IMR_MR5;		 	// Interrupt request from line 0 not masked
	NVIC_SetPriority(EXTI9_5_IRQn, 3);

	NVIC_SetPriority(TIM5_IRQn, 1);			// TIM5 interrupt priority=1

	NVIC_SetPriority(USART2_IRQn, 1);	 	// USART2 interrupt priority
	NVIC_EnableIRQ(USART2_IRQn);		 	// Enable interrupt from USART2

	NVIC_SetPriority(ADC_IRQn, 3);		 	// ADC1 interrupt priority
	NVIC_EnableIRQ(ADC_IRQn);				// Enable interrupt from ADC1

	NVIC_SetPriority(DMA2_Stream0_IRQn, 3);	// DMA2_Stream0 interrupt priority
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);		// Enable interrupt from DMA2_Stream0




}

/*========================================================
 * ADC_init
 *========================================================
 * Function configures the ADC and its channels which are used to measure
 * the voltage drop on shunt resistors (output of the INA240PW op-amp).
 * The measurement is triggered by TIM3 CC1 event.
 * Only one measurement is done in a sequence.
 * Sampling time for all the channels is set for 15 clock cycles.
 * When the conversion is finished interrupt is generated in order to extract
 * the value from DataRegister and allow further processing.
 */
void ADC_init(void){
	// ADC Clock configuration (20MHz)
	ADC1 -> CR2 &= ~ADC_CR2_ADON;		// ADC OFF
	ADC -> CCR |= 0b01 << 16;			// ADC prescaler = 4. ADC_CLK = APB2_CLK/4 = 20Mhz


	//ADC1 -> CR1 &= ~ADC_CR1_DISCNUM;		// One conversion is done in discontinuous mode
	//ADC1 -> CR1 |= ADC_CR1_RES_0;			// ADC resolution = 10 bits
	ADC1 -> CR1 |= ADC_CR1_DISCNUM_0;		// Two conversions are done in discontinous mode

	ADC1 -> CR1 |= ADC_CR1_DISCEN;			// Discontinuous mode enable
//

	ADC1 -> CR1 |= ADC_CR1_SCAN;			// Scan mode on regular channels enabled
	//ADC1 -> CR1 |= ADC_CR1_EOCIE;			// Interrupt generation enable

	ADC1 -> CR1 |= ADC_CR1_OVRIE;			// Overrun interrupt enabled
	//ADC1 -> CR1 |= ADC_CR1_EOCIE;			// End of conversion interrupt enabled
	//ADC1 -> CR2 |= ADC_CR2_DDS;				// DMA requests are made as long as data
											// is converted

	ADC1 -> SMPR1 |= ADC_SMPR1_SMP10_0;		// 15 cycles sampling time for IN10
	ADC1 -> SMPR1 |= ADC_SMPR1_SMP11_0;		// 15 cycles sampling time for IN11
	ADC1 -> SMPR1 |= ADC_SMPR1_SMP12_0;		// 15 cycles sampling time for IN12

	//ADC1 -> SQR1 &= ~ADC_SQR1_L;			// Only one conversion is done in the sequence

	ADC1 -> SQR1 |= ADC_SQR1_L_0;			// Two conversion done in a sequence
	ADC1 -> SQR3 = 0;
	ADC1 -> SQR3 |= 10 << 0;				// IN10 converted as the first channel
	ADC1 -> SQR3 |= 11 << 5;				// IN11 is converted as the second one
	ADC1 -> SQR3 |= 12 << 10;				// IN12 is converted as the third one


	ADC1 -> CR2 &= ADC_CR2_EXTSEL;
	ADC1 -> CR2 |= ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 |ADC_CR2_EXTSEL_2;	// TIM3 CC1 is the trigger for conversion
	ADC1 -> CR2 &= ~ADC_CR2_EXTEN;						// Clear the bits for trigger slope selection
	ADC1 -> CR2 |=  ADC_CR2_EXTEN_0;					// Trigger rising edge
	ADC1 -> CR2 &= ~ADC_CR2_ALIGN;						// Right data alignment
	//ADC1 -> CR2 |= ADC_CR2_CONT;						// Continous mode one

	ADC1 -> CR2 |= ADC_CR2_EOCS;						// EOC bit is set after ever finished conversion
	ADC1 -> CR2 |= ADC_CR2_DMA;							// DMA Mode enabled

	ADC1 -> CR2 |= ADC_CR2_ADON;						// ADC ON



}

/*========================================================
 * USART_init
 *========================================================
 * UART interface configuration. USART2 is used by MCU.
 * Communication with the MCU is possible via USB cable connected
 * to the PC.
 * USART2 is clocked with 20MHz.
 *
 * @param  None
 * @retval None
 */
void USART_init(void){
	USART2 -> BRR  = 20000000L/115200L;                // Baudrate = 115200
	USART2 -> CR1 |= (USART_CR1_RE | USART_CR1_TE);    // RX, TX enable
	USART2 -> CR1 |= USART_CR1_RXNEIE;				   // Enable interrupt

	USART2 -> CR1 |= USART_CR1_UE;                     // USART enable

}

/*========================================================
 * DMA_init
 *========================================================
 * Function configures DMA which is used to transfer data from
 * ADC1->DR to ADC_DMA_Values[] table. DMA works in direct circular mode
 * and that means the destination is a buffer. In one cycle two values are
 * transferred. After that the destination address is set to the first item of
 * ADC_DMA_Values[] table and after the request data is transferred again.
 *
 * DMA2_Stream0 is used to transfer the data between the peripheral to memory.
 *
 * @param  None
 * @retval None
 */
void DMA_init(void){

	DMA2_Stream0 -> CR &= ~DMA_SxCR_EN;					// Disable DMA2_Stream0
	while(DMA2_Stream0 -> CR & DMA_SxCR_EN);
	DMA2_Stream0 -> PAR = (uint32_t)&ADC1->DR;			// Peripheral address
	DMA2_Stream0 -> M0AR = (uint32_t)ADC_DMA_Values;	// Memory address
	DMA2_Stream0 -> NDTR = 50;


	DMA2_Stream0 -> CR &= ~DMA_SxCR_CT;			// Mem0 selected
	DMA2_Stream0 -> CR &= ~DMA_SxCR_CHSEL;		// Channel 0 (ADC1) selected
	DMA2_Stream0 -> CR |= DMA_SxCR_PSIZE_0;		// Half-word peripheral data size
	DMA2_Stream0 -> CR |= DMA_SxCR_MSIZE_0;		// Half-word memory data size
	DMA2_Stream0 -> CR |= DMA_SxCR_MINC;		// Memory address is incremented

	DMA2_Stream0 -> FCR &= ~DMA_SxFCR_DMDIS;	// Direct mode enabled

	DMA2_Stream0 -> CR &= ~DMA_SxCR_DIR;		// Peripheral to memory transfer direction
	DMA2_Stream0 -> CR |= DMA_SxCR_CIRC;		// Circular mode enabled

	DMA2_Stream0 -> CR |= DMA_SxCR_TCIE;		// Transfer complete interrupt enable
	DMA2 -> LIFCR |= 0b111101;					// Clear flags
	DMA2_Stream0 -> CR |= DMA_SxCR_EN;

}


/*========================================================
 * IWDG_init
 *========================================================
 * Function configures Independent Watchdog. Its functionality is
 * used in situation in which motor suddenly stops to spin
 * because of some external force (e.g. blocking the propeller).
 * IWDG is reseted in the same interrupt handler which inform the MCU about
 * rotor's zero crossing so when the motor stops to spin no zero-crossing
 * interrupts are generated, the MCU resets and the current through the
 * motor ceases to flow.
 *
 * @param  None
 * @retval None
 */
void WWDG_init(void){
	IWDG -> RLR |= 0xFF;	// IWDG reload register
	IWDG -> PR |= 0;		// Set the prescaler
}




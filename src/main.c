#include "BLDC_lib.h"
#include "GPIO_lib.h"
#include "Init.h"
#include "SVM.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "arm_common_tables.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "arm_math.h"
#include "SVM.h"


/********************GLOBAL VARIABLES*********************/
int Rotation_Time = 0;			// Variable containing time of one whole spin in us
int Set_Rotation_Time = 14000;	// Variable containing desired rotation time in us
float RPM = 0;					// Actual RPM (RotatesPerMinute) value
float Set_RPM = 0;				// Desired RPM
int Error, Integral = 0;		// Variables for the PI regulator
long double PI_Out = 0;			// PI regulator output
int Reference_Commutation_Step = 0;	// Variable used as reference to measure rotor's speed


float W = 0;					// Rotation speed rad/s
float Integral_W = 0;			// Integral from rotation speed

uint8_t PI_ON = DISABLE;		// Variable used to turn on the PI regulator
uint8_t Calculate_PI = DISABLE;	// Variable used to trigger the PI regulator
uint8_t Mode = IDLE;			// Variable describing state of algorithm
uint8_t Measure_Speed = 1;		// Variable used to trigger rotation time measurement

int Commutation = 0;			// Variable storing actual commutation step
uint16_t Send_Data = 0;			// Variable used to start sending actual motor statistics
uint8_t Begin_Measurement = DISABLE;


uint16_t ADC_DMA_Values[2];		// Actual ADC value from channels 10 and 11

float32_t Iw;		// W phase current
float32_t Iv;		// V phase current
float32_t Iu;		// U phase current


char str1[10];
char str [10];

int main(void)
{
	RCC_init();
	GPIO_init();
	TIM_init();
	NVIC_init();
	ADC_init();
	DMA_init();
	USART_init();

	send_string("\n\r");
	RCC_ClocksTypeDef clock;
	RCC_GetClocksFreq(&clock);
	sprintf(str1, "%d", (int)clock.SYSCLK_Frequency);
	send_string("SysClk: "); send_string(str1); send_string("Hz\n\r");
	sprintf(str1, "%d", (int)clock.HCLK_Frequency);
	send_string("HCLK: "); send_string(str1); send_string("Hz\n\r");
	sprintf(str1, "%d", (int)clock.PCLK1_Frequency);
	send_string("APB1CLK: "); send_string(str1); send_string("Hz\n\r");
	sprintf(str1, "%d", (int)clock.PCLK2_Frequency);
	send_string("APB2CLK: "); send_string(str1); send_string("Hz\n\r");

	LED_OFF;

	// ENABLE TIMERS
	TIM1 -> CR1 |= TIM_CR1_CEN;
	TIM1 -> CCR1 = 100;
	TIM3 -> CCER |= 0x0111;
	TIM3 -> CCR1 = TIM3->ARR/2;


	// SysTick timer configuration
	if(SysTick_Config(SystemCoreClock/1000000)){ // Interrupt comes each 1us
		while(1);
	}

	Switch_Off_Output_Stage();


	send_string("\n\rSTART\n\rPress button or 'a' to align the motor. \n\r");

	send_string("\n\r");
	send_string("########################################\n\r");

	while(((GPIOC->IDR >> 13) & 1) != 0){	// Wait for button to be pressed
		Toggle_Pin(GPIOA, 5);				// Blink led in interval of 128ms
		delay_ms(128);
		if (Mode == ALIGN) break;			// If user changed (via UART) Mode to ALIGN, break the loop
	}

	// ALIGNMENT procedure
	Mode = ALIGN;
	Alignment();
	send_string("Alignment DONE. Press button or 'b' to start the motor. \n\r");

	Switch_Off_Output_Stage();
	delay_ms(500);			// Wait 0.5s
	Commutation = 1;

	while(((GPIOC->IDR >> 13) & 1) != 0){	// Wait for button to be pressed
		Toggle_Pin(GPIOA, 5);				// Blink led in interval of 64ms
		delay_ms(64);
		if (Mode == START) break;			// If user changed (via UART) Mode to Start, break the loop
	}
	delay_ms(300);							// Wait 300ms

	// START procedure
	Mode = START;
	Start();
	Mode = RUN;
	Commutate(Commutation);
	send_string("Press button or 'p' to turn on the PI regulator. \n\r");

	Commutate(Commutation);

	TIM4 -> CCR1 = 55;
	TIM4 -> CCR2 = 55;
	TIM4 -> CCR3 = 55;

	LED_OFF;

	/****************MAIN LOOP****************/
	while (1 && Mode == RUN)
	{
		PI_regulator();

		Commutate(Commutation);

		// CALCULATE PHASE CURRENTS
		Iw = (((float)ADC_DMA_Values[0] - 2047.5f)*3.3/(AMPLIFICATION*4096.0))/R_SHUNT;
		Iv = (((float)ADC_DMA_Values[1] - 2047.5f)*3.3/(AMPLIFICATION*4096.0))/R_SHUNT;
		Iu = -Iw - Iv;

		Commutate(Commutation);

		// SEND DATA VIA UART EACH 0.5s
		if(Send_Data > 1000){
			// PRINT USER DATA INPUT
			sprintf(str, "%d", Set_Rotation_Time);
			Commutate(Commutation);
			send_string("Set rotation time: "); send_string(str); send_string("us\n\r");
			Set_RPM = 60000.0/((float)Set_Rotation_Time/1000.0);
			sprintf(str, "%d", (int)Set_RPM);
			send_string("Set RPM: "); send_string(str); send_string("\n\r");
			send_string("\n\r");

			Commutate(Commutation);

			// PRINT MEASURED DATA
			W = 2000000.0*Pi/(float)Rotation_Time;		// Calculate angular velocity
			sprintf(str, "%d", (int)(W));
			send_string("W: "); send_string(str); send_string("rad/s\n\r");	// Print angular velocity
			RPM = 60.0*1000000.0/(float)Rotation_Time;	// Calculate RPM
			sprintf(str, "%d", (int)RPM);
			send_string("RPM: "); send_string(str); send_string("\n\r");	// Print RPM
			sprintf(str, "%d", Rotation_Time);
			send_string("Rotation Time: ");
			send_string(str); send_string("us\n\r");	// Print Rotation Time
			send_string("\n\r");
			sprintf(str, "%d", (int)TIM4->CCR1);		// Print actual duty cycle
			send_string("PWM: "); send_string(str); send_string("/200 \n\r");
			sprintf(str, "%d", (int)PI_Out);			// Print PI regulator output duty
			send_string("PI regulator output: ");
			send_string(str); send_string("/200 \n\r");

			Commutate(Commutation);

			// PRINT MEASURED CURRENTS
			send_string("-----CURRENTS-----\n\r");
			sprintf(str, "%d", (int)((float)Iw*1000.0));
			send_string("W_Current: ");
			send_string(str); send_string("mA\n\r");
			sprintf(str, "%d", (int)((float)Iv*1000.0));
			send_string("V_Current: ");
			send_string(str); send_string("mA\n\r");
			sprintf(str, "%d", (int)((float)Iu*1000.0));
			send_string("U_Current: ");
			send_string(str); send_string("mA\n\r");
			send_string("########################################\n\r");
			Send_Data = 0;
		}
	}
	PI_ON = ENABLE;		// Enable PI regulator

	// Set_Rotation_Time is equal to measured time to make sure that
	// the motor won't accelerate after switching on the PI regulator
	// if it wasn't switched on earlier
	Set_Rotation_Time = Rotation_Time;

	NVIC_DisableIRQ(TIM5_IRQn);
	// STOPING THE MOTOR
	int y=1;
	while(Mode == STOP){
		// Decrease speed
		Set_Rotation_Time += 1.5*(double)y*500.0;
		TIM2->CNT=0;
		while(TIM2->CNT < 500){
			Calculate_PI = ENABLE;
			PI_regulator();
			send_string("--- ");
		}
		y++;
		if(Set_Rotation_Time > 28500){
			NVIC_DisableIRQ(EXTI4_IRQn);
			NVIC_DisableIRQ(EXTI3_IRQn);
			NVIC_DisableIRQ(EXTI2_IRQn);
			Commutation = 0;
			Switch_Off_Output_Stage();
			break;
		}
	}
	send_string(" \n\r");
	send_string("########## MOTOR STOPPED ########## \n\r");
	NVIC_DisableIRQ(EXTI4_IRQn);
	NVIC_DisableIRQ(EXTI3_IRQn);
	NVIC_DisableIRQ(EXTI2_IRQn);
	Commutation = 0;
	Switch_Off_Output_Stage();
	Change_Duty_Cycle(0);
	Mode = IDLE;



	while(1){	// Wait for button to be pressed
		Toggle_Pin(GPIOA, 5);				// Blink led in interval of 128ms
		delay_ms(500);
	}


}







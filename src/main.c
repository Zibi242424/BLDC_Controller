#include "BLDC_lib.h"
#include "GPIO_lib.h"
#include "UART_lib.h"
#include "Init.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
//#include "stm32f4xx_rcc.h"
//#include "misc.h"


/******************** GLOBAL VARIABLES *********************/
int Rotation_Time = 0;				// Variable containing time of one whole spin in us
int Set_Rotation_Time = 16000;		// Variable containing desired rotation time in us
float RPM = 0;						// Actual RPM (RotatesPerMinute) value
float Set_RPM = 0;					// Desired RPM
int Error, Integral = 0;			// Variables for the PI regulator
long double PI_Out = 0;				// PI regulator output
int Reference_Commutation_Step = 0;	// Variable used as reference to measure rotor's speed


uint8_t Display = DISABLE;			// Flag describing whether to print Manual via UART

float W = 0;						// Rotation speed [rad/s]
float Integral_W = 0;				// Integral from rotation speed

uint8_t PI_ON = DISABLE;			// Variable used to turn on the PI regulator
uint8_t Calculate_PI = DISABLE;		// Variable used to trigger the PI regulator
uint8_t Mode = IDLE;				// Variable describing state of algorithm
uint8_t Measure_Speed = 1;			// Variable used to trigger rotation time measurement

int Commutation = 0;				// Variable storing actual commutation step
uint16_t Send_Data = 0;				// Variable used to start sending actual motor statistics


uint16_t ADC_DMA_Values[2];			// Actual ADC value from channels 10 and 11

float32_t Iw;						// W phase current
float32_t Iv;						// V phase current
float32_t Iu;						// U phase current


char str1[10];		// Strings used in sprintf function
char str [10];

void Display_Current_Data(void);

/******************** MAIN *********************/
int main(void)
{

	int IWDG_Reset_Occured = RESET;		// Variable informing whether the IWDG reset has happened
	int SFTR_Reset_Occured = RESET;		// Variable informing whether the software reset has happened
	if(RCC->CSR & RCC_CSR_WDGRSTF){
		IWDG_Reset_Occured = SET;
	}
	if(RCC->CSR & RCC_CSR_SFTRSTF){
		SFTR_Reset_Occured = SET;
	}

	// PERIPHERALS' CONFIGURATION
	RCC_init();
	GPIO_init();
	TIM_init();
	NVIC_init();
	ADC_init();
	DMA_init();
	USART_init();
	WWDG_init();

	if(IWDG_Reset_Occured){			// If IWDG reset was generated
		for(int i=0; i<5; i++){		// Print message
			send_string("\n\r");
		}
		send_string("----------------------------------------------\n\r");
		send_string("----------------------------------------------\n\r");
		send_string("--- INDEPENDENT WATCHDOG RESET DETECTED!!! ---\n\r");
		send_string("----------------------------------------------\n\r");
		send_string("----------------------------------------------\n\r\n\r\n\r\n\r");
		sprintf(str1, "%d", (int)RTC -> BKP1R);	// print BKP register value
		send_string("\n\rBKP1 register: ");send_string(str1);send_string("\n\r");
		RTC -> BKP1R = 0;				// Reset the BKP register
		RCC -> CSR |= RCC_CSR_RMVF;		// Reset the WDGRST flag
		delay_ms(3000);
	}

	if(SFTR_Reset_Occured){			// If software reset was generated
		sprintf(str1, "%d", (int)RTC -> BKP1R);	// print BKP register value
		send_string("\n\rBKP1 register: ");send_string(str1);send_string("\n\r");
		RTC -> BKP1R = 0;			// Reset the BKP register
		RCC -> CSR |= RCC_CSR_RMVF;	// Reset the SFTRST flag
	}

	RCC_ClocksTypeDef clock;
	RCC_GetClocksFreq(&clock);

	send_string("\n\r");
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
		LED_ON;
		while(1);
	}


	/****************MAIN LOOP****************/
	while(1){
		IWDG -> KR |= 0xAAAA;			// Indpendent watchdog reset
		Switch_Off_Output_Stage();
		Mode = IDLE;

		Display = ENABLE;
		Display_Manual();

		send_string("\n\rSTART\n\rPress button or 's' to align and then start the motor. \n\r");

		send_string("\n\r");
		send_string("######################################################\n\r");

		// Stop the program until the button is pressed or correct command is sent via UART
		while(((GPIOC->IDR >> 13) & 1) != 0){	// Wait for button to be pressed
			Toggle_Pin(GPIOA, 5);				// Blink led in interval of 128ms
			IWDG -> KR |= 0xAAAA;
			delay_ms(128);
			if (Mode == ALIGN) break;			// If user changed (via UART) Mode to ALIGN, break the loop
		}

		/****** ALIGNMENT PROCEDUE ******/
		Mode = ALIGN;
		Alignment();
		send_string("Alignment DONE. Starting the motor...\n\r\n\r");

		/******** START PROCEDUE ********/
		Mode = START;
		Commutation = 1;
		Start();
		Mode = RUN;

		//Change_Duty_Cycle(50);		// Set default duty cycle (~3800 RPM) with 10V supply

		LED_OFF;


		Change_Duty_Cycle(30);
		IWDG -> KR |= 0xAAAA;
		delay_ms(500);
		send_string("Press button or 'p' to turn ON/OFF the PI regulator. \n\r\n\r");
		send_string("########################################\n\r");


		Send_Data = 0;

		/******** RUNNING IN THE CLOSED LOOP ********/
		while (Mode == RUN)
		{

			// CALCULATE PHASE CURRENTS
			Iw = (((float)ADC_DMA_Values[0] - 2047.5f)*3.3/(AMPLIFICATION*4096.0))/R_SHUNT;
			Iv = (((float)ADC_DMA_Values[1] - 2047.5f)*3.3/(AMPLIFICATION*4096.0))/R_SHUNT;
			Iu = -Iw - Iv;

			Display_Manual();

			Display_Current_Data();


		}

		if(Mode == FAIL){
			Switch_Off_Output_Stage();
			Change_Duty_Cycle(0);
			NVIC_DisableIRQ(EXTI4_IRQn);
			NVIC_DisableIRQ(EXTI3_IRQn);
			NVIC_DisableIRQ(EXTI2_IRQn);
			send_string("FAILURE OCCURED!!!\n\rRESETING THE SYSTEM...\n\r");
			NVIC_SystemReset();
		}

		NVIC_DisableIRQ(TIM5_IRQn);
		PI_ON = DISABLE;

		// SLOWING DOWN THE MOTOR
		int y = 1;
		int duty = TIM4->CCR1;
		while(Mode == STOP){
			// Decrease speed
			duty = duty - 5;
			TIM2->CNT = 0;
			while(TIM2->CNT < 400){
				IWDG -> KR |= 0xAAAA;
				if (duty < 0){
					Change_Duty_Cycle(0);
				}else{
					Change_Duty_Cycle(duty);
				}
				send_string("--- ");
			}
			y++;
			if(duty < 0){
				Commutation = 0;
				NVIC_DisableIRQ(EXTI4_IRQn);
				NVIC_DisableIRQ(EXTI3_IRQn);
				NVIC_DisableIRQ(EXTI2_IRQn);
				Change_Duty_Cycle(0);
				Switch_Off_Output_Stage();
				Mode = IDLE;
				break;
			}
		}
		Switch_Off_Output_Stage();
		send_string(" \n\r");
		send_string("########## MOTOR STOPPED ########## \n\r");
		send_string("   Press any key to begin again.    \n\r");

		NVIC_DisableIRQ(EXTI4_IRQn);
		NVIC_DisableIRQ(EXTI3_IRQn);
		NVIC_DisableIRQ(EXTI2_IRQn);
		Commutation = 0;
		Switch_Off_Output_Stage();
		Change_Duty_Cycle(0);
		Mode = END;

		// END OF PROGRAM
		while(Mode == END){				// Wait for button/command
			Toggle_Pin(GPIOA, 5);		// Blink led in interval of 500ms
			IWDG -> KR |= 0xAAAA;
			delay_ms(500);
		}

	}
}

/*========================================================
 * Display_Current_Data(void)
 *========================================================
 * Function sends via UART actual values of parameteres
 * like speed, phase currents, PI regulator output etc.
 *
 * @param  None
 * @retval None
 */
void Display_Current_Data(void){
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
		if(PI_ON){
			send_string("PI regulator ON\n\r");
		}else {
			send_string("PI regulator OFF\n\r");
		}
		sprintf(str, "%d", (int)PI_Out);			// Print PI regulator output duty
		send_string("PI regulator output: ");
		send_string(str); send_string("/200 \n\r");

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







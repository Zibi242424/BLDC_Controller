/*
 * Macros.h
 *
 *  Created on: Jul 18, 2018
 *      Author: Zbigniew Mansel
 */
#include "BLDC_lib.h"


// Duty ramp used during alignment procedure
uint8_t AlignRamp[25]={3, 4, 5, 6,
7, 8, 9, 10, 15, 20, 25, 30,
35, 40, 45, 50, 55, 60, 65, 70,
75, 80, 85, 90, 95
};

// Duty cycle for the start procedure
uint8_t StartRamp[30] = {33, 34, 35, 36, 37,
38, 38, 38, 39, 40, 41, 42, 43, 44, 45, 46,
47, 47, 48, 48, 49, 49, 50, 50, 50, 51, 52,
53, 54, 55
};

// Delays between changing the commutation step during start procedure
uint16_t TimeRamp[30] = {6000, 5800, 5600, 5400,
5200, 5000, 4800, 4600, 4400, 4200, 4000, 3800, 3600, 3400,
3200, 3000, 3000, 2800, 2600, 2400, 2200, 2000, 1800, 1600,
1400, 1200, 1000, 1000, 1000, 1000
};

/*========================================================
 * 			 		   Commutate(int)
 *========================================================
 * 	Function changes the state of the output stage corresponding to the commutation table.
 * 	As a parameter it takes an integer from interval 1:6. If any other value is given all
 * 	output stages are switched off.
 *
 * 	@param  Commutation: number of commutation step which is about to be set
 * 	@retval None
 */
void Commutate(int Commutation){
	switch(Commutation){
	case 1:
		// STEP 1 W+, V-
		W_HS_ON;  W_LS_OFF;
		V_LS_ON;  V_HS_OFF;
		U_HS_OFF; U_LS_OFF;
		break;
	case 2:
		// STEP 2 W+, U-
		V_LS_OFF;
		W_HS_ON;
		U_LS_ON;  U_HS_OFF;
		V_HS_OFF; W_LS_OFF;
		break;
	case 3:
		// STEP 3 V+, U-
		W_HS_OFF;
		V_HS_ON;  V_LS_OFF;
		U_LS_ON;  U_HS_OFF;
		W_LS_OFF;
		break;
	case 4:
		// STEP 4 V+, W-
		U_LS_OFF;
		W_LS_ON;
		V_HS_ON;
		V_LS_OFF;
		W_HS_OFF;
		U_HS_OFF;
		break;
	case 5:
		// STEP 5 U+, W-
		V_HS_OFF;
		U_HS_ON; U_LS_OFF;
		W_LS_ON; W_HS_OFF;
		V_LS_OFF;
		break;
	case 6:
		// STEP 6 U+, V-
		W_LS_OFF;
		V_LS_ON;
		U_HS_ON;
		U_LS_OFF;
		V_HS_OFF;
		W_HS_OFF;
		break;
	}
}

/*========================================================
 * 			 		 Alignment(void)
 *========================================================
 *  Function moves a motor to a known position in order to make start procedure possible.
 *  Alignment procedure takes around 25*30ms=750ms. During this phase W phase is connected to voltage
 *  supply and other phases are grounded.
 *
 *  @param  None
 *  @retval None
 */
void Alignment(void){
	uint8_t StepNumber=0;

	// Move rotor from its current position
	Change_Duty_Cycle(80);
	Commutate(2);
	delay_ms(75);
	Commutate(0);
	Change_Duty_Cycle(0);
	delay_ms(150);

	Switch_Off_Output_Stage();

	TIM4 -> CCR1 = 0;

	W_HS_ON;
	V_LS_ON;
	U_LS_ON;

	while(StepNumber < 25){
		TIM4 -> CCR1 = AlignRamp[StepNumber];
		delay_ms(30);
		StepNumber += 1;
	}

	TIM4 -> CCR1 = 0;
	W_HS_OFF;
	V_LS_OFF;
	U_LS_OFF;


}

/*========================================================
 * 			 			 Start(void)
 *========================================================
 * Function start the motor. It uses 2 ramps (one for duty cycle and one for switching times) in order
 * to create smooth start procedure. After 80 full commutation cycles when Back-EMF is big enough,
 * control is switched to BEMF method.
 *
 * @param  None
 * @retval None
 */
void Start(void){
	uint8_t Step = 0;
	uint16_t time = 6000;
	uint8_t turn= 0;
	int i =0;
	TIM4 -> CCR1 = StartRamp[Step];
	TIM4 -> CCR2 = StartRamp[Step];
	TIM4 -> CCR3 = StartRamp[Step];
	time = TimeRamp[Step];
	Commutation = 1;
	while(Mode == START){
		LED_OFF;			// Switch off Nucleo board LED

		// STEP 1 W+, V-
		Commutate(Commutation);
		delay_us(time);
		Commutation++;


		// STEP 2 W+, U-
		Commutate(Commutation);
		delay_us(time);
		Commutation++;


		// STEP 3 V+, U-
		Commutate(Commutation);
		delay_us(time);
		Commutation++;


		// STEP 4 V+, W-
		Commutate(Commutation);
		delay_us(time);
		Commutation++;


		// STEP 5 U+, W-
		Commutate(Commutation);
		delay_us(time);
		Commutation++;

		// STEP 6 U+, V-
		Commutate(Commutation);
		delay_us(time);

		Commutation = 1;
		turn++;

		if (Step < 29 && turn == 2) {
			Step++;
			TIM4 -> CCR1 = StartRamp[Step];
			TIM4 -> CCR2 = StartRamp[Step];
			TIM4 -> CCR3 = StartRamp[Step];
			time = TimeRamp[Step];
			turn = 0;

		}else if(i > 250){
			TIM4 -> CCR1 = 70;
			TIM4 -> CCR2 = 70;
			TIM4 -> CCR3 = 70;
		}
		else if(i > 80){
			NVIC_EnableIRQ(EXTI4_IRQn);		// Enable interrupts from the comparator
			NVIC_EnableIRQ(EXTI3_IRQn);
			NVIC_EnableIRQ(EXTI2_IRQn);
			NVIC_EnableIRQ(TIM5_IRQn);
			Commutate(Commutation);
			break;

		}
		i++;
	}
}

/*========================================================
 * 			 		Change_Duty_Cycle(int)
 *========================================================
 *  Function changes the duty cycle of all phases and returns
 *  the actual duty cycle in percent.
 *
 *  @param  Duty: value to be written into TIM4->CCRx registers
 *  @retval Set duty cycle in percents
 */
int Change_Duty_Cycle(int Duty){
	if(Duty <= 199){
		TIM4 -> CCR1 = Duty;
		TIM4 -> CCR2 = Duty;
		TIM4 -> CCR3 = Duty;
		TIM4 -> CCR4 = Duty;
		return (int)((((float)TIM4->CCR1/(float)TIM4->ARR))*100.0);
	}else{
		return (int)(((float)TIM4->CCR1/(float)TIM4->ARR)*100.0);
	}
}

/*========================================================
 * 			 			  PI(void)
 *========================================================
 *  Function calculates the output of the PI regulator.
 *  The output is PWM duty.
 *
 *  @param	None
 *  @retval Value to written into TIM4->CCRx registers
 */
long double PI_regulator(){
	LED_ON;
	Error = Set_Rotation_Time - Rotation_Time;	// Calculate error

	// If calculations enabled and PI is ON
	if (Calculate_PI == ENABLE && PI_ON == ENABLE  && (Error > 50 || Error < -50)){
		Integral += Error;		// Calculate integral of error
		// Calculate PI output
		PI_Out = -(CFG_M0_PI_ID_KP*(double)Error + CFG_M0_PI_ID_KI*(double)Integral)/100000.0;
		// If calculated PI output is less than 0, PI output is 1;
		if(PI_Out < 0){
			Change_Duty_Cycle(1);	// Set new duty cycle
		}
		else if(PI_Out <= 199){		// if calculated PI output is < 190
			Change_Duty_Cycle((int)(PI_Out));	// Set new duty Cycle
		}else if(PI_Out > 199){
			//Integral = 0;
			Change_Duty_Cycle(199);
		}
		Calculate_PI = DISABLE;		// Clear flag
	}
	LED_OFF;
	return PI_Out;
}


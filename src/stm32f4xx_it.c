/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-January-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "defines.h"

extern unsigned int ticks;
extern uint8_t Commutation;
extern int Set_Rotation_Time;
extern uint8_t Measure_Speed;
extern uint8_t Mode;
extern int Rotation_Time;
extern uint8_t Calculate_PI;
extern uint8_t PI_ON;
extern uint16_t Send_Data;
extern int Reference_Commutation_Step;
extern float Integral_W;
extern float W;
extern uint16_t ADC_DMA_Values[2];
extern uint8_t Display;
extern int Regulator_Output;

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
/*  TimingDelay_Decrement(); */
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/

/*========================================================
 * EXTI4_Handler
 *========================================================
 * 	Interrupt from comparator from COMP_W output. This interrupt informs that
 * 	zero crossing of the W phase occurred. In the interrupt commutation step is changed
 * 	to a following one.
 * 	In this interrupt rotation time is also measured. It is done by measuring time (with
 * 	TIM1) between 7 consecutive interrupts. Variable 'Measure_Speed' is used to synchronize
 * 	the measurement.
 *  Speed measurement is done because it is known that 7 full commutation cycles
 *  equal to one full spin.
 */
void EXTI4_IRQHandler(void){
	if((EXTI->IMR & EXTI_IMR_MR4) && (EXTI->PR & EXTI_PR_PR4)){
		if (Commutation == 3 || Commutation == 6){
			IWDG -> KR |= 0xAAAA;
			if(Measure_Speed == 1){			// If its first interrupt
				TIM1->CNT = 0;				// clear timer counter.
				Reference_Commutation_Step = Commutation;	// Save the Commutation step
				Measure_Speed++;				// Increase variable
			}
			// If its the same commutation step and 2<=Measure_Speed<7
			else if(Measure_Speed < POLE_PAIRS && Commutation == Reference_Commutation_Step)
			{
				Measure_Speed++;
			}
			// If the commutation step occurred 7 times save the time
			else if(Measure_Speed == POLE_PAIRS && Commutation == Reference_Commutation_Step){	// If it's 14th interrupt
				Rotation_Time = TIM1->CNT;	// read the counter value.
				Measure_Speed = 1;			// Reset variable
				Reference_Commutation_Step = 0;
			}

			if (Commutation == 6){
				Commutation = 1;
			}else Commutation++;


			Commutate(Commutation);
			RTC -> BKP1R = Commutation;
		}

	}
	EXTI -> PR |= EXTI_PR_PR4;

}
/*========================================================
 * EXTI3_Handler
 *========================================================
 * 	Interrupt from comparator from COMP_V output. This interrupt informs that
 * 	zero crossing of the V phase occured. In the interrupt commutation step is changed
 * 	to a following one.
 */
void EXTI3_IRQHandler(void){
	if((EXTI->IMR & EXTI_IMR_MR3) && (EXTI->PR & EXTI_PR_PR3)){
		if (Commutation == 2 || Commutation == 5){
			IWDG -> KR |= 0xAAAA;
			if (Commutation < 6){
				Commutation++;
			}
			else if (Commutation == 6){
				Commutation = 1;
			}
			Commutate(Commutation);
			RTC -> BKP1R = Commutation;
		}
	}
	EXTI -> PR |= EXTI_PR_PR3;		// Clear interrupt flag

	LED_OFF;
}

/*========================================================
 * EXTI2_Handler
 *========================================================
 * 	Interrupt from comparator from COMP_U output. This interrupt informs that
 * 	zero crossing of the U phase occured. In the interrupt commutation step is changed
 * 	to a following one.
 */
void EXTI2_IRQHandler(void){
	if((EXTI->IMR & EXTI_IMR_MR2) && (EXTI->PR & EXTI_PR_PR2)){
		if (Commutation == 1 || Commutation == 4){
			IWDG -> KR |= 0xAAAA;
			if (Commutation < 6){
				Commutation++;
			}
			else if (Commutation == 6){
				Commutation = 1;
			}
		}
		Commutate(Commutation);
		RTC -> BKP1R = Commutation;
	}
	EXTI -> PR |= EXTI_PR_PR2;		// Clear interrupt flag
	LED_OFF;
}

/*========================================================
 * TIM5_Handler
 *========================================================
 * 	Interrupt from TIM5 used to trigger PI regulator calculations.
 * 	In the interrupt 'Calculate_PI' flag is set
 */
void TIM5_IRQHandler(void){
	if ((TIM5 -> DIER & 1) && (TIM5 -> SR & 1)){
		GPIOC -> BSRRL |= 1 << 10;
		Calculate_PI = ENABLE;		// Set the flag to calculate the PI regulator output
		PI_regulator();
		Send_Data++;
		if(Check_For_Short()){
			Mode = FAIL;
		}
		TIM5 -> SR &= ~TIM_SR_UIF;	// Clear interrupt flag
		GPIOC -> BSRRH |= 1 << 10;
	}
}

/*========================================================
 * EXTI15_10_Handler
 *========================================================
 *	Interrupt from user button on Nucleo64 board. Used to
 *	toggle user button.
 */
void EXTI15_10_IRQHandler(void){
	if((EXTI->IMR & EXTI_IMR_MR13) && (EXTI->PR & EXTI_PR_PR13)){

		EXTI -> PR |= EXTI_PR_PR13;			// Clear interrupt flag
		if(Mode == IDLE){
			Mode = ALIGN;
		}else if (Mode == ALIGN){
			Mode = START;
		}else if (Mode == END){
			Mode = IDLE;
		}
		if(Mode == RUN && PI_ON == ENABLE && (Set_Rotation_Time - 100) >= 9000){ // If motor is running and
			Set_Rotation_Time -= 100;		// PI regulator is on, increase the speed
		}
		else if(Mode == RUN){				// If motor is running turn on
			PI_ON = ENABLE;					// the PI regulator
		}

	}


}
/*========================================================
 * USART2_IRQHandler
 *========================================================
 *	Interrupt from UART interface used to receive data from user
 *	which allows to control the work of MCU and of the algorithm.
 */
void USART2_IRQHandler(){
	if (USART2 -> SR & USART_SR_RXNE){
		char c = USART2 -> DR;			// c = data received from UART
		if(c == 's' && Mode == IDLE){	// Change mode to ALIGN
			Mode = ALIGN;
		}else if(c == 'p'  && Mode == RUN && PI_ON == DISABLE){			// Switch on PI regulator
			PI_ON = ENABLE;
		}else if (c == 'p'  && Mode == RUN && PI_ON == ENABLE){
			PI_ON = DISABLE;
			Change_Duty_Cycle(TIM4->CCR1);
		}else if(c == 's'  && Mode == RUN){			// Stop the motor
			Mode = STOP;
		}else if(c == '8' || c == 'A'){				// Decrease rotation time by 100us
			if(Set_Rotation_Time-100 >= MIN_ROTATION_TIME){
				Set_Rotation_Time -= 100;
			}
		}else if(c == '2' || c == 'B'){			// Increase rotation time by 100us
			Set_Rotation_Time += 100;
		}else if(c == '6' || c == 'C'){				// Decrease rotation time by 1000us
			if((Set_Rotation_Time - 1000) <= MIN_ROTATION_TIME)
				Set_Rotation_Time = MIN_ROTATION_TIME;
			else Set_Rotation_Time -= 1000;
		}else if(c == '4' || c == 'D'){			// Increase rotation time by 1000us
			Set_Rotation_Time += 1000;
		}else if(c == 'r'){						// Reset of MCU
			Switch_Off_Output_Stage();
			send_string("========== ");send_string("SOFTWARE RESET");send_string(" ==========\n\r");
			send_string("\n\r");
			NVIC_SystemReset();
		}else if((c == 'd' && Mode == RUN)){	// Display manual
			Display = ENABLE;
		}else if(c == 'd'){
			Display = ENABLE;
			Display_Manual();
		}else if(c == 'q' && Mode == RUN){		// Set the minimum duty cycle @10V supply
			PI_ON = DISABLE;					// Disable PI regulator
			Change_Duty_Cycle(10);
		}else if(c == 'w' && Mode == RUN){		// Set maximum duty cycle @10V supply
			PI_ON = DISABLE;					// Disable PI regulator
			Change_Duty_Cycle(180);
		}else if(Mode == END){
			Mode = IDLE;
		}else if(Mode != RUN){
			Display = ENABLE;
			Display_Manual();
		}


	}
	USART2 -> SR &= ~USART_SR_RXNE;
}

/*========================================================
 * ADC_IRQHandler
 *========================================================
 *	Handler checks if OVERRUN flag is set the it switches of
 *	the ADC configures the DMA transmission on turns on the
 *	converter. Thanks to this handler it is guaranteed that
 *	no data from the ADC is lost.
 */
void ADC_IRQHandler(void){
	if(ADC1 -> SR & ADC_SR_OVR){			// If ADC overrun occurred
		ADC1 -> CR2 &= ~ADC_CR2_ADON;
		ADC1 -> CR2 &= ~ADC_CR2_DMA;
		DMA2_Stream0 -> CR &= ~DMA_SxCR_EN;	// Disable DMA
		DMA2_Stream0 -> PAR = (uint32_t)&ADC1->DR;			// Peripheral address
		DMA2_Stream0 -> M0AR = (uint32_t)ADC_DMA_Values;	// Memory address
		DMA2_Stream0 -> NDTR = 2;
		DMA2 -> LIFCR |= 0b111101;			// Clear DMA2_Stream0 flags
		DMA2_Stream0 -> CR |= DMA_SxCR_EN;	// Enable DMA
		ADC1 -> SR &= ~ADC_SR_OVR;			// Clear flags
		ADC1 -> SR &= ~ADC_SR_EOC;
		ADC1 -> CR2 |= ADC_CR2_DMA;			// Enable DMA request from ADC
		ADC1 -> CR2 |= ADC_CR2_ADON;		// ADC ON

	}
}
/*========================================================
 * DMA2_Stream0__IRQHandler
 *========================================================
 *	If there's any interrupt from DMA2 Stream0 all the flags
 *	are cleared.
 */
void DMA2_Stream0_IRQHandler(void){
		// Clear all flags
		DMA2->LIFCR |= DMA_LIFCR_CFEIF0;
		DMA2->LIFCR |= DMA_LIFCR_CDMEIF0;
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

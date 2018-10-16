/*
 * GPIO_lib.c
 *
 *  Created on: Jul 18, 2018
 *      Author: Zbigniew Mansel
 */


#include "GPIO_lib.h"

#define LCD_DC			GPIO_Pin_1
#define LCD_CE			GPIO_Pin_2
#define LCD_RST			GPIO_Pin_3

/*========================================================
 * 			 		   delay_us(int)
 *========================================================
 * Delay function based on SysTick timer. As a parameter it takes
 * number of microseconds to wait
 *
 * @param  microseconds: number of microseconds to wait
 * @retval None
 */
void delay_us(int microseconds){
	ticks = 0;
	while (ticks < microseconds);
}

/*========================================================
 * 			 			delay_ms(int)
 *========================================================
 * Delay function based on TIM2. As a parametr it takes number of
 * milliseconds to wait. Maximum delay time is 1s.
 *
 * @param  milliseconds: number of milliseconds to wait
 * @retval None
 */
void delay_ms(int milliseconds)
{
	if(milliseconds < TIM2->ARR){
		TIM2 -> CNT = 0;
		while(TIM2->CNT < milliseconds);
	}
}

/*========================================================
 * 			 	Toggle_Pin(GPIO_TypeDef *, int)
 *========================================================
 *  Function changes the state on the output pin to the
 *  opposite one.
 *
 *  @param  GPIO: general purpose input output port
 *  @param  GPIO_Pin: number of general purpose input output pin
 *  @retval None
 */
void Toggle_Pin(GPIO_TypeDef * GPIO, int GPIO_Pin){
	if (((GPIO->MODER>>(2*GPIO_Pin)) & 0b01) == 0b01){ // Check if pin is an output
		GPIO -> ODR ^= 1 << GPIO_Pin;
	}
	/*if (((GPIO->MODER>>(2*GPIO_Pin)) & 0b01) == 0b01){ // Check if pin is an output
		if ((GPIO->ODR & (1 << GPIO_Pin)) > 0)		   // If pin is in high state
			GPIO->BSRRH |= 1 << GPIO_Pin;			   // Reset pin state
		else
			GPIO->BSRRL |= 1 << GPIO_Pin;
	}*/
}

/*========================================================
 * 			 	 Switch_Off_Output_Stage(void)
 *========================================================
 * Function switches of all the MCU's outputs to the driver.
 *
 * @param  None
 * @retval None
 */
void Switch_Off_Output_Stage(void){
	W_HS_OFF;
	V_HS_OFF;
	U_HS_OFF;
	W_LS_OFF;
	V_LS_OFF;
	U_LS_OFF;
}


/*========================================================
 * 			 		 Check_For_Short(void)
 *========================================================
 * Function checks if both highside and lowside of any of half bridges
 * are on at the same time and if so it switches off all of them.
 *
 * @param  None
 * @retval Error Code
 */
int Check_For_Short(void){
	if((TIM4->CCER & 0x0001) && (GPIOB->ODR & 0x0001)){
		W_HS_OFF;
		V_HS_OFF;
		U_HS_OFF;
		W_LS_OFF;
		V_LS_OFF;
		U_LS_OFF;
		return 1;
	}else if(((TIM4->CCER>>4) & 0x0001) && ((GPIOB->ODR >> 1) & 0x0001)){
		W_HS_OFF;
		V_HS_OFF;
		U_HS_OFF;
		W_LS_OFF;
		V_LS_OFF;
		U_LS_OFF;
		return 1;
	}else if(((TIM4->CCER>>8) & 0x0001) && ((GPIOB->ODR >> 2) & 0x0001)){
		W_HS_OFF;
		V_HS_OFF;
		U_HS_OFF;
		W_LS_OFF;
		V_LS_OFF;
		U_LS_OFF;
		return 1;
	}
	return 0;
}





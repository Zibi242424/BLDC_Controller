/*
 * IRQ_SysTick.c
 *
 *  Created on: 15.07.2018
 *      Author: Zbigniew Mansel
 */
#include "stm32f4xx.h"

volatile unsigned int ticks;


/*========================================================
 * SysTick_Handler
 *========================================================
 * 	Interrupt handler from Cortex SysTick timer.
 * 	Function adds 1 to global variable 'tick' which is used in
 * 	delay_us() function.
 */
void SysTick_Handler(void){
	ticks++;

}


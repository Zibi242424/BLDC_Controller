/*
 * GPIO_lib.h
 *
 *  Created on: Jul 18, 2018
 *      Author: adm_wdowiak
 */
#include "stm32f4xx.h"
#include "defines.h"



extern uint32_t ticks;
extern int Commutation;

void delay_ms(int);
void delay_us(int);
void Toggle_Pin(GPIO_TypeDef * GPIO, int GPIO_Pin);
void Switch_Off_Output_Stage(void);
void Check_For_Short(void);
void send_char(char c);
void send_string(const char *s);

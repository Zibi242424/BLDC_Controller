#ifndef GPIO_LIB_H_
#define GPIO_LIB_H_

#include "stm32f4xx.h"
#include "defines.h"



extern uint32_t ticks;
extern int Commutation;
extern uint8_t Display;
extern uint8_t Mode;

void delay_ms(int);
void delay_us(int);
void Toggle_Pin(GPIO_TypeDef * GPIO, int GPIO_Pin);
void Switch_Off_Output_Stage(void);
int  Check_For_Short(void);


#endif

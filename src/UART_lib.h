#ifndef UART_LIB_H_
#define UART_LIB_H_

#include "stm32f4xx.h"
#include "GPIO_lib.h"
#include "defines.h"



extern uint8_t Display;
extern uint8_t Mode;

void send_char(char);
void send_string(const char *);
void Display_Manual(void);



#endif

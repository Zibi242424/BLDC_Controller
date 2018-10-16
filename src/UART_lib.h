#include "stm32f4xx.h"
#include "GPIO_lib.h"
#include "defines.h"

#ifndef UART_H_
#define UART_H_

extern uint8_t Display;
extern uint8_t Mode;

void send_char(char);
void send_string(const char *);
void Display_Manual(void);



#endif /* UART_H_ */

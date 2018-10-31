#include "UART_lib.h"

/*========================================================
 * send_char(char)
 *========================================================
 * Function sends one single char via USART2.
 *
 * @param  c: character to be sent via UART
 * @retval None
 */
void send_char(char c){
	while((USART2->SR & USART_SR_TXE) == RESET){	// Wait until previous transfer ended
	}
	USART2->DR = (c & (uint16_t)0x01FF);
}

/*========================================================
 * send_string(const char *)
 *========================================================
 * Function sends string via USART2.
 *
 * @param  *s: pointer to the first item of a string
 * @retval None
 */
void send_string(const char *s){
	while(*s){
		send_char(*s++);
	}
}

/*========================================================
 * Display_Manual(void)
 *========================================================
 * Function sends via UART the list of available controls.
 *
 * @param  None
 * @retval None
 */
void Display_Manual(void){
	if(Display == ENABLE){
		send_string("\n\r<<<<<<<<<<<<<<<<<<<< MANUAL >>>>>>>>>>>>>>>>>>>>\n\r");
		send_string("$$$  Make sure that CAPS-LOCK is not enabled   $$$\n\r");
		send_string("'s'      => Stop/Start the motor\n\r");
		send_string("'p'      => Turn ON/OFF the PI regulator\n\r");
		send_string("'r'      => Reset MCU\n\r");
		send_string("'q'      => Set minimum possible speed (PI regulator OFF)\n\r");
		send_string("'d'      => Display manual\n\r");
		send_string("RIGHT/6  => Decrease rotation time by 1000us\n\r");
		send_string("LEFT/4   => Increase rotation time by 1000us\n\r");
		send_string("UP/8     => Decrease rotation time by 100us\n\r");
		send_string("DOWN/2   => Increase rotation time by 100us\n\r");
		send_string("****************************************************\n\r");
		Display = DISABLE;
		if(Mode == RUN){
			delay_ms(3000);
		}
	}
}

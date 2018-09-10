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
		if ((GPIO->ODR & (1 << GPIO_Pin)) > 0)		   // If pin is in high state
			GPIO->BSRRH |= 1 << GPIO_Pin;			   // Reset pin state
		else
			GPIO->BSRRL |= 1 << GPIO_Pin;
	}
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
 * @retval None
 */
void Check_For_Short(void){
	if((TIM4->CCER & 0x0001) && (GPIOB->ODR & 0x0001)){
		W_HS_OFF;
		V_HS_OFF;
		U_HS_OFF;
		W_LS_OFF;
		V_LS_OFF;
		U_LS_OFF;
	}else if(((TIM4->CCER>>4) & 0x0001) && ((GPIOB->ODR >> 1) & 0x0001)){
		W_HS_OFF;
		V_HS_OFF;
		U_HS_OFF;
		W_LS_OFF;
		V_LS_OFF;
		U_LS_OFF;
	}else if(((TIM4->CCER>>8) & 0x0001) && ((GPIOB->ODR >> 2) & 0x0001)){
		W_HS_OFF;
		V_HS_OFF;
		U_HS_OFF;
		W_LS_OFF;
		V_LS_OFF;
		U_LS_OFF;
	}
}

/*========================================================
 * 			 		 send_char(char)
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
 * 			 	  send_string(const char *)
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


/*********************SPI FUNCTIONS************************/
void GPIO_Set_Bits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIOx->BSRRL = GPIO_Pin;
}

void GPIO_Reset_Bits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{

  GPIOx->BSRRH = GPIO_Pin;
}

uint8_t SPI_SendReceiveData(uint8_t data){
	while(!(SPI2 -> SR & SPI_SR_TXE));
	SPI2 -> DR = data;

	while(!(SPI2 -> SR & SPI_SR_RXNE));
	return SPI2 -> DR;
}

void LCD_Cmd(uint8_t cmd)
{
 GPIO_Reset_Bits(GPIOC, LCD_CE|LCD_DC);
 SPI_SendReceiveData(cmd);
 GPIO_Set_Bits(GPIOC, LCD_CE);
}


void LCD_SendData(const uint8_t* data, int size)
{
 int i;
 GPIO_Set_Bits(GPIOC, LCD_DC);
 GPIO_Reset_Bits(GPIOC, LCD_CE);
 for (i = 0; i < size; i++)
 SPI_SendReceiveData(data[i]);
 GPIO_Set_Bits(GPIOC, LCD_CE);
}

void LCD_Reset()
{
 GPIO_Reset_Bits(GPIOC, LCD_RST);
 GPIO_Set_Bits(GPIOC, LCD_RST);
}


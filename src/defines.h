/*
 * defines.h
 *
 *  Created on: Jul 30, 2018
 *      Author: Zbigniew Mansel
  */
#include "stm32f4xx.h"
#ifndef DEFINES_H_
#define DEFINES_H_


#define SystemCoreClock 80000000L	// System core clock frequency in Hz

#define TwoOverPi 0.6366f			// 2/Pi
#define Pi 3.1416f					// Pi number

#define MIN_ROTATION_TIME	9500	// Minimum rotation time used in application (maximum speed)

// Motor constants
#define Li ((22.186)/(1000000.0))	// Windings inductance
#define R (0.170)					// Windings resistance
#define Vdc (10.0)					// Supply voltage
#define Bw (2000.0)					// PI regulator sampling frequency
#define POLE_PAIRS 7				// Number of pole pairs in the motor

// Hardware specifications
#define R_SHUNT 0.0045f				// Resistance of shunt resistor in Ohms
#define AMPLIFICATION 50.0f			// Amplification of an op amp


// PI controler factors calculations
#define TSTATOR  ((Li)/(R))
#define K_FACTOR (1.0/((Vdc)*TwoOverPi))
#define CALC_KI  ((R)/(1.0/(2.0*Pi*Bw)))*K_FACTOR
#define CALC_KP  (CALC_KI*TSTATOR)


#define kpd_multi 			(0.2)	//Other (with) Load	0.2
#define kid_multi 			(0.005)	//Other (with) Load 0.005
#define kpq_multi 			(0.2)	//Other (with) Load
#define kiq_multi 			(0.1)	//Other (with) Load

#define CFG_M0_PI_ID_KP 	(kpd_multi*(CALC_KP)) 	//brief Define the Kp parameter value.
#define CFG_M0_PI_ID_KI 	(kid_multi*(CALC_KI))  	// brief Define the Ki parameter value.

#define CFG_M0_PI_IQ_KP 	(kpq_multi*(CALC_KP))  	//brief Define the Kp parameter value.
#define CFG_M0_PI_IQ_KI 	(kiq_multi*(CALC_KI))   //brief Define the Ki parameter value.

enum {IDLE = 0, ALIGN = 1, START = 2, RUN = 3, STOP = 4, END = 5, FAIL = 6};

// Macros to control the output stages
#define W_HS_ON	 TIM4->CCER|=0x0001		// Enable PWM output for W phase highside switch
#define W_LS_ON  GPIOB->BSRRL|=1<<0;	// Switch on W phase lowside switch
#define W_HS_OFF TIM4->CCER&=~0x0001	// Disable PWM output for W phase highside switch
#define W_LS_OFF GPIOB->BSRRH|=1<<0;	// Switch off W phase lowside switch

#define V_HS_ON  TIM4->CCER|=0x0010		// Enable PWM output for V phase highside switch
#define V_LS_ON  GPIOB->BSRRL|=1<<1		// Switch on V phase lowside switch
#define V_HS_OFF TIM4->CCER&=~0x0010	// Disable PWM output for W phase highside switch
#define V_LS_OFF GPIOB->BSRRH|=1<<1		// Switch off V phase lowside switch

#define U_HS_ON  TIM4->CCER|=0x0100		// Enable PWM output for U phase highside switch
#define U_LS_ON  GPIOB->BSRRL|=1<<2		// Switch on U phase lowside switch
#define U_HS_OFF TIM4->CCER&=~0x0100	// Disable PWM output for W phase highside switch
#define U_LS_OFF GPIOB->BSRRH|=1<<2		// Switch off U phase lowside switch

#define LED_ON   GPIOA->BSRRL|=1<<5		// Switch on user led on Nucleo64 board
#define LED_OFF  GPIOA->BSRRH|=1<<5		// Switch off user led on Nucleo64 board

// Macros to measure phase currents
#define W_ADC_CHANNEL 10
#define V_ADC_CHANNEL 11
#define U_ADC_CHANNEL 12





#endif /* DEFINES_H_ */

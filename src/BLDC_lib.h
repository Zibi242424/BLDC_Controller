/*
 * BLDC_lib.h
 *
 *  Created on: Jul 19, 2018
 *      Author: adm_wdowiak
 */
#include "GPIO_lib.h"
#include "arm_math.h"


extern uint8_t Mode;
extern int Commutation;
extern int Rotation_Time;
extern int Error, Integral;		// Variables for the PI regulator
extern uint8_t PI_ON, Calculate_PI;
extern long double PI_Out;
extern int Duty_Cycle;
extern int Set_Rotation_Time, Rotation_Time;
extern uint8_t LS_ADD;
extern Regulator_Output;


void Alignment(void);
void Start(void);
float32_t Current_Meas(int);
void Commutate(int);
int Change_Duty_Cycle(int);
long double PI_regulator();
void ADC_GetConvValue(int, int *);
int ADC_Read(int);
float32_t CLARKE_PARK_transform();


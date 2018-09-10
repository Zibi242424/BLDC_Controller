/*
 * SVM.c
 *
 *  Created on: 10.08.2018
 *      Author: Zbigniew Mansel
 */
#include "SVM.h"

uint8_t  Vector_Table[6] = {
	    //UVW
		0b001,
		0b010,
		0b011,
		0b100,
		0b101,
		0b110,

};

void CommutateSVM(uint8_t Commutation){

	switch(Commutation){
		case 0b000:
			W_HS_OFF;
			W_LS_ON;

			V_HS_OFF;
			V_LS_ON;

			U_HS_OFF;
			U_LS_ON;
			break;
		case 0b001:
			W_LS_OFF;
			W_HS_ON;

			V_HS_OFF;
			V_LS_ON;

			U_HS_OFF;
			U_LS_ON;
			break;
		case 0b010:
			W_HS_OFF;
			W_LS_ON;

			V_LS_OFF;
			V_HS_ON;

			U_HS_OFF;
			U_LS_ON;
			break;
		case 0b011:
			W_LS_OFF;
			W_HS_ON;

			V_LS_OFF;
			V_HS_ON;

			U_HS_OFF;
			U_LS_ON;
			break;
		case 0b100:
			W_HS_OFF;
			W_LS_ON;

			V_HS_OFF;
			V_LS_ON;

			U_LS_OFF;
			U_HS_ON;
			break;
		case 0b101:
			W_LS_OFF;
			W_HS_ON;

			V_HS_OFF;
			V_LS_ON;

			U_LS_OFF;
			U_HS_ON;
			break;
		case 0b111:
			W_LS_OFF;
			W_HS_ON;

			V_LS_OFF;
			V_HS_ON;

			U_LS_OFF;
			U_HS_ON;
			break;
		default:
			W_LS_OFF;
			W_HS_OFF;

			V_LS_OFF;
			V_HS_OFF;

			U_LS_OFF;
			U_HS_OFF;
	}

}


void Sequence(uint8_t sector){
	switch(sector){
		case 1:
			CommutateSVM(0);
			//Change_Duty_Cycle(166);
			delay_us(83);
			CommutateSVM(1);
			//Change_Duty_Cycle(180);
			delay_us(90);
			CommutateSVM(2);
			//Change_Duty_Cycle(199);
			delay_us(245);
			CommutateSVM(7);
			//Change_Duty_Cycle(164);
			delay_us(82);
			CommutateSVM(2);
			//Change_Duty_Cycle(199);
			delay_us(245);
			CommutateSVM(1);
			//Change_Duty_Cycle(180);
			delay_us(90);
			CommutateSVM(0);
			//Change_Duty_Cycle(166);
			delay_us(83);
			break;
		case 2:
			CommutateSVM(0);
			delay_us(83);
			CommutateSVM(3);
			delay_us(90);
			CommutateSVM(2);
			delay_us(245);
			CommutateSVM(7);
			delay_us(82);
			CommutateSVM(2);
			delay_us(245);
			CommutateSVM(3);
			delay_us(90);
			CommutateSVM(0);
			delay_us(83);
			break;
		case 3:
			CommutateSVM(0);
			delay_us(83);
			CommutateSVM(3);
			delay_us(90);
			CommutateSVM(4);
			delay_us(245);
			CommutateSVM(7);
			delay_us(82);
			CommutateSVM(4);
			delay_us(245);
			CommutateSVM(3);
			delay_us(90);
			CommutateSVM(0);
			delay_us(83);
			break;
		case 4:
			CommutateSVM(0);
			delay_us(83);
			CommutateSVM(5);
			delay_us(90);
			CommutateSVM(4);
			delay_us(245);
			CommutateSVM(7);
			delay_us(82);
			CommutateSVM(4);
			delay_us(245);
			CommutateSVM(5);
			delay_us(90);
			CommutateSVM(0);
			delay_us(83);
			break;
		case 5:
			CommutateSVM(0);
			delay_us(83);
			CommutateSVM(5);
			delay_us(90);
			CommutateSVM(6);
			delay_us(245);
			CommutateSVM(7);
			delay_us(82);
			CommutateSVM(6);
			delay_us(245);
			CommutateSVM(5);
			delay_us(90);
			CommutateSVM(0);
			delay_us(83);
			break;
		case 6:
			CommutateSVM(0);
			delay_us(83);
			CommutateSVM(1);
			delay_us(90);
			CommutateSVM(6);
			delay_us(245);
			CommutateSVM(7);
			delay_us(82);
			CommutateSVM(6);
			delay_us(245);
			CommutateSVM(1);
			delay_us(90);
			CommutateSVM(0);
			delay_us(83);
			break;
	}
}

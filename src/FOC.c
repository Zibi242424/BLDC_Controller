/*
 * FOC.c
 *
 *  Created on: 06.08.2018
 *      Author: mansel
 */
#include "FOC.h"

void PI_FOC(int W_Set, int W){
	int Error = W_Set - W;
	Integral_FOC += Error;
	Iqref = (CFG_M0_PI_ID_KP*(double)Error + KI*CFG_M0_PI_ID_KI*(double)Integral_FOC)/100000.0;
}




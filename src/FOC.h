/*
 * FOC.h
 *
 *  Created on: 06.08.2018
 *      Author: mansel
 */

#ifndef FOC_H_
#define FOC_H_
#include "defines.h"

extern float Iqref;
int Integral_FOC;

void PI_FOC(int W_Set, int W);

#endif /* FOC_H_ */

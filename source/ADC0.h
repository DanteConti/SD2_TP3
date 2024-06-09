/*
 * ADC0.h
 *
 *  Created on: 16 abr. 2022
 *      Author: dante
 */

#ifndef ADC0_H_
#define ADC0_H_

#include "stdint.h"

void ADC0_init(void);
void ADC0_iniciarConv(void);
void ADC0_tick(void);
uint16_t ADC0_getBrightness();

#endif /* ADC0_H_ */

/*
 * adc.h
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32f4xx.h"

void ADC_Init(void);
uint16_t ADC_Read(void);
void ADC_Enable(void);
void ADC_Disable(void);


#endif /* INC_ADC_H_ */

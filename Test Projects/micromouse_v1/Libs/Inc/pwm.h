#ifndef PWM_H
#define PWM_H

#include "stm32f4xx.h"

void PWM_Init(void);
void PWM_SetDutyCycle(uint8_t dutyCycle);
void PWM_Start(void);
void PWM_Stop(void);

#endif // PWM_H

/*
 * encoder.h
 *
 *  Created on: Nov 5, 2024
 *      Author: hrish
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f4xx_hal.h"
#include <math.h>

// Initialize the encoder
void Encoder_Init(TIM_HandleTypeDef *htim, uint16_t ppr);

// Get the current encoder count for a specified timer
int Encoder_GetCount(TIM_HandleTypeDef *htim);

// Get the encoder velocity (counts per second) for a specified timer
float Encoder_GetVelocity(TIM_HandleTypeDef *htim);

float Encoder_GetVelocity_TIM1(TIM_HandleTypeDef *htim);
float Encoder_GetVelocity_TIM4(TIM_HandleTypeDef *htim);

#endif /* INC_ENCODER_H_ */

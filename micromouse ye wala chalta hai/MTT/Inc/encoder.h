/*
 * encoder.h
 *
 *  Created on: Nov 5, 2024
 *      Author: hrish
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f4xx_hal.h"

// Initialize the encoder
void Encoder_Init(TIM_HandleTypeDef *htim, uint16_t ppr);

// Get the current encoder count
int Encoder_GetCount(void);

// Get the encoder velocity (counts per second)
float Encoder_GetVelocity(void);

#endif /* INC_ENCODER_H_ */

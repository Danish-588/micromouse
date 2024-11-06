/*
 * gpio.h
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f4xx.h"

void GPIO_Init_Output(GPIO_TypeDef *GPIOx, uint16_t pin);
void GPIO_Init_Input(GPIO_TypeDef *GPIOx, uint16_t pin);
void GPIO_SetPin(GPIO_TypeDef *GPIOx, uint16_t pin);
void GPIO_ResetPin(GPIO_TypeDef *GPIOx, uint16_t pin);
void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t pin);
uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif /* INC_GPIO_H_ */

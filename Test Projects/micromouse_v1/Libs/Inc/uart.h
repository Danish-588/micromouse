/*
 * uart.h
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */

#ifndef INC_UART_H_
#define INC_UART_H_


#include "stm32f4xx.h"

void UART_Init(uint32_t baudrate);
void UART_Transmit(uint8_t *data, uint16_t size);
uint8_t UART_Receive(void);
void UART_SendString(char *str);


#endif /* INC_UART_H_ */

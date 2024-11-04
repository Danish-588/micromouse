/*
 * uart.h
 *
 *  Created on: Jun 24, 2021
 *      Author: Soham
 */

#ifndef INC_UART_H_
#define INC_UART_H_

UART_HandleTypeDef uarthandle1;
UART_HandleTypeDef uarthandle2;
UART_HandleTypeDef uarthandle3;
UART_HandleTypeDef uarthandle4;
UART_HandleTypeDef uarthandle5;
UART_HandleTypeDef uarthandle6;

typedef void (*uartrx_isrfunciton)(void);

struct uart_pins
{
	UART_HandleTypeDef *uarthandle;
	USART_TypeDef *uart;
	GPIO_TypeDef  *tx_port;
	uint16_t tx_pin;
	GPIO_TypeDef  *rx_port;
	uint16_t rx_pin;
	uint8_t *tx_buff;
	uint8_t *rx_buff;
	uartrx_isrfunciton uartrx_isrfunction;
	bool state;
}uart1,uart2,uart3,uart4,uart5,uart6;

uint8_t tx_buff1,tx_buff2,tx_buff3,tx_buff4,tx_buff5,tx_buff6;
uint8_t rx_buff1,rx_buff2,rx_buff3,rx_buff4,rx_buff5,rx_buff6;
void uart_init(struct uart_pins *uart_pin,int baudrate,void (*uartrx_isrfunction)(void));
void uart_init_norx(struct uart_pins *uart_pin,int baudrate);
void uart_transmit(struct uart_pins *uart_pin, int data);
void uart_transmit_ascii(struct uart_pins *uart_pin, char *pData);
void uartrx_isr(void isrhandler(void));

#endif /* INC_UART_H_ */

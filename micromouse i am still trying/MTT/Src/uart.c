/*
 * uart.c
 *
 *  Created on: Jun 24, 2021
 *      Author: Soham
 */
#include "main.h"

struct uart_pins
uart1 = {&uarthandle1,USART1,GPIOA,PIN9,GPIOA,PIN10,&tx_buff1,&rx_buff1,emptyfunc},
uart2 = {&uarthandle2,USART2,GPIOA,PIN2,GPIOA,PIN3,&tx_buff2,&rx_buff2,emptyfunc},
uart3 = {&uarthandle3,USART3,GPIOB,PIN10,GPIOC,PIN5,&tx_buff3,&rx_buff3,emptyfunc},
uart4 = {&uarthandle4,UART4,GPIOA,PIN0,GPIOA,PIN1,&tx_buff4,&rx_buff4,emptyfunc},
uart5 = {&uarthandle5,UART5,GPIOC,PIN12,GPIOD,PIN2,&tx_buff5,&rx_buff5,emptyfunc},
uart6 = {&uarthandle6,USART6,GPIOC,PIN6,GPIOC,PIN7,&tx_buff6,&rx_buff6,emptyfunc};

void uart_init(struct uart_pins *uart_pin,int baudrate, void (*uartrx_isrfunction)(void))
{
	uart_pin->uartrx_isrfunction = uartrx_isrfunction;
	uart_pin->uarthandle->Instance = uart_pin->uart;
	uart_pin->uarthandle->Init.BaudRate = baudrate;
	uart_pin->uarthandle->Init.WordLength = UART_WORDLENGTH_8B;
	uart_pin->uarthandle->Init.StopBits = UART_STOPBITS_1;
	uart_pin->uarthandle->Init.Parity = UART_PARITY_NONE;
	uart_pin->uarthandle->Init.Mode = UART_MODE_TX_RX;
	uart_pin->uarthandle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart_pin->uarthandle->Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(uart_pin->uarthandle) != HAL_OK)
	{
		Error_Handler();
	}
}

void uart_init_norx(struct uart_pins *uart_pin,int baudrate)
{
	uart_pin->uarthandle->Instance = uart_pin->uart;
	uart_pin->uarthandle->Init.BaudRate = baudrate;
	uart_pin->uarthandle->Init.WordLength = UART_WORDLENGTH_8B;
	uart_pin->uarthandle->Init.StopBits = UART_STOPBITS_1;
	uart_pin->uarthandle->Init.Parity = UART_PARITY_NONE;
	uart_pin->uarthandle->Init.Mode = UART_MODE_TX;
	uart_pin->uarthandle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart_pin->uarthandle->Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(uart_pin->uarthandle) != HAL_OK)
	{
		Error_Handler();
	}
}

void uart_transmit(struct uart_pins *uart_pin, int data)
{
	uint8_t temp_data = data;
	HAL_UART_Transmit(uart_pin->uarthandle, &temp_data, 1, 10);
}

void uart_transmit_ascii(struct uart_pins *uart_pin, char *pData)
{
	HAL_UART_Transmit(uart_pin->uarthandle,(uint8_t *)(pData), strlen((const char*)pData), 25);
}

void uartrx_isr(void isrhandler(void))
{
	isrhandler();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &uarthandle1)
	{
		uartrx_isr(uart1.uartrx_isrfunction);
		uart1.state = HAL_UART_Receive_IT(huart, &rx_buff1, 1);
	}
	else if(huart == &uarthandle2)
	{
		uartrx_isr(uart2.uartrx_isrfunction);
		uart2.state = HAL_UART_Receive_IT(huart, &rx_buff2, 1);
	}
	else if(huart == &uarthandle3)
	{
		uartrx_isr(uart3.uartrx_isrfunction);
		uart3.state = HAL_UART_Receive_IT(huart, &rx_buff3, 1);
	}
	else if(huart == &uarthandle4)
	{
		uartrx_isr(uart4.uartrx_isrfunction);
		uart4.state = HAL_UART_Receive_IT(huart, &rx_buff4, 1);
	}
	else if(huart == &uarthandle5)
	{
		uartrx_isr(uart5.uartrx_isrfunction);
		uart5.state = HAL_UART_Receive_IT(huart, &rx_buff5, 1);
	}
	else if(huart == &uarthandle6)
	{
		uartrx_isr(uart6.uartrx_isrfunction);
		uart6.state = HAL_UART_Receive_IT(huart, &rx_buff6, 1);
	}
}




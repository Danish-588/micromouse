/*
 * uart.c
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */


#include "stm32f4xx.h"

void UART_Init(uint32_t baudrate)
{
    // Enable clock for GPIOA and USART2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // Enable clock for GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // Enable clock for USART2

    // Configure PA2 (TX) and PA3 (RX) as alternate function (AF7 for USART2)
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);  // Clear mode bits for PA2, PA3
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1); // Set to alternate function
    GPIOA->AFR[0] |= 0x7700;                // Set AF7 for PA2 (TX) and PA3 (RX)

    // Configure USART2
    USART2->CR1 = 0;                        // Clear control register
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable Transmitter and Receiver
    USART2->BRR = SystemCoreClock / 2 / baudrate; // Set baud rate
    USART2->CR1 |= USART_CR1_UE;            // Enable USART2
}

void UART_Transmit(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        USART2->DR = data[i];               // Write data to the Data Register
        while (!(USART2->SR & USART_SR_TXE)); // Wait until transmit data register is empty
    }
    while (!(USART2->SR & USART_SR_TC));     // Wait for transmission complete
}

uint8_t UART_Receive(void)
{
    while (!(USART2->SR & USART_SR_RXNE));   // Wait until data is received
    return USART2->DR;                       // Read received data
}

void UART_SendString(char *str)
{
    while (*str)
    {
        UART_Transmit((uint8_t *)str++, 1);  // Transmit each character
    }
}

/*
 * i2c.c
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */


#include "stm32f4xx.h"

void I2C_Init(void)
{
    // Enable clocks for GPIOB and I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   // Enable clock for GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;    // Enable clock for I2C1

    // Configure PB6 (SCL) and PB7 (SDA) as alternate function (AF4 for I2C1)
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);  // Clear mode bits
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);  // Set to alternate function
    GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);  // Open-drain
    GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL6_Pos) | (4 << GPIO_AFRL_AFSEL7_Pos);  // AF4 for I2C1

    // Reset and configure I2C1
    I2C1->CR1 |= I2C_CR1_SWRST;    // Software reset I2C1
    I2C1->CR1 &= ~I2C_CR1_SWRST;   // Clear reset

    // Configure I2C timings
    I2C1->CR2 |= (16);             // Set peripheral clock to 16 MHz
    I2C1->CCR = 80;                // Set clock control register (100 kHz)
    I2C1->TRISE = 17;              // Set maximum rise time (1000 ns)

    I2C1->CR1 |= I2C_CR1_PE;       // Enable I2C1 peripheral
}

void I2C_Start(void)
{
    I2C1->CR1 |= I2C_CR1_START;     // Generate START condition
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for start bit to be set
}

void I2C_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;      // Generate STOP condition
}

void I2C_WriteAddress(uint8_t address)
{
    I2C1->DR = address;            // Send address
    while (!(I2C1->SR1 & I2C_SR1_ADDR));  // Wait for address to be acknowledged
    (void)I2C1->SR2;               // Clear the ADDR flag by reading SR1 and SR2
}

void I2C_WriteData(uint8_t data)
{
    I2C1->DR = data;               // Send data
    while (!(I2C1->SR1 & I2C_SR1_TXE));  // Wait until data register is empty
}

uint8_t I2C_ReadData(void)
{
    while (!(I2C1->SR1 & I2C_SR1_RXNE));  // Wait for data to be received
    return I2C1->DR;                      // Return received data
}

void I2C_Ack(void)
{
    I2C1->CR1 |= I2C_CR1_ACK;      // Enable ACK
}

void I2C_Nack(void)
{
    I2C1->CR1 &= ~I2C_CR1_ACK;     // Disable ACK (send NACK)
}

uint8_t I2C_Read(uint8_t address)
{
    I2C_Start();                   // Generate START condition
    I2C_WriteAddress(address | 1); // Send address with read bit
    I2C_Ack();                     // Enable ACK for receiving multiple bytes
    uint8_t data = I2C_ReadData();  // Read data
    I2C_Nack();                    // Send NACK for the last byte
    I2C_Stop();                    // Generate STOP condition
    return data;                   // Return the read data
}

void I2C_Write(uint8_t address, uint8_t data)
{
    I2C_Start();                   // Generate START condition
    I2C_WriteAddress(address);      // Send address with write bit
    I2C_WriteData(data);            // Send the data
    I2C_Stop();                     // Generate STOP condition
}

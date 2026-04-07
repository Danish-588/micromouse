/*
 * i2c.c
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */


#include "stm32f4xx.h"

void I2C_Init(void)
{
    // Enable clock for I2C1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Enable clock for GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure GPIO pins for I2C (SCL on PB6, SDA on PB7)
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // Set PB6 and PB7 to alternate function mode
    GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);        // Set PB6 and PB7 as open-drain

    // Enable pull-up resistors for PB6 and PB7
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);   // Set PB6 and PB7 to pull-up mode

    // Set alternate function AF4 (I2C1) for PB6 and PB7
    GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL6_Pos) | (4 << GPIO_AFRL_AFSEL7_Pos); // AF4 for I2C1

    // Reset and configure I2C1
    I2C1->CR1 &= ~I2C_CR1_PE;    // Disable I2C1 for configuration
    I2C1->CR2 = 16;              // Set peripheral clock to 16 MHz (adjust for your clock settings)
    I2C1->CCR = 80;              // Set clock control register (100kHz I2C)
    I2C1->TRISE = 17;            // Set maximum rise time (1000ns for 100kHz)

    I2C1->CR1 |= I2C_CR1_PE;     // Enable I2C1
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

uint8_t I2C_Read(uint8_t address, uint8_t reg)
{
    I2C_Start();                  // Start I2C communication
    I2C_WriteAddress(address);     // Send the I2C address
    I2C_WriteData(reg);            // Send the register address

    I2C_Start();                   // Send repeated start condition
    I2C_WriteAddress(address | 0x01); // Send address with read bit

    uint8_t data = I2C_ReadData(); // Read data
    I2C_Nack();                    // Send NACK
    I2C_Stop();                    // Stop I2C communication
    return data;                   // Return the data
}

void I2C_ReadMulti(uint8_t address, uint8_t reg, uint8_t *data, uint16_t length)
{
    I2C_Start();                      // Start I2C communication
    I2C_WriteAddress(address);         // Send the I2C address
    I2C_WriteData(reg);                // Send the register address

    I2C_Start();                       // Send repeated start condition
    I2C_WriteAddress(address | 0x01);  // Send address with read bit

    for (int i = 0; i < length; i++) {
        data[i] = I2C_ReadData();      // Read each byte
        if (i == length - 1)
            I2C_Nack();                // Send NACK for last byte
        else
            I2C_Ack();                 // Send ACK for each byte
    }

    I2C_Stop();                        // Stop I2C communication
}

void I2C_Write(uint8_t address, uint8_t reg, uint8_t data)
{
    I2C_Start();                  // Start I2C communication
    I2C_WriteAddress(address);     // Send the I2C address
    I2C_WriteData(reg);            // Send the register address
    I2C_WriteData(data);           // Send the data
    I2C_Stop();                    // Stop I2C communication
}


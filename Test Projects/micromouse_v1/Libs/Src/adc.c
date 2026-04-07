/*
 * adc.c
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */


#include "stm32f4xx.h"

void ADC_Init(void)
{
    // Enable the clock for ADC1 and GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // Enable clock for ADC1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable clock for GPIOA

    // Configure PA0 as analog mode (ADC Channel 0)
    GPIOA->MODER |= GPIO_MODER_MODER0;     // Set PA0 to analog mode
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;    // No pull-up/pull-down

    // ADC configuration
    ADC1->CR2 = 0;                         // Clear control register 2
    ADC1->SQR3 = 0;                        // Clear regular sequence register (Channel 0)
    ADC1->SQR1 = 0;                        // Set sequence length to 1 conversion
    ADC1->CR1 = 0;                         // 12-bit resolution (default)
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;         // Sampling time for channel 0 (15 cycles)
    ADC1->CR2 |= ADC_CR2_ADON;             // Enable ADC1
}

uint16_t ADC_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;          // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));      // Wait for conversion to finish
    return (uint16_t)ADC1->DR;             // Read ADC value
}

void ADC_Enable(void)
{
    ADC1->CR2 |= ADC_CR2_ADON;             // Enable ADC
}

void ADC_Disable(void)
{
    ADC1->CR2 &= ~ADC_CR2_ADON;            // Disable ADC
}

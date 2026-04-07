/*
 * pwm.c
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */


#include "stm32f4xx.h"

void PWM_Init(void)
{
    // Enable clock for GPIOA and TIM3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // Enable clock for GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     // Enable clock for TIM3

    // Set PA6 (TIM3 CH1) as alternate function
    GPIOA->MODER &= ~(GPIO_MODER_MODER6);  // Clear mode bits for PA6
    GPIOA->MODER |= GPIO_MODER_MODER6_1;   // Set PA6 to alternate function mode
    GPIOA->AFR[0] |= 0x02000000;           // Set AF2 (TIM3_CH1) for PA6

    // Configure TIM3 for PWM mode
    TIM3->PSC = 16 - 1;  // Set prescaler to 16 (1 MHz timer clock)
    TIM3->ARR = 1000 - 1; // Set auto-reload value (for 1 kHz PWM frequency)

    // Configure PWM mode on Channel 1
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;         // Clear the OC1M bits
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // Set PWM mode 1 on channel 1 (110 in OC1M bits)
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;         // Enable preload register on channel 1
    TIM3->CCER |= TIM_CCER_CC1E;            // Enable output for channel 1

    // Set the duty cycle (initially 0%)
    TIM3->CCR1 = 0;

    // Enable the timer
    TIM3->CR1 |= TIM_CR1_CEN;               // Enable the timer counter
}


void PWM_SetDutyCycle(uint8_t dutyCycle)
{
    // Set the duty cycle (0-100%)
    if (dutyCycle > 100) dutyCycle = 100;   // Limit duty cycle to 100%
    TIM3->CCR1 = (dutyCycle * (TIM3->ARR + 1)) / 100;
}

void PWM_Start(void)
{
    TIM3->CR1 |= TIM_CR1_CEN;             // Start Timer
}

void PWM_Stop(void)
{
    TIM3->CR1 &= ~TIM_CR1_CEN;            // Stop Timer
}

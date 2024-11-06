/*
 * gpio.c
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */


#include "stm32f4xx.h"

void GPIO_Init_Output(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    // Enable clock for the corresponding GPIO port
    if (GPIOx == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (GPIOx == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (GPIOx == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (GPIOx == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (GPIOx == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    // Set pin as output
    GPIOx->MODER &= ~(0x3 << (pin * 2));  // Clear mode bits
    GPIOx->MODER |= (0x1 << (pin * 2));   // Set pin as output

    // Set output type to push-pull
    GPIOx->OTYPER &= ~(0x1 << pin);

    // Set pin speed to high
    GPIOx->OSPEEDR |= (0x3 << (pin * 2));

    // Disable pull-up/pull-down resistors
    GPIOx->PUPDR &= ~(0x3 << (pin * 2));
}

void GPIO_Init_Input(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    // Enable clock for the corresponding GPIO port
    if (GPIOx == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (GPIOx == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (GPIOx == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (GPIOx == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (GPIOx == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    // Set pin as input
    GPIOx->MODER &= ~(0x3 << (pin * 2));  // Clear mode bits

    // Disable pull-up/pull-down resistors (optional, adjust if needed)
    GPIOx->PUPDR &= ~(0x3 << (pin * 2));
}

void GPIO_SetPin(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    GPIOx->BSRR = (1 << pin);  // Set pin (high)
}

void GPIO_ResetPin(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    GPIOx->BSRR = (1 << (pin + 16));  // Reset pin (low)
}

void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    GPIOx->ODR ^= (1 << pin);  // Toggle pin
}

uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    return (GPIOx->IDR & (1 << pin)) ? 1 : 0;  // Read pin state (0 or 1)
}

/*
 * gpio.c
 *
 *  Created on: Jun 21, 2021
 *      Author: Soham
 */

#include "main.h"

struct exti_pins
pin0 = {emptyfunc,0},
pin1 = {emptyfunc,0},
pin2 = {emptyfunc,0},
pin3 = {emptyfunc,0},
pin4 = {emptyfunc,0},
pin5 = {emptyfunc,0},
pin6 = {emptyfunc,0},
pin7 = {emptyfunc,0},
pin8 = {emptyfunc,0},
pin9 = {emptyfunc,0},
pin10 = {emptyfunc,0},
pin11 = {emptyfunc,0},
pin12 = {emptyfunc,0},
pin13 = {emptyfunc,0},
pin14 = {emptyfunc,0},
pin15 = {emptyfunc,0};


void exti_isr(void isrhandler(void))
{
	isrhandler();
}

void input(GPIO_TypeDef  *GPIOx, uint16_t pin, uint32_t pull)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = pull;

	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void output(GPIO_TypeDef  *GPIOx, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void setpin(GPIO_TypeDef  *GPIOx, uint16_t pin)
{
	HAL_GPIO_WritePin(GPIOx, pin, 1);
}

void resetpin(GPIO_TypeDef  *GPIOx, uint16_t pin)
{
	HAL_GPIO_WritePin(GPIOx, pin, 0);
}

void togglepin(GPIO_TypeDef  *GPIOx, uint16_t pin)
{
	HAL_GPIO_TogglePin(GPIOx, pin);
}

void sendtoled(int value)
{
	for(int i = 0; i<12; i++)
	{
		HAL_GPIO_WritePin(GPIOC, 1<<i, (value>>i)&0x1);
	}
}

void gpio_interrupt(GPIO_TypeDef  *GPIOx, uint16_t pin, uint32_t pull,uint32_t edge,void (*exti_handler)(void))
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = edge;
	GPIO_InitStruct.Pull = pull;

	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	if(pin&PIN0)
	{
		pin0.exti_isrfunction = exti_handler;
		HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	}
	if(pin&PIN1)
	{
		pin1.exti_isrfunction = exti_handler;
		HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	}
	if(pin&PIN2)
	{
		pin2.exti_isrfunction = exti_handler;
		HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	}
	if(pin&PIN3)
	{
		pin3.exti_isrfunction = exti_handler;
		HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}
	if(pin&PIN4)
	{
		pin4.exti_isrfunction = exti_handler;
		HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	}
	if(pin&(PIN5|PIN6|PIN7|PIN8|PIN9))
	{
		if(pin&PIN5)
		{
			pin5.enable = 1;
			pin5.exti_isrfunction = exti_handler;
		}
		if(pin&PIN6)
		{
			pin6.enable = 1;
			pin6.exti_isrfunction = exti_handler;
		}
		if(pin&PIN7)
		{
			pin7.enable = 1;
			pin7.exti_isrfunction = exti_handler;
		}
		if(pin&PIN8)
		{
			pin8.enable = 1;
			pin8.exti_isrfunction = exti_handler;
		}
		if(pin&PIN9)
		{
			pin9.enable = 1;
			pin9.exti_isrfunction = exti_handler;
		}
		HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	if(pin&(PIN10|PIN11|PIN12|PIN13|PIN14|PIN15))
	{
		if(pin&PIN10)
		{
			pin10.enable = 1;
			pin10.exti_isrfunction = exti_handler;
		}
		if(pin&PIN11)
		{
			pin11.enable = 1;
			pin11.exti_isrfunction = exti_handler;
		}
		if(pin&PIN12)
		{
			pin12.enable = 1;
			pin12.exti_isrfunction = exti_handler;
		}
		if(pin&PIN13)
		{
			pin13.enable = 1;
			pin13.exti_isrfunction = exti_handler;
		}
		if(pin&PIN14)
		{
			pin14.enable = 1;
			pin14.exti_isrfunction = exti_handler;
		}
		if(pin&PIN15)
		{
			pin15.enable = 1;
			pin15.exti_isrfunction = exti_handler;
		}
		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case PIN0:
		exti_isr(pin0.exti_isrfunction);
		isr_ext0_check++;
		break;

	case PIN1:
		exti_isr(pin1.exti_isrfunction);
		isr_ext1_check++;
		break;

	case PIN2:
		exti_isr(pin2.exti_isrfunction);
		isr_ext2_check++;
		break;

	case PIN3:
		exti_isr(pin3.exti_isrfunction);
		isr_ext3_check++;
		break;

	case PIN4:
		exti_isr(pin4.exti_isrfunction);
		isr_ext4_check++;
		break;

	case PIN5:
		exti_isr(pin5.exti_isrfunction);
		isr_ext5_check++;
		break;

	case PIN6:
		exti_isr(pin6.exti_isrfunction);
		isr_ext6_check++;
		break;

	case PIN7:
		exti_isr(pin7.exti_isrfunction);
		isr_ext7_check++;
		break;

	case PIN8:
		exti_isr(pin8.exti_isrfunction);
		isr_ext8_check++;
		break;

	case PIN9:
		exti_isr(pin9.exti_isrfunction);
		isr_ext9_check++;
		break;

	case PIN10:
		exti_isr(pin10.exti_isrfunction);
		isr_ext10_check++;
		break;

	case PIN11:
		exti_isr(pin11.exti_isrfunction);
		isr_ext11_check++;
		break;

	case PIN12:
		exti_isr(pin12.exti_isrfunction);
		isr_ext12_check++;
		break;

	case PIN13:
		exti_isr(pin13.exti_isrfunction);
		isr_ext13_check++;
		break;

	case PIN14:
		exti_isr(pin14.exti_isrfunction);
		isr_ext14_check++;
		break;

	case PIN15:
		exti_isr(pin15.exti_isrfunction);
		isr_ext15_check++;
		break;
	}
}

/*
 * timer.c
 *
 *  Created on: Jun 20, 2021
 *      Author: Soham
 */

#include "main.h"

struct isr_timer
timer1 = {TIM1,&isr_timer1,emptyfunc},
timer2 = {TIM2,&isr_timer2,emptyfunc},
timer3 = {TIM3,&isr_timer3,emptyfunc},
timer4 = {TIM4,&isr_timer4,emptyfunc},
timer5 = {TIM5,&isr_timer5,emptyfunc},
timer6 = {TIM6,&isr_timer6,emptyfunc},
timer7 = {TIM7,&isr_timer7,emptyfunc},
timer8 = {TIM8,&isr_timer8,emptyfunc},
timer9 = {TIM9,&isr_timer9,emptyfunc},
timer10 = {TIM10,&isr_timer10,emptyfunc},
timer11 = {TIM11,&isr_timer11,emptyfunc},
timer12 = {TIM12,&isr_timer12,emptyfunc},
timer13 = {TIM13,&isr_timer13,emptyfunc},
timer14 = {TIM14,&isr_timer14,emptyfunc};

volatile int isr_timer1_check = 0;
volatile int isr_timer2_check = 0;
volatile int isr_timer3_check = 0;
volatile int isr_timer4_check = 0;
volatile int isr_timer5_check = 0;
volatile int isr_timer6_check = 0;
volatile int isr_timer7_check = 0;
volatile int isr_timer8_check = 0;
volatile int isr_timer9_check = 0;
volatile int isr_timer10_check = 0;
volatile int isr_timer11_check = 0;
volatile int isr_timer12_check = 0;
volatile int isr_timer13_check = 0;
volatile int isr_timer14_check = 0;


void timer_config(struct isr_timer *timer, float freq)
{
	float temp_freq;
	float temp_prescaler;

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	(timer->timerhandle)->Instance = timer->timer;



	if((timer->timer == TIM2)||(timer->timer == TIM3)||(timer->timer == TIM4)||(timer->timer == TIM5)||(timer->timer == TIM6)||(timer->timer == TIM7)||(timer->timer == TIM12)||(timer->timer == TIM13)||(timer->timer == TIM14))
	{
		temp_freq = round(90000000.0/freq);
		if((temp_freq>65536)&&((timer->timer == TIM3)||(timer->timer == TIM4)||(timer->timer == TIM6)||(timer->timer == TIM7)||(timer->timer == TIM12)||(timer->timer == TIM13)||(timer->timer == TIM14)))
		{
			temp_prescaler = fmin(65536,ceil(temp_freq/65536));
		}
		else if((temp_freq>4294967296)&&((timer->timer == TIM2)||(timer->timer == TIM5)))
		{
			temp_prescaler = fmin(4294967296,ceil(temp_freq/4294967296));
		}
		else
		{
			temp_prescaler = 1;
		}
	}
	else if((timer->timer == TIM1)||(timer->timer == TIM8)||(timer->timer == TIM9)||(timer->timer == TIM10)||(timer->timer == TIM11))
	{
		temp_freq = round(180000000.0/freq);
		if(temp_freq>65536)
		{
			temp_prescaler = fmin(65536,ceil(temp_freq/65536));
		}
		else
		{
			temp_prescaler = 1;
		}
	}

	(timer->timerhandle)->Init.Prescaler = temp_prescaler - 1;
	(timer->timerhandle)->Init.CounterMode = TIM_COUNTERMODE_UP;
	(timer->timerhandle)->Init.Period = round(temp_freq/temp_prescaler)-1;
	(timer->timerhandle)->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	(timer->timerhandle)->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(timer->timerhandle) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	if (HAL_TIM_ConfigClockSource(timer->timerhandle, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(timer->timerhandle, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}


void timer_init(struct isr_timer *timer, float freq, void (*timer_handler)(void))
{
	timer->timerisrfunction = timer_handler;
	timer_config(timer, freq);
	HAL_TIM_Base_Start_IT(timer->timerhandle);
}

void timer_isr(void isrhandler(void))
{
	isrhandler();
}

void emptyfunc()
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &isr_timer1)
	{
		timer_isr(timer1.timerisrfunction);
		isr_timer1_check++;
	}

	else if(htim == &isr_timer2)
	{
		timer_isr(timer2.timerisrfunction);
		isr_timer2_check++;
	}

	else if(htim == &isr_timer3)
	{
		timer_isr(timer3.timerisrfunction);
		isr_timer3_check++;
	}

	else if(htim == &isr_timer4)
	{
		timer_isr(timer4.timerisrfunction);
		isr_timer4_check++;
	}

	else if(htim == &isr_timer5)
	{
		timer_isr(timer5.timerisrfunction);
		isr_timer5_check++;
	}

	else if(htim == &isr_timer6)
	{
		timer_isr(timer6.timerisrfunction);
		isr_timer6_check++;
	}

	else if(htim == &isr_timer7)
	{
		timer_isr(timer7.timerisrfunction);
		isr_timer7_check++;
	}

	else if(htim == &isr_timer8)
	{
		timer_isr(timer8.timerisrfunction);
		isr_timer8_check++;
	}

	else if(htim == &isr_timer9)
	{
		timer_isr(timer9.timerisrfunction);
		isr_timer9_check++;
	}

	else if(htim == &isr_timer10)
	{
		timer_isr(timer10.timerisrfunction);
		isr_timer10_check++;
	}

	else if(htim == &isr_timer11)
	{
		timer_isr(timer11.timerisrfunction);
		isr_timer11_check++;
	}

	else if(htim == &isr_timer12)
	{
		timer_isr(timer12.timerisrfunction);
		isr_timer12_check++;
	}

	else if(htim == &isr_timer13)
	{
		timer_isr(timer13.timerisrfunction);
		isr_timer13_check++;
	}

	else if(htim == &isr_timer14)
	{
		timer_isr(timer14.timerisrfunction);
		isr_timer14_check++;
	}
}

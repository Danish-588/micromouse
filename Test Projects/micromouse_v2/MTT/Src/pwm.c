/*
 * pwm.c
 *
 *  Created on: Jun 19, 2021
 *      Author: Soham
 */
#include "main.h"

struct pwm_pins
A11={TIM1,&pwm_timer1,12,0},
A0={TIM2,&pwm_timer2,0,0},
A1={TIM2,&pwm_timer2,4,0},
A2={TIM2,&pwm_timer2,8,0},
A3={TIM2,&pwm_timer2,12,0},
A6={TIM3,&pwm_timer3,0,0},
A7={TIM3,&pwm_timer3,4,0},
B0={TIM3,&pwm_timer3,8,0},
B1={TIM3,&pwm_timer3,12,0},
B6={TIM4,&pwm_timer4,0,0},
B7={TIM4,&pwm_timer4,4,0},
B8={TIM4,&pwm_timer4,8,0},
B9={TIM4,&pwm_timer4,12,0},
A0_t5={TIM5,&pwm_timer5,0,0},
A1_t5={TIM5,&pwm_timer5,4,0},
A2_t5={TIM5,&pwm_timer5,8,0},
A3_t5={TIM5,&pwm_timer5,12,0},
C6={TIM8,&pwm_timer8,0,0},
C7={TIM8,&pwm_timer8,4,0},
C8={TIM8,&pwm_timer8,8,0},
C9={TIM8,&pwm_timer8,12,0},
A2_t9={TIM9,&pwm_timer9,0,0},
A3_t9={TIM9,&pwm_timer9,4,0},
B14={TIM12,&pwm_timer12,0,0},
B15={TIM12,&pwm_timer12,4,0};

void pwm_config(struct pwm_pins *pin, float duty, float freq,int perc)
{
	float temp_freq;
	float temp_prescaler;
	//TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	(pin->timerhandle)->Instance = pin->timer;

	if((pin->timer == TIM2)||(pin->timer == TIM3)||(pin->timer == TIM4)||(pin->timer == TIM5)||(pin->timer == TIM12))
	{
		temp_freq = round(90000000.0/freq);
		if((temp_freq>65536)&&((pin->timer == TIM3)||(pin->timer == TIM4)||(pin->timer == TIM12)))
		{
			temp_prescaler = fmin(65536,ceil(temp_freq/65536));
		}
		else if((temp_freq>4294967296)&&((pin->timer == TIM2)||(pin->timer == TIM5)))
		{
			temp_prescaler = fmin(4294967296,ceil(temp_freq/4294967296));
		}
		else
		{
			temp_prescaler = 1;
		}
	}
	else if((pin->timer == TIM1)||(pin->timer == TIM8)||(pin->timer == TIM9))
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
//	temp_freq = round(90000000/freq);
//
//	if(((pin->timer == TIM1)||(pin->timer == TIM3)||(pin->timer == TIM4))||((pin->timer == TIM8)||(pin->timer == TIM9)||(pin->timer == TIM12)))
//	{
//		if(temp_freq>65536)
//		{
//			temp_prescaler = fmax(65536,ceil(temp_freq/65536));
//		}
//		else
//		{
//			temp_prescaler = 1;
//		}
//	}
//	else if((pin->timer == TIM2)||(pin->timer == TIM5))
//	{
//		if(temp_freq>4294967296)
//		{
//			temp_prescaler = fmax(4294967296,ceil(temp_freq/4294967296));
//		}
//		else
//		{
//			temp_prescaler = 1;
//		}
//	}

	(pin->timerhandle)->Init.Prescaler = temp_prescaler - 1;
	(pin->timerhandle)->Init.CounterMode = TIM_COUNTERMODE_UP;
	(pin->timerhandle)->Init.Period  = round(temp_freq/temp_prescaler)-1;
	(pin->timerhandle)->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	(pin->timerhandle)->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

//	if (HAL_TIM_Base_Init(pin->timerhandle) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//
//	if (HAL_TIM_ConfigClockSource(pin->timerhandle, &sClockSourceConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}

	if (HAL_TIM_PWM_Init(pin->timerhandle) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(pin->timerhandle, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;

	if(perc)
	{
				sConfigOC.Pulse = ((temp_freq/temp_prescaler)*(duty/100))-1;
	}
	else
	{
				sConfigOC.Pulse = duty-1;
	}
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(pin->timerhandle, &sConfigOC, pin->pwm_channel) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_MspPostInit(pin->timerhandle);
}

void pwm_init(struct pwm_pins *pin, float duty , float freq, int perc)
{
	pin->enable = 1;
	pwm_config(pin,duty,freq,perc);
	HAL_TIM_PWM_Start(pin->timerhandle,pin->pwm_channel);
}

void pwm_duty_update(struct pwm_pins *pin, float duty, int perc)
{
	int temp_duty = 0;
	if(perc)
	{
		temp_duty = ((pin->timerhandle)->Init.Period)*(duty/100.0);
	}
	else
	{
		temp_duty = duty;
	}
	__HAL_TIM_SET_COMPARE(pin->timerhandle,pin->pwm_channel, temp_duty);
}


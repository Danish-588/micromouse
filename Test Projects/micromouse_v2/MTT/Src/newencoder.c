/*
 * newencoder.c
 *
 *  Created on: Jul 5, 2021
 *      Author: Soham
 */

#include "main.h"

struct encoders
enc1 = {TIM1,&encoder1,0,0,0,0,1},
enc2 = {TIM6,&encoder2,0,0,0,0,1},
enc3 = {TIM3,&encoder3,0,0,0,0,1},
enc4 = {TIM4,&encoder4,0,0,0,0,1},
enc5_a = {TIM2,&encoder5_a,0,0,0,0,1},
enc5_b = {TIM5,&encoder5_b,0,0,0,0,1};

void encoder_config(TIM_HandleTypeDef *htim, TIM_TypeDef *TIM )
{
	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim->Instance = TIM;
	htim->Init.Prescaler = 0;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = 4294967295;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;

	if (HAL_TIM_Encoder_Init(htim, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void encoder_init(struct encoders *encoders, uint16_t ppr)
{
	encoders->ppr = ppr;
	encoder_config(encoders->encoderhandle,encoders->timer);
	HAL_TIM_Encoder_Start(encoders->encoderhandle,TIM_CHANNEL_ALL);
}

int get_enccount(struct encoders *encoders)
{
	int32_t temp32;
	int16_t temp16;
	if((encoders->timer == TIM1) || (encoders->timer == TIM3) || (encoders->timer == TIM4) || (encoders->timer == TIM8))
	{
		temp16 = (encoders->timer->CNT)*(encoders->encoder_dir);
		return temp16;
	}
	if((encoders->timer == TIM2) || (encoders->timer == TIM5))
	{
		temp32 = (encoders->timer->CNT)*(encoders->encoder_dir);
		return temp32;
	}
	return 0;
}

void clear_enccount(struct encoders *encoders)
{
	encoders->timer->CNT = 0;
}

void encoder_direction(struct encoders *encoders, int8_t dir)
{
	encoders->encoder_dir = dir;
}

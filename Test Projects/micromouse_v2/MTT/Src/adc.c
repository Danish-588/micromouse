/*
 * adc.c
 *
 *  Created on: Aug 27, 2021
 *      Author: Soham
 */

#include <main.h>

struct adc_pins
adc1_A0 = {ADC1,&adc1,GPIOA,0,0,ADC_CHANNEL_0},
adc1_A1 = {ADC1,&adc1,GPIOA,0,0,ADC_CHANNEL_1},
adc1_A2 = {ADC1,&adc1,GPIOA,0,0,ADC_CHANNEL_2},
adc1_A3 = {ADC1,&adc1,GPIOA,0,0,ADC_CHANNEL_3},
adc1_A4 = {ADC1,&adc1,GPIOA,0,0,ADC_CHANNEL_4},
adc1_A5 = {ADC1,&adc1,GPIOA,0,0,ADC_CHANNEL_5},
adc1_A6 = {ADC1,&adc1,GPIOA,0,0,ADC_CHANNEL_6},
adc1_A7 = {ADC1,&adc1,GPIOA,0,0,ADC_CHANNEL_7};

void adc_init(struct adc_pins *adc_pin, uint8_t rank)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	adc_pin->adc_handle->Instance = adc_pin->adc_typedef;
	adc_pin->adc_handle->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	adc_pin->adc_handle->Init.Resolution = ADC_RESOLUTION_12B;
	adc_pin->adc_handle->Init.ScanConvMode = DISABLE;
	adc_pin->adc_handle->Init.ContinuousConvMode = DISABLE;
	adc_pin->adc_handle->Init.DiscontinuousConvMode = DISABLE;
	adc_pin->adc_handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	adc_pin->adc_handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adc_pin->adc_handle->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc_pin->adc_handle->Init.NbrOfConversion = 1;
	adc_pin->adc_handle->Init.DMAContinuousRequests = DISABLE;
	adc_pin->adc_handle->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	adc_pin->adc_pin_enable = 1;
	if (HAL_ADC_Init(adc_pin->adc_handle) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = adc_pin->adc_channel;
	sConfig.Rank = rank;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(adc_pin->adc_handle, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/*
 * adc.h
 *
 *  Created on: Aug 27, 2021
 *      Author: Soham
 */

#ifndef ADC_H_
#define ADC_H_


ADC_HandleTypeDef adc1;
ADC_HandleTypeDef adc2;
ADC_HandleTypeDef adc3;
DMA_HandleTypeDef dma_adc1;

struct adc_pins
{
	ADC_TypeDef *adc_typedef;
	ADC_HandleTypeDef *adc_handle;
	GPIO_TypeDef  *adc_port;
	uint16_t adc_pin;
	uint8_t adc_pin_enable;
	uint32_t adc_channel;
}adc1_A0,adc1_A1,adc1_A2,adc1_A3,adc1_A4,adc1_A5,adc1_A6,adc1_A7;

void adc_init(struct adc_pins *adc_pin,uint8_t rank);
void adc_dma_init(struct adc_pins *adc_pin,uint8_t rank);
void adc_dma_fullinit(uint32_t *adc_storage);

#endif /* ADC_H_ */

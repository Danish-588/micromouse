/*
 * pwm.h
 *
 *  Created on: Jun 19, 2021
 *      Author: Soham
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_


TIM_HandleTypeDef pwm_timer1;
TIM_HandleTypeDef pwm_timer2;
TIM_HandleTypeDef pwm_timer3;
TIM_HandleTypeDef pwm_timer4;
TIM_HandleTypeDef pwm_timer5;
TIM_HandleTypeDef pwm_timer8;
TIM_HandleTypeDef pwm_timer9;
TIM_HandleTypeDef pwm_timer12;
struct pwm_pins
{
	TIM_TypeDef *timer;
	TIM_HandleTypeDef *timerhandle;
	uint32_t pwm_channel;
	int enable;
}A11,A0,A1,A2,A3,A6,A7,B0,B1,B6,B7,B8,B9,A0_t5,A1_t5,A2_t5,A3_t5,C6,C7,C8,C9,A2_t9,A3_t9,B14,B15;


void pwm_config(struct pwm_pins *pin, float duty, float freq,int perc);
void pwm_init(struct pwm_pins *pin, float duty , float freq, int perc);
void pwm_duty_update(struct pwm_pins *pin, float duty, int perc);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif /* INC_PWM_H_ */

/*
 * timer.h
 *
 *  Created on: Jun 20, 2021
 *      Author: Soham
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

TIM_HandleTypeDef isr_timer1;
TIM_HandleTypeDef isr_timer2;
TIM_HandleTypeDef isr_timer3;
TIM_HandleTypeDef isr_timer4;
TIM_HandleTypeDef isr_timer5;
TIM_HandleTypeDef isr_timer6;
TIM_HandleTypeDef isr_timer7;
TIM_HandleTypeDef isr_timer8;
TIM_HandleTypeDef isr_timer9;
TIM_HandleTypeDef isr_timer10;
TIM_HandleTypeDef isr_timer11;
TIM_HandleTypeDef isr_timer12;
TIM_HandleTypeDef isr_timer13;
TIM_HandleTypeDef isr_timer14;

typedef void (*timerisrfunciton)(void);

struct isr_timer
{
	TIM_TypeDef *timer;
	TIM_HandleTypeDef *timerhandle;
	timerisrfunciton timerisrfunction;
}
timer1,
timer2,
timer3,
timer4,
timer5,
timer6,
timer7,
timer8,
timer9,
timer10,
timer11,
timer12,
timer13,
timer14;

void timer_config(struct isr_timer *timer, float freq);
void timer_init(struct isr_timer *timer, float freq, void (*timerisrfunciton)(void));
void timer_isr(void isrhandler(void));
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void emptyfunc(void);

#endif /* INC_TIMER_H_ */

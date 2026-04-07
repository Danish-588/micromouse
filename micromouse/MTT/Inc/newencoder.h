/*
 * newencoder.h
 *
 *  Created on: Jul 5, 2021
 *      Author: Soham
 */

#ifndef INC_NEWENCODER_H_
#define INC_NEWENCODER_H_

struct encoders
{
	TIM_TypeDef *timer;
	TIM_HandleTypeDef *encoderhandle;
	int32_t pos;
	int16_t rpm;
	uint8_t rpm_enable;
	float ppr;
	int8_t encoder_dir;
}enc1,enc2,enc3,enc4,enc5_a,enc5_b;

TIM_HandleTypeDef encoder1;
TIM_HandleTypeDef encoder2;
TIM_HandleTypeDef encoder3;
TIM_HandleTypeDef encoder4;
TIM_HandleTypeDef encoder5_a;
TIM_HandleTypeDef encoder5_b;

void encoder_config(TIM_HandleTypeDef *htim, TIM_TypeDef *TIM );
void encoder_init(struct encoders *encoders, uint16_t ppr);
int get_enccount(struct encoders *encoders);
void clear_enccount(struct encoders *encoders);
void encoder_direction(struct encoders *encoders, int8_t dir);

#endif /* INC_NEWENCODER_H_ */

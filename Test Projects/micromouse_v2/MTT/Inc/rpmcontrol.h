/*
 * rpmcontrol.h
 *
 *  Created on: Jun 21, 2021
 *      Author: Soham
 */

#ifndef INC_RPMCONTROL_H_
#define INC_RPMCONTROL_H_

struct motor
{
	int freq;
	float kp;
	float ki;
	float kd;
	float e_p_limit;
	float e_i_limit;
	float pos_kp;
	float pos_ki;
	float pos_kd;
	float pos_e_p_limit;
	float pos_e_i_limit;
	float prev_cnt;
	uint16_t ppr;
}re40,power_window,ig32_264,ig32_35;

struct driver
{
	struct pwm_pins *pwm_pin;
	struct motor *motor;
	GPIO_TypeDef  *direction_port;
	uint32_t direction_pin;
	struct encoders *encoder;
	float past_rpm;
	float curr_rpm;
	int max_duty;
	int flag;
	int corrected_rpm;
}cytron_A6,cytron_B15,cytron_B8;

int rpm_freq;

void rpm_init(void (*rpm_isr)(void));
void driver_init(struct driver *driver,struct motor *motor);
void driver_clockwise(struct driver *driver, int duty);
void driver_anticlockwise(struct driver *driver, int duty);
void rpm_control(struct driver *driver, float rqrd_rpm);
float position_control(struct driver *driver, int rqrd_count, int max_rpm);
void driver_direction(struct driver *driver, int8_t dir);

#endif /* INC_RPMCONTROL_H_ */

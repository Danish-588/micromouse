/*
 * rpmcontrol.c
 *
 *  Created on: Jun 21, 2021
 *      Author: Soham
 */
#include "main.h"

struct motor
re40 = {32000,1,-0.0001,0.5,5,5,0,0,0,0,0,0,24500},
power_window = {32000,1,0,0,5,5,1,0,0,5,5,0,8192},
ig32_264 = {20000,10,0,0,5,5,0.15,0,0,5,5,0,7392},
ig32_35 = {20000,1,0,0,5,5,1,0,0,5,5,0,980};

struct driver
cytron_A6 = {&A6,&re40,GPIOA,PIN7,&enc1,0,0,0,0},
cytron_B15 = {&B15,&power_window,GPIOB,PIN0,&enc5_a,0,0,0,0},
cytron_B8 = {&B8,&ig32_264,GPIOB,PIN9,&enc1,0,0,0,0};

void rpm_init(void (*rpm_isr)(void))
{
	rpm_freq = 200;
	timer_init(&timer5,rpm_freq,rpm_isr);
}

void driver_init(struct driver *driver,struct motor *motor)
{
	pwm_init(driver->pwm_pin,0,motor->freq,0);
	driver->max_duty = driver->pwm_pin->timerhandle->Init.Period;
	output(driver->direction_port,driver->direction_pin);
	resetpin(driver->direction_port,driver->direction_pin);
	encoder_init(driver->encoder,driver->motor->ppr);
	pid_init(&rpm,driver->motor->kp,driver->motor->ki,driver->motor->kd,driver->motor->e_p_limit,driver->motor->e_i_limit);
	pid_init(&pos,driver->motor->pos_kp,driver->motor->pos_ki,driver->motor->pos_kd,driver->motor->pos_e_p_limit,driver->motor->pos_e_i_limit);
}

void driver_clockwise(struct driver *driver, int duty)
{
	pwm_duty_update(driver->pwm_pin, duty, 0);
	setpin(driver->direction_port,driver->direction_pin);
}

void driver_anticlockwise(struct driver *driver, int duty)
{
	pwm_duty_update(driver->pwm_pin, duty, 0);
	resetpin(driver->direction_port,driver->direction_pin);
}

void rpm_control(struct driver *driver, float rqrd_rpm)
{
	int temp_corr;
	int curr_cnt = get_enccount(driver->encoder);
	driver->curr_rpm = (float)(curr_cnt-driver->motor->prev_cnt)*((rpm_freq*60.0)/driver->encoder->ppr);
	driver->motor->prev_cnt = curr_cnt;
	pid_update(&rpm,rqrd_rpm,driver->curr_rpm);
	temp_corr = rpm.output;


	if(driver->flag)
	{
		driver->corrected_rpm-=temp_corr;
		if(driver->corrected_rpm > driver->max_duty)
		{
			driver->corrected_rpm = driver->max_duty;
		}
		else if(driver->corrected_rpm<0)
		{
			driver->flag = 0;
			driver->corrected_rpm = 0;
		}

		driver_anticlockwise(driver, driver->corrected_rpm);
	}
	else
	{
		driver->corrected_rpm+=temp_corr;
		if(driver->corrected_rpm > driver->max_duty)
		{
			driver->corrected_rpm = driver->max_duty;
		}
		else if(driver->corrected_rpm<0)
		{
			driver->flag = 1;
			driver->corrected_rpm = 0;
		}

		driver_clockwise(driver, driver->corrected_rpm);
	}
}

float position_control(struct driver *driver, int rqrd_count, int max_rpm)
{
    float rqrd_rpm = 0;

    pid_update(&pos, rqrd_count,get_enccount(driver->encoder));

    if(pos.output>max_rpm)
    	rqrd_rpm = max_rpm;
    else if(pos.output<-max_rpm)
    	rqrd_rpm = -max_rpm;
    else
    	rqrd_rpm = pos.output;

    return rqrd_rpm;
}

void driver_direction(struct driver *driver, int8_t dir)
{
	encoder_direction(driver->encoder,dir);
}

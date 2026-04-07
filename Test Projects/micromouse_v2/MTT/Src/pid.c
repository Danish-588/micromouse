/*
 * pid.c
 *
 *  Created on: Jun 21, 2021
 *      Author: Soham
 */
#include "main.h"

void pid_init(struct pid *plant, float kp, float ki, float kd,float e_p_limit, float e_i_limit)
{
	plant->kp = kp;
	plant->ki = ki;
	plant->kd = kd;
	plant->e_p_limit = e_p_limit;
	plant->e_i_limit = e_i_limit;
}

void pid_update(struct pid *plant, float rqrd, float curr)
{
	plant->e_p = rqrd-curr;
	plant->e_d = curr-plant->past;

	if(fabs(plant->e_p)<plant->e_p_limit)
	{
		plant->e_i += plant->e_p;
	}
	if(plant->e_i>plant->e_i_limit)
	{
		plant->e_i = plant->e_i_limit;
	}
	else if(plant->e_i<((-1)*plant->e_i_limit))
	{
		plant->e_i = (-1)*plant->e_i_limit;
	}
	if(plant->e_p==0)
	{
		plant->e_i = 0;
	}

	plant->output = (plant->kp*plant->e_p)+(plant->ki*plant->e_i)+(plant->kd*plant->e_d);

	plant->past = curr;
}

void set_pid(struct pid *plant, float kp, float ki, float kd)
{
	plant->kp = kp;
	plant->ki = ki;
	plant->kd = kd;
}

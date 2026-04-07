/*
 * pid.h
 *
 *  Created on: Jun 21, 2021
 *      Author: Soham
 */

#ifndef INC_PID_H_
#define INC_PID_H_

struct pid
{
	float kp;
	float ki;
	float kd;
	float e_p;
	float e_i;
	float e_d;
	float e_p_limit;
	float e_i_limit;
	float past;
	float output;
}rpm,pos,orient;

void pid_init(struct pid *plant, float kp, float ki, float kd,float e_p_limit, float e_i_limit);
void pid_update(struct pid *plant, float rqrd, float curr);
void set_pid(struct pid *plant, float kp, float ki, float kd);

#endif /* INC_PID_H_ */

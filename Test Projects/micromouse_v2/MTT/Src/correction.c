/*
 * correction.c
 *
 *  Created on: Apr 11, 2023
 *      Author: Minika
 */

#include "main.h"

volatile bool angle_correct = 0;
volatile int error_angle_dir = 0;
volatile float req_angle = 0.0;

void calculate_deadreck()
{
    if((raw_angle - angle_deadreck) < 0)
        angle = 360 + (raw_angle - angle_deadreck);
    else if ((raw_angle - angle_deadreck) > 360)
        angle = (raw_angle - angle_deadreck) - 360;
    else
        angle = (raw_angle - angle_deadreck);
}

void angle_correction(void)
{
	float error_angle = (req_angle - angle) * (error_angle_dir);
	float circular_angle = angle;

    if (error_angle > 180)
    	circular_angle += 360;
    if (error_angle < -180)
    	circular_angle -= 360;

    pid_update(&orient, req_angle, circular_angle);

    if(orient.output > w_limit)
        w = w_limit;
    else if(orient.output < -w_limit)
        w = -w_limit;
    else
        w = orient.output;
}

//void xy_correction(void)
//{
//    if (velocity_x_xy > 0)
//    {
//        if (velocity_x_xy * accs_ka_accs < 0.6)
//        {
//            accs_vx_limit = 0.6;
//        }
//        else
//        {
//            accs_vx_limit = velocity_x_xy * accs_ka_accs;
//        }
//    }
//    if (velocity_x_xy < 0)
//    {
//        if (velocity_x_xy * accs_ka_accs > -0.6)
//        {
//            accs_vx_limit_neg = -0.6;
//        }
//        else
//        {
//            accs_vx_limit_neg = velocity_x_xy * accs_ka_accs;
//        }
//    }
//    if (velocity_x_xy == 0)
//    {
//        accs_vx_limit = 0.6;
//        accs_vx_limit_neg - 0.6;
//    }
//    if (velocity_y_xy > 0)
//    {
//        if (velocity_y_xy * accs_ka_accs < 0.6)
//        {
//            accs_vy_limit = 0.6;
//        }
//        else
//        {
//            accs_vy_limit = velocity_y_xy * accs_ka_accs;
//        }
//    }
//    if (velocity_y_xy < 0)
//    {
//        if (velocity_y_xy * accs_ka_accs > -0.6)
//        {
//            accs_vy_limit_neg = -0.6;
//        }
//        else
//        {
//            accs_vy_limit_neg = velocity_y_xy * accs_ka_accs;
//        }
//    }
//    if (velocity_y_xy == 0)
//    {
//        accs_vy_limit = 0.6;
//        accs_vy_limit_neg - 0.6;
//    }
//    if (accs_vx_limit > vx_limit)
//        accs_vx_limit = vx_limit;
//    if (accs_vx_limit_neg < -vx_limit)
//        accs_vx_limit_neg = -vx_limit;
//
//    if (accs_vy_limit > vy_limit)
//        accs_vy_limit = vy_limit;
//    if (accs_vy_limit_neg < -vy_limit)
//        accs_vy_limit_neg = -vy_limit;
//
//
//    error_y = (req_y - y_final) * (error_y_dir);
//    pid_update(&y_dist, error_y);
//
//    if (y_correct && accs_enable_y)
//    {
//        if (y_dist.output >= accs_vy_limit)
//            vy = accs_vy_limit;
//        if (y_dist.output <= accs_vy_limit_neg)
//            vy = accs_vy_limit_neg;
//        if ((y_dist.output < accs_vy_limit) && (y_dist.output > accs_vy_limit_neg))
//            vy = y_dist.output;
//    }
//    else if (y_correct && !accs_enable_y)
//    {
//        if (y_dist.output >= vy_limit)
//            vy = vy_limit;
//        if (y_dist.output <= -vy_limit)
//            vy = -vy_limit;
//        if ((y_dist.output < vy_limit) && (y_dist.output > -vy_limit))
//            vy = y_dist.output;
//    }
//
//
//    error_x = (req_x - x_final) * (error_x_dir);
//    pid_update(&x_dist, error_x);
//
//    if (x_correct && accs_enable_x)
//    {
//        if (x_dist.output >= accs_vx_limit)
//            vx = accs_vx_limit;
//        if (x_dist.output <= accs_vx_limit_neg)
//            vx = accs_vx_limit_neg;
//        if ((x_dist.output < accs_vx_limit) && (x_dist.output > accs_vx_limit_neg))
//            vx = x_dist.output;
//    }
//    else if (x_correct && !accs_enable_x)
//    {
//        if (x_dist.output >= vx_limit)
//            vx = vx_limit;
//        if (x_dist.output <= -vx_limit)
//            vx = -vx_limit;
//        if ((x_dist.output < vx_limit) && (x_dist.output > -vx_limit))
//            vx = x_dist.output;
//    }
//}
//
//void distance_sensor_xy_correction(float sensor_dist_x, float sensor_dist_y)
//{
//    if (velocity_x_xy > 0)
//    {
//        if (velocity_x_xy * accs_ka_accs < 0.6)
//        {
//            accs_vx_limit = 0.6;
//        }
//        else
//        {
//            accs_vx_limit = velocity_x_xy * accs_ka_accs;
//        }
//    }
//    if (velocity_x_xy < 0)
//    {
//        if (velocity_x_xy * accs_ka_accs > -0.6)
//        {
//            accs_vx_limit_neg = -0.6;
//        }
//        else
//        {
//            accs_vx_limit_neg = velocity_x_xy * accs_ka_accs;
//        }
//    }
//    if (velocity_x_xy == 0)
//    {
//        accs_vx_limit = 0.6;
//        accs_vx_limit_neg - 0.6;
//    }
//    if (velocity_y_xy > 0)
//    {
//        if (velocity_y_xy * accs_ka_accs < 0.6)
//        {
//            accs_vy_limit = 0.6;
//        }
//        else
//        {
//            accs_vy_limit = velocity_y_xy * accs_ka_accs;
//        }
//    }
//    if (velocity_y_xy < 0)
//    {
//        if (velocity_y_xy * accs_ka_accs > -0.6)
//        {
//            accs_vy_limit_neg = -0.6;
//        }
//        else
//        {
//            accs_vy_limit_neg = velocity_y_xy * accs_ka_accs;
//        }
//    }
//    if (velocity_y_xy == 0)
//    {
//        accs_vy_limit = 0.6;
//        accs_vy_limit_neg - 0.6;
//    }
//    if (accs_vx_limit > vx_limit)
//        accs_vx_limit = vx_limit;
//    if (accs_vx_limit_neg < -vx_limit)
//        accs_vx_limit_neg = -vx_limit;
//
//    if (accs_vy_limit > vy_limit)
//        accs_vy_limit = vy_limit;
//    if (accs_vy_limit_neg < -vy_limit)
//        accs_vy_limit_neg = -vy_limit;
//
//
//    error_sensor_y = (req_sensor_y - sensor_dist_y) * (error_sensor_y_dir);
//    pid_update(&y_sensor, error_sensor_y);
//
//    if (y_sensor_correct && accs_enable_y)
//    {
//        if (y_sensor.output >= accs_vy_limit)
//            vy = accs_vy_limit;
//        if (y_sensor.output <= accs_vy_limit_neg)
//            vy = accs_vy_limit_neg;
//        if ((y_sensor.output < accs_vy_limit) && (y_sensor.output > accs_vy_limit_neg))
//            vy = y_sensor.output;
//    }
//    else if (y_sensor_correct && !accs_enable_y)
//    {
//        if (y_sensor.output >= vy_limit)
//            vy = vy_limit;
//        if (y_sensor.output <= -vy_limit)
//            vy = -vy_limit;
//        if ((y_sensor.output < vy_limit) && (y_sensor.output > -vy_limit))
//            vy = y_sensor.output;
//    }
//
//
//    error_sensor_x = (req_sensor_x - sensor_dist_x) * (error_sensor_x_dir);
//    pid_update(&x_sensor, error_sensor_x);
//
//    if (x_sensor_correct && accs_enable_x)
//    {
//        if (x_sensor.output >= accs_vx_limit)
//            vx = accs_vx_limit;
//        if (x_sensor.output <= accs_vx_limit_neg)
//            vx = accs_vx_limit_neg;
//        if ((x_sensor.output < accs_vx_limit) && (x_sensor.output > accs_vx_limit_neg))
//            vx = x_sensor.output;
//    }
//    else if (x_sensor_correct && !accs_enable_x)
//    {
//        if (x_sensor.output >= vx_limit)
//            vx = vx_limit;
//        if (x_sensor.output <= -vx_limit)
//            vx = -vx_limit;
//        if ((x_sensor.output < vx_limit) && (x_sensor.output > -vx_limit))
//            vx = x_sensor.output;
//    }
//}
//
//void distance_sensor_angle_correction(float front_sensor, float back_sensor)
//{
//    error_sensor_angle = (req_sensor_angle - (front_sensor - back_sensor)) * (error_sensor_angle_dir);
//    pid_update(&orient_sensor, error_sensor_angle);
//
//    if(orient_sensor.output > w_limit)
//        w = w_limit;
//    else if(orient_sensor.output < -w_limit)
//        w = w_limit;
//    else
//        w = orient_sensor.output;
//
//}

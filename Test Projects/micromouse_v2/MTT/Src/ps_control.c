/*
 * ps_control.c
 *
 *  Created on: Apr 11, 2023
 *      Author: Mihika
 */
#include "main.h"

volatile bool r_deadzone = 0;
volatile bool rx_deadzone = 0;
volatile bool ry_deadzone = 0;
volatile bool l_deadzone = 0;
volatile uint8_t deadzone_limit_l = 20;
volatile uint8_t deadzone_limit_r = 25;
volatile uint8_t deadzone_limit_rx = 60;
volatile uint8_t deadzone_limit_ry = 60;
volatile int8_t before_sign = 0;
volatile int8_t after_sign = 0;
volatile float v_magnitude = 0;
volatile float vx_req = 0;
volatile float vy_req = 0;
volatile float vy_temp_req = 0;
volatile float vy_temp = 0;
volatile float v_limit = 0;
volatile float error_vx = 0;
volatile float error_vy = 0;
volatile float error_vy_temp = 0;
volatile float denom_v = 0;
volatile float denom_error = 0;
volatile float denom_ps = 0;
volatile float w_req = 0;
volatile float error_w = 0;

void linear_xaxis()
{
	rx_deadzone = abs(ps_analog[rx])<deadzone_limit_rx;
	ry_deadzone = abs(ps_analog[ry])<deadzone_limit_ry;

	if(rx_deadzone)
	{
		if(vx)
		{
			before_sign = (vx>0)-(vx<0);
			vx -= deaccs*before_sign;
			after_sign = (vx>0)-(vx<0);
			if(before_sign^after_sign)
			{
				vx = 0;
			}
		}
		vx_req = 0;
	}
	else
	{
		vx_req = v_limit*(ps_analog[rx]-deadzone_limit_rx)/(128.0 - deadzone_limit_rx);
		error_vx = vx_req-vx;
		if(error_vx)
		{
			before_sign = (error_vx>0)-(error_vx<0);
			vx += accs*before_sign;
			after_sign = (error_vx>0)-(error_vx<0);
			if(before_sign^after_sign)
			{
				vx = vx_req;
			}
		}
	}

	if(ry_deadzone||(!rx_deadzone))
	{
		if(vy_temp)
		{
			before_sign = (vy_temp>0)-(vy_temp<0);
			vy_temp -= deaccs*before_sign;
			after_sign = (vy_temp>0)-(vy_temp<0);
			if(before_sign^after_sign)
			{
				vy_temp = 0;
			}
		}
		vy_temp_req = 0;
	}
	else
	{
		vy_temp_req = v_limit*(ps_analog[ry]-deadzone_limit_ry)/(128.0 - deadzone_limit_ry);
		error_vy_temp = vy_temp_req-vy_temp;
		if(error_vy_temp)
		{
			before_sign = (error_vy_temp>0)-(error_vy_temp<0);
			vy_temp += accs*before_sign;
			after_sign = (error_vy_temp>0)-(error_vy_temp<0);
			if(before_sign^after_sign)
			{
				vy_temp = vy_temp_req;
			}
		}
	}

	if (vx > vx_limit)
		vx = vx_limit;
	else if (vx < -vx_limit)
		vx = -vx_limit;

	if (vy_temp > vy_limit)
		vy_temp = vy_limit;
	else if (vy_temp < -vy_limit)
		vy_temp = -vy_limit;

	if(!angle_correct)
	{
	    l_deadzone = abs(ps_analog[lx])<deadzone_limit_l;
	    if(l_deadzone)
	    {
	        before_sign = (w>0)-(w<0);
	        w -= deaccs_w*((w>0)-(w<0));
	    }
	    else
	    {
	    	w_req = -w_limit*(ps_analog[lx]-(deadzone_limit_l*((ps_analog[lx]>0)-(ps_analog[lx]<0))))/(128.0-deadzone_limit_l);

			error_w = w_req - w;

			before_sign = (error_w>0)-(error_w<0);
			w += accs_w*((error_w>0)-(error_w<0));
			after_sign = (error_w>0)-(error_w<0);
			if(before_sign^after_sign)
			{
				w = w_req;
			}
	    }

	    if(w > w_limit)
	        w = w_limit;
	    else if(w < -w_limit)
	        w = -w_limit;
	}
}

void holo_naxis()
{
    r_deadzone = (((ps_analog[rx]*ps_analog[rx])+(ps_analog[ry]*ps_analog[ry]))<(deadzone_limit_r*deadzone_limit_r));

    if(r_deadzone)
    {
        if((vx)||(vy))
        {
            float denom_v = sqrt((vx*vx)+(vy*vy));

            before_sign = (vx>0)-(vx<0);
            vx -= deaccs*(vx/denom_v);
            after_sign = (vx>0)-(vx<0);
            if(before_sign^after_sign)
            {
                vx = 0;
                vy = 0;
            }

            before_sign = (vy>0)-(vy<0);
            vy -= deaccs*(vy/denom_v);
            after_sign = (vy>0)-(vy<0);
            if(before_sign^after_sign)
            {
                vx = 0;
                vy = 0;
            }
        }
        vy_req = 0;
        vx_req = 0;
    }
    else
    {
        float denom_ps = deadzone_limit_r/sqrt((ps_analog[rx]*ps_analog[rx])+(ps_analog[ry]*ps_analog[ry]));
        vx_req = v_limit*(ps_analog[rx]-(denom_ps*ps_analog[rx]))/(128.0 - fabs(denom_ps*ps_analog[rx]));
        vy_req = v_limit*(ps_analog[ry]-(denom_ps*ps_analog[ry]))/(128.0 - fabs(denom_ps*ps_analog[ry]));
        error_vx = vx_req-vx;
        error_vy = vy_req-vy;
        if((error_vx)||(error_vy))
        {
            float denom_error = sqrt((error_vx*error_vx)+(error_vy*error_vy));

            before_sign = (error_vx>0)-(error_vx<0);
            vx += accs*(error_vx)/denom_error;
            after_sign = (error_vx>0)-(error_vx<0);
            if(before_sign^after_sign)
            {
                vx = vx_req;
            }

            before_sign = (error_vy>0)-(error_vy<0);
            vy += accs*(error_vy)/denom_error;
            after_sign = (error_vy>0)-(error_vy<0);
            if(before_sign^after_sign)
            {
                vy = vy_req;
            }
        }
    }

    if (vy > vy_limit)
        vy = vy_limit;
    else if (vy < -vy_limit)
        vy = -vy_limit;

    if (vx > vx_limit)
        vx = vx_limit;
    else if (vx < -vx_limit)
        vx = -vx_limit;

    l_deadzone = abs(ps_analog[lx])<deadzone_limit_l;
    if(l_deadzone)
    {
        if(!angle_correct)
        {
            before_sign = (w>0)-(w<0);
            w -= deaccs_w*((w>0)-(w<0));
            after_sign = (w>0)-(w<0);
            if(before_sign^after_sign)
            {
                w = 0;
                req_angle = angle;
                angle_correct = 1;
            }
            w_req = 0;
        }
    }
    else
    {
        angle_correct = 0;
        w_req = -w_limit*(ps_analog[lx]-(deadzone_limit_l*((ps_analog[lx]>0)-(ps_analog[lx]<0))))/(128.0-deadzone_limit_l);

        error_w = w_req - w;

        before_sign = (error_w>0)-(error_w<0);
        w += accs_w*((error_w>0)-(error_w<0));
        after_sign = (error_w>0)-(error_w<0);
        if(before_sign^after_sign)
        {
            w = w_req;
        }
    }

    if(w > w_limit)
        w = w_limit;
    else if(w < -w_limit)
        w = -w_limit;
}

void ps_control_std()
{
    if(!angle_correct)
    {
        if (ps_analog[lx]<-100)
        {
            w += accs_w;
        }
        else if (ps_analog[lx]>100)
        {
            w -= accs_w;
        }
        else
        {
            if (w > 0)
                w -= deaccs_w;
            if (w < 0)
                w += deaccs_w;
            if ((w <= 0.04) && (w >= -0.04))
            {
                w = 0;
            }
        }

        if(w > w_limit)
            w = w_limit;
        else if(w < -w_limit)
            w = -w_limit;
    }

    if (ps_analog[ry]>100)
    {
        vy += accs;
    }
    else if (ps_analog[ry]<-100)
    {
        vy -= accs;
    }
    else
    {
        if (vy > 0)
            vy -= deaccs;
        if (vy < 0)
            vy += deaccs;
        if ((vy <= 0.05) && (vy >= -0.05))
            vy = 0;
    }
    if (vy > vy_limit)
        vy = vy_limit;
    else if (vy < -vy_limit)
        vy = -vy_limit;

    if (ps_analog[rx]>100)
    {
        vx += accs;
    }
    else if (ps_analog[rx]<-100)
    {
        vx -= accs;
    }
    else
    {
        if (vx > 0)
            vx -= deaccs;
        if (vx < 0)
            vx += deaccs;
        if ((vx <= 0.05) && (vx >= -0.05))
            vx = 0;
    }
    if (vx > vx_limit)
        vx = vx_limit;
    else if (vx < -vx_limit)
        vx = -vx_limit;
}


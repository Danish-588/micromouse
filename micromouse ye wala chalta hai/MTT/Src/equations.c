/*
 * equations.c
 *
 *  Created on: Apr 11, 2023
 *      Author: Mihika
 */

#include "main.h"

volatile float theta = 0;
volatile float v1 = 0, v2 = 0, v3 = 0, v4 = 0;
volatile float vx = 0, vy = 0, w = 0, v_scaling_factor = 1, rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
volatile float bot_radius = 0.242, accs = 0.016, deaccs = 0.019, accs_w = 0.015, deaccs_w = 0.02, omni_front_angle = 90;
volatile int drive = 1;
volatile bool holonomic = 1, rpm_ctrl = 1;
volatile float vx_limit = 1.5, vy_limit = 1.5, w_limit = 3;

void equations(void)
{
    if(fabs(vx) > vx_limit)
    {
        v_scaling_factor = vx_limit/vx;

        vx *= v_scaling_factor;
        vy *= v_scaling_factor;
    }

    if(fabs(vy) > vy_limit)
    {
        v_scaling_factor = vy_limit/vy;

        vy *= v_scaling_factor;
        vx *= v_scaling_factor;
    }

    if(holonomic == 1)
        theta = angle;
    else
        theta=0;

    if(drive == mecanum)
    {
        v1 = vx * (-faster_sin(45 + theta))  + vy * (faster_cos (45 + theta))  + w * bot_radius;
        v2 = vx * (-faster_sin(45 - theta))  + vy * (-faster_cos(45 - theta))  + w * bot_radius;
        v3 = vx * (faster_sin (45 + theta))  + vy * (-faster_cos(45 + theta))  + w * bot_radius;
        v4 = vx * (faster_sin (45 - theta))  + vy * (faster_cos (45 - theta))  + w * bot_radius;

        rpm1 = ((v1 * 60) / (2 * pi * 0.075));
        rpm2 = ((v2 * 60) / (2 * pi * 0.075));
        rpm3 = ((v3 * 60) / (2 * pi * 0.075));
        rpm4 = ((v4 * 60) / (2 * pi * 0.075));
    }
    else if(drive == omni)
    {
        v1 = (-vx * faster_sin(omni_front_angle + theta))       + (vy * faster_cos(omni_front_angle + theta))       + (w * bot_radius);
        v2 = (-vx * faster_sin(omni_front_angle + 120 + theta)) + (vy * faster_cos(omni_front_angle + 120 + theta)) + (w * bot_radius);
        v3 = (-vx * faster_sin(omni_front_angle + 240 + theta)) + (vy * faster_cos(omni_front_angle + 240 + theta)) + (w * bot_radius);

        rpm1 = (v1 * 60) / (2 * pi * 0.068);
        rpm2 = (v2 * 60) / (2 * pi * 0.068);
        rpm3 = (v3 * 60) / (2 * pi * 0.068);
    }
}



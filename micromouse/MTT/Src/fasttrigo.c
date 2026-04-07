/*
 * fasttrigo.c
 *
 *  Created on: Jul 6, 2021
 *      Author: Soham
 */

#include "main.h"

float faster_sin(float angle)
{
    float r_ang1,r_ang1_c;
    while(angle< -90)
    {
        angle += 360;
    }
    while(angle> 270)
    {
        angle -= 360;
    }
    if((angle>90)&&(angle<=270))
    {
        angle = 180 - angle;
    }
    r_ang1 = angle*0.0174532;
    r_ang1_c = r_ang1*r_ang1*r_ang1;
    return r_ang1 - ((r_ang1_c)/6) + ((r_ang1*r_ang1*r_ang1_c)/120) - ((r_ang1*r_ang1_c*r_ang1_c)/5040);
}

float faster_cos(float angle)
{
    float r_ang1,r_ang1_s;
    while(angle< (-90))
    {
        angle += 360;
    }
    while(angle>=270)
    {
        angle -= 360;
    }
    if((angle>90)&&(angle<270))
    {
        angle = 180 - angle;
        r_ang1 = angle*0.0174532;
        r_ang1_s = r_ang1*r_ang1;
        return -(1 - ((r_ang1_s)/2) + ((r_ang1_s*r_ang1_s)/24) - ((r_ang1_s*r_ang1_s*r_ang1_s)/720) + ((r_ang1_s*r_ang1_s*r_ang1_s*r_ang1_s)/40320));
    }
    else
    {
        r_ang1 = angle*0.0174532;
        r_ang1_s = r_ang1*r_ang1;
        return 1 - ((r_ang1_s)/2) + ((r_ang1_s*r_ang1_s)/24) - ((r_ang1_s*r_ang1_s*r_ang1_s)/720) + ((r_ang1_s*r_ang1_s*r_ang1_s*r_ang1_s)/40320);
    }
}

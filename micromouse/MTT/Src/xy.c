/*
 * xy.c
 *
 *  Created on: Jul 6, 2021
 *      Author: Soham
 */

#include "main.h"

struct odometry
twowheel = {2,&enc1,&enc5_a,&enc3,30,2000,twowheel_isr},
threewheel = {3,&enc1,&enc5_a,&enc3,30,2000,threewheel_isr};

volatile float xy_enc1_angle = 30;

void xy_init(struct odometry *odometry)
{
		encoder_init(odometry->wheel1,odometry->ppr);
		encoder_init(odometry->wheel2,odometry->ppr);
		if(odometry->type==3)
		{
			encoder_init(odometry->wheel3,odometry->ppr);
		}
		timer_init(&timer5,odometry->xy_freq,odometry->xyisrfunciton);
}

void twowheel_isr()
{

}

void threewheel_isr()
{
	threewheel.wheel1cnt = get_enccount(&enc1);
	threewheel.wheel2cnt = get_enccount(&enc5_a);
	threewheel.wheel3cnt = get_enccount(&enc3);
	clear_enccount(&enc1);
	clear_enccount(&enc5_a);
	clear_enccount(&enc3);

	float c1,c2,c3,s1,s2,s3,k,v1_xy,v2_xy,v3_xy,vx_xy,vy_xy;

    c1 = faster_cos(angle + xy_enc1_angle);
    c2 = faster_cos(angle + xy_enc1_angle + 120);
    c3 = faster_cos(angle + xy_enc1_angle + 240);

    s1 = faster_sin(angle + xy_enc1_angle);
    s2 = faster_sin(angle + xy_enc1_angle + 120);
    s3 = faster_sin(angle + xy_enc1_angle + 240);

    k = ((s1 * c3) + (c1 * s2) + (c2 * s3) - (s1 * c2) - (s3 * c1) - (s2 * c3));

    v1_xy = 2 * 3.1415 * 0.0248 * 0.015 * threewheel.wheel1cnt;
    v2_xy = 2 * 3.1415 * 0.0248 * 0.015 * threewheel.wheel2cnt;
    v3_xy = 2 * 3.1415 * 0.0248 * 0.015 * threewheel.wheel3cnt;

    vx_xy = ((c2 - c3) * v1_xy + (c3 - c1) * v2_xy + (c1 - c2) * v3_xy) / k;
    vy_xy = ((s2 - s3) * v1_xy + v2_xy * (s3 - s1) + v3_xy * (s1 - s2)) / k;

    threewheel.X_pos += vx_xy / threewheel.xy_freq;
    threewheel.Y_pos += vy_xy / threewheel.xy_freq;
}

/*
 * xy.h
 *
 *  Created on: Jul 6, 2021
 *      Author: Soham
 */

#ifndef INC_XY_H_
#define INC_XY_H_

typedef void (*xyisrfunciton)(void);
struct odometry
{
	uint8_t type;
	struct encoders *wheel1;
	struct encoders *wheel2;
	struct encoders *wheel3;
	uint16_t xy_freq;
	uint16_t ppr;
	xyisrfunciton xyisrfunciton;
	float X_pos;
	float Y_pos;
	int wheel1cnt;
	int wheel2cnt;
	int wheel3cnt;
}twowheel,threewheel;

void twowheel_isr();
void threewheel_isr();
void xy_init(struct odometry *odometry);

#endif /* INC_XY_H_ */

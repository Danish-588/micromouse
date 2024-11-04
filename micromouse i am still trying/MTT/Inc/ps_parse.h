/*
 * ps_parse.h
 *
 *  Created on: 23-Jun-2022
 *      Author: Soham
 */

#ifndef INC_PS_PARSE_H_
#define INC_PS_PARSE_H_

volatile bool ps_digital[40];
volatile int16_t ps_analog[7];

enum digital
{
    share_hold,
    l3_hold,
    r3_hold,
    options_hold,
    up_hold,
    right_hold,
    down_hold,
    left_hold,
    triangle_hold,
    circle_hold,
    cross_hold,
    square_hold,
    l1_hold,
    r1_hold,
    l2_hold,
    r2_hold,
    share_toggle,
    l3_toggle,
    r3_toggle,
    options_toggle,
    up_toggle,
    right_toggle,
    down_toggle,
    left_toggle,
    triangle_toggle,
    circle_toggle,
    cross_toggle,
    square_toggle,
    l1_toggle,
    r1_toggle,
    l2_toggle,
    r2_toggle,
    ps,
    touchpad,
	ps_up,
	ps_down,
	touchpad_up,
	touchpad_down,
	r2_up,
	r2_down,
};

enum analog
{
    battery,
    l2val,
    r2val,
    lx,
    ly,
    rx,
    ry,
};

void ps_getdata(uint32_t digi_id,uint32_t ana_id);
void ps_senddata(uint64_t red, uint64_t green, uint64_t blue, uint64_t left_rumble, uint64_t right_rumble);

#endif /* INC_PS_PARSE_H_ */

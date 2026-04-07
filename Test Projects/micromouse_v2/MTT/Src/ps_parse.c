/*
 * ps_parse.c
 *
 *  Created on: 23-Jun-2022
 *      Author: Soham
 */
#include <main.h>
void ps_getdata(uint32_t digi_id,uint32_t ana_id)
{
    uint8_t i;
    uint64_t temp;
    uint64_t data;
    data = can_receive(digital_id);
    for(i = 0; i<40 ; i++)
    {
        ps_digital[i] = (data>>i)&0x1;
    }

    data = can_receive(analog_id);
    for(i = 0; i<3 ; i++)
    {
        temp = (data>>(i*8))&0xFF;
        ps_analog[i] = (*(uint8_t*)&temp);
    }
    for(i = 0; i<4 ; i++)
    {
        temp = (data>>(24+(i*8)))&0xFF;
        ps_analog[i+3] = (*(int8_t*)&temp);
    }
}

void ps_senddata(uint64_t red, uint64_t green, uint64_t blue, uint64_t left_rumble, uint64_t right_rumble)
{
	can_transmit_64(pssend, red|(green<<8)|(blue<<16)|(left_rumble<<24)|(right_rumble<<32));
}

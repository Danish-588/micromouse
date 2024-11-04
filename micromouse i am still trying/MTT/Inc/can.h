/*
 * can.h
 *
 *  Created on: Jul 13, 2021
 *      Author: Soham
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#define wheel1_id       1
#define wheel2_id       2
#define wheel3_id       3
#define wheel4_id       4

#define angle_id        5
#define dead_angle_id   6
#define req_angle_id    7

#define x_id            8
#define y_id            9
#define dead_x_id       10
#define dead_y_id       11
#define req_x_id        12
#define req_y_id        13

#define vx_xy_id        14
#define vy_xy_id        15

#define vx_id           16
#define vy_id           17

#define navi_case_id    18

#define digital_id    	20

#define analog_id    	21

#define pssend    		22


uint8_t can_array[8];

CAN_HandleTypeDef hcan1;
CAN_FilterTypeDef can_filter;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

uint64_t *can_id[28];

struct can_speed
{
	uint32_t Prescaler;
	uint32_t SyncJumpWidth;
	uint32_t TimeSeg1;
	uint32_t TimeSeg2;

}can_100,can_250,can_500,can_1000;

void can_set_ids();
void can_init(struct can_speed *can_speed);
void can_transmit(int msg_id, int data);
uint64_t can_receive(uint32_t can_data);
void can_isr();
void CAN_setFilter(uint8_t bank, uint8_t fifo, uint16_t mode, uint32_t id1, uint32_t id2);
void CAN_setFilter_all();
void can_pack_data(int data, uint8_t *data_arr);

#endif /* INC_CAN_H_ */

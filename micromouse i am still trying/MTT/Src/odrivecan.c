/*
 * odrivecan.c
 *
 *  Created on: Apr 7, 2022
 *      Author: Mihika Moni
 */

#include "main.h"

int f2b(float fl)
{
	return (*(int*)&fl);
}

float b2f(int b)
{
	return (*(float*)&b);
}

void can_transmit_64(int msg_id, uint64_t data)
{
	uint32_t TxMailbox;
	uint8_t *frame = (uint8_t *)&data;

	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = msg_id;

	if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,frame,&TxMailbox)!=HAL_OK)
	{
		Error_Handler();
	}

	while(HAL_CAN_IsTxMessagePending(&hcan1,TxMailbox));
}

void can_set_controller_mode(int node, uint8_t con_mode, uint8_t input_mode)
{
    can_transmit_64(((node<<5)|0xB),(((uint64_t)input_mode)<<32)|con_mode);
}

void set_limits(int node, float v_lim, float c_lim)
{
	can_transmit_64(((node<<5)|0xF),(((uint64_t)f2b(c_lim))<<32)|f2b(v_lim));
}

void set_traj_limits(int node, float traj_accel, float traj_decel)
{
	can_transmit_64(((node<<5)|0x12),(((uint64_t)f2b(traj_decel))<<32)|f2b(traj_accel));
}

void odcansend(int node, int command, float value)
{
    int temp_value = value;
    switch(command)
    {
        case axis_node_id:
        {
            command_id = 0x006;
            break;
        }
        case req_state:
        {
            command_id = 0x007;
            break;
        }

        case controller_mode:
        {
            command_id = 0x00B;
            break;
        }

        case input_pos:
        {
            command_id = 0x00C;
            temp_value = f2b(value);
            break;
        }

        case input_vel:
        {
            command_id = 0x00D;
            temp_value = f2b(value);
            break;
        }

        case input_torque:
        {
            command_id = 0x00E;
            temp_value = f2b(value);
            break;
        }

        case vel_limit:
        {
            command_id = 0x00F;
            temp_value = f2b(value);
            break;
        }

        case curr_lim:
        {
            command_id = 0x00F;
            temp_value = ((uint64_t)f2b(value)<<32);
            break;
        }

        case traj_vel_lim:
        {
            command_id = 0x011;
            temp_value = f2b(value);
            break;
        }

        case traj_accel_lim:
        {
            command_id = 0x012;
            temp_value = f2b(value);
            break;
        }

        case traj_decel_lim:
        {
            command_id = 0x012;
            temp_value = ((uint64_t)f2b(value)<<32);
            break;
        }

        case traj_inertia:
        {
            command_id = 0x013;
            temp_value = f2b(value);
            break;
        }

        case reboot:
        {
            command_id = 0x016;
            break;
        }

        case clear_errors:
        {
            command_id = 0x018;
            break;
        }

        case linear_count:
        {
            command_id = 0x019;
            break;
        }

        case pos_gain:
        {
            command_id = 0x01A;
            temp_value = f2b(value);
            break;
        }

        case vel_gains:
        {
            command_id = 0x01B;
            temp_value = f2b(value);
            break;
        }

    }
    can_transmit_64((node<<5)|command_id, temp_value);
}

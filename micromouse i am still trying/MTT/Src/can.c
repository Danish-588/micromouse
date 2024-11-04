/*
 * can.c
 *
 *  Created on: Jul 13, 2021
 *      Author: Soham
 */

#include "main.h"

uint64_t msg1;
uint64_t msg2;
uint64_t msg3;
uint64_t msg4;
uint64_t msg5;
uint64_t msg6;
uint64_t msg7;
uint64_t msg8;
uint64_t msg9;
uint64_t msg10;
uint64_t msg11;
uint64_t msg12;
uint64_t msg13;
uint64_t msg14;
uint64_t msg15;
uint64_t msg16;
uint64_t msg17;
uint64_t msg18;
uint64_t msg19;
uint64_t msg20;
uint64_t msg21;
uint64_t msg22;
uint64_t msg23;
uint64_t msg24;
uint64_t msg25;
uint64_t msg26;
uint64_t msg27;
uint64_t msg28;

struct can_speed
can_100 = { 25 , CAN_SJW_1TQ , CAN_BS1_15TQ , CAN_BS2_2TQ },
can_250 = { 10 , CAN_SJW_1TQ , CAN_BS1_15TQ , CAN_BS2_2TQ },
can_500 = { 5 , CAN_SJW_1TQ , CAN_BS1_15TQ , CAN_BS2_2TQ },
can_1000 = { 3 , CAN_SJW_1TQ , CAN_BS1_12TQ , CAN_BS2_2TQ };

void can_set_ids()
{
	can_id[0] = &msg1;
	can_id[1] = &msg2;
	can_id[2] = &msg3;
	can_id[3] = &msg4;
	can_id[4] = &msg5;
	can_id[5] = &msg6;
	can_id[6] = &msg7;
	can_id[7] = &msg8;
	can_id[8] = &msg9;
	can_id[9] = &msg10;
	can_id[10] = &msg11;
	can_id[11] = &msg12;
	can_id[12] = &msg13;
	can_id[13] = &msg14;
	can_id[14] = &msg15;
	can_id[15] = &msg16;
	can_id[16] = &msg17;
	can_id[17] = &msg18;
	can_id[18] = &msg19;
	can_id[19] = &msg20;
	can_id[20] = &msg21;
	can_id[21] = &msg22;
	can_id[22] = &msg23;
	can_id[23] = &msg24;
	can_id[24] = &msg25;
	can_id[25] = &msg26;
	can_id[26] = &msg27;
	can_id[27] = &msg28;
}

void can_init(struct can_speed *can_speed)
{
	  /* USER CODE BEGIN CAN1_Init 0 */

	  /* USER CODE END CAN1_Init 0 */

	  /* USER CODE BEGIN CAN1_Init 1 */

	  /* USER CODE END CAN1_Init 1 */
	  hcan1.Instance = CAN1;
	  hcan1.Init.Prescaler = can_speed->Prescaler;
	  hcan1.Init.Mode = CAN_MODE_NORMAL;
	  hcan1.Init.SyncJumpWidth = can_speed->SyncJumpWidth;
	  hcan1.Init.TimeSeg1 = can_speed->TimeSeg1;
	  hcan1.Init.TimeSeg2 = can_speed->TimeSeg2;
	  hcan1.Init.TimeTriggeredMode = DISABLE;
	  hcan1.Init.AutoBusOff = ENABLE;
	  hcan1.Init.AutoWakeUp = DISABLE;
	  hcan1.Init.AutoRetransmission = DISABLE;
	  hcan1.Init.ReceiveFifoLocked = DISABLE;
	  hcan1.Init.TransmitFifoPriority = DISABLE;
	  if (HAL_CAN_Init(&hcan1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN CAN1_Init 2 */

	  /* USER CODE END CAN1_Init 2 */

	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

	  HAL_CAN_Start(&hcan1);

	  CAN_setFilter_all();

	  can_set_ids();
}

void CAN_setFilter(uint8_t bank, uint8_t fifo, uint16_t mode, uint32_t id1, uint32_t id2)
{
	can_filter.FilterActivation = ENABLE;
	can_filter.FilterBank = bank;
	can_filter.FilterFIFOAssignment = fifo;
	can_filter.FilterIdHigh = (id1)<<5;
	can_filter.FilterIdLow = 0;
	can_filter.FilterMaskIdHigh = (id2)<<5;
	can_filter.FilterMaskIdLow = 0;
	can_filter.FilterMode = mode;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.SlaveStartFilterBank = 27;

	if((HAL_CAN_ConfigFilter(&hcan1,&can_filter))!=HAL_OK)
	{
		Error_Handler();
	}
}

void CAN_setFilter_all()
{
	for(int i = 0; i<25; i++)
	{
		CAN_setFilter(i, i%2==0, CAN_FILTERMODE_IDLIST, i+1, i+1);
	}
//
//	can_filter.FilterActivation = ENABLE;
//	can_filter.FilterBank = 25;
//	can_filter.FilterFIFOAssignment = 0;
//	can_filter.FilterIdHigh = (((4096)+42)>>13);
//	can_filter.FilterIdLow = ((((4096)+42)&0x1FFF)<<3)|CAN_ID_EXT;
//	can_filter.FilterMaskIdHigh = (((4096)+42)>>13);
//	can_filter.FilterMaskIdLow = ((((4096)+42)&0x1FFF)<<3)|CAN_ID_EXT;
//	can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
//	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
//	can_filter.SlaveStartFilterBank = 27;
//
//	if((HAL_CAN_ConfigFilter(&hcan1,&can_filter))!=HAL_OK)
//	{
//		Error_Handler();
//	}
	CAN_setFilter(26, 1, CAN_FILTERMODE_IDLIST, 41, 41);//Temporarily for ODRIVE
}

uint64_t can_receive(uint32_t can_data)
{
	return *(can_id[can_data-1]);
}
void can_transmit(int msg_id, int data)
{
	uint32_t TxMailbox;

	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = msg_id;

    if (data < 0)
	{
		data = ((0x80000000) | (-(data)));
	}

	can_pack_data(data,can_array);

	if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,can_array,&TxMailbox)!=HAL_OK)
	{
		Error_Handler();
	}

	while(HAL_CAN_IsTxMessagePending(&hcan1,TxMailbox));
}

void can_pack_data(int data, uint8_t *data_arr)
{
	for(int i = 0 ; i<4 ; i++)
	{
		data_arr[i] = (data>>(i*8))&0xFF;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t*)(can_id[(((hcan1.Instance->sFIFOMailBox[0].RDTR)&0xFF00)>>8)+1]));
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, (uint8_t*)(can_id[((hcan1.Instance->sFIFOMailBox[1].RDTR)&0xFF00)>>8]));
}

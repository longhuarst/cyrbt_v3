#include "cylib_can.h"
#include "can.h"
#include <stdint.h>
#include "../cylib.h"



typedef struct{
	CAN_RxHeaderTypeDef        rxMsg;
	uint8_t rxData[8];
	CAN_TxHeaderTypeDef        txMsg;
	uint8_t txData[8];
}cylib_can_def;


cylib_can_def cylib_can;








void cylib_can_init(void)
{
	CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	
	

	HAL_CAN_Start(&hcan);
	
	
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	
	
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1){
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,&cylib_can.rxMsg,cylib_can.rxData);
		
		if (cylib_can.rxMsg.ExtId == 0x1310 || cylib_can.rxMsg.ExtId == 0x1311 || cylib_can.rxMsg.ExtId == 0x1312 || cylib_can.rxMsg.ExtId == 0x1313){
		uint8_t val = cylib_can.rxMsg.ExtId %10;
		if (val >= 4)
			return;
		
		float x,y;
		
		mBYTE0(x) = cylib_can.rxData[0];
		mBYTE1(x) = cylib_can.rxData[1];
		mBYTE2(x) = cylib_can.rxData[2];
		mBYTE3(x) = cylib_can.rxData[3];
		mBYTE0(y) = cylib_can.rxData[4];
		mBYTE1(y) = cylib_can.rxData[5];
		mBYTE2(y) = cylib_can.rxData[6];
		mBYTE3(y) = cylib_can.rxData[7];
		
		cylib_angle_update(val,x,y);
		

		
	}
	
		
		
		
	}
}




void cylib_can_rx_callback(void)
{

	
}


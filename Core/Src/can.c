/*
 * can.c
 *
 *  Created on: Oct 24, 2020
 *      Author: zandor
 */

#include <../Inc/can.h>

extern CAN_HandleTypeDef hcan1;

CANStruct s_can1;


void enable_can1(){
	  s_can1.FilterConfig.SlaveStartFilterBank = 14;
	  s_can1.FilterConfig.FilterScale      = CAN_FILTERSCALE_32BIT;
	  s_can1.FilterConfig.FilterMode       = CAN_FILTERMODE_IDMASK;
	  s_can1.FilterConfig.FilterMaskIdLow  = 0x0000;
	  s_can1.FilterConfig.FilterMaskIdHigh = 0x0000;
	  s_can1.FilterConfig.FilterIdLow 	  = 0x0000;
	  s_can1.FilterConfig.FilterIdHigh     = 0x0000;
	  s_can1.FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  s_can1.FilterConfig.FilterBank       = 0;
	  s_can1.FilterConfig.FilterActivation = ENABLE;


	  if(HAL_CAN_ConfigFilter(&hcan1, &s_can1.FilterConfig) != HAL_OK)
	  {
	    /* Filter configuration Error */
	    Error_Handler();
	  }
	  if(HAL_CAN_Start(&hcan1) != HAL_OK)
	  {
	    /* Start Error */
	    Error_Handler();
	  }
	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts

}

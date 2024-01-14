//
// Created by Trisoil on 2023/5/30.
//

#include <math.h>
#include "CAN_Handler.h"
#include "Go_Driver.h"
#include "main.h"
#include "can.h"

typedef struct{
    uint32_t ID;
    float data;
}control_data;

union CAN_Decode_Box{
    uint8_t data[8];
    control_data control_message;
}CAN_Decode_Box_u;

int cnt = 0;

void CAN1_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x310;
    sFilterConfig.FilterIdLow = 0x300;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* 启动CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* 使能CAN的FIFO0接收通知（中断） */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) {
        CAN_RxHeaderTypeDef Control_Message_Box;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Control_Message_Box, CAN_Decode_Box_u.data);
        uint32_t ID = Control_Message_Box.StdId;
        uint8_t run = 1;
        // cnt++;
        if (ID == 0x301){
            // speed control
            Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].des_vel = CAN_Decode_Box_u.control_message.data;
            Go_State[CAN_Decode_Box_u.control_message.ID].State = PIDVelControl;
        }else if (ID == 0x302){
            // position control
            Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].des_pos = CAN_Decode_Box_u.control_message.data + Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].start_pos;
            Go_State[CAN_Decode_Box_u.control_message.ID].State = PIDPosControl;
        }else if (ID == 0x303){
            // curve position control
            float des = CAN_Decode_Box_u.control_message.data + Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].start_pos;
            if (fabsf(Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].des_pos-des)>0.01f){
                Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].des_pos = CAN_Decode_Box_u.control_message.data + Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].start_pos;
                Go_State[CAN_Decode_Box_u.control_message.ID].State = AsCurveControl;
                Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].have_calculated = 0;
            }else run = 0;
        }
        if (!run) return;
        if (Go_State[CAN_Decode_Box_u.control_message.ID].State == PosControl) Go_Motor_Position_Control(CAN_Decode_Box_u.control_message.ID, Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].des_pos, Go_Ctrl_Data[0].k_pos);
        else if (Go_State[CAN_Decode_Box_u.control_message.ID].State == VelControl) Go_Motor_Speed_Control(CAN_Decode_Box_u.control_message.ID, Go_Ctrl_Data[CAN_Decode_Box_u.control_message.ID].des_vel, Go_Ctrl_Data[0].k_vel);
        else if (Go_State[CAN_Decode_Box_u.control_message.ID].State == AsCurveControl) Go_Motor_Curve_Control(CAN_Decode_Box_u.control_message.ID);
    }
}



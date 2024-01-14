//
// Created by Trisoil on 2023/6/22.
//

#include <stdio.h>
#include "stdarg.h"
#include "usart_cmd.h"
#include "main.h"
#include "usart.h"
#include <string.h>
#include <math.h>
#include "Go_Driver.h"

extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;

uint8_t cmd_data[2][99];

uint8_t interrupted = 0;

void Usart6_TX_DMA_Init(void)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);
}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }
    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
    //clear flag
    //清除标志位
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_HTIF7);

    //set data address
    //设置数据地址
    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    //set data length
    //设置数据长度
    hdma_usart6_tx.Instance->NDTR = len;

    //
    //enable DMA
    //使能DMA
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);

    while (!(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)){
        __HAL_DMA_ENABLE(&hdma_usart6_tx);
    }

}

void Usart_Printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string
    //返回字符串长度
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

//     usart6_tx_dma_enable(tx_buf,len);
    HAL_UART_Transmit_DMA(&huart6, tx_buf, len);
//    HAL_UART_Transmit(&huart6,tx_buf,len,0x7fff);
//    HAL_Delay(10);
}

// TODO: 多电机参数写入支持
void motor_WriteParam(int id)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    HAL_FLASH_Unlock(); // FLASH解锁

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR_5;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &PageError))
    {
        HAL_FLASH_Lock();
        return;
    }
    Flash_Data.s_data = Go_Ctrl_Data[id].k_vel;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&FLASH_Param[0], Flash_Data.t_data);
    Flash_Data.s_data = Go_Ctrl_Data[id].k_pos;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&FLASH_Param[1], Flash_Data.t_data);
    Flash_Data.s_data = Go_Ctrl_Data[id].J_a;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&FLASH_Param[2], Flash_Data.t_data);
    Flash_Data.s_data = Go_Ctrl_Data[id].J_d;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&FLASH_Param[3], Flash_Data.t_data);
    Flash_Data.s_data = Go_Ctrl_Data[id].A_a;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&FLASH_Param[4], Flash_Data.t_data);
    Flash_Data.s_data = Go_Ctrl_Data[id].A_d;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&FLASH_Param[5], Flash_Data.t_data);
    Flash_Data.s_data = Go_Ctrl_Data[id].V_m;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&FLASH_Param[6], Flash_Data.t_data);

    HAL_FLASH_Lock(); // FLASH上锁
}

void USART6_Receive_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //HAL_UART_Receive_IT()
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) &(USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t) (rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t) (rx2_buf);
    //data length
    //数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

}

void USART6_Rx_Init() {
    USART6_Receive_init(cmd_data[0], cmd_data[1], 50);
}

// TODO: 将命令识别使用cpp的map实现
void CMD_Dealer(char cmd_buffer[], uint8_t data_length){
    if (cmd_buffer[data_length-1] == '\n') cmd_buffer[data_length-1] = '\0';
    if (cmd_buffer[data_length-2] == '\r') cmd_buffer[data_length-2] = '\0';
    char cmd[20];
    sscanf(cmd_buffer, "%s", cmd);
    interrupted = 1;
    if (!strcmp(cmd, "help")){
        Usart_Printf("To be added\r\n");
    }else if (!strcmp(cmd, "velctrl")){
        uint8_t id;
        float vel;
        int ans = sscanf(cmd_buffer, "velctrl %d %f", &id, &vel);
        if (ans < 2 || ans == EOF){
            Usart_Printf("Invalid args!\r\n");
        }else{
            Usart_Printf(">>> %s\r\n",cmd_buffer);
            Go_State[id].State = VelControl;
            Go_Ctrl_Data[id].des_vel = vel;
//            Motor_Ctrl(id);
        }
    }else if (!strcmp(cmd, "posctrl")){
        uint8_t id;
        float pos;
        int ans = sscanf(cmd_buffer,"posctrl %d %f", &id, &pos);
        if (ans < 2 || ans == EOF){
            Usart_Printf("Invalid args!\r\n");
        }else{
            Usart_Printf(">>>posctrl %d %f\r\n",id,pos);
            Go_State[id].State = PosControl;
            Go_Ctrl_Data[id].des_pos = pos + Go_Ctrl_Data[id].start_pos;
//            Motor_Ctrl(id);
        }
    }else if (!strcmp(cmd, "pidposctrl")){
        uint8_t id;
        float pos;
        int ans = sscanf(cmd_buffer,"pidposctrl %d %f", &id, &pos);
        if (ans < 2 || ans == EOF){
            Usart_Printf("Invalid args!\r\n");
        }else{
            Usart_Printf(">>>pidposctrl %d %f\r\n",id,pos);
            Go_State[id].State = PIDPosControl;
            Go_Ctrl_Data[id].des_pos = pos + Go_Ctrl_Data[id].start_pos;
//            Motor_Ctrl(id);
        }
    }else if (!strcmp(cmd, "pidvelctrl")){
        uint8_t id;
        float vel;
        int ans = sscanf(cmd_buffer,"pidvelctrl %d %f", &id, &vel);
        if (ans < 2 || ans == EOF){
            Usart_Printf("Invalid args!\r\n");
        }else{
            Usart_Printf(">>>pidvelctrl %d %f\r\n",id,vel);
            Go_State[id].State = PIDVelControl;
            Go_Ctrl_Data[id].des_vel = vel;
//            Motor_Ctrl(id);
        }
    }else if (!strcmp(cmd, "sposctrl")){
        uint8_t id;
        float pos;
        int ans = sscanf(cmd_buffer,"sposctrl %d %f", &id, &pos);
        if (ans < 2 || ans == EOF){
            Usart_Printf("Invalid args!\r\n");
        }else{
            Usart_Printf(">>>sposctrl %d %f\r\n",id,pos);
            Go_State[id].State = SPosControl;
            Go_Ctrl_Data[id].des_pos = pos;
//            Motor_Ctrl(id);
        }
    }else if (!strcmp(cmd, "crvctrl")){
        uint8_t id;
        float pos;
        int ans = sscanf(cmd_buffer, "crvctrl %d %f", &id, &pos);
        if (ans < 2 || ans == EOF){
            Usart_Printf("Invalid args!\r\n");
        }else{
            Usart_Printf(">>> %s\r\n",cmd_buffer);
            if (fabsf(pos-Go_Ctrl_Data[id].des_pos) > 0.001f){
//                if (Go_State[id].State != AsCurveControl) Go_Update_data(id);
                Go_State[id].State = AsCurveControl;
                Go_Ctrl_Data[id].des_pos = pos + Go_Ctrl_Data[id].start_pos;
                Go_Ctrl_Data[id].have_calculated = 0;
            }
//            Motor_Ctrl(id);
        }
    }else if (!strcmp(cmd, "check")){
        uint8_t id;
        int ans = sscanf(cmd_buffer, "check %d", &id);
        if (ans < 1 || ans == EOF)
            Usart_Printf("Invalid args!\r\n");
        else{
//            Motor_Ctrl(id);
//            HAL_Delay(5);
            Usart_Printf("id: %d\r\npos: %f\r\nvel: %f\r\nstate: %d\r\n",id, Go_State[id].pos-Go_Ctrl_Data[id].start_pos,Go_State[id].omega,Go_State[0].State);
        }
    }else if (!strcmp(cmd, "cfgset")){
        uint8_t id;
        float J_a, J_d, A_a, A_d, V_m, k_vel, k_pos;
        int ans = sscanf(cmd_buffer, "cfgset %d %f %f %f %f %f %f %f", &id, &k_vel, &k_pos, &J_a, &J_d, &A_a, &A_d, &V_m);
        if (ans < 8 || ans == EOF)
            Usart_Printf("Invalid args!\r\n");
        else{
            if (k_vel != -1) Go_Ctrl_Data[id].k_vel = k_vel;
            if (k_pos != -1) Go_Ctrl_Data[id].k_pos = k_pos;
            if (J_a != -1) Go_Ctrl_Data[id].J_a = J_a;
            if (J_d != -1) Go_Ctrl_Data[id].J_d = J_d;
            if (A_a != -1) Go_Ctrl_Data[id].A_a = A_a;
            if (A_d != -1) Go_Ctrl_Data[id].A_d = A_d;
            if (V_m != -1) Go_Ctrl_Data[id].V_m = V_m;
            motor_WriteParam(id);
            Usart_Printf("id: %d\r\nk_vel: %f\r\nk_pos: %f\r\nJ_a: %f\r\nJ_d: %f\r\nA_a: %f\r\nA_d: %f\r\nV_m: %f\r\n",id ,Go_Ctrl_Data[id].k_vel, Go_Ctrl_Data[id].k_pos, Go_Ctrl_Data[id].J_a, Go_Ctrl_Data[id].J_d, Go_Ctrl_Data[id].A_a, Go_Ctrl_Data[id].A_d, Go_Ctrl_Data[id].V_m);
        }
    }else if (!strcmp(cmd, "getcfg")){
        uint8_t id;
        int ans = sscanf(cmd_buffer, "getcfg %d", &id);
        if (ans < 1 || ans == EOF)
            Usart_Printf("Invalid args!\r\n");
        else{
            Usart_Printf("id: %d\r\nk_vel: %f\r\nk_pos: %f\r\nJ_a: %f\r\nJ_d: %f\r\nA_a: %f\r\nA_d: %f\r\nV_m: %f\r\n",id ,Go_Ctrl_Data[id].k_vel, Go_Ctrl_Data[id].k_pos, Go_Ctrl_Data[id].J_a, Go_Ctrl_Data[id].J_d, Go_Ctrl_Data[id].A_a, Go_Ctrl_Data[id].A_d, Go_Ctrl_Data[id].V_m);
        }
    }
    Motor_Ctrl(0);
}

void USART6_IRQHandler(void) {
    if (huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

    } else if (USART6->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 99 - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = 99;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            CMD_Dealer((char *)cmd_data[0], this_time_rx_len);
        } else {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 99 - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_length
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = 99;

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            CMD_Dealer((char *)cmd_data[1], this_time_rx_len);
        }
    }else{
        HAL_UART_IRQHandler(&huart6);
    }
}




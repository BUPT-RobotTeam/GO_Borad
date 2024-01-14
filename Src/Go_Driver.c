#include <main.h>
#include "dma.h"
#include <usart.h>
#include <math.h>
#include "crc_ccitt.h"
#include "pid.h"
#include "Go_Driver.h"
#include "usart_cmd.h"

#define PI 3.1415926535f

// Uart Send Part

extern DMA_HandleTypeDef hdma_usart3_tx;
Go_Send_Data Go_Tx_Data;
Curve_Data Go_Ctrl_Data[14];
Go_Status Go_State[14];

union Body_Parameters{
    uint8_t data[12];
    Body_Data parameters;
}Body_Parameters_u;

Flash_Data_u Flash_Data;

union Rev_Parameters{
    Go_Rev_Data rx_data;
    uint8_t data[8];
}Rev_Parameters_u;

union CRCC{
    uint8_t data[2];
    uint16_t crc;
}CRC_u;

struct S_Data P_sata;
uint8_t s_First;

void Go_Motor_Curve_Control(uint8_t id);
float S_Curve_cal(Curve_Data *AS_Curve_Data, int id);

void Usart3_TX_DMA_Init(void)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAT);
}

void usart3_tx_dma_enable(const uint8_t *data, uint16_t len)
{

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_tx);

    while(hdma_usart3_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_tx);
    }
    hdma_usart3_tx.Instance->PAR = (uint32_t) & (USART3->DR);
    //clear flag
    //清除标志位
    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_HISR_HTIF7);

    //set data address
    //设置数据地址
    hdma_usart3_tx.Instance->M0AR = (uint32_t)(data);
    //set data length
    //设置数据长度
    hdma_usart3_tx.Instance->NDTR = len;

    //
    //enable DMA
    //使能DMA
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAT);

    __HAL_DMA_ENABLE(&hdma_usart3_tx);

    while (!(hdma_usart3_tx.Instance->CR & DMA_SxCR_EN)){
        __HAL_DMA_ENABLE(&hdma_usart3_tx);
    }

}

// Send data generation and sent.

void Go_Control_Send()
{
    uint8_t data[23];
    // set head
    data[0] = 0xFE;
    data[1] = 0xEE;

    // set mode
    data[2] = (Go_Tx_Data.status << 4) | Go_Tx_Data.ID;

    // set control parameters
    Body_Parameters_u.parameters = Go_Tx_Data.parameters;
    int cnt = 0;
    for (int i=3;i<15;i++) data[i] = Body_Parameters_u.data[cnt++];

    // set CRC
    uint16_t crc = crc_ccitt(0,data,15);
    CRC_u.crc = crc;
    data[15] = CRC_u.data[0];
    data[16] = CRC_u.data[1];

    // dma send data
    usart3_tx_dma_enable(data, 17);
//    HAL_UART_Transmit_DMA(&huart3, data, 17);
    Go_Ctrl_Data[Go_Tx_Data.ID].have_send = 1;
}

void Go_Motor_Torque_Control(uint8_t ID, float Torque)
{
    Go_Tx_Data.ID = ID;
    Go_Tx_Data.status = 1;
    Go_Tx_Data.parameters.torque = (int16_t)(Torque*256.0f);
    Go_Tx_Data.parameters.omega = 0;
    Go_Tx_Data.parameters.pos = 0;
    Go_Tx_Data.parameters.k_pos = 0;
    Go_Tx_Data.parameters.k_vel = 0;
    Go_Control_Send();
}

void Go_Motor_Speed_Control(uint8_t ID, float W, float K_vel)
{
    Go_Tx_Data.ID = ID;
    Go_Tx_Data.status = 1;
    Go_Tx_Data.parameters.torque = 0;
    Go_Tx_Data.parameters.omega = (int16_t)(((W*6.33f) / (2.0f * PI)) * 256.0f);
    Go_Tx_Data.parameters.pos = 0;
    Go_Tx_Data.parameters.k_pos = 0;
    Go_Tx_Data.parameters.k_vel = (int16_t)(K_vel * 1280.0f);
    Go_Control_Send();
}

void Go_Motor_Position_Control(uint8_t ID, float pos, float K_pos)
{
    Go_Tx_Data.ID = ID;
    Go_Tx_Data.status = 1;
    Go_Tx_Data.parameters.torque = 0;
    Go_Tx_Data.parameters.omega = 0;
    Go_Tx_Data.parameters.pos = (int32_t)(((pos*6.33f)/(2.0f*PI))*32768.0f);
    Go_Tx_Data.parameters.k_pos = (int16_t)(K_pos*1280.0f);
    Go_Tx_Data.parameters.k_vel = 0;
    Go_Control_Send();
}

void Go_Encoder_Correction(uint8_t ID)
{
    Go_Tx_Data.ID = ID;
    Go_Tx_Data.status = 2;
    Go_Tx_Data.parameters.torque = 0;
    Go_Tx_Data.parameters.omega = 0;
    Go_Tx_Data.parameters.pos = 0;
    Go_Tx_Data.parameters.k_pos = 0;
    Go_Tx_Data.parameters.k_vel = 0;
    Go_Control_Send();
}

void Go_Vel_PID_Control(uint8_t ID)
{
    float err = Go_Ctrl_Data[ID].des_vel -  Go_State[ID].omega ;
    float output_torque = PID_Calculate(&Go_Vel_PID_Num[ID], err);
    Go_Motor_Torque_Control(ID, output_torque);
}

void Go_Pos_PID_Control(uint8_t ID)
{
    static int time;
    time++;
    float err = Go_Ctrl_Data[ID].des_pos - Go_State[ID].pos;
    Go_Ctrl_Data[ID].des_vel = PID_Calculate(&Go_Pos_PID_Num[ID], err);
    if(time % 400 == 0)
        Usart_Printf("%f,%f\r\n",Go_Ctrl_Data[ID].des_pos,Go_State[ID].pos);
    Go_Vel_PID_Control(ID);
}

void Go_Pos_S_Control(uint8_t ID)
{
    float err = Go_Ctrl_Data[ID].des_pos - Go_State[ID].pos;

}
// Uart Receive part
extern DMA_HandleTypeDef hdma_usart3_rx;
uint8_t rx_data_box[2][SBUS_RX_BUF_NUM];

void USART3_Receive_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    //HAL_UART_Receive_IT()
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) &(USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t) (rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t) (rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

void USART3_Rx_Init() {
    USART3_Receive_init(rx_data_box[0], rx_data_box[1], SBUS_RX_BUF_NUM);
}

void Motor_Ctrl(uint8_t id){
    Go_Ctrl_Data[id].have_send = 0;
    if (Go_State[id].State == PosControl) Go_Motor_Position_Control(id, Go_Ctrl_Data[id].des_pos, Go_Ctrl_Data[id].k_pos);
    else if (Go_State[id].State == VelControl) Go_Motor_Speed_Control(id, Go_Ctrl_Data[id].des_vel, Go_Ctrl_Data[id].k_vel);
    else if (Go_State[id].State == AsCurveControl) Go_Motor_Curve_Control(id);
    else if (Go_State[id].State == PIDVelControl) Go_Vel_PID_Control(id);
    else if (Go_State[id].State == PIDPosControl) Go_Pos_PID_Control(id);
    else if (Go_State[id].State == SPosControl) Go_Pos_S_Control(id);
}

void Received_Data_Dealer(const uint8_t *sbus_buf)
{
    interrupted = 0;
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
    idle_time = 0;
    uint8_t id = sbus_buf[2]<<4;
    id = id >> 4;
    for (int i=0,j=3;i<8;i++,j++) Rev_Parameters_u.data[i] = sbus_buf[j];
    Go_State[id].torque = (float)Rev_Parameters_u.rx_data.torque/256.0f;
    Go_State[id].omega = (((float)Rev_Parameters_u.rx_data.omega/256.0f)*2.0f*PI)/6.33f;
    Go_State[id].pos = (((float)Rev_Parameters_u.rx_data.pos/32768.0f)*2.0f*PI)/6.33f;
    if (Go_State[id].have_init == 0){
        Go_Ctrl_Data[id].des_pos = Go_State[id].pos;
        Go_Ctrl_Data[id].start_pos = Go_State[id].pos;
        Go_Motor_Position_Control(id, Go_Ctrl_Data[id].des_pos, Go_Ctrl_Data[id].k_pos);
        Go_State[id].State = PosControl;
        Go_State[id].have_init = 1;
        return;
    }
    Motor_Ctrl(id);
//    if (Go_State[id].State == AsCurveControl) Go_Motor_Curve_Control(id);
}

void USART3_IRQHandler(void) {
    idle_time = 0;
    if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);

    } else if (USART3->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == 16u){
                Received_Data_Dealer(rx_data_box[0]);
            }
        } else {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_length
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == 16u){
                Received_Data_Dealer(rx_data_box[1]);
            }
        }
    }
}


// AS_Curve Control Part
void AS_Curve_Init(Curve_Data *AS_Curve_Data, float J_a, float J_d, float A_a, float A_d, float V_m)
{
    AS_Curve_Data->J_a = J_a;
    AS_Curve_Data->J_d = J_d;
    AS_Curve_Data->A_a = A_a;
    AS_Curve_Data->A_d = A_d;
    AS_Curve_Data->V_m = V_m;
}

void AS_Curve_Related_Data_Cal(Curve_Data *AS_Curve_Data, float start_angle)
{
    float Tar_Pos = AS_Curve_Data->des_pos - start_angle;
    float Vm = AS_Curve_Data->V_m;

    if (Tar_Pos < 0){
        AS_Curve_Data->reverse = -1;
        Tar_Pos *= -1.0f;
    }else{
        AS_Curve_Data->reverse = 1;
    }
//    uprintf("Tar_Pos is: %f\r\n",Tar_Pos);
//    uprintf("start_angle is: %f\r\n",start_angle);
//    uprintf("reverse is %d\r\n",AS_Curve_Data->reverse);

    if (Vm*AS_Curve_Data->J_a >= AS_Curve_Data->A_a*AS_Curve_Data->A_a){
        AS_Curve_Data->T[1] = AS_Curve_Data->A_a/AS_Curve_Data->J_a;
        AS_Curve_Data->T[2] = (Vm/AS_Curve_Data->A_a)-AS_Curve_Data->T[1];
    }else{
        AS_Curve_Data->T[1] = sqrtf(Vm / AS_Curve_Data->J_a);
        AS_Curve_Data->T[2] = 0;
    }
    AS_Curve_Data->T[3] = AS_Curve_Data->T[1];
    if (Vm*AS_Curve_Data->J_d >= AS_Curve_Data->A_d*AS_Curve_Data->A_d){
        AS_Curve_Data->T[5] = AS_Curve_Data->A_d / AS_Curve_Data->J_d;
        AS_Curve_Data->T[6] = (Vm / AS_Curve_Data->A_d) - (AS_Curve_Data->A_d / AS_Curve_Data->J_d);
    }else{
        AS_Curve_Data->T[6] = 0;
        AS_Curve_Data->T[5] = sqrtf(Vm / AS_Curve_Data->J_d);
    }
    AS_Curve_Data->T[7] = AS_Curve_Data->T[5];
    AS_Curve_Data->T[4] = (Tar_Pos / Vm) - (0.5f * AS_Curve_Data->T[2] + AS_Curve_Data->T[1] + AS_Curve_Data->T[5] + 0.5f * AS_Curve_Data->T[6]);

    if (AS_Curve_Data->T[4] < 0){
        float L,R;
        L = 0, R = Vm;
        for (;fabsf(AS_Curve_Data->T[4])>=0.001f;){
            Vm = (L + R)/2.0f;
            if (Vm*AS_Curve_Data->J_a >= AS_Curve_Data->A_a*AS_Curve_Data->A_a){
                AS_Curve_Data->T[1] = AS_Curve_Data->A_a/AS_Curve_Data->J_a;
                AS_Curve_Data->T[2] = (Vm/AS_Curve_Data->A_a)-AS_Curve_Data->T[1];
            }else{
                AS_Curve_Data->T[1] = sqrtf(Vm / AS_Curve_Data->J_a);
                AS_Curve_Data->T[2] = 0;
            }
            AS_Curve_Data->T[3] = AS_Curve_Data->T[1];
            if (Vm*AS_Curve_Data->J_d >= AS_Curve_Data->A_d*AS_Curve_Data->A_d){
                AS_Curve_Data->T[5] = AS_Curve_Data->A_d / AS_Curve_Data->J_d;
                AS_Curve_Data->T[6] = (Vm / AS_Curve_Data->A_d) - (AS_Curve_Data->A_d / AS_Curve_Data->J_d);
            }else{
                AS_Curve_Data->T[6] = 0;
                AS_Curve_Data->T[5] = sqrtf(Vm / AS_Curve_Data->J_d);
            }
            AS_Curve_Data->T[7] = AS_Curve_Data->T[5];
            AS_Curve_Data->T[4] = (Tar_Pos / Vm) - (0.5f * AS_Curve_Data->T[2] + AS_Curve_Data->T[1] + AS_Curve_Data->T[5] + 0.5f * AS_Curve_Data->T[6]);
            if (AS_Curve_Data->T[4] > 0){
                L = Vm;
            }else{
                R = Vm;
            }
        }
    }

    AS_Curve_Data->P[0] = 0;
    AS_Curve_Data->P[1] = (1.0f/6.0f)*AS_Curve_Data->J_a*AS_Curve_Data->T[1]*AS_Curve_Data->T[1]*AS_Curve_Data->T[1];
    AS_Curve_Data->P[2] = AS_Curve_Data->P[1] + 0.5f*AS_Curve_Data->J_a*AS_Curve_Data->T[1]*AS_Curve_Data->T[2]*(AS_Curve_Data->T[1]+AS_Curve_Data->T[2]);
    AS_Curve_Data->P[3] = AS_Curve_Data->P[2] + AS_Curve_Data->J_a*AS_Curve_Data->T[1]*AS_Curve_Data->T[1]*((5.0f/6.0f)*AS_Curve_Data->T[1]+AS_Curve_Data->T[2]);
    AS_Curve_Data->P[4] = AS_Curve_Data->P[3] + AS_Curve_Data->J_a*AS_Curve_Data->T[1]*(AS_Curve_Data->T[1]+AS_Curve_Data->T[2])*AS_Curve_Data->T[4];
    AS_Curve_Data->P[5] = AS_Curve_Data->P[4] + AS_Curve_Data->J_d*AS_Curve_Data->T[5]*AS_Curve_Data->T[5]*((5.0f/6.0f)*AS_Curve_Data->T[5]+AS_Curve_Data->T[6]);
    AS_Curve_Data->P[6] = AS_Curve_Data->P[5] + 0.5f*AS_Curve_Data->T[6]*AS_Curve_Data->J_a*AS_Curve_Data->T[1]*(AS_Curve_Data->T[1]+AS_Curve_Data->T[2]);
    AS_Curve_Data->P[7] = AS_Curve_Data->P[6] + (1.0f/6.0f)*AS_Curve_Data->J_d*AS_Curve_Data->T[5]*AS_Curve_Data->T[5]*AS_Curve_Data->T[5];

    AS_Curve_Data->V[1] = 0.5f*AS_Curve_Data->J_a*AS_Curve_Data->T[1]*AS_Curve_Data->T[1];
    AS_Curve_Data->V[2] = AS_Curve_Data->J_a*AS_Curve_Data->T[1]*(0.5f*AS_Curve_Data->T[1]+AS_Curve_Data->T[2]);
    AS_Curve_Data->V[3] = AS_Curve_Data->J_a*AS_Curve_Data->T[1]*(AS_Curve_Data->T[1]+AS_Curve_Data->T[2]);
    AS_Curve_Data->V[4] = AS_Curve_Data->V[3];
    AS_Curve_Data->V[5] = AS_Curve_Data->J_d*AS_Curve_Data->T[5]*(0.5f*AS_Curve_Data->T[5]+AS_Curve_Data->T[6]);
    AS_Curve_Data->V[6] = 0.5f*AS_Curve_Data->J_d*AS_Curve_Data->T[5]*AS_Curve_Data->T[5];
    for (int i=2;i<=7;i++) AS_Curve_Data->T[i] += AS_Curve_Data->T[i-1]; // 时间前项和
    AS_Curve_Data->start_angle = start_angle;

    // 倒转处理
    for (int i=0;i<=7;i++){
        AS_Curve_Data->P[i] *= (float)AS_Curve_Data->reverse;
        AS_Curve_Data->P[i] += start_angle;
    }
}

float Find_Triple_root(float a, float b, float c, float d, float l, float r)
{
    float ans = (l+r)/2.0f;
    int cnt = 0;
    while (fabsf(a*powf(ans,3.0f)+b*powf(ans,2.0f)+c*ans+d) > 0.01f){
        if (a*powf(ans,3.0f)+b*powf(ans,2.0f)+c*ans+d<0) l = ans;
        else r = ans;
        ans = (l+r)/2.0f;
        if (fabsf(l-r)<=0.0001f) return ans;
    }
//    uprintf("ans is %f\r\n",ans);
    // float _ans = ans*cosf()
    return ans;
}

float S_Curve_cal(Curve_Data *AS_Curve_Data, int id)
{
    float Now_Pos = Go_State[id].pos;
    float Next_Speed;
    uint8_t stage = Go_State[id].State;

    if (stage == PosControl){
        // Next_Speed = PosCtrl(&(Driver->posCtrl), id);
        Next_Speed = 0;
        return Next_Speed;
    }

    // Calculate related data
    if (!AS_Curve_Data->have_calculated){
        AS_Curve_Related_Data_Cal(AS_Curve_Data, Now_Pos);
        AS_Curve_Data->have_calculated = 1;
        // uprintf("Finish calculation!\r\n");
    }
    float J_a = AS_Curve_Data->J_a;
    float J_d = AS_Curve_Data->J_d;
    float A_a = AS_Curve_Data->A_a;
    float A_d = AS_Curve_Data->A_d;

    // Run the algorithm
    float P[8],V[8],T[8];
    for (int j=0;j<=7;j++) {
        P[j] = AS_Curve_Data->P[j];
        V[j] = AS_Curve_Data->V[j];
        T[j] = AS_Curve_Data->T[j];
    }
    float reverse = (float)AS_Curve_Data->reverse;
    if (Now_Pos*reverse >= (P[0]-0.2f*reverse)*reverse && Now_Pos*reverse <= P[1]*reverse){
        float delta_pos = (Now_Pos - P[0])*reverse;
        float t = Find_Triple_root((1.0f/6.0f)*J_a,0,0,-delta_pos,0,T[1]);
        Next_Speed = 0.5f*J_a*t*t;
//        if (cnt %100 == 0){
//            uprintf("delta_pos: %f\r\n",delta_pos);
//            uprintf("output: %f\r\n",cbrtf(((6.0f*delta_pos)/J_a)*((6.0f*delta_pos)/J_a)));
//        }
        if (Next_Speed <= 1.2f) Next_Speed = 1.2f;
    }else if (Now_Pos*reverse >= P[1]*reverse && Now_Pos*reverse <= P[2]*reverse){
        float delta_pos = (Now_Pos - P[1])*reverse;
        Next_Speed = sqrtf(V[1]*V[1]+2.0f*A_a*delta_pos);
    }else if (Now_Pos*reverse >= P[2]*reverse && Now_Pos*reverse <= P[3]*reverse){
        float delta_pos = (Now_Pos - P[2])*reverse;
        float t = Find_Triple_root(-(1.0f/6.0f)*J_a,0.5f*J_a*T[1],V[2],-delta_pos,0,T[3]-T[2]);
        Next_Speed = V[2] + J_a*T[1]*t - 0.5f*J_a*t*t;
        // if (Next_Speed >= V[3]) Next_Speed = V[3];
    }else if (Now_Pos*reverse >= P[3]*reverse && Now_Pos*reverse <= P[4]*reverse){
        Next_Speed = V[3];
    }else if (Now_Pos*reverse >= P[4]*reverse && Now_Pos*reverse <= P[5]*reverse){
        float delta_pos = (Now_Pos - P[4])*reverse;
        float t = Find_Triple_root(-(1.0f/6.0f)*J_d,0,V[4],-delta_pos,0,T[5]-T[4]);
        Next_Speed = V[4] - 0.5f*J_d*t*t;
    }else if (Now_Pos*reverse >= P[5]*reverse && Now_Pos*reverse <= P[6]*reverse){
        float delta_pos = (Now_Pos - P[5])*reverse;
        Next_Speed = sqrtf(V[5]*V[5]-2.0f*A_d*delta_pos);
    }else if (Now_Pos*reverse >= P[6]*reverse && Now_Pos*reverse <= (P[7])*reverse){
        float delta_pos = (Now_Pos - P[6])*reverse;
        T[6] = T[6] - T[5];
        T[5] = T[5] - T[4];
        float t = Find_Triple_root((1.0f/6.0f)*J_d,-0.5f*J_d*T[5],V[4]-0.5f*T[5]*T[5]*J_d-J_d*T[5]*T[6],-delta_pos,0,T[7]-T[6]);
        Next_Speed = V[4]+J_d*(0.5f*t*t-T[5]*t-0.5f*T[5]*T[5]-T[5]*T[6]);
        if (fabsf(Now_Pos-P[7])<=0.001f){
            Next_Speed = 0;
            Go_State[id].State = PosControl;
        }
    }else{
        Next_Speed = 0;
        HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
        Go_State[id].State = PosControl;
    }

//    if (cnt%100 == 0) uprintf("Next_Speed is %f\r\n",Next_Speed);
    Next_Speed *= (float)reverse;
//    if (cnt%100 == 0){
//        uprintf("Next_Speed is %f\r\n",Next_Speed);
//        uprintf("Now_Speed is %f\r\n", Driver->velCtrl.actualSpeed);
//        uprintf("Now_Pos is %f\r\n\r\n",Now_Pos);
//    }
    return Next_Speed;
}


void Go_Motor_Curve_Control(uint8_t id)
{
    float des_vel = S_Curve_cal(&Go_Ctrl_Data[id], id);
    if (Go_State[id].State == PosControl) Go_Motor_Position_Control(0, Go_Ctrl_Data[id].des_pos, Go_Ctrl_Data[id].k_pos);
    else {
//        if (des_vel != Go_Ctrl_Data[id].des_vel) Go_Ctrl_Data[id].des_vel = des_vel;
        Go_Motor_Speed_Control(id, des_vel, Go_Ctrl_Data[0].k_vel);
    }
}

void Go_Update_data(uint8_t id)
{
    if (Go_State[id].State == AsCurveControl) return;
    else{
        int temp = idle_time;
        Motor_Ctrl(id);
        while (temp <= idle_time) continue;
    }
}

void motor_ReadParam(int id)
{
    Flash_Data.t_data = (uint32_t)FLASH_Param[0];
    Go_Ctrl_Data[id].k_vel = Flash_Data.s_data;
    Flash_Data.t_data = (uint32_t)FLASH_Param[1];
    Go_Ctrl_Data[id].k_pos = Flash_Data.s_data;
    Flash_Data.t_data = (uint32_t)FLASH_Param[2];
    Go_Ctrl_Data[id].J_a = Flash_Data.s_data;
    Flash_Data.t_data = (uint32_t)FLASH_Param[3];
    Go_Ctrl_Data[id].J_d = Flash_Data.s_data;
    Flash_Data.t_data = (uint32_t)FLASH_Param[4];
    Go_Ctrl_Data[id].A_a = Flash_Data.s_data;
    Flash_Data.t_data = (uint32_t)FLASH_Param[5];
    Go_Ctrl_Data[id].A_d = Flash_Data.s_data;
    Flash_Data.t_data = (uint32_t)FLASH_Param[6];
    Go_Ctrl_Data[id].V_m = Flash_Data.s_data;
}

void motorOn(uint8_t id)
{
    motor_ReadParam(id);
    Go_Pos_PID_Init(id);
    Go_Vel_PID_Init(id);
    Go_State[id].State = PIDPosControl;
    Go_State[id].have_init = 0;
    Go_Pos_PID_Control(0);
    while (!Go_State[id].have_init) continue;

}
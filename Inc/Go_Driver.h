#include <main.h>

#define SBUS_RX_BUF_NUM 32u

enum CurveFlag{
    PosControl = 0,
    VelControl,
    AsCurveControl,
    PIDVelControl,
    PIDPosControl,
    SPosControl
};

typedef struct{
    int16_t torque;
    int16_t omega;
    int32_t pos;
    int16_t k_pos;
    int16_t k_vel;
}Body_Data;

typedef struct{
    uint8_t status;
    uint8_t ID;
    Body_Data parameters;
}Go_Send_Data;

typedef struct{
    int16_t torque;
    int16_t omega;
    int32_t pos;
}Go_Rev_Data;

typedef struct{
    float torque;
    float omega;
    float pos;
    uint8_t State;
    uint8_t have_init;
}Go_Status;

typedef struct{
    float A_a;
    float A_d;
    float J_a;
    float J_d;
    float V_m;
    float des_pos;
    float des_vel;
    float start_pos;
    int8_t reverse;
    uint8_t have_calculated;
    uint8_t have_init;
    uint8_t have_send;
    float P[8];
    float T[8];
    float V[8];
    float start_angle;
    float k_vel;
    float k_pos;
}Curve_Data;

typedef union{
    uint32_t t_data;
    float s_data
}Flash_Data_u;

struct S_Data{
    int beginTime;
    int endTime;
    float Begin;
    float End;
    float A;
    float V;
};

extern Flash_Data_u Flash_Data;

extern Curve_Data Go_Ctrl_Data[14];
extern Go_Status Go_State[14];
extern uint8_t s_First;

void Go_Motor_Speed_Control(uint8_t ID, float W, float K_vel);
void Usart3_TX_DMA_Init(void);
void USART3_Rx_Init();
void Go_Motor_Position_Control(uint8_t ID, float pos, float K_pos);
void Go_Encoder_Correction(uint8_t ID);
void AS_Curve_Init(Curve_Data *AS_Curve_Data, float J_a, float J_d, float A_a, float A_d, float V_m);
void Go_Motor_Curve_Control(uint8_t id);
void Go_Vel_PID_Control(uint8_t ID);
void Go_Pos_PID_Control(uint8_t ID);
void Go_Pos_S_Control(uint8_t ID);
void Motor_Ctrl(uint8_t id);
void Go_Update_data(uint8_t id);
void motorOn(uint8_t id);
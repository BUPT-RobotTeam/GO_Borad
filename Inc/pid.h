//
// Created by Trisoil on 2023/6/1.
//
#include "main.h"

#ifndef GO_BOARD_PID_H
#define GO_BOARD_PID_H

#define LIMIT(x,max) (x>max) ? max : x

typedef struct {
    float kp;
    float ki;
    float kd;
    float kp_output;
    float ki_output;
    float kd_output;
    float output;
    float integral_limit_err;
    float now_err;
    float last_err;
    float deadband;
    float max_output;
    float integral_limit;
    float max_err;
}PID_Number;

extern PID_Number Go_Vel_PID_Num[14];
extern PID_Number Go_Pos_PID_Num[14];

void Go_Vel_PID_Init(uint8_t id);
void Go_Pos_PID_Init(uint8_t id);
float PID_Calculate(PID_Number *pid, float err);


#endif //GO_BOARD_PID_H

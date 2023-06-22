//
// Created by Trisoil on 2023/6/1.
//

#include "pid.h"
#include "math.h"

PID_Number Go_Vel_PID_Num[14];
PID_Number Go_Pos_PID_Num[14];

void Go_Vel_PID_Init(uint8_t id)
{
    Go_Vel_PID_Num[id].kp = 0.25f;
    Go_Vel_PID_Num[id].ki = 0;
    Go_Vel_PID_Num[id].kd = 9.0f;
    Go_Vel_PID_Num[id].max_output  = 20.0f;
    Go_Vel_PID_Num[id].integral_limit = 100.0f;
    Go_Vel_PID_Num[id].integral_limit_err = 1.0f;
    Go_Vel_PID_Num[id].deadband = 10;
    Go_Vel_PID_Num[id].max_err = 8000;
}

void Go_Pos_PID_Init(uint8_t id)
{
    Go_Pos_PID_Num[id].kp = 0.1f;
    Go_Pos_PID_Num[id].ki = 0;
    Go_Pos_PID_Num[id].kd = 0;
    Go_Pos_PID_Num[id].max_output  = 200.0f;
    Go_Pos_PID_Num[id].integral_limit = 100.0f;
    Go_Pos_PID_Num[id].integral_limit_err = 1.0f;
    Go_Pos_PID_Num[id].deadband = 10;
    Go_Pos_PID_Num[id].max_err = 8000;
}

float PID_Calculate(PID_Number *pid, float err)
{
    pid->now_err = err;
    if (fabsf(err) <= pid->integral_limit_err) pid->ki_output += pid->ki*pid->now_err;
    pid->ki_output = LIMIT(pid->ki_output, pid->integral_limit); // ki_output limit
    pid->output = pid->kp*pid->now_err + pid->ki_output + pid->kd*(pid->now_err - pid->last_err);
    pid->last_err = pid->now_err;
    pid->output = LIMIT(pid->output, pid->max_output);
    if (fabsf(pid->output) > pid->max_output) pid->output = pid->max_output;
    return pid->output;
}

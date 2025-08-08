/**
 * @file pid.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float dt, float output_min, float output_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->setpoint = 0.0f;
}

float PID_Compute(PID_HandleTypeDef *pid, float measurement)
{
    float error = pid->setpoint - measurement;
    pid->integral += error * pid->dt;

    float derivative = (error - pid->last_error) / pid->dt;
    pid->last_error = error;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // �޷�
    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;

    return output;
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

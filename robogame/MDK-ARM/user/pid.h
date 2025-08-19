/**
 * @file pid.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef PID_HPP
#define PID_HPP

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
typedef struct 
{
    float Kp;
    float Ki;
    float Kd;

    float setpoint;      
    float integral;      
    float last_error;   

    float output_max;    
    float output_min;   

    float dt;            
} PID_HandleTypeDef;

/* Exported function declarations --------------------------------------------*/

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float dt, float output_min, float output_max);
float PID_Compute(PID_HandleTypeDef *pid, float measurement);
void PID_Reset(PID_HandleTypeDef *pid);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

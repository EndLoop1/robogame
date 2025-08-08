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

    float setpoint;      // 目标值
    float integral;      // 积分项累积
    float last_error;    // 上次误差

    float output_max;    // 输出最大值限制
    float output_min;    // 输出最小值限制

    float dt;            // 采样周期，单位秒
} PID_HandleTypeDef;

/* Exported function declarations --------------------------------------------*/

// 初始化PID结构体
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float dt, float output_min, float output_max);
// 计算PID输出
float PID_Compute(PID_HandleTypeDef *pid, float measurement);
// 重置PID状态
void PID_Reset(PID_HandleTypeDef *pid);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

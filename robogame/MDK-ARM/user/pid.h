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

    float setpoint;      // Ŀ��ֵ
    float integral;      // �������ۻ�
    float last_error;    // �ϴ����

    float output_max;    // ������ֵ����
    float output_min;    // �����Сֵ����

    float dt;            // �������ڣ���λ��
} PID_HandleTypeDef;

/* Exported function declarations --------------------------------------------*/

// ��ʼ��PID�ṹ��
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float dt, float output_min, float output_max);
// ����PID���
float PID_Compute(PID_HandleTypeDef *pid, float measurement);
// ����PID״̬
void PID_Reset(PID_HandleTypeDef *pid);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

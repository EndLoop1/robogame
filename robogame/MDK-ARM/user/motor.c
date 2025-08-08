/**
 * @file motor.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

// PID控制器数组
static PID_HandleTypeDef motorPID[MOTOR_NUM];

// 电机控制数据结构
typedef struct {
    int32_t last_encoder_count;
    MotorDirection direction;
    float target_speed_rpm;
} Motor_t;

static Motor_t motors[MOTOR_NUM];

// 工具函数：获取编码器计数
static int32_t GetEncoderCount(uint8_t motor_id) 
{
    switch(motor_id) 
    {
        case 0: return __HAL_TIM_GET_COUNTER(&htim2);
        case 1: return __HAL_TIM_GET_COUNTER(&htim3);
        case 2: return __HAL_TIM_GET_COUNTER(&htim4);
        case 3: return __HAL_TIM_GET_COUNTER(&htim5);
        default: return 0;
    }
}

// 设置电机方向
static void SetMotorDirection(uint8_t motor_id, MotorDirection dir) 
{
    switch(motor_id) 
    {
        case 0:
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR1_DIR1_PIN, (dir == MOTOR_DIR_FORWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR1_DIR2_PIN, (dir == MOTOR_DIR_BACKWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR2_DIR1_PIN, (dir == MOTOR_DIR_FORWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR2_DIR2_PIN, (dir == MOTOR_DIR_BACKWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR3_DIR1_PIN, (dir == MOTOR_DIR_FORWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR3_DIR2_PIN, (dir == MOTOR_DIR_BACKWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR4_DIR1_PIN, (dir == MOTOR_DIR_FORWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR4_DIR2_PIN, (dir == MOTOR_DIR_BACKWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

// 设置PWM占空比（0~PWM_MAX）
static void SetMotorPWM(uint8_t motor_id, uint16_t pwm) 
{
    if (pwm > PWM_MAX) pwm = PWM_MAX;

    switch(motor_id) 
    {
        case 0:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
            break;
        case 1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm);
            break;
        default:
            break;
    }
}

// 计算速度（单位：RPM）
static float CalculateSpeedRPM(int32_t count_diff, float dt_s) 
{
    // 转速 = (脉冲数/编码器PPR) / 时间(s) * 60秒/分钟
    return ((float)count_diff / ENCODER_PPR) / dt_s * 60.0f;
}

// 初始化
void MotorControl_Init(void) 
{
    for (int i = 0; i < MOTOR_NUM; i++) 
    {
        motors[i].last_encoder_count = GetEncoderCount(i);
        motors[i].direction = MOTOR_DIR_STOP;
        motors[i].target_speed_rpm = 0;

        PID_Init(&motorPID[i], 1.0f, 50.0f, 0.0f, 0.001f, 0.0f, PWM_MAX); // PID参数需调试
        motorPID[i].setpoint = 0;
    }
}

// 设置目标速度，正数正转，负数反转
void MotorControl_SetTargetSpeed(uint8_t motor_id, float speed_rpm) 
{
    if (motor_id >= MOTOR_NUM) return;

    motors[motor_id].target_speed_rpm = speed_rpm;
    motorPID[motor_id].setpoint = fabsf(speed_rpm);
}

// 1ms周期调用的更新函数
void MotorControl_Update(void) 
{
    const float dt_s = 0.001f; // 1ms

    for (int i = 0; i < MOTOR_NUM; i++) 
    {
        int32_t current_count = GetEncoderCount(i);
        int32_t last_count = motors[i].last_encoder_count;

        int32_t diff = current_count - last_count;
        if (diff > (COUNTER_MAX / 2)) diff -= (COUNTER_MAX + 1);
        else if (diff < -(COUNTER_MAX / 2)) diff += (COUNTER_MAX + 1);

        motors[i].last_encoder_count = current_count;

        float speed = CalculateSpeedRPM(diff, dt_s);

        float pid_output = PID_Compute(&motorPID[i], speed);

        // 设置方向
        if (motors[i].target_speed_rpm > 0) 
        {
            motors[i].direction = MOTOR_DIR_FORWARD;
        } 
        else if (motors[i].target_speed_rpm < 0) 
        {
            motors[i].direction = MOTOR_DIR_BACKWARD;
        } 
        else 
        {
            motors[i].direction = MOTOR_DIR_STOP;
            pid_output = 0;
        }

        SetMotorDirection(i, motors[i].direction);
        SetMotorPWM(i, (uint16_t)pid_output);
    }
}

void MotorControl_Forward(void)
{

}

void MotorControl_Backward(void)
{

}

void MotorControl_Stop(void)
{

}

void MotorControl_Rotate(void)
{

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

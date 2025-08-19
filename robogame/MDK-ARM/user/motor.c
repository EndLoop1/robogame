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
static void SetMotorPWM(uint8_t motor_id, uint16_t pwm, MotorDirection dir) 
{
    if (pwm > PWM_MAX) pwm = PWM_MAX;

    switch(motor_id) 
    {
        case 0:
            if (dir == MOTOR_DIR_FORWARD) 
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm); // RPWM
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);   // LPWM
            } 
            else if (dir == MOTOR_DIR_BACKWARD) 
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
            } 
            else 
            { // STOP
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            }
            break;
        case 1:
            if (dir == MOTOR_DIR_FORWARD) 
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm); // RPWM
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);   // LPWM
            } 
            else if (dir == MOTOR_DIR_BACKWARD) 
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
            } 
            else 
            { // STOP
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            }
            break;
        case 2:
            if (dir == MOTOR_DIR_FORWARD) 
            {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm); // RPWM
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);   // LPWM
            } 
            else if (dir == MOTOR_DIR_BACKWARD) 
            {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm);
            } 
            else 
            { // STOP
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
            }
            break;
        case 3:
            if (dir == MOTOR_DIR_FORWARD) 
            {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm); // RPWM
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);   // LPWM
            } 
            else if (dir == MOTOR_DIR_BACKWARD) 
            {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm);
            } 
            else 
            { // STOP
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
            }
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
        // 编码器差值
        int32_t current_count = GetEncoderCount(i);
        int32_t diff = current_count - motors[i].last_encoder_count;

        if (diff > (COUNTER_MAX / 2)) 
            diff -= (COUNTER_MAX + 1);
        else if (diff < -(COUNTER_MAX / 2)) 
            diff += (COUNTER_MAX + 1);

        motors[i].last_encoder_count = current_count;

        // 当前速度
        float speed = CalculateSpeedRPM(diff, dt_s);

        // PID 计算（目标值用绝对值，方向单独判断）
        float pid_output = PID_Compute(&motorPID[i], fabsf(speed));

        // 确定方向
        MotorDirection dir;
        if (motors[i].target_speed_rpm > 0) 
        {
            dir = MOTOR_DIR_FORWARD;
        } 
        else if (motors[i].target_speed_rpm < 0) 
        {
            dir = MOTOR_DIR_BACKWARD;
        } 
        else 
        {
            dir = MOTOR_DIR_STOP;
            pid_output = 0;
        }
        motors[i].direction = dir;

        // 直接调用统一的 IBT-2 PWM 输出
        SetMotorPWM(i, (uint16_t)pid_output, dir);
    }
}

void MotorControl_SetVelocityVector(float Vx, float Vy, float Omega)
{
    // 轮子参数（米）
    const float r = 0.04f;   // 轮半径 4cm
    const float Lx = 0.15f;  // 前后轮中心到质心距离
    const float Ly = 0.20f;  // 左右轮中心到质心距离

    // 计算四个轮子目标速度（单位 RPM）
    float rpm0 = ((Vx - Vy - (Lx+Ly)*Omega) / (2.0f * 3.1415926f * r)) * 60.0f; // 前左
    float rpm1 = ((Vx + Vy + (Lx+Ly)*Omega) / (2.0f * 3.1415926f * r)) * 60.0f; // 前右
    float rpm2 = ((Vx - Vy + (Lx+Ly)*Omega) / (2.0f * 3.1415926f * r)) * 60.0f; // 后右
    float rpm3 = ((Vx + Vy - (Lx+Ly)*Omega) / (2.0f * 3.1415926f * r)) * 60.0f; // 后左

    // 设置四个电机目标速度
    MotorControl_SetTargetSpeed(0, rpm0);
    MotorControl_SetTargetSpeed(1, rpm1);
    MotorControl_SetTargetSpeed(2, rpm2);
    MotorControl_SetTargetSpeed(3, rpm3);
}

// 前进，speed_m_s 为线速度，单位 m/s
void MotorControl_Forward(float speed_m_s)
{
    MotorControl_SetVelocityVector(speed_m_s, 0.0f, 0.0f);
}

// 后退，speed_m_s 为线速度，单位 m/s
void MotorControl_Backward(float speed_m_s)
{
    MotorControl_SetVelocityVector(-speed_m_s, 0.0f, 0.0f);
}

// 左平移，speed_m_s 为线速度，单位 m/s
void MotorControl_Left(float speed_m_s)
{
    MotorControl_SetVelocityVector(0.0f, -speed_m_s, 0.0f);
}

// 右平移，speed_m_s 为线速度，单位 m/s
void MotorControl_Right(float speed_m_s)
{
    MotorControl_SetVelocityVector(0.0f, speed_m_s, 0.0f);
}

// 顺时针原地旋转，omega_rad_s 单位 rad/s
void MotorControl_RotateCW(float omega_rad_s)
{
    MotorControl_SetVelocityVector(0.0f, 0.0f, omega_rad_s);
}

// 逆时针原地旋转
void MotorControl_RotateCCW(float omega_rad_s)
{
    MotorControl_SetVelocityVector(0.0f, 0.0f, -omega_rad_s);
}

// 停止
void MotorControl_Stop(void)
{
    MotorControl_SetVelocityVector(0.0f, 0.0f, 0.0f);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

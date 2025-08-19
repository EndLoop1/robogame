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

// PID����������
static PID_HandleTypeDef motorPID[MOTOR_NUM];

// ����������ݽṹ
typedef struct {
    int32_t last_encoder_count;
    MotorDirection direction;
    float target_speed_rpm;
} Motor_t;

static Motor_t motors[MOTOR_NUM];

// ���ߺ�������ȡ����������
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

// ���õ������
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

// ����PWMռ�ձȣ�0~PWM_MAX��
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

// �����ٶȣ���λ��RPM��
static float CalculateSpeedRPM(int32_t count_diff, float dt_s) 
{
    // ת�� = (������/������PPR) / ʱ��(s) * 60��/����
    return ((float)count_diff / ENCODER_PPR) / dt_s * 60.0f;
}

// ��ʼ��
void MotorControl_Init(void) 
{
    for (int i = 0; i < MOTOR_NUM; i++) 
    {
        motors[i].last_encoder_count = GetEncoderCount(i);
        motors[i].direction = MOTOR_DIR_STOP;
        motors[i].target_speed_rpm = 0;

        PID_Init(&motorPID[i], 1.0f, 50.0f, 0.0f, 0.001f, 0.0f, PWM_MAX); // PID���������
        motorPID[i].setpoint = 0;
    }
}

// ����Ŀ���ٶȣ�������ת��������ת
void MotorControl_SetTargetSpeed(uint8_t motor_id, float speed_rpm) 
{
    if (motor_id >= MOTOR_NUM) return;

    motors[motor_id].target_speed_rpm = speed_rpm;
    motorPID[motor_id].setpoint = fabsf(speed_rpm);
}

// 1ms���ڵ��õĸ��º���
void MotorControl_Update(void) 
{
    const float dt_s = 0.001f; // 1ms

    for (int i = 0; i < MOTOR_NUM; i++) 
    {
        // ��������ֵ
        int32_t current_count = GetEncoderCount(i);
        int32_t diff = current_count - motors[i].last_encoder_count;

        if (diff > (COUNTER_MAX / 2)) 
            diff -= (COUNTER_MAX + 1);
        else if (diff < -(COUNTER_MAX / 2)) 
            diff += (COUNTER_MAX + 1);

        motors[i].last_encoder_count = current_count;

        // ��ǰ�ٶ�
        float speed = CalculateSpeedRPM(diff, dt_s);

        // PID ���㣨Ŀ��ֵ�þ���ֵ�����򵥶��жϣ�
        float pid_output = PID_Compute(&motorPID[i], fabsf(speed));

        // ȷ������
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

        // ֱ�ӵ���ͳһ�� IBT-2 PWM ���
        SetMotorPWM(i, (uint16_t)pid_output, dir);
    }
}

void MotorControl_SetVelocityVector(float Vx, float Vy, float Omega)
{
    // ���Ӳ������ף�
    const float r = 0.04f;   // �ְ뾶 4cm
    const float Lx = 0.15f;  // ǰ�������ĵ����ľ���
    const float Ly = 0.20f;  // ���������ĵ����ľ���

    // �����ĸ�����Ŀ���ٶȣ���λ RPM��
    float rpm0 = ((Vx - Vy - (Lx+Ly)*Omega) / (2.0f * 3.1415926f * r)) * 60.0f; // ǰ��
    float rpm1 = ((Vx + Vy + (Lx+Ly)*Omega) / (2.0f * 3.1415926f * r)) * 60.0f; // ǰ��
    float rpm2 = ((Vx - Vy + (Lx+Ly)*Omega) / (2.0f * 3.1415926f * r)) * 60.0f; // ����
    float rpm3 = ((Vx + Vy - (Lx+Ly)*Omega) / (2.0f * 3.1415926f * r)) * 60.0f; // ����

    // �����ĸ����Ŀ���ٶ�
    MotorControl_SetTargetSpeed(0, rpm0);
    MotorControl_SetTargetSpeed(1, rpm1);
    MotorControl_SetTargetSpeed(2, rpm2);
    MotorControl_SetTargetSpeed(3, rpm3);
}

// ǰ����speed_m_s Ϊ���ٶȣ���λ m/s
void MotorControl_Forward(float speed_m_s)
{
    MotorControl_SetVelocityVector(speed_m_s, 0.0f, 0.0f);
}

// ���ˣ�speed_m_s Ϊ���ٶȣ���λ m/s
void MotorControl_Backward(float speed_m_s)
{
    MotorControl_SetVelocityVector(-speed_m_s, 0.0f, 0.0f);
}

// ��ƽ�ƣ�speed_m_s Ϊ���ٶȣ���λ m/s
void MotorControl_Left(float speed_m_s)
{
    MotorControl_SetVelocityVector(0.0f, -speed_m_s, 0.0f);
}

// ��ƽ�ƣ�speed_m_s Ϊ���ٶȣ���λ m/s
void MotorControl_Right(float speed_m_s)
{
    MotorControl_SetVelocityVector(0.0f, speed_m_s, 0.0f);
}

// ˳ʱ��ԭ����ת��omega_rad_s ��λ rad/s
void MotorControl_RotateCW(float omega_rad_s)
{
    MotorControl_SetVelocityVector(0.0f, 0.0f, omega_rad_s);
}

// ��ʱ��ԭ����ת
void MotorControl_RotateCCW(float omega_rad_s)
{
    MotorControl_SetVelocityVector(0.0f, 0.0f, -omega_rad_s);
}

// ֹͣ
void MotorControl_Stop(void)
{
    MotorControl_SetVelocityVector(0.0f, 0.0f, 0.0f);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

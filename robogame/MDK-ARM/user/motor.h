/**
 * @file motor.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "pid.h"

/* Exported macros -----------------------------------------------------------*/
#define MOTOR_DIR_GPIO_PORT GPIOF

#define MOTOR1_DIR1_PIN GPIO_PIN_0
#define MOTOR1_DIR2_PIN GPIO_PIN_1
#define MOTOR2_DIR1_PIN GPIO_PIN_2
#define MOTOR2_DIR2_PIN GPIO_PIN_3
#define MOTOR3_DIR1_PIN GPIO_PIN_4
#define MOTOR3_DIR2_PIN GPIO_PIN_5
#define MOTOR4_DIR1_PIN GPIO_PIN_6
#define MOTOR4_DIR2_PIN GPIO_PIN_7

#define MOTOR_NUM 4
#define PWM_MAX 3599
#define ENCODER_PPR 1000       
#define COUNTER_MAX 0xFFFF     

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD = 1,
    MOTOR_DIR_STOP = 2,
} MotorDirection;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void MotorControl_Init(void);
void MotorControl_SetTargetSpeed(uint8_t motor_id, float speed_rpm);
void MotorControl_Update(void);
void MotorControl_Forward(void);
void MotorControl_Backward(void);
void MotorControl_Stop(void);
void MotorControl_Rotate(void);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

/**
 * @file stepmotor.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-019
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef STEPMOTOR_HPP
#define STEPMOTOR_HPP

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Exported macros -----------------------------------------------------------*/
#define STEPMOTOR_EN_GPIO_PORT   GPIOF
#define STEPMOTOR_EN_PIN         GPIO_PIN_8

#define STEPMOTOR_DIR_GPIO_PORT  GPIOF
#define STEPMOTOR_DIR_PIN        GPIO_PIN_9

#define STEPMOTOR_PUL_GPIO_PORT  GPIOA
#define STEPMOTOR_PUL_PIN        GPIO_PIN_2

/* Exported types ------------------------------------------------------------*/
typedef enum {
    STEPMOTOR_DIR_CW = 0,   // 顺时针
    STEPMOTOR_DIR_CCW       // 逆时针
} StepMotorDir_t;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void StepMotor_Init(void);
void StepMotor_Enable(uint8_t enable);
void StepMotor_SetDir(StepMotorDir_t dir);
void StepMotor_Step(uint16_t steps, uint16_t delay_us);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

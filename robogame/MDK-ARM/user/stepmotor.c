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
#include "stepmotor.h"
#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

// 步进电机初始化
void StepMotor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ENA 引脚 */
    GPIO_InitStruct.Pin = STEPMOTOR_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;   // 开漏输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // 共阳极，不需要下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STEPMOTOR_EN_GPIO_PORT, &GPIO_InitStruct);

    /* DIR 引脚 */
    GPIO_InitStruct.Pin = STEPMOTOR_DIR_PIN;
    HAL_GPIO_Init(STEPMOTOR_DIR_GPIO_PORT, &GPIO_InitStruct);

    /* PUL 引脚 */
    GPIO_InitStruct.Pin = STEPMOTOR_PUL_PIN;
    HAL_GPIO_Init(STEPMOTOR_PUL_GPIO_PORT, &GPIO_InitStruct);

    /* 默认拉高 ENA（禁止） */
    HAL_GPIO_WritePin(STEPMOTOR_EN_GPIO_PORT, STEPMOTOR_EN_PIN, GPIO_PIN_SET);
}

/* 电机使能控制 */
void StepMotor_Enable(uint8_t enable)
{
    if (enable)
        HAL_GPIO_WritePin(STEPMOTOR_EN_GPIO_PORT, STEPMOTOR_EN_PIN, GPIO_PIN_RESET); // ENA=0 使能
    else
        HAL_GPIO_WritePin(STEPMOTOR_EN_GPIO_PORT, STEPMOTOR_EN_PIN, GPIO_PIN_SET);   // ENA=1 禁止
}

/* 设置方向 */
void StepMotor_SetDir(StepMotorDir_t dir)
{
    if (dir == STEPMOTOR_DIR_CW)
        HAL_GPIO_WritePin(STEPMOTOR_DIR_GPIO_PORT, STEPMOTOR_DIR_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(STEPMOTOR_DIR_GPIO_PORT, STEPMOTOR_DIR_PIN, GPIO_PIN_SET);
}

/* 产生脉冲控制步进 */
void StepMotor_Step(uint16_t steps, uint16_t delay_us)
{
    for (uint16_t i = 0; i < steps; i++)
    {
        HAL_GPIO_WritePin(STEPMOTOR_PUL_GPIO_PORT, STEPMOTOR_PUL_PIN, GPIO_PIN_SET);
        for (volatile uint32_t d = 0; d < delay_us * 8; d++); // 简单延时
        HAL_GPIO_WritePin(STEPMOTOR_PUL_GPIO_PORT, STEPMOTOR_PUL_PIN, GPIO_PIN_RESET);
        for (volatile uint32_t d = 0; d < delay_us * 8; d++);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

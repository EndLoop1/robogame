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


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Motor_Output(int num)
{
	if(num == 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(num > 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(num < 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, (num > 0)?num : -num);
}

int Motor_Conter(int counter)
{
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == GPIO_PIN_SET)
	{
		counter++;
	}
	else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == GPIO_PIN_RESET)
	{
		counter--;
	}
	return(counter);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

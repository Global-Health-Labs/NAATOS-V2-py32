/**
  ******************************************************************************
  * @file    py32f0xx_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
  * @brief 初始化全局MSP
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();                   /* Enable TIM3 clock */
        HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);        /* Set interrupt priority */
        HAL_NVIC_EnableIRQ(TIM3_IRQn);                /* Enable TIM3 interrupt */
    }
    else if (htim->Instance == TIM14)
    {
        __HAL_RCC_TIM14_CLK_ENABLE();                 /* Enable TIM14 clock */
        HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);       /* Set interrupt priority */
        HAL_NVIC_EnableIRQ(TIM14_IRQn);               /* Enable TIM14 interrupt */
    }
    else if (htim->Instance == TIM16)
    {
        __HAL_RCC_TIM16_CLK_ENABLE();                 /* Enable TIM16 clock */
        HAL_NVIC_SetPriority(TIM16_IRQn, 0, 0);       /* Set interrupt priority */
        HAL_NVIC_EnableIRQ(TIM16_IRQn);               /* Enable TIM16 interrupt */
    }
    else if (htim->Instance == TIM17)
    {
        __HAL_RCC_TIM17_CLK_ENABLE();                 /* Enable TIM17 clock */
        HAL_NVIC_SetPriority(TIM17_IRQn, 0, 0);       /* Set interrupt priority */
        HAL_NVIC_EnableIRQ(TIM17_IRQn);               /* Enable TIM17 interrupt */
    }
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

/**
  ******************************************************************************
  * @file    main.h
  * @author  MCU Application Team
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "timers.h"
#include "py32f0xx_hal.h"
#include "app_data.h"

/* Defines ------------------------------------------------------------------*/

/* Exported variables prototypes ---------------------------------------------*/
extern struct app_data_t data;
extern struct flags_t flags;
    
/* Exported functions prototypes ---------------------------------------------*/
void APP_ErrorHandler(void);

void GPIO_Init(void);
void UART_Init(void);

void PWMTimer_ISR(void);
void LEDTimer_ISR(void);
void SecondTimer_ISR(void);
void MinuteTimer_ISR(void);
void DataCollection_ISR(void);
void DelayedStart_ISR(void);
void Pushbutton_ISR(void);

void print_log_data(void);

typedef struct {
    uint8_t enabled;
    uint8_t pwm_setting;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint16_t pwm_state;
    uint64_t pwm_bits[4];
} Pin_pwm_t;

    
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

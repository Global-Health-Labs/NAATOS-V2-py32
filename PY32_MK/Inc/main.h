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
#include <py32f0xx_hal_iwdg.h>
#include "py32f0xx_hal_flash_ex.h"
//#include "py32f002x5.h"
#include "app_data.h"


/* Defines ------------------------------------------------------------------*/

//#define DEBUG 1

typedef enum {
    // define alarm states here, probably related to LED indicators
    SAMPLE_HEATER = 0,
    VALVE_HEATER = 1
} heater_t;

/* Exported functions prototypes ---------------------------------------------*/
void APP_ErrorHandler(uint8_t errnum);

void GPIO_Init(void);
void UART_Init(void);

void PWMTimer_ISR(void);
void LEDTimer_ISR(void);
void PIDTimer_ISR(void);
void MinuteTimer_ISR(void);
void DataCollection_ISR(void);
void DelayedStart_ISR(void);
void Pushbutton_ISR(void);
void LogData_ISR(void);
void ActuationDelay_ISR(void);

void print_log_data(void);
void send_max_temps(void);
bool Validate_USB_Power_Source(void);
bool Validate_Power_Supply(void);

void ADC_data_collect(void);
void Update_PID(void);

typedef struct {
    bool    enabled;
    bool    suspended;
    bool    heater_level_high;
    uint16_t pwm_state;
    uint64_t pwm_bits[4];
    uint8_t  pwm_tick_count;
} Pin_pwm_t;

// Store the pin assignments in this structure
typedef struct {
    GPIO_TypeDef    *GPIOx_AMP_TEMP_V;
    uint16_t        GPIO_Pin_AMP_TEMP_V;
    uint32_t        ADC_CHANNEL_AMP_TEMP_V;
    
    GPIO_TypeDef    *GPIOx_AMP_VALVE_TEMP_V;
    uint16_t        GPIO_Pin_AMP_VALVE_TEMP_V;
    uint32_t        ADC_CHANNEL_VALVE_TEMP_V;
    
    GPIO_TypeDef    *GPIOx_AMP_V_BATT_SENSE;
    uint16_t        GPIO_Pin_V_BATT_SENSE;
    uint32_t        ADC_CHANNEL_V_BATT_SENSE;

    GPIO_TypeDef    *GPIOx_USB_CC1;
    uint16_t        GPIO_Pin_USB_CC1;
    uint32_t        ADC_CHANNEL_USB_CC1;
    
    GPIO_TypeDef    *GPIOx_USB_CC2;
    uint16_t        GPIO_Pin_USB_CC2;
    uint32_t        ADC_CHANNEL_USB_CC2;
    
    GPIO_TypeDef    *GPIOx_LED1;
    uint16_t        GPIO_Pin_LED1;
    GPIO_TypeDef    *GPIOx_LED2;
    uint16_t        GPIO_Pin_LED2;
    GPIO_TypeDef    *GPIOx_ADC_SPARE;
    uint16_t        GPIO_Pin_ADC_SPARE;
    GPIO_TypeDef    *GPIOx_AMP_CTRL1;
    uint16_t        GPIO_Pin_AMP_CTRL1;
    GPIO_TypeDef    *GPIOx_AMP_CTRL2;
    uint16_t        GPIO_Pin_AMP_CTRL2;
    GPIO_TypeDef    *GPIOx_VALVE_CTRL1;
    uint16_t        GPIO_Pin_VALVE_CTRL1;
    GPIO_TypeDef    *GPIOx_VALVE_CTRL2;
    uint16_t        GPIO_Pin_VALVE_CTRL2;
    GPIO_TypeDef    *GPIOx_PUSHBUTTON;
    uint16_t        GPIO_Pin_PUSHBUTTON;
} Pin_assignments_t;

/* Exported variables prototypes ---------------------------------------------*/
extern struct app_data_t data;
extern struct flags_t flags;
extern Pin_assignments_t Pins;


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

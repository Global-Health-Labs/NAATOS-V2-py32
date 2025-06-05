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

//#define DEBUG 1
//#define DEBUG_HEATERS


#if defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
    #define NUM_HEATERS 4
#else
    #define NUM_HEATERS 2
#endif

typedef enum {
    H1_HEATER = 0,
    H2_HEATER = 1,
    H3_HEATER = 2,
    H4_HEATER = 3
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
    GPIO_TypeDef    *GPIOx_H1_TEMP_V;
    uint16_t        GPIO_Pin_H1_TEMP_V;
    uint32_t        ADC_CHANNEL_H1_TEMP_V;
    
    GPIO_TypeDef    *GPIOx_AMP_H2_TEMP_V;
    uint16_t        GPIO_Pin_AMP_H2_TEMP_V;
    uint32_t        ADC_CHANNEL_H2_TEMP_V;
    
    GPIO_TypeDef    *GPIOx_H3_TEMP_V;
    uint16_t        GPIO_Pin_H3_TEMP_V;
    uint32_t        ADC_CHANNEL_H3_TEMP_V;
    
    GPIO_TypeDef    *GPIOx_AMP_H4_TEMP_V;
    uint16_t        GPIO_Pin_AMP_H4_TEMP_V;
    uint32_t        ADC_CHANNEL_H4_TEMP_V;
    
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
    GPIO_TypeDef    *GPIOx_H1_CTRL;
    uint16_t        GPIO_Pin_H1_CTRL;
    GPIO_TypeDef    *GPIOx_H2_CTRL;
    uint16_t        GPIO_Pin_H2_CTRL;
    GPIO_TypeDef    *GPIOx_H3_CTRL;
    uint16_t        GPIO_Pin_H3_CTRL;
    GPIO_TypeDef    *GPIOx_H4_CTRL;
    uint16_t        GPIO_Pin_H4_CTRL;
    GPIO_TypeDef    *GPIOx_PUSHBUTTON;
    uint16_t        GPIO_Pin_PUSHBUTTON;
} Pin_assignments_t;

/* Exported variables prototypes ---------------------------------------------*/
extern struct app_data_t data;
extern struct flags_t flags;
extern Pin_assignments_t Pins;


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

/*
*   File: timers.cpp
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*
*   This module manages a set of up to MAX_REGISTERED_TIMERS timers that are managed from a single physical timer.
*   Timers are configured (registered) once, and may be enabled or disabled as needed.
*   The ISRTimerData_t structure sets the TimerTickInterval and TimerCallbackFunc
*
*   When a timer fires, its TimerCallbackFunc is called as part of the timer ISR.
*   Minimal processing should be done in this ISR. 
*   Flags are recommended to alert user code to handle most events.
*/

#include "timers.h"
#include "main.h"
#include "app_data.h"

TIM_HandleTypeDef tim1Handle;
uint32_t TIM1_tick_count;
uint8_t Registered_ISRTimers;

ISRTimerData_t ISRTimers[MAX_REGISTERED_TIMERS];

void TIMER_Init(void)
{
    TIM1_tick_count = 0;
    
    // HAL_SetTickFreq configures the HAL tick timer
    // This has a minimum frequency of 1 kHz, which is too low for our application
    // This is not currently used.
    HAL_SetTickFreq(1);     // set the tick timer to 1 KHz (1 msec per tick)    
    
	/* 	Trigger 1000x per second, or every 1 ms
			Clock at 24MHz -> 8 million cycles
			Period to 10,000, prescaler to 800
			10,000*800=8,000,000 */
	
	tim1Handle.Instance = TIM1;																						//Timer 1 advanced timer
    tim1Handle.Init.Period            = 30 - 1;				    //Timer count = (period+1)*(prescaler+1), Period of 30 = 1 msec and 15 = 500 usec
    tim1Handle.Init.Prescaler         = 800 - 1;
    tim1Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;						//Use full clock rate
    tim1Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    tim1Handle.Init.RepetitionCounter = 1 - 1;
    tim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&tim1Handle) != HAL_OK)
    {
    APP_ErrorHandler();
    }

    if (HAL_TIM_Base_Start_IT(&tim1Handle) != HAL_OK)
    {
    APP_ErrorHandler();
    }
    
    Registered_ISRTimers = 0;
    
    for (int i=0; i < MAX_REGISTERED_TIMERS; i++) {
        ISRTimers[i].enabled = false;
        ISRTimers[i].TimerCallbackFunc = (TimerCallback) 0;
    }
}

int8_t Register_timer(TimerCallback CallbackFunc, uint32_t TickInterval)
{
    if (Registered_ISRTimers >= MAX_REGISTERED_TIMERS) 
        return -1;
     
    ISRTimers[Registered_ISRTimers].enabled = false;
    ISRTimers[Registered_ISRTimers].TimerCallbackFunc = CallbackFunc;
    ISRTimers[Registered_ISRTimers].TimerTickInterval = TickInterval;
    
    return Registered_ISRTimers++;     
}

bool Enable_timer(int8_t TimerNumber) {
    if (TimerNumber < 0 || TimerNumber >= Registered_ISRTimers) return false;
    
    ISRTimers[TimerNumber].TimerLastTickTime = 0;
    
    // align timers >= 1 second on the 1 second boundary:
    if (ISRTimers[TimerNumber].TimerTickInterval >= TICKS_PER_SEC) {
        ISRTimers[TimerNumber].TimerNextTickTime = TIM1_tick_count + TICKS_PER_SEC - (TIM1_tick_count%TICKS_PER_SEC) + ISRTimers[TimerNumber].TimerTickInterval;
    } else {
        ISRTimers[TimerNumber].TimerNextTickTime = TIM1_tick_count + ISRTimers[TimerNumber].TimerTickInterval;
    }
    ISRTimers[TimerNumber].enabled = true;
    return true;
}

bool Disable_timer(int8_t TimerNumber) {
    if (TimerNumber < 0 || TimerNumber >= Registered_ISRTimers) return false;
    
    ISRTimers[TimerNumber].enabled = false;
    return true;
}

// Wait_until_tick uses the Timer1 callback variable TIM1_tick_count for precise timing delays in user space
uint32_t Wait_until_tick(uint32_t tick, uint32_t max_wait)
{
    uint32_t start_tick, current_tick, wait_time;
	start_tick = TIM1_tick_count;
    if (start_tick >= tick) return 0;
    
    do {
        current_tick = TIM1_tick_count;
        wait_time = current_tick - start_tick;
    } while (current_tick < tick  && wait_time < max_wait);
    return wait_time;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{   
    static uint8_t msec_subcounter = 0;
    TIM1_tick_count++;
    
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);    
    
    // __disable_irq(); // Set PRIMASK
    
    for (int timer=0; timer < MAX_REGISTERED_TIMERS; timer++) {
        if (ISRTimers[timer].enabled) {
            if (TIM1_tick_count >= ISRTimers[timer].TimerNextTickTime) {
                ISRTimers[timer].TimerLastTickTime = ISRTimers[timer].TimerNextTickTime;
                ISRTimers[timer].TimerNextTickTime = ISRTimers[timer].TimerLastTickTime + ISRTimers[timer].TimerTickInterval;
                
                // This means that we are not keeping up or that the timer has just been enabled:
                if (ISRTimers[timer].TimerNextTickTime <= TIM1_tick_count) {        
                    ISRTimers[timer].TimerNextTickTime = TIM1_tick_count + ISRTimers[timer].TimerTickInterval;
                }
                
                if (ISRTimers[timer].TimerCallbackFunc)
                    ISRTimers[timer].TimerCallbackFunc();  // call the callback function
            }
        }
    }
    
    if (msec_subcounter == (TICKS_PER_MSEC - 1)) {
        msec_subcounter = 0;
        data.msec_tick_count++;
    } else {
        msec_subcounter++;
    }
    
    // __enable_irq(); // Clear PRIMASK    
}

void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI; /* ?????HSE,HSI,LSI */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* ??HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI 1?? */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;  /* Clock at 24MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* ??HSE */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* ??LSI */

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* ?????? HCLK,SYSCLK,PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; /* ??HSI?????? */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* AHB?? 1?? */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* APB?? 1?? */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}



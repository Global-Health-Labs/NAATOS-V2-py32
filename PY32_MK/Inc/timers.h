/*
*   File: timers.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef TIMERS_H
#define TIMERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "py32f0xx_hal.h"

#define TICKS_PER_MSEC 1
#define TICKS_PER_SEC (1000 * TICKS_PER_MSEC)

#define MAX_REGISTERED_TIMERS 12

extern TIM_HandleTypeDef tim3Handle;
extern uint32_t SYS_tick_count;

typedef void (*TimerCallback) ();
typedef struct 
  {
    bool          enabled;
    TimerCallback TimerCallbackFunc;
    uint32_t      TimerTickInterval;
    uint32_t      TimerLastTickTime;
    uint32_t      TimerNextTickTime;
  } ISRTimerData_t;
  

void TIMER_Init(void);
uint32_t Wait_until_tick(uint32_t tick, uint32_t max_wait);
void APP_SystemClockConfig(void);

int8_t Register_timer(TimerCallback CallbackFunc, uint32_t TickInterval);
bool Enable_timer(int8_t TimerNumber);
bool Disable_timer(int8_t TimerNumber);    
bool Update_TimerTickInterval(int8_t TimerNumber, uint32_t new_TimerTickInterval);  

#ifdef __cplusplus
}
#endif

#endif
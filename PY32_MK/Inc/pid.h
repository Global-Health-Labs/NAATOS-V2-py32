/*
*   File: pid.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef PID_H
#define PID_H

/* Based on PID Conroller:
   https://github.com/geekfactory/PID
*/

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* PID Controller Parameters */
#define PID_LIM_MIN 0.0f
#define PID_ANTIWINDUP_ERR_RANGE 6.0f
#define PID_DELTA_T 0.5f        // PID is run twice per second

/*CONTROL structure
  holds process steps for each STATE in application. Each [INDEX] maps to enum state_machine
*/

#ifdef BOARDCONFIG_MK6F
#define PID_SH_P_TERM 5
#define PID_SH_I_TERM 0.5
#define PID_SH_D_TERM 0.333
#define PID_VH_P_TERM 5
#define PID_VH_I_TERM 0.5
#define PID_VH_D_TERM 0.333
#else
#define PID_SH_P_TERM 45.0
#define PID_SH_I_TERM 1
#define PID_SH_D_TERM 0.333
#define PID_VH_P_TERM 45.0
#define PID_VH_I_TERM 2
#define PID_VH_D_TERM 10

#define PID_P_RAMP_TERM 300

//#define PID_SH_P_TERM 200
//#define PID_SH_I_TERM 15
//#define PID_SH_D_TERM 0
//#define PID_VH_P_TERM 75
//#define PID_VH_I_TERM 10
//#define PID_VH_D_TERM .25

#define PID_VH_P_TERM_ACT 75
#define PID_VH_I_TERM_ACT 10
#define PID_VH_D_TERM_ACT 6

//#define PID_SH_P_TERM 45
//#define PID_SH_I_TERM 0.5
//#define PID_SH_D_TERM 0.333
//#define PID_VH_P_TERM 45
//#define PID_VH_I_TERM 0.5
//#define PID_VH_D_TERM 0.333
#endif

typedef struct {
  /* Controller Gains */
  float k_p;
  float k_i;
  float k_d;
  /* Output Limits */
  float lim_max;
  float lim_min;
  float slew_rate;
  /* Controller */
  float integrator;
  float prevMesurement;
  /* Controller Output */
  float out;
  /* Controller Setpoint */
  float setpoint;
  float error, lastError, lastInput;
  float outputSum;
  uint32_t lastTime;
  bool  ramp_start;
  float pTerm;
  float iTerm;
  float dTerm;
  
} pid_controller_t;

void pid_controller_init(heater_t heater, float setpoint, float k_p, float k_i, float k_d, int pid_max, float slew_rate);
void pid_controller_compute(heater_t heater, float measurement);

extern pid_controller_t pid_data[];

#endif
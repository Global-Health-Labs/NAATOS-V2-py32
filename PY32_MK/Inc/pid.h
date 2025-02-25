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

/* PID Controller Parameters */
#define PID_LIM_MIN 0.0f

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
  float pTerm;
  float iTerm;
  float dTerm;
  
} pid_controller_t;

void pid_controller_init(pid_controller_t *pid, float setpoint, float k_p, float k_i, float k_d, int pid_max, float slew_rate);
void pid_controller_compute(pid_controller_t *pid, float measurement);

#endif